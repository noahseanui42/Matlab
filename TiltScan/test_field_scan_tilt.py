"""Tests for TiltScan/field_scan_tilt.py with the simulated robot and tilting sensor.

    python -m pytest TiltScan/ -v
"""
import csv
import json
import math
import sys
import threading
import time
from types import SimpleNamespace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import field_scan_tilt as ft
fs = ft.fs


def make_cfg(tmp_path, **kw):
    cfg = ft.TiltScanConfig(out_dir=str(tmp_path), settle_s=5.0, sample_dt_s=0.02, n_avg=5,
                            xr=(-50, 50), nx=3, yr=(-50, 50), ny=2, zr=(-700, -600), nz=2)
    for k, v in kw.items():
        setattr(cfg, k, v)
    return cfg


def scan(tmp_path, link=None, **kw):
    cfg = make_cfg(tmp_path, **kw)
    clock = ft.VirtualClock()
    link = link or fs.FakeLink()
    sensor = ft.FakeTiltSensor(link, clock)
    p = ft.run_scan(link, sensor, cfg, "t", confirm=lambda *_: None, out=lambda *_: None, clock=clock)
    return p, list(csv.DictReader(open(p))), json.loads(p.with_suffix(".meta.json").read_text())


def test_columns_extend_field_scan(tmp_path):
    p, rows, meta = scan(tmp_path)
    assert list(rows[0].keys()) == fs.CSV_COLUMNS + ft.TILT_COLUMNS
    assert ft.CSV_COLUMNS[:len(fs.CSV_COLUMNS)] == fs.CSV_COLUMNS   # existing MATLAB scripts still read it
    assert len(rows) == 12 and meta["points_done"] == 12 and meta["tilt"]["settle_mode"] == "fixed"


def test_accelerometer_gives_the_simulated_tilt(tmp_path):
    p, rows, _ = scan(tmp_path)
    for r in rows:
        a = [float(r[k]) for k in ("ax_g", "ay_g", "az_g")]
        assert abs(math.sqrt(sum(v * v for v in a)) - 1.0) < 2e-3          # gravity, 1 g
        tilt = math.degrees(math.acos(-a[2] / math.sqrt(sum(v * v for v in a))))
        x, y = float(r["x_mm"]), float(r["y_mm"])
        expect = math.degrees(math.acos(math.cos(math.radians(0.01 * x)) * math.cos(math.radians(0.01 * y))))
        assert abs(tilt - expect) < 0.05


def test_fixed_settle_waits_the_set_time(tmp_path):
    p, rows, _ = scan(tmp_path, settle_s=2.0)
    assert all(r["settled"] == "-1" and abs(float(r["settle_s"]) - 2.0) < 1e-6 for r in rows)


def test_gyro_settle_stops_when_quiet(tmp_path):
    # rings down from 5 deg/s with tau 1 s: below 0.5 deg/s after about 2.3 s, then 1 s quiet
    p, rows, meta = scan(tmp_path, gyro_settle_dps=0.5, still_s=1.0, settle_min_s=0.5, settle_s=10.0)
    waits = [float(r["settle_s"]) for r in rows]
    assert all(r["settled"] == "1" for r in rows)
    assert all(3.0 <= t <= 4.5 for t in waits), waits
    assert meta["tilt"]["settle_mode"] == "gyro" and meta["points_not_settled"] == 0


def test_gyro_settle_times_out(tmp_path):
    # threshold below the gyro noise (0.05 deg/s per axis): never quiet, waits the full --settle
    p, rows, meta = scan(tmp_path, gyro_settle_dps=0.01, settle_s=3.0)
    assert all(r["settled"] == "0" and abs(float(r["settle_s"]) - 3.0) < 0.05 for r in rows)
    assert meta["points_not_settled"] == 12


def test_unreachable_points_have_nan_tilt(tmp_path):
    p, rows, meta = scan(tmp_path, zr=(-700, -400), nz=2)    # z = -400 is out of reach
    bad = [r for r in rows if r["err"] == "1"]
    assert len(bad) == 6 and meta["points_unreachable"] == 6
    assert all(r["ax_g"] == "nan" and r["gyro_rms_dps"] == "nan" and r["settled"] == "-1" for r in bad)


def test_cli_simulate(tmp_path):
    ft.main(["--simulate", "--label", "cli", "--out-dir", str(tmp_path), "--x", "-50", "50", "2",
             "--y", "0", "0", "1", "--z", "-650", "-650", "1", "--gyro-settle", "0.5"])
    rows = list(csv.DictReader(open(next(tmp_path.glob("cli_*.csv")))))
    assert len(rows) == 2 and all(r["settled"] == "1" for r in rows)


def test_pitch_and_roll_columns(tmp_path):
    p, rows, _ = scan(tmp_path)
    for r in rows:
        x, y = float(r["x_mm"]), float(r["y_mm"])
        pitch, roll = float(r["pitch_deg"]), float(r["roll_deg"])
        assert abs(pitch - 0.01 * x) < 0.02 and abs(roll + 0.01 * y) < 0.02          # board's filter
        assert abs(float(r["acc_pitch_deg"]) - pitch) < 0.05                       # from gravity
        assert abs(float(r["acc_roll_deg"]) - roll) < 0.05


def test_accel_pitch_roll_either_way_up():
    a = math.radians(2.0)
    down = (math.sin(a), 0.0, -math.cos(a))      # board reads -1 g on z when level
    up = (-math.sin(a), 0.0, math.cos(a))        # ... or +1 g
    assert all(abs(v - w) < 1e-9 for v, w in zip(ft.accel_pitch_roll(down), ft.accel_pitch_roll(up)))
    assert abs(ft.accel_pitch_roll(down)[0] - 2.0) < 1e-9
    assert all(math.isnan(v) for v in ft.accel_pitch_roll((math.nan, 0, 1)))


class StandInSpatial:
    """Stands in for Phidget22's Spatial channel: Euler angles only."""
    def __init__(self):
        self.closed = False

    def getEulerAngles(self):
        return SimpleNamespace(pitch=1.5, roll=-0.5, heading=10.0)

    def close(self):
        self.closed = True


def test_spatial_channel_buffers_events():
    sensor = ft.Phidget1044Spatial(channel=StandInSpatial())
    sensor.algorithm = "imu"
    stop = threading.Event()

    def feed():                      # the Phidget library's event thread, 250 Hz
        k = 0
        while not stop.is_set():
            mag = (1e300, 0.0, 0.0) if k == 3 else (0.2, 0.05, -0.5)    # one unknown reading
            sensor._on_data(None, (0.0, 0.0, -1.0), (0.1, 0.0, 0.0), mag, k * 4.0)
            k += 1
            time.sleep(0.004)

    t = threading.Thread(target=feed, daemon=True)
    t.start()
    try:
        r = ft.read_point(sensor, 20, 0.004)
        assert abs(sensor.gyro()[0] - 0.1) < 1e-12
    finally:
        stop.set()
        t.join()
    assert 18 <= r["n"] <= 20                       # the 1e300 reading (if in this window) dropped
    assert abs(r["B"][2] + 0.5) < 1e-12 and abs(r["a"][2] + 1.0) < 1e-12
    assert r["pitch"] == 1.5 and r["roll"] == -0.5
    assert all(abs(v) < 1e-9 for v in r["acc_pr"])
    t0, t1 = r["ts"]                                # consecutive events, 4 ms apart
    assert t1 - t0 == 4.0 * 19 and t0 % 4.0 == 0
    sensor.close()


def test_spatial_without_algorithm_gives_nan_pitch_roll():
    sensor = ft.Phidget1044Spatial(channel=StandInSpatial())     # algorithm stays "none"
    sensor._on_data(None, (0.0, 0.0, -1.0), (0.0, 0.0, 0.0), (0.2, 0.0, 0.0), 0.0)
    assert sensor.euler() is None
    assert sensor.info()["algorithm"] == "none"


class FakeSpatialChannel:
    """Phidget22 Spatial look-alike: sends events from a thread once opened."""
    refuse_algorithm = False

    def __init__(self):
        self.handler, self.algorithm, self.interval, self.serial = None, None, None, None
        self.zeroed = False
        self._stop = threading.Event()

    def setDeviceSerialNumber(self, n): self.serial = n
    def setOnSpatialDataHandler(self, h): self.handler = h
    def getMinDataInterval(self): return 4
    def setDataInterval(self, ms): self.interval = ms
    def getDataInterval(self): return self.interval
    def getDeviceSerialNumber(self): return self.serial
    def getDeviceName(self): return "PhidgetSpatial Precision 3/3/3 High Resolution"
    def getMaxMagneticField(self): return [5.6, 5.6, 5.6]
    def zeroGyro(self): self.zeroed = True
    def getEulerAngles(self): return SimpleNamespace(pitch=0.3, roll=0.2, heading=0.0)

    def setAlgorithm(self, alg):
        if self.refuse_algorithm:
            raise RuntimeError("Not Supported")
        self.algorithm = alg

    def openWaitForAttachment(self, ms):
        def run():
            k = 0
            while not self._stop.is_set():
                self.handler(self, [0.0, 0.0, -1.0], [0.0, 0.0, 0.0], [0.2, 0.05, -0.5], 20.0 * k)
                k += 1
                time.sleep(0.004)
        threading.Thread(target=run, daemon=True).start()

    def close(self): self._stop.set()


def test_spatial_open_path(monkeypatch):
    import Phidget22.Devices.Spatial  # noqa: F401
    mod = sys.modules["Phidget22.Devices.Spatial"]   # the module (the package exports the class)
    from Phidget22.SpatialAlgorithm import SpatialAlgorithm
    monkeypatch.setattr(mod, "Spatial", FakeSpatialChannel)
    sensor = ft.Phidget1044Spatial(302277, 0.02, "imu")
    try:
        assert sensor.sp.serial == 302277 and sensor.sp.interval == 20
        assert sensor.sp.algorithm == SpatialAlgorithm.SPATIAL_ALGORITHM_IMU
        info = sensor.info()
        assert info["algorithm"] == "imu" and info["max_field_G"] == [5.6] * 3
        r = ft.read_point(sensor, 5, 0.02)
        assert r["n"] == 5 and (r["pitch"], r["roll"]) == (0.3, 0.2)
        assert r["ts"][1] - r["ts"][0] == 20.0 * 4 and sensor.latest_timestamp() >= r["ts"][1]
        sensor.zero_gyro(ft.VirtualClock())
        assert sensor.sp.zeroed
    finally:
        sensor.close()


def test_spatial_board_refuses_algorithm(monkeypatch, capsys):
    import Phidget22.Devices.Spatial  # noqa: F401
    mod = sys.modules["Phidget22.Devices.Spatial"]   # the module (the package exports the class)
    monkeypatch.setattr(FakeSpatialChannel, "refuse_algorithm", True)
    monkeypatch.setattr(mod, "Spatial", FakeSpatialChannel)
    sensor = ft.Phidget1044Spatial(0, 0.02, "ahrs")
    try:
        assert "would not run the ahrs algorithm" in capsys.readouterr().out
        r = ft.read_point(sensor, 5, 0.02)
        assert math.isnan(r["pitch"]) and r["n"] == 5 and all(math.isfinite(v) for v in r["acc_pr"])
        assert sensor.info()["algorithm"] == "none" and "Not Supported" in sensor.info()["algorithm_error"]
    finally:
        sensor.close()


def test_timestamps_per_row(tmp_path):
    p, rows, meta = scan(tmp_path, zr=(-700, -400), nz=2)    # some unreachable rows too
    t0 = meta["spatial_timestamp_at_t0_ms"]
    prev = t0
    for r in rows:
        if r["err"] == "1":
            assert r["spatial_t_first_ms"] == "nan" and r["spatial_t_last_ms"] == "nan"
            continue
        a, b = float(r["spatial_t_first_ms"]), float(r["spatial_t_last_ms"])
        assert abs((b - a) - 4 * 20.0) < 1e-6          # n_avg 5 readings, 20 ms apart
        assert a > prev                                  # after the previous row, and after t0
        # the window ends just before the row is stamped (t_s, s since the scan start)
        assert 0 <= float(r["t_s"]) * 1000 - (b - t0) <= 25
        prev = b


def test_magnet_position_in_name_and_meta(tmp_path):
    cfg = make_cfg(tmp_path, nx=1, ny=1, nz=1)
    clock = ft.VirtualClock()
    link = fs.FakeLink()
    p = ft.run_scan(link, ft.FakeTiltSensor(link, clock), cfg, "magnet_tilt", confirm=lambda *_: None,
                    out=lambda *_: None, clock=clock, magnet="Underneath")
    assert p.name.startswith("magnet_tilt_underneath_") and p.suffix == ".csv"
    meta = json.loads(p.with_suffix(".meta.json").read_text())
    assert meta["magnet"] == "Underneath" and meta["csv"] == p.name
    assert ft.magnet_slug("under centre, N up") == "under-centre-n-up" and ft.magnet_slug("") == ""
    # no --magnet: names as before
    p2 = ft.run_scan(link, ft.FakeTiltSensor(link, clock), cfg, "bg_tilt", confirm=lambda *_: None,
                     out=lambda *_: None, clock=clock)
    assert p2.name.startswith("bg_tilt_2") and json.loads(p2.with_suffix(".meta.json").read_text())["magnet"] == ""
