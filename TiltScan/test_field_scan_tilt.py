"""Tests for TiltScan/field_scan_tilt.py with the simulated robot and tilting sensor.

    python -m pytest TiltScan/ -v
"""
import csv
import json
import math
import sys
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
