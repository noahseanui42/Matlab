"""Field scan with tilt logging: the same scan as delta_app/field_scan.py, plus the
1044's accelerometer (tilt) and gyroscope (is the probe still?) at every point.

    python TiltScan/field_scan_tilt.py --label magnet_tilt --note "strong magnet"
    python TiltScan/field_scan_tilt.py --label magnet_tilt --correction hybrid
    python TiltScan/field_scan_tilt.py --label t --gyro-settle 0.5   # wait for the gyro, not a fixed time
    python TiltScan/field_scan_tilt.py --simulate --label demo        # no hardware

field_scan.py is NOT changed: this script imports its robot link, moves, grid and
position correction, and only replaces the per-point reading and the CSV writing.
All field_scan.py options work the same way. Output goes to TiltScan/data/ by default.

Per point: move, settle, then read the magnetometer AND the accelerometer in the same
loop (n_avg samples) and log both in one row. While the probe is still, the
accelerometer measures gravity only, so its direction gives the board's tilt.
MATLAB (TiltScan/tilt_correct.m) turns that into a tilt angle per point and
rotates B back to a reference orientation.

Settling: by default a fixed --settle wait, as in field_scan.py. With
--gyro-settle RATE (deg/s) it waits until the gyro's angular rate has stayed below
RATE for --still seconds, at least --settle-min and at most --settle seconds. Run
a scan with the fixed wait first and look at gyro_rms_dps to choose RATE.

Extra CSV columns, after field_scan.py's 19 (so existing MATLAB scripts still read it):
    ax_g, ay_g, az_g            mean acceleration, g, SENSOR axes (gravity when still)
    ax_std_g, ay_std_g, az_std_g  std over the samples
    gyro_rms_dps, gyro_max_dps  angular rate magnitude while sampling, deg/s
    settle_s                    time waited after the move, s
    settled                     1 gyro went quiet, 0 gyro wait timed out, -1 fixed wait
"""

import argparse
import csv
import json
import math
import random
import statistics
import sys
import time
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "engr489-firmware" / "delta_app"))
import field_scan as fs   # noqa: E402  (reused unchanged)

TILT_COLUMNS = [
    "ax_g", "ay_g", "az_g", "ax_std_g", "ay_std_g", "az_std_g",
    "gyro_rms_dps", "gyro_max_dps", "settle_s", "settled",
]
CSV_COLUMNS = fs.CSV_COLUMNS + TILT_COLUMNS


@dataclass
class TiltScanConfig(fs.ScanConfig):
    out_dir: str = str(HERE / "data")
    gyro_settle_dps: float = 0.0     # 0: fixed settle_s wait; > 0: wait for the gyro to go quiet
    still_s: float = 1.0             # ... and stay below gyro_settle_dps this long
    settle_min_s: float = 0.5        # gyro mode: always wait at least this
    zero_gyro: bool = True           # zero the gyro at the park position before the scan


class Clock:
    """Real time. The simulation swaps in a virtual clock so waits take no time."""
    now = staticmethod(time.monotonic)
    sleep = staticmethod(time.sleep)


# --------------------------------------------------------------------------
# Sensor: three channels of the same 1044
# --------------------------------------------------------------------------
class Phidget1044Tilt:
    """Magnetometer, accelerometer and gyroscope channels of one 1044, opened together."""

    GETTERS = {"mag": "getMagneticField", "acc": "getAcceleration", "gyro": "getAngularRate"}

    def __init__(self, serial_number=0, sample_dt_s=0.02):
        from Phidget22.Devices.Magnetometer import Magnetometer
        from Phidget22.Devices.Accelerometer import Accelerometer
        from Phidget22.Devices.Gyroscope import Gyroscope
        self.ch = {}
        try:
            for name, cls in (("mag", Magnetometer), ("acc", Accelerometer), ("gyro", Gyroscope)):
                c = cls()
                if serial_number:
                    c.setDeviceSerialNumber(int(serial_number))
                c.openWaitForAttachment(5000)
                self.ch[name] = c
            # The 1044 may share one data rate between its channels: set all, then read back.
            for c in self.ch.values():
                c.setDataInterval(max(c.getMinDataInterval(), round(sample_dt_s * 1000)))
            t0 = time.monotonic()
            for name, getter in self.GETTERS.items():   # first sample takes a moment after attach
                while True:
                    try:
                        getattr(self.ch[name], getter)()
                        break
                    except Exception:
                        if time.monotonic() - t0 > 3:
                            raise fs.ScanError(f"1044 {name} channel attached but sent no data within 3 s.")
                        time.sleep(0.05)
        except Exception:
            self.close()
            raise

    def info(self):
        m = self.ch["mag"]
        d = {"serial": m.getDeviceSerialNumber(), "name": m.getDeviceName(),
             "data_interval_ms": {k: c.getDataInterval() for k, c in self.ch.items()}}
        for key, ch, getter in (("max_field_G", "mag", "getMaxMagneticField"),
                                ("max_accel_g", "acc", "getMaxAcceleration"),
                                ("max_rate_dps", "gyro", "getMaxAngularRate")):
            try:
                d[key] = getattr(self.ch[ch], getter)()
            except Exception:
                pass
        return d

    def mag(self):
        return tuple(self.ch["mag"].getMagneticField())

    def accel(self):
        return tuple(self.ch["acc"].getAcceleration())

    def gyro(self):
        return tuple(self.ch["gyro"].getAngularRate())

    def zero_gyro(self, clock=Clock):
        self.ch["gyro"].zero()      # Phidget: re-zeros in 1-2 s, board must be still
        clock.sleep(2.5)

    def after_move(self):
        pass

    def close(self):
        for c in self.ch.values():
            try:
                c.close()
            except Exception:
                pass


# --------------------------------------------------------------------------
# Per-point reading
# --------------------------------------------------------------------------
def _norm(v):
    return math.sqrt(sum(x * x for x in v))


def _mean_std(rows):
    if not rows:
        return [math.nan] * 3, [math.nan] * 3
    cols = list(zip(*rows))
    return ([statistics.fmean(c) for c in cols],
            [statistics.stdev(c) if len(c) > 1 else 0.0 for c in cols])


def wait_settled(sensor, cfg, clock=Clock):
    """Wait after a move. Returns (seconds waited, settled flag: 1 / 0 / -1)."""
    t0 = clock.now()
    if cfg.gyro_settle_dps <= 0:
        clock.sleep(cfg.settle_s)
        return clock.now() - t0, -1
    clock.sleep(min(cfg.settle_min_s, cfg.settle_s))
    still_since = None
    while True:
        now = clock.now()
        try:
            rate = _norm(sensor.gyro())
        except Exception:
            rate = math.inf
        if rate < cfg.gyro_settle_dps:
            if still_since is None:
                still_since = now
            if now - still_since >= cfg.still_s:
                return now - t0, 1
        else:
            still_since = None
        if now - t0 >= cfg.settle_s:
            return now - t0, 0
        clock.sleep(max(cfg.sample_dt_s, 0.01))


def read_point(sensor, n, dt_s, clock=Clock):
    """n samples dt_s apart of field, acceleration and gyro, read in the same loop.
    Returns (B mean, B std, n_B, a mean, a std, gyro rms, gyro max). Readings the
    sensor rejects are dropped (per sensor)."""
    B, A, W = [], [], []
    for _ in range(n):
        for read, store in ((sensor.mag, B), (sensor.accel, A), (sensor.gyro, W)):
            try:
                store.append(read())
            except Exception:
                pass
        clock.sleep(dt_s)
    Bm, Bs = _mean_std(B)
    Am, As = _mean_std(A)
    rates = [_norm(w) for w in W]
    g_rms = math.sqrt(statistics.fmean(r * r for r in rates)) if rates else math.nan
    g_max = max(rates) if rates else math.nan
    return Bm, Bs, len(B), Am, As, g_rms, g_max


# --------------------------------------------------------------------------
# Scan (field_scan.run_scan with the tilt reading and columns)
# --------------------------------------------------------------------------
def run_scan(link, sensor, cfg, label="scan", note="", coil_current_A=None,
             confirm=input, out=print, clock=Clock):
    pts = fs.scan_grid(cfg.xr, cfg.yr, cfg.zr, cfg.nx, cfg.ny, cfg.nz)
    n = len(pts)
    corr = fs.load_correction(cfg)
    n_outside = sum(not corr.in_valid_box(p) for p in pts) if corr else 0
    mismatch = {}
    if corr:
        import pose_correction
        mismatch = pose_correction.geometry_mismatch()
        if mismatch:
            out("WARNING: robot_config.py geometry differs from the geometry the correction was "
                f"calibrated with {mismatch} (robot_config value, calibrated value). The correction uses "
                "the calibrated geometry. Make sure the robot runs the calibrated firmware "
                "(branch claude/amazing-lovelace-onixrv).")
    if n_outside:
        out(f"WARNING: {n_outside} of {n} points are outside the box the {cfg.correction} "
            f"correction was validated in ({corr.valid_box}); it is extrapolating there.")
    out_dir = Path(cfg.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = out_dir / f"{label}_{stamp}.csv"
    meta_path = csv_path.with_suffix(".meta.json")

    fs.ensure_enabled(link, confirm)
    out(f"Moving to park position {cfg.park_xyz}...")
    err, _ = fs.move_probe(link, cfg, cfg.park_xyz)
    if err == 1:
        raise fs.ScanError(f"Park position {cfg.park_xyz} is unreachable.")
    sensor.after_move()
    gyro_zeroed = False
    if cfg.zero_gyro:
        out("Zeroing the gyro at the park position (keep the robot still)...")
        clock.sleep(cfg.settle_s)
        sensor.zero_gyro(clock)
        gyro_zeroed = True

    meta = {
        "label": label, "note": note, "started": datetime.now().isoformat(timespec="seconds"),
        "coil_current_A": coil_current_A, "n_points": n, "csv": csv_path.name,
        "script": "TiltScan/field_scan_tilt.py",
        "frame": "sensor axes, raw: B in gauss, acceleration in g, angular rate in deg/s "
                 "(tilt_correct.m and R_sensor_to_robot in MATLAB)",
        "config": asdict(cfg), "sensor": sensor.info() if hasattr(sensor, "info") else {},
        "geometry": fs.CALIBRATED_GEOMETRY,
        "correction": dict(corr.info(), points_outside_valid_box=n_outside,
                           geometry_mismatch_with_robot_config={k: list(v) for k, v in mismatch.items()})
                      if corr else {"name": "off"},
        "positions": "x_mm..z_mm = target; sent_x_mm..sent_z_mm = coordinates sent to the robot",
        "tilt": {"columns": TILT_COLUMNS, "gyro_zeroed": gyro_zeroed,
                 "settle_mode": "gyro" if cfg.gyro_settle_dps > 0 else "fixed",
                 "settled": "1 gyro quiet, 0 gyro wait timed out, -1 fixed wait"},
        "finished": None, "points_done": 0, "points_unreachable": 0,
        "points_not_settled": 0, "aborted": False,
    }

    def write_meta():
        meta_path.write_text(json.dumps(meta, indent=2))

    write_meta()
    t_scan = clock.now()
    n_skipped = n_unsettled = 0
    nan3 = [math.nan] * 3
    try:
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(CSV_COLUMNS)
            f.flush()
            out(f"Scanning {n} points -> {csv_path}")
            for k, p in enumerate(pts, 1):
                send, why = fs.corrected_target(corr, p)
                if send is None:
                    out(f"  point {k} {p}: correction {why}; logged as unreachable")
                    err, st = 1, {}
                else:
                    err, st = fs.move_probe(link, cfg, send)
                    sensor.after_move()
                if err == 1:
                    B, Bsd, ns, a, asd, g_rms, g_max = nan3, nan3, 0, nan3, nan3, math.nan, math.nan
                    waited, settled = 0.0, -1
                    n_skipped += 1
                else:
                    waited, settled = wait_settled(sensor, cfg, clock)
                    n_unsettled += settled == 0
                    B, Bsd, ns, a, asd, g_rms, g_max = read_point(sensor, cfg.n_avg, cfg.sample_dt_s, clock)
                deg = list(st.get("deg", nan3))
                w.writerow([k, *(f"{v:.2f}" for v in p),
                            *(f"{v:.6f}" for v in B), *(f"{v:.6f}" for v in Bsd),
                            err, f"{clock.now() - t_scan:.2f}", ns,
                            *(f"{d:.3f}" for d in deg),
                            *(f"{v:.2f}" for v in (send or nan3)),
                            *(f"{v:.6f}" for v in a), *(f"{v:.6f}" for v in asd),
                            f"{g_rms:.4f}", f"{g_max:.4f}", f"{waited:.2f}", settled])
                f.flush()
                meta["points_done"] = k
                an = _norm(a)
                tilt = math.degrees(math.acos(min(1.0, abs(a[2]) / an))) if an > 0 else math.nan
                eta = (clock.now() - t_scan) / k * (n - k)
                out(f"{k:4d}/{n}  [{p[0]:7.1f} {p[1]:7.1f} {p[2]:7.1f}]  "
                    f"|B| = {_norm(B):.4f} G  tilt from sensor z {tilt:6.3f} deg  "
                    f"gyro {g_rms:.2f} deg/s  settle {waited:.1f} s{'' if settled else ' (gyro not quiet)'}  "
                    f"e={err}  ({eta:.0f} s left)")
    except KeyboardInterrupt:
        meta["aborted"] = True
        out("Interrupted; data so far is saved.")
    finally:
        meta["points_unreachable"] = n_skipped
        meta["points_not_settled"] = n_unsettled
        meta["finished"] = datetime.now().isoformat(timespec="seconds")
        write_meta()

    out(f"Done in {clock.now() - t_scan:.0f} s. {n_skipped} of {n} points unreachable"
        + (f", {n_unsettled} where the gyro never went quiet." if cfg.gyro_settle_dps > 0 else "."))
    try:
        fs.move_probe(link, cfg, cfg.park_xyz)
        out("Parked. Robot left ENABLED (disabling lets the arms drop).")
    except fs.ScanError as e:
        out(f"Could not park: {e}")
    return csv_path


# --------------------------------------------------------------------------
# Simulated sensor (for --simulate and tests)
# --------------------------------------------------------------------------
class VirtualClock:
    """Time that only moves when something sleeps, so simulated waits are instant."""

    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def sleep(self, s):
        self.t += max(0.0, s)


def _rot(axis, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    if axis == "x":
        return [[1, 0, 0], [0, c, -s], [0, s, c]]
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def _matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _apply_T(R, v):          # R^T v: a world vector seen in the tilted sensor's axes
    return tuple(sum(R[k][i] * v[k] for k in range(3)) for i in range(3))


class FakeTiltSensor:
    """A 1044 on a platform that tips outwards by tilt_deg_per_mm per mm off the axis.
    The field (Earth, constant) and gravity are fixed in the world, so both readings
    rotate with the tilt; tilt_correct.m should take that rotation back out. The gyro
    rings down exponentially after each move. Accelerometer reads (0, 0, -1) g level
    (simulation only: check the real 1044's sign)."""

    def __init__(self, link, clock, B_world=(0.2, 0.05, -0.5), tilt_deg_per_mm=0.01,
                 noise_B=0.0005, noise_a=0.0003, noise_w=0.05, ring_dps=5.0, ring_tau_s=1.0, seed=0):
        self.link, self.clock = link, clock
        self.B_world, self.k = B_world, tilt_deg_per_mm
        self.noise_B, self.noise_a, self.noise_w = noise_B, noise_a, noise_w
        self.ring, self.tau = ring_dps, ring_tau_s
        self.t_move = -1e9
        self.r = random.Random(seed)

    def _R(self):
        x, y, _ = self.link.pos or (0.0, 0.0, -650.0)
        return _matmul(_rot("y", self.k * x), _rot("x", -self.k * y))

    def tilt_deg(self):
        return math.degrees(math.acos(max(-1.0, min(1.0, self._R()[2][2]))))

    def info(self):
        return {"serial": 0, "name": "simulated (tilting platform)", "max_field_G": [5.6] * 3}

    def mag(self):
        return tuple(b + self.r.gauss(0, self.noise_B) for b in _apply_T(self._R(), self.B_world))

    def accel(self):
        return tuple(a + self.r.gauss(0, self.noise_a) for a in _apply_T(self._R(), (0.0, 0.0, -1.0)))

    def gyro(self):
        amp = self.ring * math.exp(-(self.clock.now() - self.t_move) / self.tau)
        return tuple(amp * d + self.r.gauss(0, self.noise_w) for d in (0.6, 0.8, 0.0))

    def zero_gyro(self, clock=None):
        pass

    def after_move(self):
        self.t_move = self.clock.now()

    def close(self):
        pass


# --------------------------------------------------------------------------
def main(argv=None):
    cfg = TiltScanConfig()
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--label", default="scan")
    ap.add_argument("--note", default="", help="free text stored in the metadata, e.g. 'magnet at (200, 0, -720)'")
    ap.add_argument("--coil-current", type=float, default=None, help="coil current (A), stored in metadata")
    ap.add_argument("--port", default=cfg.port)
    ap.add_argument("--speed", type=int, choices=range(1, 11), default=cfg.speed_v, help="1..10 (5..50 mm/s)")
    ap.add_argument("--settle", type=float, default=cfg.settle_s,
                    help="fixed wait after each move, s; with --gyro-settle, the longest wait")
    ap.add_argument("--gyro-settle", type=float, default=cfg.gyro_settle_dps, metavar="DEG_PER_S",
                    help="wait until the gyro stays below this rate instead of a fixed time (0 = off)")
    ap.add_argument("--still", type=float, default=cfg.still_s, help="gyro mode: how long it must stay quiet, s")
    ap.add_argument("--settle-min", type=float, default=cfg.settle_min_s, help="gyro mode: shortest wait, s")
    ap.add_argument("--no-zero-gyro", action="store_true", help="don't zero the gyro at the park position")
    ap.add_argument("--n-avg", type=int, default=cfg.n_avg)
    ap.add_argument("--mag-serial", type=int, default=cfg.mag_serial)
    ap.add_argument("--out-dir", default=cfg.out_dir)
    ap.add_argument("--x", nargs=3, type=float, metavar=("MIN", "MAX", "N"))
    ap.add_argument("--y", nargs=3, type=float, metavar=("MIN", "MAX", "N"))
    ap.add_argument("--z", nargs=3, type=float, metavar=("MIN", "MAX", "N"))
    ap.add_argument("--correction", choices=fs.CORRECTIONS, default=cfg.correction,
                    help="position correction: off (default) or hybrid (pose_correction.py)")
    ap.add_argument("--simulate", action="store_true", help="fake robot and tilting sensor, no hardware")
    a = ap.parse_args(argv)

    cfg.port, cfg.speed_v, cfg.settle_s = a.port, a.speed, a.settle
    cfg.gyro_settle_dps, cfg.still_s, cfg.settle_min_s = a.gyro_settle, a.still, a.settle_min
    cfg.zero_gyro = not a.no_zero_gyro
    cfg.n_avg, cfg.mag_serial, cfg.out_dir = a.n_avg, a.mag_serial, a.out_dir
    cfg.correction = a.correction
    for axis, v in (("x", a.x), ("y", a.y), ("z", a.z)):
        if v:
            setattr(cfg, axis + "r", (v[0], v[1]))
            setattr(cfg, "n" + axis, int(v[2]))

    if a.simulate:
        clock = VirtualClock()
        link = fs.FakeLink()
        sensor = FakeTiltSensor(link, clock)
        confirm = lambda prompt="": None
    else:
        clock = Clock
        sensor = Phidget1044Tilt(cfg.mag_serial, cfg.sample_dt_s)   # sensor first: fail before the robot moves
        link = fs.DeltaLink(cfg.port)
        confirm = input
    try:
        run_scan(link, sensor, cfg, a.label, a.note, a.coil_current, confirm=confirm, clock=clock)
    except fs.ScanError as e:
        sys.exit(f"ERROR: {e}")
    finally:
        sensor.close()
        link.close()


if __name__ == "__main__":
    main()
