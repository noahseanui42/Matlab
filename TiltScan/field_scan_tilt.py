"""Field scan with tilt logging: the same scan as delta_app/field_scan.py, plus the
1044's pitch and roll (tilt) and gyroscope (is the probe still?) at every point.

    python TiltScan/field_scan_tilt.py --label magnet_tilt --note "strong magnet"
    python TiltScan/field_scan_tilt.py --label magnet_tilt --correction hybrid
    python TiltScan/field_scan_tilt.py --label t --gyro-settle 0.5   # wait for the gyro, not a fixed time
    python TiltScan/field_scan_tilt.py --simulate --label demo        # no hardware

field_scan.py is NOT changed: this script imports its robot link, moves, grid and
position correction, and only replaces the per-point reading and the CSV writing.
All field_scan.py options work the same way. Output goes to TiltScan/data/ by default.

The 1044 is read through its Spatial channel (the separate Magnetometer /
Accelerometer / Gyroscope channels don't open on our board). Every Spatial event
carries acceleration, angular rate and field from the same instant, and with
--algorithm imu (default) or ahrs the board's own orientation filter also gives
pitch and roll (Spatial.getEulerAngles).

Per point: move, settle, then collect n_avg Spatial events and average them into one
row: field, acceleration, gyro, and pitch/roll both from the board's filter and
from the averaged acceleration (while still they should agree; the acceleration
ones need no filter). MATLAB (TiltScan/tilt_correct.m) turns the gravity vector
into a tilt per point and rotates B back to a reference orientation.

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
    pitch_deg, roll_deg         mean pitch and roll from the board's IMU/AHRS filter
                                (NaN with --algorithm none or if the board refuses it)
    acc_pitch_deg, acc_roll_deg pitch (about sensor y) and roll (about sensor x) from
                                the mean acceleration; signs may differ from the
                                board's convention, compare them on the first run
"""

import argparse
import collections
import csv
import json
import math
import random
import statistics
import sys
import threading
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
    "pitch_deg", "roll_deg", "acc_pitch_deg", "acc_roll_deg",
]
ALGORITHMS = ("imu", "ahrs", "none")
UNKNOWN = 1e100        # Phidget22 reports unknown / out-of-range values as 1e300
CSV_COLUMNS = fs.CSV_COLUMNS + TILT_COLUMNS


@dataclass
class TiltScanConfig(fs.ScanConfig):
    out_dir: str = str(HERE / "data")
    gyro_settle_dps: float = 0.0     # 0: fixed settle_s wait; > 0: wait for the gyro to go quiet
    still_s: float = 1.0             # ... and stay below gyro_settle_dps this long
    settle_min_s: float = 0.5        # gyro mode: always wait at least this
    zero_gyro: bool = True           # zero the gyro at the park position before the scan
    algorithm: str = "imu"           # board's orientation filter for pitch/roll: imu, ahrs or none


class Clock:
    """Real time. The simulation swaps in a virtual clock so waits take no time."""
    now = staticmethod(time.monotonic)
    sleep = staticmethod(time.sleep)


# --------------------------------------------------------------------------
# Sensor: the 1044's Spatial channel
# --------------------------------------------------------------------------
class Phidget1044Spatial:
    """The 1044 through its Spatial channel. Each SpatialData event carries
    acceleration (g), angular rate (deg/s) and field (G) from the same instant; the
    events are buffered by a handler (Spatial has no getters for them). Pitch and
    roll come from the board's IMU/AHRS filter via getEulerAngles()."""

    def __init__(self, serial_number=0, sample_dt_s=0.02, algorithm="imu", channel=None):
        self._lock = threading.Lock()
        self._buf = collections.deque(maxlen=5000)
        self._latest = None
        self._new = threading.Event()
        self.algorithm, self.algorithm_error = "none", ""
        if channel is not None:              # tests: a stand-in channel, events fed by hand
            self.sp = channel
            return
        from Phidget22.Devices.Spatial import Spatial
        self.sp = Spatial()
        try:
            if serial_number:
                self.sp.setDeviceSerialNumber(int(serial_number))
            self.sp.setOnSpatialDataHandler(self._on_data)
            self.sp.openWaitForAttachment(5000)
            self.sp.setDataInterval(max(self.sp.getMinDataInterval(), round(sample_dt_s * 1000)))
            if algorithm != "none":
                from Phidget22.SpatialAlgorithm import SpatialAlgorithm
                try:
                    self.sp.setAlgorithm({"imu": SpatialAlgorithm.SPATIAL_ALGORITHM_IMU,
                                          "ahrs": SpatialAlgorithm.SPATIAL_ALGORITHM_AHRS}[algorithm])
                    self.algorithm = algorithm
                except Exception as e:      # not supported: log acceleration pitch/roll only
                    self.algorithm_error = str(e)
                    print(f"WARNING: the 1044 would not run the {algorithm} algorithm ({e}); "
                          "pitch_deg/roll_deg will be NaN, acc_pitch_deg/acc_roll_deg still work.")
            if not self._new.wait(3.0):     # first event takes a moment after attach
                raise fs.ScanError("1044 Spatial channel attached but sent no data within 3 s.")
        except Exception:
            self.close()
            raise

    def _on_data(self, ch, acceleration, angularRate, magneticField, timestamp):
        ev = (tuple(acceleration), tuple(angularRate), tuple(magneticField), timestamp)
        with self._lock:
            self._buf.append(ev)
            self._latest = ev
        self._new.set()

    def info(self):
        d = {"channel": "Spatial", "algorithm": self.algorithm}
        if self.algorithm_error:
            d["algorithm_error"] = self.algorithm_error
        for key, getter in (("serial", "getDeviceSerialNumber"), ("name", "getDeviceName"),
                            ("data_interval_ms", "getDataInterval"),
                            ("max_field_G", "getMaxMagneticField"),
                            ("max_accel_g", "getMaxAcceleration"),
                            ("max_rate_dps", "getMaxAngularRate")):
            try:
                d[key] = getattr(self.sp, getter)()
            except Exception:
                pass
        return d

    def gyro(self):
        """Latest angular rate (deg/s), for the settle wait."""
        with self._lock:
            ev = self._latest
        if ev is None or not _known(ev[1]):
            raise fs.ScanError("no gyro reading")
        return ev[1]

    def euler(self):
        """(pitch, roll) in degrees from the board's filter, or None."""
        if self.algorithm == "none":
            return None
        try:
            e = self.sp.getEulerAngles()
            return (e.pitch, e.roll)
        except Exception:
            return None

    def samples(self, n, dt_s, clock=None):
        """The next n events after this call: lists of field, acceleration, angular
        rate and (pitch, roll). Unknown values are left out, per sensor."""
        with self._lock:
            self._buf.clear()
        got, E = [], []
        deadline = time.monotonic() + max(3.0, 5 * n * max(dt_s, 0.004))
        while len(got) < n and time.monotonic() < deadline:
            time.sleep(max(dt_s, 0.004))
            with self._lock:
                got.extend(self._buf)
                self._buf.clear()
            e = self.euler()
            if e is not None and _known(e):
                E.append(e)
        got = got[:n]
        return ([ev[2] for ev in got if _known(ev[2])], [ev[0] for ev in got if _known(ev[0])],
                [ev[1] for ev in got if _known(ev[1])], E)

    def zero_gyro(self, clock=Clock):
        self.sp.zeroGyro()          # Phidget: re-zeros in 1-2 s, board must be still
        clock.sleep(2.5)

    def after_move(self):
        pass

    def close(self):
        try:
            self.sp.close()
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


def _known(v):
    return all(math.isfinite(x) and abs(x) < UNKNOWN for x in v)


def accel_pitch_roll(a):
    """Pitch (about sensor y) and roll (about sensor x), degrees, from a gravity
    vector. Works whichever way up the board reads 1 g (the sign of az)."""
    ax, ay, az = a
    if not all(math.isfinite(v) for v in a) or _norm(a) == 0:
        return math.nan, math.nan
    s = 1.0 if az >= 0 else -1.0
    return (math.degrees(math.atan2(-s * ax, math.hypot(ay, az))),
            math.degrees(math.atan2(s * ay, abs(az))))


def poll_samples(sensor, n, dt_s, clock=Clock):
    """samples() for a sensor with mag()/accel()/gyro()/euler() getters (the simulation)."""
    B, A, W, E = [], [], [], []
    for _ in range(n):
        for read, store in ((sensor.mag, B), (sensor.accel, A), (sensor.gyro, W), (sensor.euler, E)):
            try:
                v = read()
                if v is not None and _known(v):
                    store.append(v)
            except Exception:
                pass
        clock.sleep(dt_s)
    return B, A, W, E


def read_point(sensor, n, dt_s, clock=Clock):
    """Average n samples of field, acceleration, gyro and pitch/roll."""
    B, A, W, E = sensor.samples(n, dt_s, clock)
    Bm, Bs = _mean_std(B)
    Am, As = _mean_std(A)
    rates = [_norm(w) for w in W]
    pitch = statistics.fmean(e[0] for e in E) if E else math.nan
    roll = statistics.fmean(e[1] for e in E) if E else math.nan
    return {"B": Bm, "Bsd": Bs, "n": len(B), "a": Am, "asd": As,
            "g_rms": math.sqrt(statistics.fmean(r * r for r in rates)) if rates else math.nan,
            "g_max": max(rates) if rates else math.nan,
            "pitch": pitch, "roll": roll, "acc_pr": accel_pitch_roll(Am)}


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
        "frame": "sensor axes, raw: B in gauss, acceleration in g, angular rate in deg/s, "
                 "pitch/roll in deg (tilt_correct.m and R_sensor_to_robot in MATLAB)",
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
                    r = {"B": nan3, "Bsd": nan3, "n": 0, "a": nan3, "asd": nan3, "g_rms": math.nan,
                         "g_max": math.nan, "pitch": math.nan, "roll": math.nan,
                         "acc_pr": (math.nan, math.nan)}
                    waited, settled = 0.0, -1
                    n_skipped += 1
                else:
                    waited, settled = wait_settled(sensor, cfg, clock)
                    n_unsettled += settled == 0
                    r = read_point(sensor, cfg.n_avg, cfg.sample_dt_s, clock)
                B = r["B"]
                deg = list(st.get("deg", nan3))
                w.writerow([k, *(f"{v:.2f}" for v in p),
                            *(f"{v:.6f}" for v in B), *(f"{v:.6f}" for v in r["Bsd"]),
                            err, f"{clock.now() - t_scan:.2f}", r["n"],
                            *(f"{d:.3f}" for d in deg),
                            *(f"{v:.2f}" for v in (send or nan3)),
                            *(f"{v:.6f}" for v in r["a"]), *(f"{v:.6f}" for v in r["asd"]),
                            f"{r['g_rms']:.4f}", f"{r['g_max']:.4f}", f"{waited:.2f}", settled,
                            f"{r['pitch']:.4f}", f"{r['roll']:.4f}",
                            *(f"{v:.4f}" for v in r["acc_pr"])])
                f.flush()
                meta["points_done"] = k
                pr = (r["pitch"], r["roll"]) if math.isfinite(r["pitch"]) else r["acc_pr"]
                eta = (clock.now() - t_scan) / k * (n - k)
                out(f"{k:4d}/{n}  [{p[0]:7.1f} {p[1]:7.1f} {p[2]:7.1f}]  "
                    f"|B| = {_norm(B):.4f} G  pitch {pr[0]:7.3f}  roll {pr[1]:7.3f} deg  "
                    f"gyro {r['g_rms']:.2f} deg/s  settle {waited:.1f} s{'' if settled else ' (gyro not quiet)'}  "
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

    def euler(self):
        """(pitch, roll) as an IMU filter would give them (simulation: true tilt + noise)."""
        x, y, _ = self.link.pos or (0.0, 0.0, -650.0)
        return (self.k * x + self.r.gauss(0, 0.01), -self.k * y + self.r.gauss(0, 0.01))

    def samples(self, n, dt_s, clock=None):
        return poll_samples(self, n, dt_s, clock or self.clock)

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
    ap.add_argument("--algorithm", choices=ALGORITHMS, default=cfg.algorithm,
                    help="board's orientation filter for pitch/roll: imu (accel + gyro, default), "
                         "ahrs (also uses the magnetometer, which the magnet disturbs) or none")
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
    cfg.algorithm = a.algorithm
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
        sensor = Phidget1044Spatial(cfg.mag_serial, cfg.sample_dt_s, cfg.algorithm)   # sensor first: fail before the robot moves
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
