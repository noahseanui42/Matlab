"""Field-scan runner: move the probe through a 3D grid with the delta robot,
read the Phidget 1044 at every point, and log to CSV (+ a metadata JSON).

Real-time half of the pipeline; MATLAB does calibration/filtering/analysis
offline from the CSV. Python owns the serial port and the sensor, so the
GUI's 50-point program cap does not apply. Close the delta GUI first (only
one program can hold the port).

    python field_scan.py --label baseline --note "coils off"
    python field_scan.py --label coils_on --coil-current 2.0
    python field_scan.py --simulate --label demo        # no hardware
    python field_scan.py --label coils_on --correction hybrid   # with position correction

Per point: send a joint move, wait for the status stream's mv 1->0 (arrived),
settle, average N magnetometer samples, append a CSV row (flushed at once,
so Ctrl+C keeps the data). Points the firmware reports unreachable (e=1) are
logged as NaN. The robot is left ENABLED at the end (disabling drops the arms).

--correction hybrid aims each move past its target by the servos' predicted
shortfall (pose_correction.py), so the probe lands on the grid point. x_mm..z_mm
are always the TARGET (where the reading belongs); sent_x_mm..sent_z_mm are the
coordinates actually sent. Validated inside x +-50, y +-150, z -600..-700.

Field values are raw gauss in SENSOR axes. Apply the sensor->robot rotation
(cfg R_sensor_to_robot) in the MATLAB calibration stage.
"""

import argparse
import csv
import json
import math
import queue
import random
import statistics
import sys
import threading
import time
from dataclasses import dataclass, asdict, field
from datetime import datetime
from pathlib import Path

import robot_config

CSV_COLUMNS = [
    "idx", "x_mm", "y_mm", "z_mm", "Bx_G", "By_G", "Bz_G",
    "Bx_std_G", "By_std_G", "Bz_std_G", "err", "t_s",
    "n_samples", "deg1", "deg2", "deg3",
    "sent_x_mm", "sent_y_mm", "sent_z_mm",
]   # first 12 match FieldScan/run_field_scan.m, so plot_field_map.m reads both
CORRECTIONS = ("off", "hybrid")
# Geometry of the calibrated firmware (branch claude/amazing-lovelace-onixrv),
# from the correction file; no numpy needed, so it is recorded with the correction off too.
CALIBRATED_GEOMETRY = json.loads(
    (Path(__file__).resolve().parent / "pose_correction_hybrid.json").read_text())["geometry"]


@dataclass
class ScanConfig:
    port: str = robot_config.SERIAL_PORT_DEFAULT
    tcp: tuple = robot_config.TCP_DEFAULT
    speed_v: int = 2                 # 1..10 -> 5..50 mm/s; must be an int (firmware reads 2.0 as 0)
    move_timeout_s: float = 30.0
    settle_s: float = 0.5
    park_xyz: tuple = (0.0, 0.0, -650.0)
    xr: tuple = (-50.0, 50.0); nx: int = 5
    yr: tuple = (-50.0, 50.0); ny: int = 5
    zr: tuple = (-700.0, -600.0); nz: int = 5
    mag_serial: int = 302277         # 0 = first 1044 found
    n_avg: int = 20
    sample_dt_s: float = 0.02
    out_dir: str = str(Path(__file__).resolve().parent.parent.parent / "FieldScan" / "data")
    correction: str = "off"          # "off" or "hybrid" (pose_correction.py)


class ScanError(RuntimeError):
    pass


def scan_grid(xr, yr, zr, nx, ny, nz):
    """Serpentine order, identical to Kinematics/scan_grid.m."""
    def lin(r, n):
        return [r[0]] if n == 1 else [r[0] + (r[1] - r[0]) * i / (n - 1) for i in range(n)]
    xs, ys, zs = lin(xr, nx), lin(yr, ny), lin(zr, nz)
    pts = []
    for z in zs:
        for j, y in enumerate(ys):
            for x in (xs if j % 2 == 0 else reversed(xs)):
                pts.append((x, y, z))
    return pts


def frame(mode, payload=None):
    """One transaction in the GUI's framing: <mode><{json}><#>."""
    body = f"<{mode}>"
    if payload is not None:
        body += "<" + json.dumps(payload, separators=(",", ":")) + ">"
    return (body + "<#>").encode()


# --------------------------------------------------------------------------
# Robot link
# --------------------------------------------------------------------------
class DeltaLink:
    """Serial link to delta_servo. Sets DTR (the R4's USB CDC needs it).

    A background thread reads the port all the time. The firmware streams a
    status line every 20 ms; if nothing reads them (while the scan waits at a
    prompt or reads the sensor) they back up in the Arduino, and after
    flush_input() the old lines still arrive first. An old en=0 line made the
    enable check fail although the robot had enabled; an old mv=0 line could
    look like an arrival."""

    def __init__(self, port, dtr=robot_config.SERIAL_DTR, ser=None):
        if ser is None:
            import serial   # pyserial; imported here so --simulate needs nothing
            ser = serial.Serial()
            ser.port = port
            ser.baudrate = 115200
            ser.timeout = 0.05
            ser.dtr = dtr       # set before open() so it is asserted on connect
            ser.open()
            time.sleep(1.0)
            ser.reset_input_buffer()
        self.ser = ser
        self._lines = queue.Queue()
        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def _read_loop(self):
        while not self._stop.is_set():
            try:
                line = self.ser.readline()
            except Exception:   # port closed or unplugged: status() then times out
                return
            if line:
                self._lines.put(line)

    def status(self, timeout_s):
        """Next status dict {"deg","mv","run","en","e"}, or None on timeout.
        Partial/garbled lines are skipped."""
        deadline = time.monotonic() + timeout_s
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                return None
            try:
                line = self._lines.get(timeout=left).strip()
            except queue.Empty:
                return None
            try:
                j = json.loads(line)
            except ValueError:
                continue
            if isinstance(j, dict) and all(k in j for k in ("deg", "mv", "en", "e")):
                return j

    def send(self, mode, payload=None):
        self.ser.write(frame(mode, payload))

    def flush_input(self):
        """Drop every status line received so far."""
        while True:
            try:
                self._lines.get_nowait()
            except queue.Empty:
                return

    def close(self):
        self._stop.set()
        self._reader.join(timeout=1.0)
        self.ser.close()


def ensure_enabled(link, confirm=input):
    st = link.status(2.0)
    if st is None:
        raise ScanError("No status lines. Is delta_servo flashed and the port right?")
    if st["en"] == 0:
        print("\nRobot is DISABLED. Enabling snaps the arms to their last commanded pose.")
        print("Support the arms near flat and switch servo power ON.")
        confirm("Press Enter to enable (Ctrl+C to abort)... ")
        link.send(8, {"enable": 0})      # enable:0 means ENABLE (inverted)
        time.sleep(0.2)
        link.flush_input()
        t0 = time.monotonic()
        while True:                      # look at every line for up to 3 s, not just the first
            st = link.status(1.0)
            if st is not None and st["en"] == 1:
                break
            if time.monotonic() - t0 > 3.0:
                raise ScanError("Robot did not enable.")
    return st


def move_probe(link, cfg, xyz):
    """Joint move of the PROBE to xyz and block until arrived (mv 1->0).
    Returns (err_code, status). err 1 = unreachable (robot did not move),
    5 = servo pulse clamped. xyz is what is SENT: pass the corrected target
    when the position correction is on."""
    c = [round(a - b, 2) for a, b in zip(xyz, cfg.tcp)]   # firmware takes effector centre
    link.send(2, {"n": 0, "i": 0, "v": int(cfg.speed_v), "a": 0, "c": c})
    # Every move lasts >= 200 ms (T_MIN_MS): after 100 ms, anything still
    # buffered from before the command is stale.
    time.sleep(0.1)
    link.flush_input()
    t0 = time.monotonic()
    seen_moving = False
    while True:
        st = link.status(2.0)
        if st is None:
            raise ScanError("Robot stopped sending status during a move.")
        if st["en"] == 0:
            raise ScanError("Robot is disabled. Enable it before moving.")
        if st["e"] in (3, 4):
            raise ScanError(f"Firmware rejected the move (e={st['e']}).")
        if st["e"] == 1:                 # unreachable: the firmware never starts the move
            return 1, st
        if st["mv"] == 1:
            seen_moving = True
        elif seen_moving or time.monotonic() - t0 > 1.0:
            # arrived: mv 1 -> 0. A move lasts >= 200 ms (10+ lines with mv=1), so
            # an mv=0 line before any mv=1 is from before the command; after 1 s
            # with no mv=1 at all, trust mv=0.
            return st["e"], st
        if time.monotonic() - t0 > cfg.move_timeout_s:
            raise ScanError(f"Move to {tuple(xyz)} timed out.")


# --------------------------------------------------------------------------
# Magnetometer
# --------------------------------------------------------------------------
class Phidget1044:
    def __init__(self, serial_number=0, sample_dt_s=0.02):
        from Phidget22.Devices.Magnetometer import Magnetometer
        self.m = Magnetometer()
        if serial_number:
            self.m.setDeviceSerialNumber(int(serial_number))
        self.m.openWaitForAttachment(5000)
        self.m.setDataInterval(max(self.m.getMinDataInterval(), round(sample_dt_s * 1000)))
        t0 = time.monotonic()
        while True:           # first sample takes a moment after attach
            try:
                self.m.getMagneticField()
                break
            except Exception:
                if time.monotonic() - t0 > 3:
                    raise ScanError("Magnetometer attached but sent no data within 3 s.")
                time.sleep(0.05)

    def info(self):
        d = {"serial": self.m.getDeviceSerialNumber(), "name": self.m.getDeviceName()}
        try:
            d["max_field_G"] = self.m.getMaxMagneticField()
        except Exception:
            pass
        return d

    def sample(self):
        return tuple(self.m.getMagneticField())

    def close(self):
        self.m.close()


def read_avg(mag, n, dt_s):
    """Average n samples dt_s apart. Returns (mean[3], std[3], n_valid), gauss.
    Samples the sensor rejects (e.g. out of range) are dropped."""
    got = []
    for _ in range(n):
        try:
            got.append(mag.sample())
        except Exception:
            pass
        time.sleep(dt_s)
    if not got:
        return [math.nan] * 3, [math.nan] * 3, 0
    cols = list(zip(*got))
    mean = [statistics.fmean(c) for c in cols]
    std = [statistics.stdev(c) if len(c) > 1 else 0.0 for c in cols]
    return mean, std, len(got)


# --------------------------------------------------------------------------
# Scan
# --------------------------------------------------------------------------
def load_correction(cfg):
    """The PoseCorrection for cfg.correction, or None when it's off."""
    if cfg.correction not in CORRECTIONS:
        raise ScanError(f"Unknown correction {cfg.correction!r}; choose from {CORRECTIONS}.")
    if cfg.correction == "off":
        return None
    import pose_correction   # numpy; only needed with the correction on
    return pose_correction.PoseCorrection.load(tcp=cfg.tcp)


def corrected_target(corr, p):
    """(target to send, None) or (None, reason) if the correction can't reach it."""
    if corr is None:
        return tuple(p), None
    import pose_correction
    try:
        return tuple(float(v) for v in corr.compensate(p)), None
    except pose_correction.Unreachable as e:
        return None, str(e)


def run_scan(link, mag, cfg, label="scan", note="", coil_current_A=None,
             confirm=input, out=print):
    pts = scan_grid(cfg.xr, cfg.yr, cfg.zr, cfg.nx, cfg.ny, cfg.nz)
    n = len(pts)
    corr = load_correction(cfg)
    n_outside = sum(not corr.in_valid_box(p) for p in pts) if corr else 0
    mismatch = {}
    if corr:
        import pose_correction
        mismatch = pose_correction.geometry_mismatch()
        if mismatch:
            out("WARNING: robot_config.py geometry differs from the geometry the correction was "
                f"calibrated with {mismatch} (robot_config value, calibrated value). The correction uses "
                "the calibrated geometry. Make sure the robot runs the calibrated firmware "
                "(branch claude/amazing-lovelace-onixrv), not this branch's delta_servo.")
    if n_outside:
        out(f"WARNING: {n_outside} of {n} points are outside the box the {cfg.correction} "
            f"correction was validated in ({corr.valid_box}); it is extrapolating there.")
    out_dir = Path(cfg.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = out_dir / f"{label}_{stamp}.csv"
    meta_path = csv_path.with_suffix(".meta.json")

    ensure_enabled(link, confirm)
    out(f"Moving to park position {cfg.park_xyz}...")
    err, _ = move_probe(link, cfg, cfg.park_xyz)
    if err == 1:
        raise ScanError(f"Park position {cfg.park_xyz} is unreachable.")

    meta = {
        "label": label, "note": note, "started": datetime.now().isoformat(timespec="seconds"),
        "coil_current_A": coil_current_A, "n_points": n, "csv": csv_path.name,
        "frame": "sensor axes, gauss, raw (apply R_sensor_to_robot in MATLAB)",
        "config": asdict(cfg), "sensor": mag.info() if hasattr(mag, "info") else {},
        # The firmware does the IK, so the geometry that matters is the flashed one.
        # Record the calibrated firmware's (as the correction does), not this
        # branch's robot_config.py, which still has the older master values.
        "geometry": CALIBRATED_GEOMETRY,
        "correction": dict(corr.info(), points_outside_valid_box=n_outside,
                           geometry_mismatch_with_robot_config={k: list(v) for k, v in mismatch.items()})
                      if corr else {"name": "off"},
        "positions": "x_mm..z_mm = target; sent_x_mm..sent_z_mm = coordinates sent to the robot",
        "finished": None, "points_done": 0, "points_unreachable": 0, "aborted": False,
    }

    def write_meta():
        meta_path.write_text(json.dumps(meta, indent=2))

    write_meta()
    t_scan = time.monotonic()
    n_skipped = 0
    try:
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(CSV_COLUMNS)
            f.flush()
            out(f"Scanning {n} points -> {csv_path}")
            for k, p in enumerate(pts, 1):
                send, why = corrected_target(corr, p)
                if send is None:                 # correction would leave the reach: don't move
                    out(f"  point {k} {p}: correction {why}; logged as unreachable")
                    err, st = 1, {}
                else:
                    err, st = move_probe(link, cfg, send)
                if err == 1:
                    B, Bsd, ns = [math.nan] * 3, [math.nan] * 3, 0
                    n_skipped += 1
                else:
                    time.sleep(cfg.settle_s)
                    B, Bsd, ns = read_avg(mag, cfg.n_avg, cfg.sample_dt_s)
                deg = list(st.get("deg", [math.nan] * 3))
                w.writerow([k, *(f"{v:.2f}" for v in p),
                            *(f"{v:.6f}" for v in B), *(f"{v:.6f}" for v in Bsd),
                            err, f"{time.monotonic() - t_scan:.2f}", ns,
                            *(f"{d:.3f}" for d in deg),
                            *(f"{v:.2f}" for v in (send or [math.nan] * 3))])
                f.flush()
                meta["points_done"] = k
                eta = (time.monotonic() - t_scan) / k * (n - k)
                out(f"{k:4d}/{n}  [{p[0]:7.1f} {p[1]:7.1f} {p[2]:7.1f}]  "
                    f"B = [{B[0]:7.4f} {B[1]:7.4f} {B[2]:7.4f}]  |B| = {math.sqrt(sum(b*b for b in B)):.4f} G  "
                    f"e={err}  ({eta:.0f} s left)")
    except KeyboardInterrupt:
        meta["aborted"] = True
        out("Interrupted; data so far is saved.")
    finally:
        meta["points_unreachable"] = n_skipped
        meta["finished"] = datetime.now().isoformat(timespec="seconds")
        write_meta()

    out(f"Done in {time.monotonic() - t_scan:.0f} s. {n_skipped} of {n} points unreachable.")
    try:
        move_probe(link, cfg, cfg.park_xyz)
        out("Parked. Robot left ENABLED (disabling lets the arms drop).")
    except ScanError as e:
        out(f"Could not park: {e}")
    return csv_path


# --------------------------------------------------------------------------
# Simulated hardware (for --simulate and tests)
# --------------------------------------------------------------------------
class FakeLink:
    """Mimics delta_servo: replays mv=1 for a few status lines after a move,
    then mv=0. Points with |x|,|y| > reach or z outside [-790,-550] -> e=1."""

    # reach 200: with --correction hybrid, targets at y = +-150 are sent out to about
    # +-170 mm, which the real robot reaches (pose_correction.ik checks it properly)
    def __init__(self, move_lines=3, reach=200.0):
        self.en, self.pos, self.pending, self.e = 0, None, 0, 0
        self.move_lines, self.reach = move_lines, reach
        self.sent = []

    def send(self, mode, payload=None):
        self.sent.append((mode, payload))
        if mode == 8:
            self.en = 0 if payload["enable"] == 1 else 1
        elif mode == 2:
            x, y, z = payload["c"]
            if abs(x) > self.reach or abs(y) > self.reach or not -790 <= z <= -550:
                self.e, self.pending = 1, 0
            else:
                self.e, self.pending = 0, self.move_lines
                self.pos = (x, y, z)

    def status(self, timeout_s):
        mv = 1 if self.pending > 0 else 0
        self.pending = max(0, self.pending - 1)
        return {"deg": [10.0, 11.0, 12.0], "mv": mv, "run": 0, "en": self.en, "e": self.e}

    def flush_input(self):
        pass

    def close(self):
        pass


class FakeMag:
    def __init__(self, B=(0.2, 0.05, -0.5), noise=0.002, seed=0):
        self.B, self.noise, self.r = B, noise, random.Random(seed)

    def info(self):
        return {"serial": 0, "name": "simulated", "max_field_G": 8.0}

    def sample(self):
        return tuple(b + self.r.gauss(0, self.noise) for b in self.B)

    def close(self):
        pass


def main(argv=None):
    cfg = ScanConfig()
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--label", default="scan")
    ap.add_argument("--note", default="", help="free text stored in the metadata, e.g. 'coils off'")
    ap.add_argument("--coil-current", type=float, default=None, help="coil current (A), stored in metadata")
    ap.add_argument("--port", default=cfg.port)
    ap.add_argument("--speed", type=int, choices=range(1, 11), default=cfg.speed_v,
                    help="1..10 (5..50 mm/s)")
    ap.add_argument("--settle", type=float, default=cfg.settle_s)
    ap.add_argument("--n-avg", type=int, default=cfg.n_avg)
    ap.add_argument("--mag-serial", type=int, default=cfg.mag_serial)
    ap.add_argument("--out-dir", default=cfg.out_dir)
    ap.add_argument("--x", nargs=3, type=float, metavar=("MIN", "MAX", "N"))
    ap.add_argument("--y", nargs=3, type=float, metavar=("MIN", "MAX", "N"))
    ap.add_argument("--z", nargs=3, type=float, metavar=("MIN", "MAX", "N"))
    ap.add_argument("--correction", choices=CORRECTIONS, default=cfg.correction,
                    help="position correction: off (default) or hybrid (pose_correction.py)")
    ap.add_argument("--simulate", action="store_true", help="fake robot and sensor, no hardware")
    a = ap.parse_args(argv)

    cfg.port, cfg.speed_v, cfg.settle_s = a.port, a.speed, a.settle
    cfg.n_avg, cfg.mag_serial, cfg.out_dir = a.n_avg, a.mag_serial, a.out_dir
    cfg.correction = a.correction
    for axis, v in (("x", a.x), ("y", a.y), ("z", a.z)):
        if v:
            setattr(cfg, axis + "r", (v[0], v[1]))
            setattr(cfg, "n" + axis, int(v[2]))

    if a.simulate:
        cfg.settle_s, cfg.sample_dt_s = 0.0, 0.0
        link, mag = FakeLink(), FakeMag()
        confirm = lambda prompt="": None
    else:
        mag = Phidget1044(cfg.mag_serial, cfg.sample_dt_s)   # sensor first: fail before the robot moves
        link = DeltaLink(cfg.port)
        confirm = input
    try:
        run_scan(link, mag, cfg, a.label, a.note, a.coil_current, confirm=confirm)
    except ScanError as e:
        sys.exit(f"ERROR: {e}")
    finally:
        mag.close()
        link.close()


if __name__ == "__main__":
    main()
