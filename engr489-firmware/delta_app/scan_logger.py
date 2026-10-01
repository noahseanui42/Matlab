"""Log every robot status line plus the latest Phidget 1044 reading to CSV.

Used by deltagui.py (File > Start data log). Nothing here talks to the robot:
the GUI hands each parsed status line to ScanLogger.log(), and the 1044 is
read on its own Phidget event thread, so logging never blocks the Tk loop.
FieldScan/compile_scan.m turns the log into one row per program point
afterwards.
"""
import csv
import datetime as dt
import os
import threading
import time

try:
    from Phidget22.Devices.Magnetometer import Magnetometer
except ImportError:  # GUI still runs without the package; logging just can't start
    Magnetometer = None

HEADER = ["t_s", "x_mm", "y_mm", "z_mm", "deg1", "deg2", "deg3",
          "mv", "run", "en", "e", "Bx_G", "By_G", "Bz_G", "b_seq"]


class ScanLogger:
    def __init__(self, out_dir, mag_serial=0, data_interval_ms=20):
        self.out_dir = out_dir
        self.mag_serial = mag_serial
        self.data_interval_ms = data_interval_ms
        self.path = None
        self._mag = None
        self._file = None
        self._writer = None
        self._t0 = 0.0
        self._lock = threading.Lock()
        self._field = (float("nan"),) * 3
        self._seq = 0

    @property
    def running(self):
        return self._writer is not None

    def start(self):
        """Open the 1044 and a new CSV. Returns the CSV path; raises on failure."""
        if self.running:
            return self.path
        if Magnetometer is None:
            raise RuntimeError("Phidget22 is not installed in this Python.\n"
                               "Run:  python -m pip install Phidget22")

        mag = Magnetometer()
        if self.mag_serial:
            mag.setDeviceSerialNumber(self.mag_serial)
        mag.setOnMagneticFieldChangeHandler(self._on_field)
        mag.openWaitForAttachment(5000)
        try:
            mag.setDataInterval(max(mag.getMinDataInterval(), self.data_interval_ms))
            try:
                mag.setMagneticFieldChangeTrigger(0)  # report every interval, not only on change
            except Exception:
                pass  # not supported on every board version; the default is fine
            os.makedirs(self.out_dir, exist_ok=True)
            name = "deltalog_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
            self.path = os.path.join(self.out_dir, name)
            self._file = open(self.path, "w", newline="", buffering=1)
        except Exception:
            mag.close()
            raise
        self._writer = csv.writer(self._file)
        self._writer.writerow(HEADER)
        self._t0 = time.time()
        self._mag = mag
        return self.path

    def _on_field(self, ch, field, timestamp):
        with self._lock:
            self._field = tuple(field)
            self._seq += 1

    def log(self, status, xyz):
        """Write one row for a parsed status line {"deg":[..],"mv","run","en","e"}."""
        if not self.running:
            return
        with self._lock:
            field, seq = self._field, self._seq
        if xyz is None:
            xyz = (float("nan"),) * 3
        deg = status.get("deg", [float("nan")] * 3)
        self._writer.writerow([
            f"{time.time() - self._t0:.3f}",
            *(f"{v:.2f}" for v in xyz),
            *deg,
            status.get("mv", ""), status.get("run", ""), status.get("en", ""), status.get("e", ""),
            *(f"{v:.6f}" for v in field),
            seq,
        ])

    def stop(self):
        if self._mag is not None:
            try:
                self._mag.close()
            except Exception:
                pass
            self._mag = None
        if self._file is not None:
            self._file.close()
        self._file = None
        self._writer = None
