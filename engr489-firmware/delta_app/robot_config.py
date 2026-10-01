# Derived from grzesiek2201/Delta-Robot (GPL-3.0) @ 48a8038
# https://github.com/grzesiek2201/Delta-Robot
"""Single source of truth for the Python GUI's geometry and limits.

Must stay numerically in sync with delta_servo/config.h — see
tests/test_config_sync.py, which enforces this.
"""
import os

# Geometry (mm). SB/SP are equilateral-triangle SIDE lengths through the
# joint centres (HANDOFF §4 Q1: confirmed as side lengths).
SB = 175.0
SP = 150.0
L_UP = 180.0   # bicep, pivot to pivot
L_LO = 625.0   # forearm, pivot to pivot (HANDOFF §4 Q2: confirmed 625 mm)

# Per-arm joint angle limits, degrees. index = arm number - 1.
# Reconfirmed with the full 3-arm assembly (bring-up §10 steps 4-5, all
# three enabled and jogged together) -- wider than the single-arm-measured
# numbers this replaced, since driving one arm with the other two slack let
# the effector plate sag out of level and hit an early false limit. -20/70
# is a safe margin, not necessarily the true mechanical stop. Must equal
# delta_servo/config.h's CAL[i] min_deg/max_deg exactly (enforced by
# tests/test_config_sync.py).
ANGLE_LIMITS_DEG = [(-20.0, 70.0), (-20.0, 70.0), (-20.0, 70.0)]

Z_MAX = -300.0          # upper z guard (TCP point), replaces upstream's hard-coded -70
Z_LIMIT_DEFAULT = -800.0
TCP_DEFAULT = (0.0, 0.0, -21.0)  # HANDOFF §4 Q6: TCP centred, 21 mm below the effector's ball-joint axis
JOG_STEP_DEFAULT = 5.0

ELBOW_JOINT_WIDTH = 35.0  # user's actual parallelogram rod spacing, confirmed (plot only)

SERIAL_PORT_DEFAULT = "/dev/cu.usbmodem14101"  # confirmed during bring-up on the GUI Mac
SERIAL_DTR = True               # confirmed needed during bring-up: no stream arrived at False

# Scan data log (File > Start data log, see scan_logger.py). The CSVs land next
# to the MATLAB scan data so FieldScan/compile_scan.m can find them.
MAG_SERIAL = 302277        # Phidget 1044 serial number (0 = first one found)
MAG_DATA_INTERVAL_MS = 20
SCAN_LOG_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                             "..", "..", "FieldScan", "data"))
