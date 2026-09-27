# Derived from grzesiek2201/Delta-Robot (GPL-3.0) @ 48a8038
# https://github.com/grzesiek2201/Delta-Robot
"""Single source of truth for the Python GUI's geometry and limits.

Must stay numerically in sync with delta_servo/config.h — see
tests/test_config_sync.py, which enforces this.
"""

# Geometry (mm). SB/SP are equilateral-triangle SIDE lengths through the
# joint centres (HANDOFF §4 Q1: confirmed as side lengths).
SB = 175.0
SP = 150.0
L_UP = 180.0   # bicep, pivot to pivot
L_LO = 625.0   # forearm, pivot to pivot (HANDOFF §4 Q2: confirmed 625 mm)

# Per-arm joint angle limits, degrees. index = arm number - 1.
# TODO(calibrate): placeholders until the user measures mechanical limits
# with servo_calibration_v3.ino. Must equal delta_servo/config.h's CAL[i]
# min_deg/max_deg exactly (enforced by tests/test_config_sync.py).
ANGLE_LIMITS_DEG = [(-30.0, 80.0), (-30.0, 80.0), (-30.0, 80.0)]

Z_MAX = -300.0          # upper z guard (TCP point), replaces upstream's hard-coded -70
Z_LIMIT_DEFAULT = -800.0
TCP_DEFAULT = (0.0, 0.0, -21.0)  # HANDOFF §4 Q6: TCP centred, 21 mm below the effector's ball-joint axis
JOG_STEP_DEFAULT = 5.0

ELBOW_JOINT_WIDTH = 45.0  # TODO: replace with the user's actual parallelogram rod spacing (plot only)

SERIAL_PORT_DEFAULT = "COM3"  # TODO: set to the actual port once the GUI host is known
SERIAL_DTR = False             # flip to True if no stream arrives from the R4
