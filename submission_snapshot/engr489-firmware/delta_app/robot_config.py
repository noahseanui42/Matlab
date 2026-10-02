# Derived from grzesiek2201/Delta-Robot (GPL-3.0) @ 48a8038
# https://github.com/grzesiek2201/Delta-Robot
"""Single source of truth for the Python GUI's geometry and limits.

Must stay numerically in sync with delta_servo/config.h — see
tests/test_config_sync.py, which enforces this.
"""

# Geometry (mm). SB/SP are equilateral-triangle SIDE lengths through the
# joint centres (HANDOFF §4 Q1: confirmed as side lengths).
SB = 175.0
# SP is the triangle through the forearm BALL-JOINT centres: the 75 mm inner
# triangle of the 150 mm platform (circumradius 43.3 mm), NOT the 150 mm outline.
SP = 75.0
L_UP = 177.0   # bicep, pivot to pivot
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

# Which physical arm (index into ANGLE_LIMITS_DEG / CAL, i.e. servo pin) sits
# at each geometric position deltarobot.py's IK/FK assume: index 0 is the
# model's "-y axis" arm, 1 is +120 deg CCW from it, 2 is +240 deg CCW.
# Confirmed during the pen-holder bench test (engr489-firmware/README.md's
# "Pen-holder end effector" section): physical Arm 3 (D11) is the one
# actually sitting at the model's assumed -y position, not physical Arm 1
# (D9). Angle limits above are unaffected: they're measured per physical
# servo and stay indexed by pin regardless of which geometric role that
# servo plays. Must equal delta_servo/config.h's GEOM_TO_PHYS exactly
# (enforced by tests/test_config_sync.py).
GEOM_TO_PHYS = [2, 0, 1]

Z_MAX = -300.0          # upper z guard (TCP point), replaces upstream's hard-coded -70
Z_LIMIT_DEFAULT = -800.0
TCP_DEFAULT = (0.0, 0.0, -21.0)  # HANDOFF §4 Q6: TCP centred, 21 mm below the effector's ball-joint axis
JOG_STEP_DEFAULT = 5.0

ELBOW_JOINT_WIDTH = 35.0  # user's actual parallelogram rod spacing, confirmed (plot only)

SERIAL_PORT_DEFAULT = "/dev/cu.usbmodem14101"  # confirmed during bring-up on the GUI Mac
SERIAL_DTR = True               # confirmed needed during bring-up: no stream arrived at False
