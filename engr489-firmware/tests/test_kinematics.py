"""Python-side kinematics tests (HANDOFF §9.1).

Regression values below are computed for THIS robot's actual configured
geometry (SB=175, SP=150 as triangle side lengths, L_UP=180, L_LO=625 mm,
HANDOFF §4 answers) -- not the handoff's illustrative L_LO=600 example --
and its real calibrated per-arm angle limits (measured single-arm, rest of
linkage flat; see config.h/robot_config.py). Those limits are much tighter
than the earlier ±(-30,80) placeholder, which shrinks the on-axis reachable
z range to roughly -561..-692mm (was -530..-750mm under the placeholder).
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "delta_app"))

import deltarobot  # noqa: E402


def make_robot():
    """A robot with TCP=(0,0,0).

    calculateIPK() takes a TCP-frame target and subtracts self.TCP before
    solving, but the firmware's ik() (kinematics.cpp) always solves directly
    for the effector centre -- the GUI removes the TCP offset before sending
    (HANDOFF §5). These tests exercise the shared geometric solve, so they
    zero out TCP to match what ik() sees, per HANDOFF §9.1's own caveat
    ("Regression values ... only if ... TCP = 0").
    """
    d = deltarobot.DeltaRobot()
    d.TCP = [0.0, 0.0, 0.0]
    return d


def test_zhome_on_axis_regression():
    d = make_robot()
    d.calculateIPK((0, 0, -608.2049800218946))
    for theta in d.fi:
        assert abs(math.degrees(theta)) < 0.01


def test_ik_on_axis_regression():
    # z -> expected joint angle (deg), all three arms equal by symmetry.
    # Within the real calibrated limits' intersection (-16..26.5 deg).
    expected = {
        -570: -12.582295,
        -600: -2.626863,
        -650: 13.071569,
        -680: 22.446806,
    }
    d = make_robot()
    for z, exp_deg in expected.items():
        d.calculateIPK((0, 0, z))
        for theta in d.fi:
            assert abs(math.degrees(theta) - exp_deg) < 0.01, (z, math.degrees(theta), exp_deg)


def test_ik_unreachable_above_workspace():
    # z=-450 needs the bicep well above horizontal; outside the real
    # calibrated joint limits (config.h / robot_config.py CAL/ANGLE_LIMITS_DEG).
    d = make_robot()
    try:
        d.calculateIPK((0, 0, -450))
        assert False, "expected TypeError for an out-of-range point"
    except TypeError:
        pass


def test_ik_unreachable_below_workspace():
    # z=-750 (once reachable under the old ±(-30,80) placeholder) now needs
    # theta well past the real calibrated 26.5 deg max.
    d = make_robot()
    try:
        d.calculateIPK((0, 0, -750))
        assert False, "expected TypeError for an out-of-range point"
    except TypeError:
        pass


def test_fk_ik_round_trip_grid():
    """FK(IK(p)) ~= p over the HANDOFF §9.1 grid, skipping unreachable points.

    Tolerance is 0.1 mm rather than the handoff's aspirational 0.01 mm:
    upstream's calculateFPK() nudges z by +-0.01 mm whenever two computed
    elbow heights coincide (its own div-by-zero guard, deltarobot.py
    ~L151-156, untouched here), which happens whenever two joint angles are
    exactly equal -- e.g. every point with x=0. That nudge measurably biases
    the FK result (~0.02-0.08 mm here) independent of any change made in
    this port. IK itself (what the firmware actually runs) has no such
    nudge, so this does not affect real accuracy; see also
    test_ik_parity.py, which holds the C++/Python IK match to 0.01 deg.
    """
    d = make_robot()
    tested = 0
    for x in range(-80, 81, 20):
        for y in range(-80, 81, 20):
            for z in range(-680, -559, 15):
                try:
                    d.calculateIPK((x, y, z))
                except TypeError:
                    continue
                fk = d.calculateFPK([math.degrees(a) for a in d.fi])
                assert abs(fk[0] - x) < 0.1, (x, y, z, fk)
                assert abs(fk[1] - y) < 0.1, (x, y, z, fk)
                assert abs(fk[2] - z) < 0.1, (x, y, z, fk)
                tested += 1
    assert tested > 100, "grid produced too few reachable points to be a meaningful test"


def test_ik_fk_round_trip_symmetric_poses():
    """IK(FK(theta)) for theta1=theta2=theta3, per HANDOFF §9.1."""
    d = make_robot()
    for theta_deg in (-15, -5, 0, 10, 20, 25):
        fi = (theta_deg, theta_deg, theta_deg)
        xyz = d.calculateFPK(fi)
        d.calculateIPK(xyz)
        for theta in d.fi:
            assert abs(math.degrees(theta) - theta_deg) < 0.01, (theta_deg, xyz, math.degrees(theta))
