"""Python-side kinematics tests (HANDOFF §9.1).

Regression values below are computed for THIS robot's actual configured
geometry (SB=175, SP=75 as triangle side lengths, L_UP=177, L_LO=625 mm,
HANDOFF §4 answers) -- not the handoff's illustrative L_LO=600 example --
and its real calibrated per-arm angle limits: -20/70 deg on all three arms,
confirmed with the full 3-arm assembly jogged together (the single-arm
measurement that preceded it was falsely tight -- see README). This gives
an on-axis reachable z range of roughly -540..-788mm.
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
    d.calculateIPK((0, 0, -597.2345784744069))
    for theta in d.fi:
        assert abs(math.degrees(theta)) < 0.01


def test_ik_on_axis_regression():
    # z -> expected joint angle (deg), all three arms equal by symmetry.
    # Within the real calibrated -20/70 deg limits.
    expected = {
        -560: -12.572069,
        -600: 0.89307,
        -650: 16.59158,
        -700: 32.43681,
        -750: 50.504961,
        -770: 59.536649,
    }
    d = make_robot()
    for z, exp_deg in expected.items():
        d.calculateIPK((0, 0, z))
        for theta in d.fi:
            assert abs(math.degrees(theta) - exp_deg) < 0.01, (z, math.degrees(theta), exp_deg)


def test_ik_unreachable_above_workspace():
    # z=-450 needs the bicep well above horizontal; outside the real
    # calibrated joint limits (config.h / robot_config.py CAL/ANGLE_LIMITS_DEG)
    # even after widening them to -20/70 with the full-assembly recheck.
    d = make_robot()
    try:
        d.calculateIPK((0, 0, -450))
        assert False, "expected TypeError for an out-of-range point"
    except TypeError:
        pass


def test_ik_unreachable_past_800mm():
    # Past the deep end of the -20/70 deg envelope.
    d = make_robot()
    try:
        d.calculateIPK((0, 0, -800))
        assert False, "expected TypeError for an out-of-range point"
    except TypeError:
        pass


def test_fk_ik_round_trip_grid():
    """FK(IK(p)) ~= p over the HANDOFF §9.1 grid, skipping unreachable points.

    Tolerance is 0.2 mm rather than the handoff's aspirational 0.01 mm:
    upstream's calculateFPK() nudges z by +-0.01 mm whenever two computed
    elbow heights coincide (its own div-by-zero guard, deltarobot.py
    ~L151-156, untouched here), which happens whenever two joint angles are
    exactly equal -- e.g. every point with x=0. That nudge measurably biases
    the FK result (up to ~0.17 mm here, growing with depth/joint angle)
    independent of any change made in this port. IK itself (what the
    firmware actually runs) has no such nudge, so this does not affect real
    accuracy; see also test_ik_parity.py, which holds the C++/Python IK
    match to 0.02 deg.
    """
    d = make_robot()
    tested = 0
    for x in range(-150, 151, 25):
        for y in range(-150, 151, 25):
            for z in range(-790, -549, 25):
                try:
                    d.calculateIPK((x, y, z))
                except TypeError:
                    continue
                fk = d.calculateFPK([math.degrees(a) for a in d.fi])
                assert abs(fk[0] - x) < 0.2, (x, y, z, fk)
                assert abs(fk[1] - y) < 0.2, (x, y, z, fk)
                assert abs(fk[2] - z) < 0.2, (x, y, z, fk)
                tested += 1
    assert tested > 100, "grid produced too few reachable points to be a meaningful test"


def test_ik_fk_round_trip_symmetric_poses():
    """IK(FK(theta)) for theta1=theta2=theta3, per HANDOFF §9.1."""
    d = make_robot()
    for theta_deg in (-18, -5, 0, 20, 40, 60):
        fi = (theta_deg, theta_deg, theta_deg)
        xyz = d.calculateFPK(fi)
        d.calculateIPK(xyz)
        for theta in d.fi:
            assert abs(math.degrees(theta) - theta_deg) < 0.01, (theta_deg, xyz, math.degrees(theta))
