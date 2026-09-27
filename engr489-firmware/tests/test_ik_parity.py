"""C++ <-> Python IK parity (HANDOFF §9.2).

Builds tests/ik_cli from kinematics.cpp, feeds it the same grid as
test_kinematics.py, and checks it agrees with deltarobot.py's IK -- both on
the resulting angles and on which points are reachable. Both sides use
TCP=(0,0,0): ik_cli (kinematics.cpp) has no concept of TCP, it always solves
for the effector centre, so the Python side is zeroed to match (see
test_kinematics.make_robot).
"""
import math
import os
import subprocess
import sys

TESTS_DIR = os.path.dirname(__file__)
FIRMWARE_DIR = os.path.join(TESTS_DIR, "..", "delta_servo")
IK_CLI = os.path.join(TESTS_DIR, "ik_cli")

sys.path.insert(0, os.path.join(TESTS_DIR, "..", "delta_app"))
import deltarobot  # noqa: E402


def build_ik_cli():
    subprocess.run(
        [
            "g++", "-O2", "-Wall", "-Wextra", "-std=c++17",
            os.path.join(TESTS_DIR, "ik_cli.cpp"),
            os.path.join(FIRMWARE_DIR, "kinematics.cpp"),
            "-o", IK_CLI,
        ],
        check=True,
    )


def grid_points():
    for x in range(-150, 151, 25):
        for y in range(-150, 151, 25):
            for z in range(-750, -549, 25):
                yield (x, y, z)


def test_ik_parity_over_grid():
    build_ik_cli()
    points = list(grid_points())
    stdin_text = "\n".join(f"{x} {y} {z}" for x, y, z in points) + "\n"
    proc = subprocess.run([IK_CLI], input=stdin_text, capture_output=True, text=True, check=True)
    cpp_lines = proc.stdout.strip("\n").split("\n")
    assert len(cpp_lines) == len(points)

    d = deltarobot.DeltaRobot()
    d.TCP = [0.0, 0.0, 0.0]

    max_delta = 0.0
    checked = 0
    for (x, y, z), line in zip(points, cpp_lines):
        cpp_reachable = line != "UNREACHABLE"
        try:
            d.calculateIPK((x, y, z))
            py_reachable = True
        except TypeError:
            py_reachable = False

        assert cpp_reachable == py_reachable, (x, y, z, "cpp=", line, "py_reachable=", py_reachable)

        if cpp_reachable:
            cpp_theta = [float(v) for v in line.split()]
            py_theta = [math.degrees(a) for a in d.fi]
            Ei, Fi, Gi = d.calculateConstants((x, y, z))
            for i, (c, p) in enumerate(zip(cpp_theta, py_theta)):
                # Near a configuration where |G-E| is small relative to E/G's
                # own magnitude, theta = 2*atan((-F+-root)/(G-E)) is
                # ill-conditioned: float32 (kinematics.cpp) and float64
                # (deltarobot.py) can round to meaningfully different angles
                # even though each is "correct" to its own precision. This
                # is a property of the shared formula, not a porting bug, so
                # skip the strict check there rather than mask it with a
                # tolerance loose enough to hide a real mismatch elsewhere.
                denom_scale = max(abs(Ei[i]), abs(Gi[i]), 1.0)
                if abs(Gi[i] - Ei[i]) / denom_scale < 1e-3:
                    continue
                max_delta = max(max_delta, abs(c - p))
                # kinematics.cpp uses float32, deltarobot.py float64; 0.02 deg
                # comfortably covers that precision gap while still catching
                # a real algorithmic mismatch.
                assert abs(c - p) < 0.02, (x, y, z, cpp_theta, py_theta)
            checked += 1

    assert checked > 100, "grid produced too few reachable points to be a meaningful test"
