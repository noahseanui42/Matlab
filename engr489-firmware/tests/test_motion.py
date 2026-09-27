"""Builds and runs motion_test_runner.cpp (HANDOFF §9.4)."""
import os
import subprocess

TESTS_DIR = os.path.dirname(__file__)
FIRMWARE_DIR = os.path.join(TESTS_DIR, "..", "delta_servo")
AJ_INCLUDE = os.path.join(TESTS_DIR, "..", "third_party", "ArduinoJson", "src")
BIN = os.path.join(TESTS_DIR, "motion_test")


def test_motion_suite():
    subprocess.run(
        [
            "g++", "-O0", "-g", "-Wall", "-Wextra", "-std=c++17",
            "-I", AJ_INCLUDE,
            os.path.join(TESTS_DIR, "motion_test_runner.cpp"),
            os.path.join(FIRMWARE_DIR, "motion.cpp"),
            os.path.join(FIRMWARE_DIR, "kinematics.cpp"),
            "-o", BIN,
        ],
        check=True,
    )
    result = subprocess.run([BIN], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "ALL PASS" in result.stdout
