"""HANDOFF §9.5: config.h and robot_config.py must agree on geometry and
per-arm angle limits. Regex-parses config.h rather than compiling it, since
its whole point is being the single source both a C++ build and this test
can read without extra tooling.
"""
import os
import re
import sys

TESTS_DIR = os.path.dirname(__file__)
CONFIG_H = os.path.join(TESTS_DIR, "..", "delta_servo", "config.h")

sys.path.insert(0, os.path.join(TESTS_DIR, "..", "delta_app"))
import robot_config  # noqa: E402


def _read_config_h():
    with open(CONFIG_H) as f:
        return f.read()


def _constexpr_float(text, name):
    m = re.search(rf"constexpr\s+float\s+{name}\s*=\s*([-\d.]+)f?\s*;", text)
    assert m, f"{name} not found in config.h"
    return float(m.group(1))


def _cal_entries(text):
    m = re.search(r"constexpr ServoCal CAL\[3\]\s*=\s*\{(.*?)\};", text, re.DOTALL)
    assert m, "CAL[3] not found in config.h"
    body = m.group(1)
    rows = re.findall(
        r"\{\s*(\d+)\s*,\s*([-\d.]+)f?\s*,\s*([-\d.]+)f?\s*,\s*([-+\d]+)\s*,\s*([-\d.]+)f?\s*,\s*([-\d.]+)f?\s*\}",
        body,
    )
    assert len(rows) == 3, f"expected 3 CAL rows, found {len(rows)}"
    entries = []
    for pin, centre_us, us_per_deg, dir_, min_deg, max_deg in rows:
        entries.append(
            {
                "pin": int(pin),
                "centre_us": float(centre_us),
                "us_per_deg": float(us_per_deg),
                "dir": int(dir_),
                "min_deg": float(min_deg),
                "max_deg": float(max_deg),
            }
        )
    return entries


def test_geometry_matches():
    text = _read_config_h()
    assert _constexpr_float(text, "SB") == robot_config.SB
    assert _constexpr_float(text, "SP") == robot_config.SP
    assert _constexpr_float(text, "L_UP") == robot_config.L_UP
    assert _constexpr_float(text, "L_LO") == robot_config.L_LO


def test_angle_limits_match():
    text = _read_config_h()
    cal = _cal_entries(text)
    assert len(cal) == len(robot_config.ANGLE_LIMITS_DEG) == 3
    for i, entry in enumerate(cal):
        lo, hi = robot_config.ANGLE_LIMITS_DEG[i]
        assert entry["min_deg"] == lo, (i, entry["min_deg"], lo)
        assert entry["max_deg"] == hi, (i, entry["max_deg"], hi)


def test_pin_mapping_matches_handoff_default():
    # HANDOFF §4 Q3 (confirmed default): D9=arm1, D10=arm2, D11=arm3.
    text = _read_config_h()
    cal = _cal_entries(text)
    assert [c["pin"] for c in cal] == [9, 10, 11]


def _geom_to_phys(text):
    m = re.search(r"constexpr int GEOM_TO_PHYS\[3\]\s*=\s*\{([^}]*)\};", text)
    assert m, "GEOM_TO_PHYS not found in config.h"
    return [int(v.strip()) for v in m.group(1).split(",")]


def test_geom_to_phys_matches():
    text = _read_config_h()
    assert _geom_to_phys(text) == robot_config.GEOM_TO_PHYS
