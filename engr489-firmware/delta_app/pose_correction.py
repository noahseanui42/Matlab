"""Pose-dependent position correction for field scans (open loop).

Off-centre, the servos give way slightly under the load, so the probe falls short
of its target, pulled back towards the axis (about 13% of the distance from the
centre, about 19 mm rms at the corners of the scan box uncorrected). This module
predicts each servo's shortfall from the robot's Jacobian and aims past the target
by that much, so the probe lands on it:

    dtheta(p) = beta_x f_cx(p) + beta_y f_cy(p) + beta_mid f_cx(p) (1 - (y/150)^2)
    f_cx = J(p)^T (-x, 0, 0),  f_cy = J(p)^T (0, -y, 0)          (in degrees)
    send  = FK( IK(p) - dtheta(p) )

The firmware's own IK of `send` then gives IK(p) - dtheta, and the servos settle
on IK(p). Nothing changes in the firmware.

The coefficients (pose_correction_hybrid.json) come from the calibration tests in
the ENGR489 report repo (noahseanui42/engr489-REPORT, analysis/refit_v2.py
--hybrid; notes in calibration/2026-10-01_refit-v2.md and
calibration/2026-10-01_calibration-summary.md). Validated inside x +-50,
y +-150, z -600..-700 (probe): corner error about 19 -> 2.3 mm rms; box spans
100 x 299 mm at z -650. Not validated outside that box.

GEOMETRY: the model only holds for the geometry it was fitted with, so the
geometry travels with the coefficients in the JSON file (SB 175, SP 75, L_UP 177,
L_LO 625, arm remap GEOM_TO_PHYS [2, 0, 1]; the calibrated firmware from branch
claude/amazing-lovelace-onixrv, now merged). It is deliberately NOT read from
robot_config.py, so a later geometry change can't silently invalidate the fit;
field_scan.py warns if robot_config.py stops matching it.
The TCP offset comes from the scan config.
"""
import json
import math
from pathlib import Path

import numpy as np

import robot_config

HERE = Path(__file__).resolve().parent
DEFAULT_FILE = HERE / "pose_correction_hybrid.json"

GEOMETRY = json.loads(DEFAULT_FILE.read_text())["geometry"]
SB, SP = GEOMETRY["SB"], GEOMETRY["SP"]
WB = SB / (2 * math.sqrt(3))                # centre to servo shaft (base side midpoint)
UP = SP / math.sqrt(3)                      # centre to platform ball joint (corner)
L_UP, L_LO = GEOMETRY["L_UP"], GEOMETRY["L_LO"]
GEOM_TO_PHYS = list(GEOMETRY["GEOM_TO_PHYS"])   # physical servo (D9, D10, D11 = 0, 1, 2) per slot
PHYS_TO_GEOM = [GEOM_TO_PHYS.index(p) for p in range(3)]
LIMITS_RAD = [(math.radians(lo), math.radians(hi)) for lo, hi in GEOMETRY["ANGLE_LIMITS_DEG"]]


def geometry_mismatch():
    """Differences between this geometry and robot_config.py (empty if none)."""
    diffs = {}
    for key in ("SB", "SP", "L_UP", "L_LO"):
        if abs(getattr(robot_config, key) - GEOMETRY[key]) > 1e-9:
            diffs[key] = (getattr(robot_config, key), GEOMETRY[key])
    rc_map = list(getattr(robot_config, "GEOM_TO_PHYS", [0, 1, 2]))
    if rc_map != GEOM_TO_PHYS:
        diffs["GEOM_TO_PHYS"] = (rc_map, GEOM_TO_PHYS)
    return diffs

_ANG = np.radians([-90.0, 30.0, 150.0])     # geometric slots, as deltarobot.py's B1..B3
U = np.stack([np.cos(_ANG), np.sin(_ANG), np.zeros(3)], axis=1)
Z = np.array([0.0, 0.0, 1.0])


class Unreachable(ValueError):
    pass


def _elbows(theta_geom):
    th = np.asarray(theta_geom, float)
    return (WB + L_UP * np.cos(th))[:, None] * U - (L_UP * np.sin(th))[:, None] * Z


def ik(probe, tcp=robot_config.TCP_DEFAULT):
    """Joint angles (rad, physical servo order D9, D10, D11) for a probe position.
    Same equation and root choice as deltarobot.calculateIPK."""
    p = np.asarray(probe, float) - np.asarray(tcp, float)
    out = np.zeros(3)
    for g in range(3):
        phys = GEOM_TO_PHYS[g]
        lo, hi = LIMITS_RAD[phys]
        d = p + (UP - WB) * U[g]
        E = -2 * L_UP * (d @ U[g])
        F = 2 * L_UP * d[2]
        G = d @ d + L_UP ** 2 - L_LO ** 2
        disc = E * E + F * F - G * G
        if disc < 0:
            raise Unreachable("out of reach")
        for sgn in (+1, -1):
            th = 2 * math.atan((-F + sgn * math.sqrt(disc)) / (G - E))
            if lo <= th <= hi:
                out[phys] = th
                break
        else:
            raise Unreachable("outside the joint limits")
    return out


def fk(theta_phys, guess=(0.0, 0.0, -650.0), tcp=robot_config.TCP_DEFAULT, tol=1e-10):
    """Probe position from joint angles (rad, physical order). Newton on the three
    forearm-length constraints (no divide-by-zero poses)."""
    A = _elbows(np.asarray(theta_phys, float)[GEOM_TO_PHYS])
    p = np.asarray(guess, float) - np.asarray(tcp, float)
    for _ in range(50):
        s = p + UP * U - A
        step = np.linalg.solve(2 * s, np.einsum("ij,ij->i", s, s) - L_LO ** 2)
        p -= step
        if np.max(np.abs(step)) < tol:
            return p + np.asarray(tcp, float)
    raise RuntimeError("FK did not converge")


def jacobian(probe, theta_phys, tcp=robot_config.TCP_DEFAULT):
    """J (mm/rad, columns D9 D10 D11): dp = J dtheta."""
    thg = np.asarray(theta_phys, float)[GEOM_TO_PHYS]
    s = np.asarray(probe, float) - np.asarray(tcp, float) + UP * U - _elbows(thg)
    dA = -(L_UP * np.sin(thg))[:, None] * U - (L_UP * np.cos(thg))[:, None] * Z
    J_geom = np.linalg.solve(s, np.diag(np.einsum("ij,ij->i", s, dA)))
    return J_geom[:, PHYS_TO_GEOM]


def features(probe, theta_phys=None, tcp=robot_config.TCP_DEFAULT):
    p = np.asarray(probe, float)
    th = ik(p, tcp) if theta_phys is None else theta_phys
    J = jacobian(p, th, tcp)
    cx = np.degrees(J.T @ np.array([-p[0], 0.0, 0.0]))
    cy = np.degrees(J.T @ np.array([0.0, -p[1], 0.0]))
    return {"cx": cx, "cy": cy, "cx_mid": cx * (1.0 - (p[1] / 150.0) ** 2)}


class PoseCorrection:
    def __init__(self, forces, beta, valid_box=None, name="", source="", tcp=robot_config.TCP_DEFAULT):
        self.forces, self.beta = tuple(forces), np.asarray(beta, float)
        self.valid_box = valid_box or {}
        self.name, self.source, self.tcp = name, source, tuple(tcp)

    @classmethod
    def load(cls, path=DEFAULT_FILE, tcp=robot_config.TCP_DEFAULT):
        d = json.loads(Path(path).read_text())
        return cls(d["forces"], d["beta"], d.get("valid_box"), d.get("name", ""), d.get("source", ""), tcp)

    def predict_offset_deg(self, probe):
        """Predicted measured - commanded joint angle (deg, D9 D10 D11) at a target."""
        f = features(probe, tcp=self.tcp)
        return sum(b * f[k] for b, k in zip(self.beta, self.forces))

    def compensate(self, probe):
        """Probe target to send so the robot lands on `probe`. Raises Unreachable
        if the aimed angles are out of reach."""
        p = np.asarray(probe, float)
        aim = ik(p, self.tcp) - np.radians(self.predict_offset_deg(p))
        for phys, (lo, hi) in enumerate(LIMITS_RAD):
            if not lo <= aim[phys] <= hi:
                raise Unreachable("corrected angles outside the joint limits")
        return fk(aim, p, self.tcp)

    def in_valid_box(self, probe):
        b = self.valid_box
        if not b:
            return True
        return all(b[k][0] - 1e-6 <= v <= b[k][1] + 1e-6 for k, v in zip(("x", "y", "z"), probe))

    def info(self):
        return {"name": self.name, "forces": list(self.forces), "beta": self.beta.tolist(),
                "valid_box": self.valid_box, "source": self.source, "geometry": GEOMETRY}
