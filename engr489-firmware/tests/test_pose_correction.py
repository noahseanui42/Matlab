"""Tests for delta_app/pose_correction.py and its use in field_scan.py."""
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "delta_app"))
import field_scan as fs    # noqa: E402
import pose_correction as pc   # noqa: E402
import robot_config        # noqa: E402

POINTS = [(0, 0, -650), (50, 150, -650), (-50, -150, -680), (50, 0, -650),
          (25, 75, -600), (-50, 0, -700), (0, -150, -640), (30, -120, -610)]

# Reference values from the report repo's model (analysis/error_model.py with
# error_model_coeffs_hybrid.json): target -> (send, predicted offset deg D9 D10 D11)
REFERENCE = {
    (50, 150, -650): ((55.4032, 170.3634, -644.4391), (1.3617, 0.3725, -1.9784)),
    (50, 0, -650): ((59.8111, -0.1589, -649.1574), (0.7912, -0.8210, -0.0042)),
    (-50, -150, -680): ((-58.6538, -168.9985, -674.4535), (-1.5929, -0.3611, 1.7839)),
    (0, 0, -650): ((0.0, 0.0, -650.0), (0.0, 0.0, 0.0)),
    (25, 75, -600): ((28.8404, 83.1935, -598.7902), (0.7454, 0.0627, -0.8731)),
    (-50, 0, -700): ((-61.4250, -0.4307, -699.0330), (-0.8885, 0.8510, 0.0080)),
    (0, -150, -640): ((0.0, -167.4692, -635.4432), (-0.9348, -0.9348, 1.7088)),
}


def model():
    return pc.PoseCorrection.load()


def test_ik_matches_calibrated_robot():
    # on-axis regression values from the calibrated branch's tests/test_kinematics.py
    # (platform z with TCP 0, i.e. probe z - 21) and the expected angles the GUI gave
    # for the corner test (calibration run 5), physical order D9 D10 D11
    for z_platform, deg in {-600: 0.89307, -650: 16.59158, -700: 32.43681, -750: 50.504961}.items():
        assert np.allclose(np.degrees(pc.ik((0, 0, z_platform - 21))), deg, atol=1e-4)
    assert np.allclose(np.degrees(pc.ik((100, 100, -740))), (32.61, 47.83, 52.60), atol=0.006)
    assert np.allclose(np.degrees(pc.ik((-100, -100, -600))), (13.40, -4.17, -10.61), atol=0.006)


def test_fk_inverts_ik():
    for p in POINTS:
        assert np.max(np.abs(pc.fk(pc.ik(p), p) - p)) < 1e-6


def test_jacobian_matches_finite_differences():
    for p in POINTS:
        th = pc.ik(p)
        J = pc.jacobian(p, th)
        h = 1e-6
        Jfd = np.column_stack([(pc.fk(th + h * e, p) - pc.fk(th - h * e, p)) / (2 * h) for e in np.eye(3)])
        assert np.max(np.abs(J - Jfd)) < 1e-4 * np.max(np.abs(J))


def test_matches_report_repo_reference():
    m = model()
    for p, (send, off) in REFERENCE.items():
        assert np.allclose(m.compensate(p), send, atol=1e-3), p
        assert np.allclose(m.predict_offset_deg(p), off, atol=1e-3), p


def test_compensated_target_lands_on_the_wanted_angles():
    m = model()
    for p in POINTS:
        landed = pc.ik(m.compensate(p)) + np.radians(m.predict_offset_deg(p))
        assert np.max(np.abs(landed - pc.ik(p))) < 1e-9, p


def test_corners_get_no_mid_box_term():
    # the mid-box x term fades to zero at y = +-150: corners are pure v1 there
    f = pc.features((50, 150, -650))
    assert np.allclose(f["cx_mid"], 0.0, atol=1e-12)


def test_valid_box():
    m = model()
    assert m.in_valid_box((50, -150, -600)) and m.in_valid_box((0, 0, -700))
    assert not m.in_valid_box((60, 0, -650)) and not m.in_valid_box((0, 0, -720))


def make_cfg(tmp_path, **kw):
    cfg = fs.ScanConfig(out_dir=str(tmp_path), settle_s=0.0, sample_dt_s=0.0, n_avg=3,
                        xr=(-50, 50), nx=3, yr=(-150, 150), ny=3, zr=(-650, -650), nz=1)
    for k, v in kw.items():
        setattr(cfg, k, v)
    return cfg


def sent_moves(link):
    return [payload["c"] for mode, payload in link.sent if mode == 2]


def test_correction_off_sends_target_minus_tcp(tmp_path):
    cfg = make_cfg(tmp_path)
    link = fs.FakeLink()
    path = fs.run_scan(link, fs.FakeMag(), cfg, "t", confirm=lambda *_: None, out=lambda *_: None)
    pts = fs.scan_grid(cfg.xr, cfg.yr, cfg.zr, cfg.nx, cfg.ny, cfg.nz)
    moves = sent_moves(link)[1:-1]        # drop the park moves
    for p, c in zip(pts, moves):
        assert c == [round(a - b, 2) for a, b in zip(p, cfg.tcp)]
    rows = list(csv.DictReader(open(path)))
    assert all(float(r["sent_x_mm"]) == float(r["x_mm"]) for r in rows)
    meta = json.loads(path.with_suffix(".meta.json").read_text())
    assert meta["correction"] == {"name": "off"}


def test_correction_hybrid_sends_compensated_target(tmp_path):
    cfg = make_cfg(tmp_path, correction="hybrid")
    link = fs.FakeLink()
    path = fs.run_scan(link, fs.FakeMag(), cfg, "t", confirm=lambda *_: None, out=lambda *_: None)
    m = model()
    pts = fs.scan_grid(cfg.xr, cfg.yr, cfg.zr, cfg.nx, cfg.ny, cfg.nz)
    moves = sent_moves(link)[1:-1]
    assert len(moves) == len(pts)
    for p, c in zip(pts, moves):
        want = [round(a - b, 2) for a, b in zip(m.compensate(p), cfg.tcp)]
        assert c == want, p
    rows = list(csv.DictReader(open(path)))
    assert list(rows[0].keys())[:12] == fs.CSV_COLUMNS[:12]       # MATLAB-readable
    r = next(r for r in rows if float(r["x_mm"]) == 50 and float(r["y_mm"]) == 0)
    assert float(r["x_mm"]) == 50.0 and abs(float(r["sent_x_mm"]) - 59.81) < 0.01   # target vs sent
    meta = json.loads(path.with_suffix(".meta.json").read_text())
    assert meta["correction"]["name"] == "hybrid" and meta["correction"]["points_outside_valid_box"] == 0


def test_point_the_correction_cannot_reach_is_logged_not_sent(tmp_path, monkeypatch):
    cfg = make_cfg(tmp_path, correction="hybrid", xr=(0, 0), nx=1, yr=(0, 150), ny=2)
    real = pc.PoseCorrection.compensate

    def flaky(self, p):
        if p[1] > 100:
            raise pc.Unreachable("corrected angles outside the joint limits")
        return real(self, p)
    monkeypatch.setattr(pc.PoseCorrection, "compensate", flaky)
    link = fs.FakeLink()
    path = fs.run_scan(link, fs.FakeMag(), cfg, "t", confirm=lambda *_: None, out=lambda *_: None)
    rows = list(csv.DictReader(open(path)))
    assert len(sent_moves(link)) == 1 + 1 + 1     # park, the reachable point, park
    bad = rows[1]
    assert bad["err"] == "1" and math.isnan(float(bad["Bx_G"])) and math.isnan(float(bad["sent_x_mm"]))


def test_warns_outside_validated_box(tmp_path):
    cfg = make_cfg(tmp_path, correction="hybrid", xr=(-60, 60), nx=2, yr=(0, 0), ny=1)
    said = []
    path = fs.run_scan(fs.FakeLink(), fs.FakeMag(), cfg, "t", confirm=lambda *_: None, out=said.append)
    assert any("outside the box" in s for s in said)
    meta = json.loads(path.with_suffix(".meta.json").read_text())
    assert meta["correction"]["points_outside_valid_box"] == 2


def test_cli_accepts_correction(tmp_path):
    fs.main(["--simulate", "--label", "cli", "--out-dir", str(tmp_path), "--correction", "hybrid",
             "--x", "-50", "50", "2", "--y", "-150", "150", "2", "--z", "-650", "-650", "1"])
    meta = json.loads(next(Path(tmp_path).glob("cli_*.meta.json")).read_text())
    assert meta["correction"]["name"] == "hybrid" and meta["points_unreachable"] == 0


def test_geometry_is_the_calibrated_one():
    assert (pc.SB, pc.SP, pc.L_UP, pc.L_LO) == (175.0, 75.0, 177.0, 625.0)
    assert pc.GEOM_TO_PHYS == [2, 0, 1]


def test_scan_warns_when_robot_config_differs(tmp_path, monkeypatch):
    monkeypatch.setattr(robot_config, "SP", 150.0)
    cfg = make_cfg(tmp_path, correction="hybrid", nx=1, xr=(0, 0), ny=1, yr=(0, 0))
    said = []
    path = fs.run_scan(fs.FakeLink(), fs.FakeMag(), cfg, "t", confirm=lambda *_: None, out=said.append)
    assert any("geometry differs" in s for s in said)
    meta = json.loads(path.with_suffix(".meta.json").read_text())
    assert meta["correction"]["geometry_mismatch_with_robot_config"]["SP"] == [150.0, 75.0]
