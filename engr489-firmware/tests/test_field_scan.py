"""Host tests for delta_app/field_scan.py using the simulated robot and sensor."""
import csv
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "delta_app"))
import field_scan as fs


def make_cfg(tmp_path, **kw):
    cfg = fs.ScanConfig(out_dir=str(tmp_path), settle_s=0.0, sample_dt_s=0.0, n_avg=5,
                        xr=(-50, 50), nx=3, yr=(-50, 50), ny=2, zr=(-700, -600), nz=2)
    for k, v in kw.items():
        setattr(cfg, k, v)
    return cfg


def test_grid_is_serpentine():
    pts = fs.scan_grid((0, 2), (0, 1), (5, 5), 3, 2, 1)
    assert pts == [(0, 0, 5), (1, 0, 5), (2, 0, 5), (2, 1, 5), (1, 1, 5), (0, 1, 5)]


def test_frame_matches_gui_framing():
    assert fs.frame(8, {"enable": 0}) == b'<8><{"enable":0}><#>'
    assert fs.frame(2, {"n": 0, "i": 0, "v": 2, "a": 0, "c": [1, 2, -3]}) == \
        b'<2><{"n":0,"i":0,"v":2,"a":0,"c":[1,2,-3]}><#>'


def test_scan_writes_csv_and_meta(tmp_path):
    cfg = make_cfg(tmp_path)
    link = fs.FakeLink()
    p = fs.run_scan(link, fs.FakeMag(), cfg, "t", note="n", coil_current_A=1.5,
                    confirm=lambda *_: None, out=lambda *_: None)
    rows = list(csv.DictReader(open(p)))
    assert list(rows[0].keys()) == fs.CSV_COLUMNS
    assert len(rows) == 12
    assert float(rows[0]["Bz_G"]) < -0.4 and int(rows[0]["n_samples"]) == 5
    meta = json.loads(p.with_suffix(".meta.json").read_text())
    assert meta["points_done"] == 12 and meta["coil_current_A"] == 1.5 and not meta["aborted"]
    # TCP offset subtracted from the probe position before sending
    first_move = [pl for m, pl in link.sent if m == 2][1]
    assert first_move["c"][2] == -600 + 21 or first_move["c"][2] == -700 + 21


def test_unreachable_points_logged_as_nan(tmp_path):
    cfg = make_cfg(tmp_path, zr=(-700, -400), nz=2)    # z=-400 is out of reach
    p = fs.run_scan(fs.FakeLink(), fs.FakeMag(), cfg, "t", confirm=lambda *_: None,
                    out=lambda *_: None)
    rows = list(csv.DictReader(open(p)))
    bad = [r for r in rows if r["err"] == "1"]
    assert len(bad) == 6 and all(r["Bx_G"] == "nan" and r["n_samples"] == "0" for r in bad)
    assert json.loads(p.with_suffix(".meta.json").read_text())["points_unreachable"] == 6


def test_speed_sent_as_int(tmp_path):
    # ArduinoJson reads "v":5.0 as 0 (firmware then falls back to V_DEFAULT),
    # so the speed must go out as a JSON integer even if set as a float.
    cfg = make_cfg(tmp_path, speed_v=5.0, nx=1, ny=1, nz=1)
    link = fs.FakeLink()
    fs.run_scan(link, fs.FakeMag(), cfg, "t", confirm=lambda *_: None, out=lambda *_: None)
    moves = [pl for m, pl in link.sent if m == 2]
    assert moves and all(type(pl["v"]) is int and pl["v"] == 5 for pl in moves)
    assert b'"v":5,' in fs.frame(2, moves[0])
