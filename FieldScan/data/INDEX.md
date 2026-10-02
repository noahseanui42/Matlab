# Scan index

All scans: 2 Oct 2026, coils off, `field_scan.py`, 5 s settle unless the name says
`settle0p5s`. Positions are probe (TCP) coordinates in mm, robot frame.
The magnet's side is named by robot axis, not by where you stand:
`xpos` = magnet on the +x side of the grid, `xneg` = the −x side.

## Naming convention

```
<magnet>_<correction>[_<extra>]_run<N>_<YYYYMMDD_HHMMSS>.csv   (+ matching .meta.json)
```

| Part | Values |
|---|---|
| magnet | `nomagnet`, or `magnet-` + side of the grid it sits on (`xpos`, `xneg`, `ypos`, `yneg`, or `under` the grid centre), + `-close` if moved in close, + `-replaced` if taken away and put back |
| correction | `corr-off`, `corr-hybrid` (`field_scan.py --correction`) |
| extra | only when a setting differs from the default, e.g. `settle0p5s` |
| run | repeat number for that exact setup |
| time stamp | added by `field_scan.py` itself |

For a new scan, pass everything before the time stamp as the label and
`field_scan.py` adds the stamp:

```
python field_scan.py --label magnet-xpos_corr-off_run3 --note "..."
python field_scan.py --label magnet-xpos_corr-hybrid_run3 --correction hybrid --note "..."
```

Test, simulated and failed scans go in `checks/`.

## Main dataset (evening, 17:22–19:12)

Grid 5 × 5 × 3: x, y ±50, z −675 / −650 / −625. 75 points each, all reached.
Magnet fixed on the +x side at y ≈ 0, about z −650 (from the notes). The field
is strongest at (+50, 0, −675), the bottom layer, which may be the magnet's
orientation rather than its height.

| File | Setup | Old name |
|---|---|---|
| `nomagnet_corr-off_run1_20261002_172221` | **Background**, no magnet | `2026-10-02_dryrun1_no_magnet` |
| `magnet-xpos_corr-off_run1_20261002_180000` | Magnet, correction off | `2026-10-02_dryrun1_magnet_20261002_180000` |
| `magnet-xpos_corr-hybrid_run1_20261002_182828` | Magnet, hybrid correction | `2026-10-02_dryrun1_magnet_hybrid_20261002_182828` |
| `magnet-xpos_corr-off_run2_20261002_184415` | Magnet, correction off, repeat | `2026-10-02_dryrun1_magnet_off2_20261002_184415` |
| `magnet-xpos_corr-hybrid_run2_20261002_185818` | Magnet, hybrid, repeat | `2026-10-02_dryrun1_magnet_hybrid2_20261002_185818` |

## Earlier magnet scans (early morning, 02:27–03:24)

Same grid. The magnet was moved between these.

| File | Setup | Old name |
|---|---|---|
| `magnet-xneg_corr-off_settle0p5s_run1_20261002_022713` | Magnet on the −x side (field strongest at x −50), 0.5 s settle | `settle_0p5s/mag_off_1_20261002_022713` |
| `magnet-xneg_corr-off_run1_20261002_024004` | Same magnet position, 5 s settle | `mag_off_1_20261002_024004` |
| `magnet-xpos-close_corr-off_run1_20261002_031036` | Magnet moved to the +x side, close: 1.1 G at (+50, 0, −650), peak 1.87 G | `mag_off_1_20261002_031036` |

The first two compare 0.5 s against 5 s settle with the magnet in the same place.

## Magnet under the centre (evening, 20:54–21:56)

Same grid. Strong magnet under the grid centre, pole up, coils off, hybrid
correction. Strongest on the bottom layer (z −675), slightly towards −y.

| File | Setup | Old name |
|---|---|---|
| `magnet-under_corr-hybrid_run1_20261002_205751` | First placement | `2026-10-02_magunder_hybrid_20261002_205751` |
| `magnet-under-replaced_corr-hybrid_run1_20261002_212800` | Magnet taken away and put back | `2026-10-02_magunder_hybrid2_20261002_212800` |
| `magnet-under-replaced_corr-hybrid_run2_20261002_214238` | Same placement, back-to-back repeat | `2026-10-02_magunder_hybrid3_20261002_214238` |

The two `-replaced` runs are the repeatability pair: same placement, 0.005 G
median difference per point. The first placement differs from them by about
0.06 G median, so treat it as a separate map.

## checks/ (not copied into submission_snapshot)

| File | What it is | Old name |
|---|---|---|
| `SIMULATED_demo_corr-hybrid_20261002_013031` | `--simulate`, no hardware. Not real data | `demo_20261002_013031` |
| `check_robot-dryrun-3x3x3_corr-hybrid_settle0p5s_20261002_014611` | First real dry run, 3 × 3 × 3 grid, x, y ±25 | `dryrun_20261002_014611` |
| `ABORTED-3pts_corr-off_settle0p5s_20261002_015453` | Stopped after 3 of 75 points | `settle_0p5s/mag_off_1_20261002_015453` |
| `CRASHED-4pts_corr-off_settle0p5s_20261002_015519` | Died after 4 points; its meta.json was never finished | `settle_0p5s/mag_off_2_20261002_015519` |
| `check_magnet-2pts-xneg-xpos_corr-off_settle0p5s_20261002_015819` | 2 points, x −50 and +50 at (y 0, z −650) | `magcheck_20261002_015819` |
| `check_magnet-2pts-xneg-xpos_corr-off_settle0p5s_20261002_020554` | Same, repeated | `magcheck_20261002_020554` |
| `check_hybrid-2pts-xpos_corr-hybrid_20261002_175834` | 2 points at x +50, z −675 and −650, before the evening set | `hybridcheck_20261002_175834` |
| `ABORTED-9pts_magnet-under_corr-off_20261002_205432` | Magnet under centre, correction off, stopped after 9 of 75 points | `2026-10-02_magunder_off_20261002_205432` |

The `label` and `csv` fields inside each `.meta.json` still show the old name,
because those files are the scan's original record.
