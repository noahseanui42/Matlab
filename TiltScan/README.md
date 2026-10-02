# TiltScan

The FieldScan scan with the 1044's **pitch and roll** (tilt at every point) and
**gyroscope** (was the probe still?) logged in the same row as the field.

The 1044 is read through its **Spatial** channel, the one that works on our board
(the separate Magnetometer / Accelerometer / Gyroscope channels don't open). Every
Spatial event carries acceleration, angular rate and field from the same instant.
The board's own orientation filter (`--algorithm imu`, the default) gives pitch and
roll; the script also works them out from the averaged acceleration as a check.

Nothing in `FieldScan/` or `engr489-firmware/delta_app/` is changed.
`field_scan_tilt.py` imports `field_scan.py` for the robot link, moves, grid and
position correction, and replaces only the per-point reading and the CSV.

| File | What it does |
|---|---|
| `field_scan_tilt.py` | Runs the scan: move, settle, read field + acceleration + gyro together, one CSV row per point |
| `tilt_correct.m` | Tilt per point from the accelerometer; field rotated back to a reference orientation; optional `_tiltcorr.csv` |
| `plot_tilt.m` | Tilt map per z layer, tilt vs distance off-axis, gyro rate and settle time per point |
| `test_field_scan_tilt.py`, `test_tilt_correct.m` | Checks with a simulated tilting sensor (no hardware) |

## Running a scan

Same set-up as FieldScan (close the delta app first; Phidget22 drivers installed).
From the repo root:

```
python TiltScan/field_scan_tilt.py --simulate --label demo          # try it, no hardware
python TiltScan/field_scan_tilt.py --label magnet_tilt --note "magnet at ..." --correction hybrid \
       --x -50 50 5 --y -50 50 5 --z -675 -625 3
```

All of `field_scan.py`'s options work the same way. New ones:

| Option | Meaning | Default |
|---|---|---|
| `--gyro-settle RATE` | Wait until the gyro's angular rate stays below RATE (deg/s), instead of a fixed `--settle` | 0 (off: fixed wait) |
| `--still` | Gyro mode: how long the rate must stay below RATE, s | 1.0 |
| `--settle-min` | Gyro mode: shortest wait, s | 0.5 |
| `--settle` | Fixed wait, or in gyro mode the **longest** wait | 5.0 |
| `--no-zero-gyro` | Don't zero the gyro at the park position before the scan | zero it |
| `--magnet TEXT` | Where the magnet is, e.g. `underneath` or `none`. Goes in the file name (`<label>_<magnet>_<time>.csv`) and the `.meta.json` | not in the name |
| `--algorithm` | Board's orientation filter for pitch/roll: `imu` (accelerometer + gyro), `ahrs` (also the magnetometer) or `none` | `imu` |

Use `imu`, not `ahrs`. AHRS also steers by the magnetometer, which the magnet being
mapped disturbs. Heading isn't logged: it needs the magnetometer (AHRS) or drifts
(IMU), and tilt only needs pitch and roll. If the board refuses the algorithm, the
script warns, logs `pitch_deg`/`roll_deg` as NaN and carries on; the
acceleration-based pitch and roll still work.

Output goes to `TiltScan/data/<label>_<time>.csv` and `.meta.json`.

**Choosing `--gyro-settle`:** do the first scan with the fixed 5 s wait and look at
`gyro_rms_dps` (or figure 2 of `plot_tilt`). Points that were still give you the
gyro's noise level. Set RATE a few times above that. Points where the gyro never
went quiet are logged with `settled = 0` and counted in the metadata.

## CSV columns

The first 19 are exactly `field_scan.py`'s, so every FieldScan MATLAB script reads
these files. Then:

| Column | Meaning |
|---|---|
| `ax_g, ay_g, az_g` | Mean acceleration over the n_avg samples, g, **sensor axes**. Gravity, when still. |
| `ax_std_g, ay_std_g, az_std_g` | Std over the samples (vibration shows up here) |
| `gyro_rms_dps, gyro_max_dps` | Angular rate magnitude while the field was sampled, deg/s |
| `settle_s` | Time waited after the move, s |
| `settled` | 1 = gyro went quiet, 0 = gyro wait timed out, −1 = fixed wait |
| `pitch_deg, roll_deg` | Mean pitch and roll from the board's IMU/AHRS filter, deg (absolute: level = 0 plus any mounting offset) |
| `acc_pitch_deg, acc_roll_deg` | Pitch (about sensor y) and roll (about sensor x) from the mean acceleration, deg. Signs may differ from the board's convention. |
| `spatial_t_first_ms, spatial_t_last_ms` | The 1044's own timestamps of the first and last reading averaged into the row, ms since the Spatial channel opened. With 20 readings at 20 ms they are 380 ms apart. |

**Timing of a row:** after the settle wait, the script clears what the 1044 sent
during the wait and averages the next `n_avg` readings (20 × 20 ms ≈ 0.4 s). Each
reading holds field, acceleration and gyro from the same instant, so the field and
the tilt in a row cover the same window. `t_s` is stamped when the row is written,
just after the window ends. To put the 1044's timestamps on the `t_s` clock, use
`spatial_timestamp_at_t0_ms` from the `.meta.json` (the 1044's timestamp when the
scan's t_s = 0): window start in scan seconds = (spatial_t_first_ms − that) / 1000.

## Afterwards in MATLAB

```matlab
cd TiltScan; addpath ../FieldScan
plot_tilt("data/magnet_tilt_….csv")                     % tilt map, settling, pitch/roll maps, check
S = tilt_correct("data/magnet_tilt_….csv");             % S.tilt_deg, S.B_corr_G, S.pitch_deg, S.roll_deg, ...

% magnet minus background, both tilt-corrected to the SAME reference:
tilt_correct("data/no_magnet_tilt_….csv", "Write", true);
tilt_correct("data/magnet_tilt_….csv", "Reference", "data/no_magnet_tilt_….csv", "Write", true);
fit_dipole("data/magnet_tilt_…_tiltcorr.csv", "data/no_magnet_tilt_…_tiltcorr.csv")
plot_field_layers("data/magnet_tilt_…_tiltcorr.csv", "data/no_magnet_tilt_…_tiltcorr.csv")
```

The `_tiltcorr.csv` has the corrected field in `Bx_G..Bz_G` (raw kept in
`Bx_raw_G..Bz_raw_G`, plus `tilt_deg`), so `fit_dipole`, `plot_field_layers` and
the rest use it unchanged. Compare a fit on the raw and on the corrected files to
see how much of the dipole residual was tilt.

## How the tilt is worked out

**Pitch and roll** are logged as numbers per point (`pitch_deg`, `roll_deg`) and
mapped by `plot_tilt` (figure 3, relative to the grid centre). Figure 4 plots the
board's pitch/roll against the ones from acceleration: while the probe is still they
should agree (slope ±1). The sign tells you the board's convention.

**The correction** uses the gravity vector (`ax_g..az_g`) rather than the pitch/roll
numbers. While the probe is still it is the same information, and it doesn't depend
on the order or signs of the board's Euler angles. At rest the accelerometer measures
gravity, so `g = a/|a|` is "down" in the sensor's axes. The tilt at a point is the angle between its `g` and a reference
`g_ref`. The default reference is the point nearest the middle of the grid; you can
also use another scan's centre, the scan mean, or a given vector. The field is
rotated by the smallest rotation that takes `g` onto `g_ref` (Rodrigues).

Limits:
- **Only roll and pitch.** A turn about the vertical doesn't change gravity, so it
  is not measured or corrected. A delta platform shouldn't turn that way, but
  combined tilts leave a tiny, second-order turn (about 1e-5 G of field at
  0.7° tilt in the test).
- **Relative, not absolute.** The tilt is relative to the reference point, so the
  board's mounting and the accelerometer's offset cancel out. Absolute levelness
  is not measured.
- **Only while still.** Vibration adds to `a`. Check `ax_std_g..az_std_g` and the
  gyro columns.
- **Sign check:** tip the 1044 by hand once and see which way `az_g` moves, so you
  know which way the `plot_tilt` arrows point (downhill if gravity reads as down).

## Not yet checked on hardware

Everything was tested with the simulated robot, a simulated tilting sensor and a
stand-in for Phidget22's Spatial channel: `python -m pytest TiltScan/` (13 tests)
and `test_tilt_correct` in MATLAB/Octave. On the real 1044_0, check on the first run:
- **The algorithm is accepted.** The `.meta.json` `sensor.algorithm` says `imu` (or
  `none` with an `algorithm_error`). `pitch_deg` should be finite in the CSV.
- **The board's pitch/roll agree with the acceleration ones** (`plot_tilt` figure 4).
  If the filter values lag or wander, give it a longer settle time or rely on the
  acceleration values.
- **The gyro zeroes at the park position.** `Spatial.zeroGyro()` takes 1–2 s and
  needs the board still. It runs after the park move and one settle wait.
- **Accelerometer noise and the tilt resolution** you actually get. Take a scan at one
  point with several repeats and look at the spread of `tilt_deg`.
