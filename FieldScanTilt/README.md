# FieldScanTilt

A clone of `FieldScan/` that also logs the 1044's **pitch and roll** (the tilt of
the probe) and its **gyroscope** (was the probe still?) at every point, in the same
CSV row as the field. `FieldScan/` itself is unchanged; use this folder for scans
that need the tilt.

The 1044 is read through its **Spatial** channel. Every reading holds field,
acceleration and angular rate from the same instant, so the field and the tilt in a
row are from the same 0.4 s window. The board's own orientation filter
(`imu`, the default) gives pitch and roll; they are also worked out from the
averaged acceleration as a check.

| Part | Program |
|---|---|
| Set-up: connect, enable, jog, check reach, calibrate servos | **delta app** (`engr489-firmware/delta_app/main.py`) |
| The scan: move through the grid, read field + tilt + gyro, write a CSV | **`field_scan_tilt.py`** (this folder) |
| Afterwards: tilt correction, baseline subtraction, plots, repeatability, dipole fit | **MATLAB** (this folder) |

Only one program can use the Arduino's serial port at a time, so **close the
delta app before running a scan**.

## What's here

| File | What it does |
|---|---|
| `field_scan_tilt.py` | The scan. Uses `delta_app/field_scan.py` for the robot link, moves, grid and position correction, and reads field + acceleration + gyro + pitch/roll at each point |
| `tilt_correct.m` | Tilt per point from the accelerometer; field rotated back to a reference orientation; optional `_tiltcorr.csv` |
| `plot_tilt.m` | Tilt map per z layer, tilt vs distance off-axis, gyro rate and settle time, pitch/roll maps, board vs acceleration check |
| `plot_field_map.m`, `plot_field_layers.m`, `plot_field_arrows3d.m` | Field plots, as in FieldScan (read the tilt CSVs and the `_tiltcorr.csv` unchanged) |
| `compare_runs.m` | Magnet repeatability, as in FieldScan |
| `fit_dipole.m` | Point-dipole fit (moment + position) with residuals |
| `run_field_scan.m`, `scan_config.m` | MATLAB-only scan (fallback), now with tilt and gyro; settings in `scan_config.m` |
| `mag_open.m`, `mag_read.m`, `mag_settle.m` | The 1044 from MATLAB: open the Spatial channel, read a point, wait for the gyro |
| `test_magnetometer.m` | Live field, pitch, roll and gyro readings |
| `delta_*.m` | Serial link to `delta_servo`, as in FieldScan (`delta_move` also returns the servo angles) |
| `make_demo_scan.m` | Fake coils-on and baseline scans **with tilt columns**, for trying the plots without hardware |
| `test_field_scan_tilt.py`, `test_tilt_correct.m`, `test_fit_dipole.m` | Checks with simulated data (no hardware) |

## One-time setup (macOS)

Same as FieldScan:

1. Install the **Phidget22 drivers** and check that the 1044 shows up in the
   Phidget Control Panel. Calibrate the magnetometer there with the coils off.
2. Install the Python packages into the Python you run the delta app with:
   ```
   cd engr489-firmware/delta_app
   python -m pip install -r requirements.txt     # pyserial, matplotlib, pandas, Phidget22
   python -m pip install numpy                   # only for --correction hybrid
   ```
3. Find the Arduino's port: `ls /dev/cu.usbmodem*`. Pass `--port` if it isn't
   `robot_config.SERIAL_PORT_DEFAULT`.
4. Try a scan with no hardware connected (from the repo root):
   ```
   python FieldScanTilt/field_scan_tilt.py --simulate --label demo
   ```

## Running a scan

From the repo root:

```
python FieldScanTilt/field_scan_tilt.py --label baseline --note "coils off"
python FieldScanTilt/field_scan_tilt.py --label magnet_tilt --note "magnet at ..." --correction hybrid \
       --x -50 50 5 --y -50 50 5 --z -675 -625 3
```

What happens (as in FieldScan, plus the tilt):
- The script opens the 1044 first, then the robot.
- If the robot is disabled, it asks you to support the arms and switch servo
  power on, then enables it.
- It parks at `(0, 0, -650)` and **zeroes the gyro** there (keep the robot still),
  then goes through the grid in serpentine order. At each point it moves, waits
  for the robot to report it has stopped, settles (fixed 5 s, or until the gyro
  goes quiet), averages 20 readings of field + acceleration + gyro + pitch/roll and
  writes the row straight away. Ctrl+C keeps everything measured so far.
- Points the firmware rejects as unreachable are written as NaN.
- At the end it parks and leaves the robot **enabled**, because disabling lets
  the arms drop.

Output goes to `FieldScanTilt/data/<label>_<time>.csv`, plus a `.meta.json`
recording the label, note, coil current, every setting, the sensor's serial and
range, the orientation filter used, whether the gyro was zeroed, and whether the
scan finished.

All of `field_scan.py`'s options work the same way:

| Option | Meaning | Default |
|---|---|---|
| `--x MIN MAX N`, `--y …`, `--z …` | Grid (probe coordinates, mm) and points per axis | ±50, ±50, −700…−600, 5 each |
| `--speed` | 1–10 (5–50 mm/s) | 2 |
| `--settle` | Fixed wait after each move, s; in gyro mode the **longest** wait | 5.0 |
| `--n-avg` | Readings averaged per point | 20 |
| `--port` | Arduino serial port | from `robot_config.py` |
| `--note`, `--coil-current` | Stored in the `.meta.json` | |
| `--correction` | Position correction: `off` or `hybrid` (see `FieldScan/README.md`) | `off` |
| `--simulate` | Fake robot and a fake sensor on a tilting platform | |

New ones:

| Option | Meaning | Default |
|---|---|---|
| `--gyro-settle RATE` | Wait until the gyro's angular rate stays below RATE (deg/s), instead of a fixed `--settle` | 0 (off: fixed wait) |
| `--still` | Gyro mode: how long the rate must stay below RATE, s | 1.0 |
| `--settle-min` | Gyro mode: shortest wait, s | 0.5 |
| `--no-zero-gyro` | Don't zero the gyro at the park position before the scan | zero it |
| `--algorithm` | Board's orientation filter for pitch/roll: `imu` (accelerometer + gyro), `ahrs` (also the magnetometer) or `none` | `imu` |

Use `imu`, not `ahrs`: AHRS also steers by the magnetometer, which the magnet being
mapped disturbs. Heading isn't logged (it needs the magnetometer or drifts), and
tilt only needs pitch and roll. If the board refuses the filter, the script warns,
logs `pitch_deg`/`roll_deg` as NaN and carries on; the acceleration-based pitch and
roll still work.

**Choosing `--gyro-settle`:** do the first scan with the fixed 5 s wait and look at
`gyro_rms_dps` (or figure 2 of `plot_tilt`). Points that were still give you the
gyro's noise level. Set RATE a few times above that. Points where the gyro never
went quiet are logged with `settled = 0` and counted in the metadata.

## CSV columns

The first 19 are exactly FieldScan's:

`idx, x_mm, y_mm, z_mm, Bx_G, By_G, Bz_G, Bx_std_G, By_std_G, Bz_std_G, err, t_s, n_samples, deg1, deg2, deg3, sent_x_mm, sent_y_mm, sent_z_mm`

so every FieldScan MATLAB script reads these files. Then the new data at each point:

| Column | Meaning |
|---|---|
| `ax_g, ay_g, az_g` | Mean acceleration over the readings, g, **sensor axes**. Gravity, when still. |
| `ax_std_g, ay_std_g, az_std_g` | Std over the readings (vibration shows up here) |
| `gyro_rms_dps, gyro_max_dps` | Angular rate magnitude while the field was sampled, deg/s |
| `settle_s` | Time waited after the move, s |
| `settled` | 1 = gyro went quiet, 0 = gyro wait timed out, −1 = fixed wait |
| `pitch_deg, roll_deg` | Mean pitch and roll from the board's IMU/AHRS filter, deg (absolute: level = 0 plus any mounting offset) |
| `acc_pitch_deg, acc_roll_deg` | Pitch (about sensor y) and roll (about sensor x) from the mean acceleration, deg. Signs may differ from the board's convention. |
| `spatial_t_first_ms, spatial_t_last_ms` | The 1044's own timestamps of the first and last reading in the row, ms since the channel opened (20 readings at 20 ms: 380 ms apart) |

Positions (`x_mm`…`z_mm`) are the target probe position; `sent_*` is what was sent
to the robot. Field values are raw gauss in the **sensor's** axes (1 G = 100 µT).
`err` is the firmware's code for the move: 0 = ok, 1 = unreachable, 5 = servo
pulse clamped.

**Timing of a row:** after the settle wait, the script drops what the 1044 sent
during the wait and averages the next `n_avg` readings (20 × 20 ms ≈ 0.4 s). `t_s`
is stamped when the row is written, just after that window. To put the 1044's
timestamps on the `t_s` clock, use `spatial_timestamp_at_t0_ms` from the
`.meta.json`: window start in scan seconds = (spatial_t_first_ms − that) / 1000.

## Afterwards in MATLAB

```matlab
cd FieldScanTilt
plot_tilt("data/magnet_tilt_….csv")                     % tilt map, settling, pitch/roll maps, check
S = tilt_correct("data/magnet_tilt_….csv");             % S.tilt_deg, S.B_corr_G, S.pitch_deg, S.roll_deg, ...

% everything FieldScan does works on these CSVs as they are:
plot_field_map("data/coils_on_….csv", "data/baseline_….csv")
plot_field_layers("data/magnet_tilt_….csv", "data/no_magnet_tilt_….csv")
plot_field_arrows3d("data/magnet_tilt_….csv")
R = compare_runs("data/magnet_run*.csv", "data/no_magnet_….csv");

% magnet minus background, both tilt-corrected to the SAME reference:
tilt_correct("data/no_magnet_tilt_….csv", "Write", true);
tilt_correct("data/magnet_tilt_….csv", "Reference", "data/no_magnet_tilt_….csv", "Write", true);
plot_field_layers("data/magnet_tilt_…_tiltcorr.csv", "data/no_magnet_tilt_…_tiltcorr.csv")
fit_dipole("data/magnet_tilt_…_tiltcorr.csv", "data/no_magnet_tilt_…_tiltcorr.csv")
```

The `_tiltcorr.csv` has the corrected field in `Bx_G..Bz_G` (raw kept in
`Bx_raw_G..Bz_raw_G`, plus `tilt_deg`), so every plot uses it unchanged. Compare a
plot or fit on the raw and on the corrected files to see how much was tilt.

To try it all without hardware:
```matlab
[f, f0] = make_demo_scan(); plot_field_map(f, f0); plot_tilt(f)
```

## How the tilt is worked out

**Pitch and roll** are logged as numbers per point and mapped by `plot_tilt`
(figure 3, relative to the grid centre). Figure 4 plots the board's pitch/roll
against the ones from acceleration: while the probe is still they should agree
(slope ±1). The sign tells you the board's convention.

**The correction** uses the gravity vector (`ax_g..az_g`) rather than the pitch/roll
numbers. While the probe is still it is the same information, and it doesn't depend
on the order or signs of the board's angles. At rest the accelerometer measures
gravity, so `g = a/|a|` is "down" in the sensor's axes. The tilt at a point is the
angle between its `g` and a reference `g_ref` (default: the point nearest the middle
of the grid; or another scan's centre, the scan mean, or a given vector). The field
is rotated by the smallest rotation that takes `g` onto `g_ref`.

Limits:
- **Only roll and pitch.** A turn about the vertical doesn't change gravity, so it
  is not measured or corrected.
- **Relative, not absolute.** The board's mounting and the accelerometer's offset
  cancel out; absolute levelness is not measured.
- **Only while still.** Vibration adds to `a`. Check `ax_std_g..az_std_g` and the
  gyro columns.
- **Sign check:** tip the 1044 by hand once (`test_magnetometer`) and see which way
  pitch, roll and `az_g` move.

## MATLAB-only scan (fallback)

`run_field_scan.m` does the same scan from MATLAB and writes the same 35 columns
(no position correction: `sent_*` = target). It reads the 1044 through the same
Python code as `field_scan_tilt.py` (`mag_open`, `mag_read`, `mag_settle`), so set
`cfg.python` in `scan_config.m` to the Python that has Phidget22. The 1044's
readings arrive on a Python thread, so MATLAB must run Python **out of process**:
`mag_open` sets this if Python isn't loaded yet; otherwise restart MATLAB and run
`pyenv('ExecutionMode', 'OutOfProcess')` first. The tilt settings
(`algorithm`, `gyro_settle_dps`, `still_s`, `settle_min_s`, `zero_gyro`) are in
`scan_config.m`. Unlike FieldScan's `run_field_scan.m`, the field is written in
**sensor axes** (the plots apply `R_sensor_to_robot`, so it isn't applied twice).
`test_magnetometer.m` prints live field, pitch, roll and gyro.

## Not yet checked on hardware

Tested with the simulated robot, a simulated tilting sensor and a stand-in for the
Spatial channel (`python -m pytest FieldScanTilt/`, 14 tests), and the MATLAB side in
Octave (`test_tilt_correct`, `test_fit_dipole`, and `make_demo_scan` through
`plot_tilt`, `tilt_correct`, `plot_field_layers`, `plot_field_arrows3d`). The
MATLAB→Python link (`mag_open`, `mag_read`, `mag_settle`, `run_field_scan`,
`test_magnetometer`) has not been run in MATLAB. On the first real run, check:
- **The filter is accepted.** The `.meta.json` `sensor.algorithm` says `imu` (or
  `none` with an `algorithm_error`), and `pitch_deg` is finite in the CSV.
- **The board's pitch/roll agree with the acceleration ones** (`plot_tilt` figure 4).
- **The gyro zeroes at the park position.** It takes 1–2 s and needs the board still.
- **The tilt resolution you actually get.** Take a scan at one point with several
  repeats and look at the spread of `tilt_deg`.
