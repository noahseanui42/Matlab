# FieldScan

Maps the magnetic field inside the Helmholtz coil. The work is split into
three parts:

| Part | Program |
|---|---|
| Set-up: connect, enable, jog, check reach, calibrate servos | **delta app** (`engr489-firmware/delta_app/main.py`) |
| The scan: move through the grid, read the 1044, write a CSV | **`field_scan.py`** (`engr489-firmware/delta_app/`) |
| Afterwards: baseline subtraction, plots, repeatability | **MATLAB** (this folder) |

Only one program can use the Arduino's serial port at a time, so **close the
delta app before running `field_scan.py`**. Open it again afterwards if you
need to jog.

## One-time setup (macOS)

1. Install the **Phidget22 drivers** and check that the 1044 shows up in the
   Phidget Control Panel. Calibrate the magnetometer there with the coils off.
2. In Terminal, install the Python packages into the Python you run the delta
   app with:
   ```
   cd engr489-firmware/delta_app
   python -m pip install -r requirements.txt     # pyserial, matplotlib, pandas, Phidget22
   ```
3. Find the Arduino's port: `ls /dev/cu.usbmodem*`. The default is
   `robot_config.SERIAL_PORT_DEFAULT`. Pass `--port` if yours is different.
4. Try a scan with no hardware connected:
   ```
   python field_scan.py --simulate --label demo
   ```

## Running a scan

```
cd engr489-firmware/delta_app
python field_scan.py --label baseline --note "coils off"
python field_scan.py --label coils_on --coil-current 2.0 --note "coils on"
```

What happens:
- The script opens the 1044 first, then the robot.
- If the robot is disabled, it asks you to support the arms and switch servo
  power on, then enables it.
- It parks at `(0, 0, -650)`, then goes through the grid in serpentine order.
  At each point it moves, waits for the robot to report it has stopped
  (`mv` 1→0, after the dip and rise), settles 5 s, averages 20 sensor readings and writes the row
  straight away. Ctrl+C keeps everything measured so far.
- Points the firmware rejects as unreachable are written as NaN.
- At the end it parks and leaves the robot **enabled**, because disabling
  lets the arms drop.

Output goes to `FieldScan/data/<label>_<time>.csv`, plus a `.meta.json`
recording the label, note, coil current, every setting, the sensor's serial
and range, and whether the scan finished.

Useful options:

| Option | Meaning | Default |
|---|---|---|
| `--x MIN MAX N`, `--y …`, `--z …` | Grid (probe coordinates, mm) and points per axis | ±50, ±50, −700…−600, 5 each |
| `--speed` | 1–10 (5–50 mm/s) | 2 |
| `--settle` | Wait after each move (after the rise), s | 5.0 |
| `--n-avg` | Readings averaged per point | 20 |
| `--port` | Arduino serial port | from `robot_config.py` |
| `--note`, `--coil-current` | Stored in the `.meta.json` | |
| `--correction` | Position correction: `off` or `hybrid` (see below) | `off` |
| `--simulate` | Fake robot and sensor | |

### Position correction (`--correction hybrid`)

Off-centre, the servos give way slightly under load and the probe falls short of
its target, pulled back towards the axis. Uncorrected, the corners of the scan box
land about 19 mm in. `--correction hybrid` predicts each servo's shortfall from the
robot's Jacobian and aims past the target by that much (`delta_app/pose_correction.py`,
coefficients in `delta_app/pose_correction_hybrid.json`).

- **Validated** with pen-and-ruler tests inside x ±50, y ±150, z −600 to −700 (probe):
  corner error about 19 → 2.3 mm rms; box 100 × 299 mm at z −650. It warns if the
  grid goes outside that box. Repeatability is 2–2.5 mm. About 5 mm remains at the
  box centre at z −650 from platform tilt, which no servo-angle correction can remove.
  The calibration notes are in the ENGR489 report repo, `calibration/2026-10-01_*.md`.
- **Needs the calibrated firmware** (branch `claude/amazing-lovelace-onixrv`: geometry
  SB 175, SP 75, L_UP 177, arm remap, refitted servo calibration, upward approach).
  This branch's `robot_config.py` and `delta_servo/` are older (SP 150, L_UP 180, no
  remap); the script warns about that and the correction uses the calibrated
  geometry from its JSON file.
- `x_mm, y_mm, z_mm` stay the **target** (where the reading belongs); the coordinates
  actually sent are in `sent_x_mm, sent_y_mm, sent_z_mm`. The `.meta.json` records
  the correction and its coefficients.
- A point the correction would push out of reach is logged as unreachable and not sent.

## Afterwards in MATLAB

```matlab
plot_field_map("data/coils_on_….csv", "data/baseline_….csv")   % coil field only
plot_field_layers("data/mag_off_1_….csv")                       % heat map, direction map, side views, 3D stacked map
plot_field_layers("data/mag_off_1_….csv", "", "Side", "yz")     % side views in y-z instead of x-z
plot_field_arrows3d("data/mag_off_1_….csv")                     % 3D arrows coloured by layer on see-through planes
plot_field_arrows3d("data/mag_off_1_….csv", "", "Equal", true)  % same, all arrows the same length
plot_field_layers("data/mag_off_1_….csv", "data/no_magnet_….csv") % same, magnet (or coil) field only
plot_field_layers("data/mag_off_1_….csv", "", "Save", true)     % also save PNGs next to the CSV
```

The baseline scan records the Earth's field, the servo motors' magnets and
any sensor offset. Subtracting it leaves just the coils' field. Points are
matched by position, so a point skipped in one scan doesn't shift the rest.

Magnet repeatability: run the same grid several times with a fixed magnet,
plus once with the magnet removed, then:
```matlab
R = compare_runs("data/magnet_run*.csv", "data/no_magnet_….csv");
```
This reports the spread of |B| across runs and converts it to a position
repeatability in mm (spread ÷ field gradient).

### Magnet at several positions

Repeat the magnet test with the magnet in several places, with a few runs at
each place. Moving the magnet moves the strong gradient around the grid, so
the repeatability figure covers more of the volume and doesn't depend on one
magnet placement.

1. Keep the grid, speed, settle time and correction exactly as in the first magnet
   run so every scan matches `2026-10-02_dryrun1_no_magnet.csv`:
   `--z -675 -625 3` (x and y stay at the default ±50 mm, 5 points).
2. Fix the magnet down with tape or a printed holder, nothing ferrous, and
   write down where it is. **Don't touch the magnet or the robot base between runs
   at the same position.** Only move it when you start the next position.
3. Name the runs `magP<position>_run<repeat>`:
   ```
   python field_scan.py --label magP1_run1 --z -675 -625 3 --note "magnet P1: <where>, coils off"
   python field_scan.py --label magP1_run2 --z -675 -625 3 --note "magnet P1: <where>, coils off"
   python field_scan.py --label magP1_run3 --z -675 -625 3 --note "magnet P1: <where>, coils off"
   # move the magnet
   python field_scan.py --label magP2_run1 --z -675 -625 3 --note "magnet P2: <where>, coils off"
   ...
   ```
   Each run takes about 14 minutes (75 points), so 3 positions × 3 runs is about
   2 hours 10 minutes.
4. Finish with another magnet-absent scan (`--label no_magnet_end`). Comparing it with
   the first one shows how much the background drifted over the session.
5. In MATLAB:
   ```matlab
   S = compare_positions("data/magP*_run*.csv", "data/2026-10-02_dryrun1_no_magnet.csv", ...
                         "Names", ["P1 near +x", "P2 ...", "P3 ..."]);
   S.table      % one row per position: runs, peak magnet field and where, sigma_B, sigma_pos
   S.pooled     % all positions together
   ```
   Figure 1 shows the magnet's field on each z layer for each position. Check that
   the peak (×) moves when you move the magnet. Figure 2 shows sigma_B and sigma_pos
   at every point, per position and pooled.
   If the magnet hasn't moved since the first magnet run, add that run to
   position 1 by giving the groups yourself:
   ```matlab
   S = compare_positions({["data/2026-10-02_dryrun1_magnet_20261002_180000.csv" "data/magP1_*.csv"], ...
                          "data/magP2_*.csv", "data/magP3_*.csv"}, ...
                         "data/2026-10-02_dryrun1_no_magnet.csv");
   ```

To try the plots without hardware:
```matlab
[f, f0] = make_demo_scan(); plot_field_map(f, f0)
```

## CSV columns

`idx, x_mm, y_mm, z_mm, Bx_G, By_G, Bz_G, Bx_std_G, By_std_G, Bz_std_G, err, t_s, n_samples, deg1, deg2, deg3, sent_x_mm, sent_y_mm, sent_z_mm`

Positions (`x_mm`…`z_mm`) are the target probe position; `sent_*` is what was sent
to the robot (the same unless `--correction` is on). Field values are raw gauss in
the **sensor's** axes (1 G = 100 µT). `err` is the firmware's code for the
move: 0 = ok, 1 = unreachable, 5 = servo pulse clamped.

## MATLAB-only scan (fallback)

`run_field_scan.m` does the same scan from MATLAB, reading the 1044 through
MATLAB's Python interface. Its settings are in `scan_config.m`
(`cfg.python`, `cfg.port`, grid). Its CSV has the first 12 columns above.
`test_magnetometer.m` prints live readings and the sensor's range.
