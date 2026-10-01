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
  (`mv` 1→0), settles 0.5 s, averages 20 sensor readings and writes the row
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
| `--settle` | Wait after each move, s | 0.5 |
| `--n-avg` | Readings averaged per point | 20 |
| `--port` | Arduino serial port | from `robot_config.py` |
| `--note`, `--coil-current` | Stored in the `.meta.json` | |
| `--simulate` | Fake robot and sensor | |

## Afterwards in MATLAB

```matlab
plot_field_map("data/coils_on_….csv", "data/baseline_….csv")   % coil field only
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

To try the plots without hardware:
```matlab
[f, f0] = make_demo_scan(); plot_field_map(f, f0)
```

## CSV columns

`idx, x_mm, y_mm, z_mm, Bx_G, By_G, Bz_G, Bx_std_G, By_std_G, Bz_std_G, err, t_s, n_samples, deg1, deg2, deg3`

Positions are the commanded probe position. Field values are raw gauss in
the **sensor's** axes (1 G = 100 µT). `err` is the firmware's code for the
move: 0 = ok, 1 = unreachable, 5 = servo pulse clamped.

## MATLAB-only scan (fallback)

`run_field_scan.m` does the same scan from MATLAB, reading the 1044 through
MATLAB's Python interface. Its settings are in `scan_config.m`
(`cfg.python`, `cfg.port`, grid). Its CSV has the first 12 columns above.
`test_magnetometer.m` prints live readings and the sensor's range.
