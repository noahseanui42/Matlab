# FieldScan

MATLAB scripts that move the magnetometer probe through a 3D grid with the delta
robot, read the Phidget 1044 at each point, save the readings to CSV and plot
the field map.

MATLAB reads the 1044 through its **Python interface** using the `Phidget22`
Python package. This avoids the Phidget22 MATLAB/C library, so you don't need
Xcode or MinGW.

## One-time setup (macOS)

1. Install the **Phidget22 drivers** and check that the 1044 shows up in the
   Phidget Control Panel. Calibrate the magnetometer there with the coils off.
2. In Terminal, find your Python and install the Phidget package into it:
   ```
   which python                 # e.g. /opt/anaconda3/bin/python
   python -m pip install Phidget22
   ```
3. Put that path in `scan_config.m` → `cfg.python`.
   MATLAB only works with certain Python versions (check "Versions of Python
   Compatible with MATLAB" for your release). If MATLAB rejects your Python, make
   a compatible environment and use its path instead:
   ```
   conda create -n matlabpy python=3.11
   conda activate matlabpy
   python -m pip install Phidget22
   which python
   ```
4. Find the Arduino's port and put it in `cfg.port`:
   ```matlab
   serialportlist("available")
   ```

## Use

Run these in MATLAB from this folder. **Close the Python delta GUI first**,
because only one program can use the serial port at a time.

```matlab
test_magnetometer                         % live readings; rotate the board to check
[f, f0] = make_demo_scan(); plot_field_map(f, f0)   % try the plots with fake data

b = run_field_scan("baseline");           % coils OFF
c = run_field_scan("coils_on");           % coils ON, same grid
plot_field_map(c, b)                      % coil field only
```

The baseline scan records the Earth's field, the servo motors' magnets and
any sensor offset. Subtracting it leaves just the coils' field.

## Alternative: run the delta app, compile afterwards

If you'd rather drive the robot from the delta app's own programs, the app
can log everything to CSV and MATLAB sorts it into points afterwards.

1. **Make the programs** in MATLAB. Each point gets a 1.5 s "Wait time", and
   grids over 50 points are split into several files:
   ```matlab
   make_program("magnet")          % grid from scan_config.m -> FieldScan/programs/
   ```
2. **In the delta app:** Connect → Enable → **File → Start data log** →
   File → Program, then in that window File → Open the program → Upload → Start. For split grids, load and run
   each file in turn. Finish with **File → Stop data log**. The log goes to
   `FieldScan/data/deltalog_<time>.csv`.
   (In the Python the delta app runs with, do `python -m pip install Phidget22` once.)
3. **Compile and plot** in MATLAB:
   ```matlab
   f = compile_scan("data/deltalog_20261002_141500.csv");   % -> ..._points.csv
   plot_field_map(f)
   ```

The log has one row per robot status line (about 50 per second): time,
probe x/y/z (from the joint angles), angles, `mv/run/en/e` and the latest
1044 reading. `compile_scan` finds each stop as a stretch where the joint
angles don't change. `mv` can't be used for this, because the firmware
reports `mv = 1` during a program's wait time. It then drops the first
0.5 s, averages the rest and rounds positions to the nearest 1 mm. The output
has the same columns as `run_field_scan`, plus `run_no` (which program run)
and `n_samples`.

## Settings (`scan_config.m`)

| Setting | Meaning |
|---|---|
| `xr, yr, zr, nx, ny, nz` | Scan volume (probe coordinates, mm) and number of points per axis |
| `n_avg`, `sample_dt_s` | Samples averaged per point and the time between them |
| `settle_s` | Wait after each move before sampling |
| `speed_v` | Move speed, 1–10 (5–50 mm/s) |
| `R_sensor_to_robot` | Rotation from the sensor's axes to the robot's, if the 1044 isn't mounted square |

Points the firmware reports as unreachable are logged as NaN and skipped.
Each row is written as soon as it's measured, so a scan stopped with Ctrl+C
keeps its data. CSVs go to `FieldScan/data/`.

## CSV columns

`idx, x_mm, y_mm, z_mm, Bx_G, By_G, Bz_G, Bx_std_G, By_std_G, Bz_std_G, err, t_s`

Field values are in gauss (1 G = 100 µT). `err` is the firmware's error code
for the move: 0 = ok, 1 = unreachable, 5 = servo pulse clamped.
