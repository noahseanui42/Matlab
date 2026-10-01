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
