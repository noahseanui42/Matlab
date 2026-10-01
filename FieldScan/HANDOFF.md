# HANDOFF: FieldScan (magnetometer + delta robot scan)

This note is for whoever picks up the field-mapping work next. Read it before
changing anything in `FieldScan/`.

Repo: `noahseanui42/Matlab`, branch `claude/brave-edison-mui291`
(FieldScan was added in commit "Add FieldScan: MATLAB field-mapping scan with
the Phidget 1044"; no PR opened yet).

## Project in one paragraph

This is an ENGR489 capstone at VUW, supervised by Christopher Hollit. A delta
robot moves a magnetometer probe through a 3D volume inside a Helmholtz coil
to map the field. **Hard rule: nothing magnetic near the probe** (PLA, carbon
fibre, aluminium, nylon only). The Arduino is an UNO R4 **Minima**, not the
WiFi model, to avoid RF near the sensor. It drives 3 hobby servos on
D9/D10/D11 from a separate 6 V AA pack. The firmware and a patched Python GUI
("delta app") are in `engr489-firmware/`; read its README for geometry,
calibration and the serial protocol. The MATLAB kinematics are in
`Kinematics/`.

## Sensor: Phidget 1044 (PhidgetSpatial Precision 3/3/3)

- The board is the **1044_0** (original version), USB serial number **302277**.
  It plugs into the PC over USB, not into the Arduino.
- The magnetometer range of the 1044_0 hasn't been confirmed. ±8 G was quoted
  from the current 1044_1 spec. `test_magnetometer.m` prints the real range
  using `getMaxMagneticField()`. Keep the coil or reference-magnet field inside
  that range.
- Expected reading with the coils off is about 0.55–0.6 G total (the Earth's
  field in Wellington).

### Which computers work
| Machine | Result |
|---|---|
| Rochelle's MacBook Pro (Intel, T2 chip, Touch Bar, Anaconda `(base)`) | **Works.** `ioreg` shows `1044_0`, the Phidget22 drivers are installed, the kernel extension is allowed and the board **shows in the Phidget Control Panel**. **This is the scan machine.** |
| Windows PC | Detected: `USB\VID_06C2&PID_0033\302277`, status OK. Phidget22 drivers not confirmed installed. |
| The user's M2 MacBook Air, Sonoma 14, through a USB-C adapter | **Not detected at the USB level** (missing from System Information → USB), even after allowing the kernel extension and restarting. It showed up once at first. The "Allow accessories to connect" option didn't appear in the settings. Most likely the adapter. Not resolved; the user moved to the MacBook Pro. |

Still to do on the MacBook Pro: calibrate the magnetometer in the Control
Panel with the coils off. The user hadn't confirmed doing this.

## What's in `FieldScan/`

| File | Purpose |
|---|---|
| `scan_config.m` | All settings: serial port, TCP offset `[0 0 -21]`, speed, settle time, grid, Python path, sensor serial, averaging, sensor→robot rotation, output folder |
| `mag_open.m` / `mag_read.m` | Read the 1044 through **MATLAB's Python interface** (`py.` calls to the `Phidget22` pip package). This was chosen over the Phidget22 MATLAB/C library so the user doesn't need Xcode. Integer arguments are passed as `int32` because ctypes rejects floats. |
| `test_magnetometer.m` | Prints the sensor range, then 20 live readings |
| `delta_connect.m`, `delta_status.m`, `delta_send.m`, `delta_move.m` | Serial link to `delta_servo`. Sets DTR (required on the R4's USB). Uses the GUI's framing `<2><{"n":0,"i":0,"v":V,"a":0,"c":[x,y,z]}><#>`, where `c` = probe position − TCP offset. Joint moves only (`i=0`). |
| `run_field_scan.m` | Opens the magnetometer, then the robot. Asks before enabling (`<8><{"enable":0}>` means ENABLE; the logic is inverted). Parks the robot, then works through the serpentine grid from `Kinematics/scan_grid.m`. For each point: move, wait for `mv==0`, settle, average N samples, append a CSV row. Points the firmware reports as unreachable (`e=1`) are logged as NaN. Leaves the robot **enabled** at the end, because disabling lets the arms drop. |
| `plot_field_map.m` | Field vectors coloured by \|B\|, \|B\| slices, and % deviation from the centre on the middle z plane. Optionally subtracts a coils-off baseline CSV taken on the same grid. Plots in µT; CSVs are in gauss. |
| `make_demo_scan.m` | Fake Helmholtz-pair scan (Biot–Savart, R = 0.3 m, 50 A-turns) plus an Earth-field baseline, for trying the plots without hardware |
| `README.md` | Setup and usage steps for the user |

CSV columns: `idx,x_mm,y_mm,z_mm,Bx_G,By_G,Bz_G,Bx_std_G,By_std_G,Bz_std_G,err,t_s`

## What has been checked and what hasn't

Checked:
- MATLAB-style move and enable frames were fed through the real
  `delta_servo/protocol.cpp`, compiled on the host. They parse correctly,
  including integer JSON values read as floats.
- The Phidget22 Python method names exist in the current pip package.
- The demo Biot–Savart centre field (1.4986 G) matches the Helmholtz formula
  (1.4985 G).

**Not checked: none of the `.m` files has been run in MATLAB.** No MATLAB
was available. Expect small fixes on the first real run. Specific risks:
- MATLAB ↔ Anaconda Python version compatibility (`pyenv`). The README gives
  a conda python=3.11 fallback.
- `cellfun(@double, cell(pylist))` conversion of Python lists
- `serialport` `readline`/`flush` behaviour with the 20 ms status stream
- Handling of `Bsd` in `run_field_scan.m`: rotating the standard deviation
  with `abs(R)` is only approximate unless R is axis-aligned.

## Timing and sync (explained to the user)

The firmware streams `{"deg":[..],"mv":0|1,"run":0|1,"en":0|1,"e":0..5}`
every 20 ms. **The `mv` change from 1 to 0 is the "arrived" trigger.** Every
move takes at least 200 ms, which is why `delta_move` throws away buffered
lines 100 ms after sending. Only one program can hold the serial port, so the
delta GUI and MATLAB can't run at the same time.

Recommended to the user (**Option 1**): use the delta GUI to jog and set up,
close it, then let MATLAB drive the scan. The GUI's program feature is capped
at 50 points (`MAX_POINTS`).

**Option 2** (offered, not chosen): add Phidget reads to the Python delta
GUI, triggered on `mv` 1→0 while `run==1`. Each point's dwell would need to
be at least about 1.5 s. The status line has no point index, so points are
numbered by counting arrivals.

## Delta-app logging route (added after the first handoff)

The user chose to **drive scans from the delta app and compile the data
afterwards**:
- `engr489-firmware/delta_app/scan_logger.py` writes a CSV row for each status
  line the GUI parses. Each row holds time, FK probe xyz, deg, mv/run/en/e, the
  latest 1044 field from a Phidget event thread, and a `b_seq` counter. In
  `deltagui.py` this is wired to **File → Start/Stop data log** and a hook in
  `readEncoders()`. The settings are `MAG_SERIAL`, `MAG_DATA_INTERVAL_MS` and
  `SCAN_LOG_DIR` (defaults to `FieldScan/data`) in `robot_config.py`.
  `Phidget22` was added to `requirements.txt`. All 12 tests pass. The logger
  was checked with a fake magnetometer; the GUI hasn't been run with real
  hardware.
- `FieldScan/make_program.m` writes delta-app program files (the GUI's
  2-line JSON format) with a "Wait time" after every point, in chunks of 50
  or fewer (`MAX_POINTS`).
- `FieldScan/compile_scan.m` splits the log into stops. **Correction to the
  timing note above:** during a program's dwell the firmware's `isMoving()`
  is true, so **`mv` stays 1 through the wait time**, and mv 1→0 is no good
  as a per-point trigger for programs. Stops are found where the joint
  angles are unchanged (|Δdeg| < 0.005 between status lines) for at least
  `min_dwell_s`. The first `settle_s` is dropped, the rest averaged (one
  sample per new `b_seq`), and xyz snapped to 1 mm. A Python port of the
  algorithm was checked against a simulated 6-point log (firmware
  smoothstep, dwell with mv=1): 6 of 6 stops found at the right positions.
  The `.m` file itself has not been run in MATLAB.
- `plot_field_map` now matches the baseline to the data **by position**
  (0.1 mm) instead of requiring identical row order.

## Next steps (the user hasn't chosen between these)

1. **Magnet repeatability test (the user's stated next step).** Fix a
   reference magnet firmly, out of the arm's path, far enough away not to
   overload the sensor. Run the same small grid 5–10 times
   (`run_field_scan("magnet_run"+k)`), plus one run without the magnet for
   the noise floor. **Offered: `compare_runs.m`.** It would compute the spread
   of B across runs at each point, convert that to position repeatability as
   σ_pos ≈ σ_B / |∇B| using the local gradient, and plot the result.
2. Or Option 2 above (magnetometer inside the delta GUI).
3. On the first real run, fix whatever breaks in MATLAB (see the risks above).
4. Set `cfg.R_sensor_to_robot` once the 1044's mounting in the PLA holder
   is known.

## Known inconsistencies elsewhere in the repo (not fixed)

- `Kinematics/init_scan.m` still has `L2 = 600` with the comment "600 vs 700
  unresolved". The firmware and README confirm the forearm is **625 mm**.
  `sb=175`, `sp=150` match the firmware. FieldScan doesn't use `init_scan.m`;
  it only calls `scan_grid.m`.
- Reachable volume, from the firmware README: on-axis z ≈ −550 to −793 mm
  (the z-limit is −800). The default grid is x, y ±50 and z −600 to −700
  (probe coordinates), 5×5×5. The firmware rejects unreachable points itself.
- `robot_config.SERIAL_PORT_DEFAULT = /dev/cu.usbmodem14101` was found on the
  GUI Mac and may be different on the MacBook Pro. Use
  `serialportlist("available")` to find the port.
