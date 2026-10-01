# HANDOFF: FieldScan (magnetometer + delta robot scan)

This note is for whoever picks up the field-mapping work next. Read it before
changing anything in `FieldScan/`.

Repo: `noahseanui42/Matlab`, branch `claude/brave-edison-mui291`
(no PR opened yet). This branch now also contains the other session's
`claude/gallant-planck-stiamj` work (`field_scan.py`, `compare_runs.m`).

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
| `plot_field_map.m` | Field vectors coloured by \|B\|, \|B\| slices, and % deviation from the centre on the middle z plane. Optionally subtracts a coils-off baseline, matched to the data by position (0.1 mm). Plots in µT; CSVs are in gauss. Reads CSVs from both `field_scan.py` and `run_field_scan.m`. |
| `compare_runs.m` | Magnet repeatability, see below |
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

## Decided: how scans are run

**`engr489-firmware/delta_app/field_scan.py` runs the scans and writes a CSV.
MATLAB only post-processes.** The delta app is still used for set-up (connect,
enable, jog, check reach, calibrate) but must be **closed** during a scan,
because only one program can hold the serial port.

- `field_scan.py` was written by another session (branch
  `claude/gallant-planck-stiamj`) and merged here. It is a Python port of
  `run_field_scan.m`: the same framing, DTR, TCP offset, park position,
  serpentine grid, mv 1→0 arrival, settle, average and NaN-for-unreachable
  logic. It adds `n_samples, deg1..3` columns (the first 12 columns match
  the MATLAB CSV), a `.meta.json` per scan (label, note, coil current, config,
  sensor info, done/aborted), `--simulate` with `FakeLink`/`FakeMag`, and
  `tests/test_field_scan.py`.
- **Fixed after the merge:** the speed was sent as a JSON float (`"v":2.0`).
  ArduinoJson's `doc["v"] | 0` reads a float as 0, and the firmware then
  falls back to `V_DEFAULT` = 2, so `--speed` was silently ignored. This was
  confirmed by feeding both frames through the compiled `protocol.cpp`. It is
  now sent as an int, `--speed` is an int from 1 to 10, and a regression test
  was added. All 17 tests pass.
- **Removed:** the delta-app data log (`scan_logger.py`, File → Start data
  log, `compile_scan.m`, `make_program.m`). `deltagui.py` and
  `robot_config.py` are back to their master versions.
- **Timing facts worth keeping:** status lines arrive every 20 ms, every move
  takes at least 200 ms, and buffered lines are flushed 100 ms after a
  command. **During a delta-app program's "Wait time" the firmware reports
  `mv = 1`** (`isMoving()` includes DWELL), so mv can't mark stops inside
  programs. That doesn't matter for `field_scan.py`, which sends single
  manual moves.

## compare_runs.m (magnet repeatability)

Also from the other session. It loads N runs on an identical grid, computes
σ of |B| across runs per point, optionally subtracts a magnet-absent noise
floor in quadrature, and divides by |∇|B|| to get σ_pos in mm. Points with a
weak gradient are flagged. **Fixed after the merge:**
- `gradient()` was called on V(x,y,z) as if dim 1 were x. MATLAB treats dim 2
  as x, so the call errored on grids with nx ≠ ny and mixed up spacings
  otherwise. It is now `[Gy,Gx,Gz] = gradient(V, ys, xs, zs)`.
- `prctile` (Statistics Toolbox in older releases) was replaced with a local
  `pct95`.
- The noise floor now uses the CSV's `n_samples` column when present, not
  `scan_config.m`'s `n_avg`.

Not run in MATLAB.

## Next steps

1. Bench dry run: `python field_scan.py --label dryrun --x -25 25 3 --y -25 25 3 --z -675 -625 3`
   with the robot outside the coil and the coils off. Check every point
   logs `err=0` and |B| ≈ 0.55–0.6 G.
2. Magnet repeatability: run the same small grid 5–10 times
   (`--label magnet_run1` …), plus `--label no_magnet` with the magnet
   removed, then `compare_runs("data/magnet_run*.csv", "data/no_magnet_….csv")`.
3. Baseline (coils off) then coils-on scans in the coil, then
   `plot_field_map(coils_on, baseline)`.
4. Apply the sensor→robot rotation in MATLAB once the 1044's mounting is
   known. `field_scan.py` logs raw sensor axes. `run_field_scan.m` applies
   `cfg.R_sensor_to_robot` itself.

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
