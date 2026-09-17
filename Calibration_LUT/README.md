# Calibration_LUT

Experimental, standalone alternative to the linear per-servo calibration in
`Kinematics/calibrate_servos.m` + `Kinematics/angle_to_pulse.m`. Nothing
here is imported by, or modifies, the existing pipeline - `pulse_table.csv`
still comes out of the original `init_scan.m` -> Simulink ->
`Export_table.m` path exactly as before.

## What's here

- `calibrate_servos_lut.m` - builds a per-servo (angle, pulse-width) lookup
  table from bidirectional (forward + backward) calibration sweeps, instead
  of fitting a single line. Reports backlash (forward/backward gap) per
  servo.
- `angle_to_pulse_lut.m` - same input/output shape as `angle_to_pulse.m`,
  but interpolates against the lookup table (`interp1`) instead of using
  `k_us`/`sgn`/`pw_home`.
- `servo_calibration_sweep/servo_calibration_sweep.ino` - Arduino sketch
  that drives the rig sweep and logs the raw (pulse_width, angle) pairs
  these two functions consume.

## Running the sweep

Upload `servo_calibration_sweep.ino`, open the Serial Monitor at 9600
baud, mount one servo in the rig, then:

1. `S1` (or `S2`/`S3`) to select and attach that servo.
2. `F` to run the forward sweep (`PW_MIN` -> `PW_MAX`, 11 evenly spaced
   points by default). At each point the servo holds and the sketch
   prompts 3 times for the angle read off the current photo - type the
   number, press enter, repeat.
3. `B` to run the same points backward (`PW_MAX` -> `PW_MIN`) - this is
   what lets `calibrate_servos_lut.m` compute backlash, so don't skip it.
4. `Q` to detach, swap in the next servo, repeat from step 1.

Every line the sketch prints starts with `# ` except the actual data
lines, which are bare `pulse_width_us,angle_deg`. Capture the Serial
Monitor output to a file and keep only the un-prefixed lines - those go
straight into `cal_servo<N>_fwd.csv` (from the `F` run) and
`cal_servo<N>_bwd.csv` (from the `B` run).

## Input format expected

Per servo, two CSVs: a forward sweep (`pulse_width_us, angle_deg`,
increasing pulse width) and a backward sweep (decreasing pulse width),
using the *same* set of commanded pulse widths in both directions so
`calibrate_servos_lut.m` can pair them up and compute backlash. Repeat
rows at the same pulse width are averaged automatically. This is exactly
what the `.ino` sketch above produces.

## Adopting this later

Once real hardware data has validated this against held-out points,
swapping it in is a one-line change per call site: replace
`calibrate_servos.m` + `angle_to_pulse.m` calls with
`calibrate_servos_lut.m` + `angle_to_pulse_lut.m` in whatever
script/model currently calls them (`init_scan.m` and the Simulink
`S01_Scan_pipe.slx` MATLAB Function block, respectively). Nothing in this
folder does that automatically - it's kept opt-in.
