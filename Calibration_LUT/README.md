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

## Input format expected

Per servo, two CSVs: a forward sweep (`pulse_width_us, angle_deg`,
increasing pulse width) and a backward sweep (decreasing pulse width),
using the *same* set of commanded pulse widths in both directions so
`calibrate_servos_lut.m` can pair them up and compute backlash. Repeat
rows at the same pulse width are averaged automatically.

## Adopting this later

Once real hardware data has validated this against held-out points,
swapping it in is a one-line change per call site: replace
`calibrate_servos.m` + `angle_to_pulse.m` calls with
`calibrate_servos_lut.m` + `angle_to_pulse_lut.m` in whatever
script/model currently calls them (`init_scan.m` and the Simulink
`S01_Scan_pipe.slx` MATLAB Function block, respectively). Nothing in this
folder does that automatically - it's kept opt-in.
