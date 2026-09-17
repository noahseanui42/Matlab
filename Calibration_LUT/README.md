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
  these two functions consume. Also takes `P<us>` to log a hold-out point at
  an arbitrary pulse width, for the comparison below.
- `parse_sweep_log.m` - turns one saved Serial Monitor capture into every
  CSV both calibration paths need (`cal_servoN_fwd.csv`, `cal_servoN_bwd.csv`,
  the combined `cal_servoN.csv` for `Kinematics/calibrate_servos.m`, and
  `cal_servoN_holdout.csv`). No hand-stripping of `# ` lines.
- `compare_calibrations.m` - residuals of the linear fit vs the lookup table
  on the hold-out points, per servo, in degrees and in microseconds through
  the real `angle_to_pulse.m` / `angle_to_pulse_lut.m` code paths. This is
  the number that decides adoption.

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
4. `P<us>` for 2-3 hold-out points at pulse widths *not* on the sweep grid
   (e.g. `P1050`, `P1450`, `P1850`). Same prompting as a sweep point.
5. `Q` to detach, swap in the next servo, repeat from step 1.

Every line the sketch prints starts with `# ` except the actual data
lines, which are bare `pulse_width_us,angle_deg`. Save the whole Serial
Monitor session (all three servos can be in one file) and run

```matlab
out = parse_sweep_log('sweep_log.txt', 'cal_data');
```

It splits on the sketch's `# BEGIN ... / # END ...` markers and writes
`cal_servo<N>_fwd.csv`, `cal_servo<N>_bwd.csv`, `cal_servo<N>.csv` (fwd+bwd
combined, the Phase 1 input) and `cal_servo<N>_holdout.csv` into `cal_data/`.

## Input format expected

Per servo, two CSVs: a forward sweep (`pulse_width_us, angle_deg`,
increasing pulse width) and a backward sweep (decreasing pulse width),
using the *same* set of commanded pulse widths in both directions so
`calibrate_servos_lut.m` can pair them up and compute backlash. Repeat
rows at the same pulse width are averaged automatically. This is exactly
what the `.ino` sketch above produces.

## Comparing against the linear fit

With Phase 1's `k_us`/`sgn`/`pw_home` and this folder's `cal_angle`/`cal_pw`
in the workspace:

```matlab
res = compare_calibrations({out.holdout}, k_us, sgn, pw_home, ...
                           cal_angle, cal_pw, pw_min, pw_max);
```

prints RMS and max residual per servo for both methods. Weigh the gap
against the repeat spread and the backlash `calibrate_servos_lut.m` reports;
a difference smaller than those is noise, not a result.

## Adopting this later

Once `compare_calibrations.m` shows a real gain on held-out points,
swapping it in is a one-line change per call site: replace
`calibrate_servos.m` + `angle_to_pulse.m` calls with
`calibrate_servos_lut.m` + `angle_to_pulse_lut.m` in whatever
script/model currently calls them (`init_scan.m` and the Simulink
`S01_Scan_pipe.slx` MATLAB Function block, respectively). Nothing in this
folder does that automatically - it's kept opt-in.
