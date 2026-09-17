# HANDOFF — Servo Calibration (Method & Routine)

Scope: this file covers **only** the servo PWM-to-angle calibration work — not the wider delta robot project (mechanical design, magnetometer interface, full scan runtime). Repo: `noahseanui42/Matlab`, branch `claude/zen-cray-hudl5s`.

---

## The problem this solves

Each of the 3 bicep servos will end up with its own PWM-to-angle relationship once measured on real hardware — different slope (µs/degree), different center pulse width, possibly different direction sign, and not perfectly linear across its range. `Kinematics/calibrate_servos.m` and `Kinematics/angle_to_pulse.m` already handle per-servo independence correctly (every constant is a 3-element vector, one per servo, fit/applied independently) — that part of the design was never in question. What's in progress is *measuring the real values* and deciding whether a simple straight-line fit is good enough or whether a lookup table is worth the extra work.

## Two tables — do not conflate them

- **`pulse_table.csv`** (produced by `Kinematics/Export_table.m`) — the big scan lookup table: 2541 rows, one per scan point, each with a desired joint angle + pulse width per servo. This is a pipeline *output*. Its structure doesn't change no matter which calibration approach is used.
- **Calibration table** — a small, separate, per-servo table (~10-15 rows) of measured `(pulse_width, angle)` pairs from the physical sweep rig. It's an *input* consumed inside the angle→pulse conversion step, applied once per scan point (2541 times) to produce the pulse-width column of `pulse_table.csv`. It is never merged or joined with `pulse_table.csv` — it's a function, not a second dataset sitting alongside it.

## Current physical state

- 3D-printed calibration-rig part: **done**
- Printed protractor dial (marked per angle): **done**
- Rig assembly (mounting part + protractor + fixed camera): **not yet done — next step**
- Real sweep data: **not yet collected**
- `k_us` / `sgn` / `pw_home` in `Kinematics/init_scan.m`: still placeholders (`[11.67 11.67 11.67]`, `[1 -1 -1]`) — these need real measured values

## Two calibration implementations in the repo

1. **`Kinematics/calibrate_servos.m`** (original/baseline) — fits a single line per servo, `pw = pw_home + sgn*k_us*theta`, via `polyfit`, independently per servo, warns if R² < 0.98. This is what `init_scan.m` → `angle_to_pulse.m` → `Export_table.m` currently uses in production. **Untouched.**
2. **`Calibration_LUT/`** (new, isolated, opt-in — does not touch or call anything in `Kinematics/`) — implements a direction-averaged lookup-table upgrade, based on research into how others do this (see `reports/Servo PWM calibration methods.md` and `research_notes/Servo PWM calibration methods/` for full sourcing):
   - `calibrate_servos_lut.m` — builds a per-servo `(angle, pulse_width)` table from forward + backward sweep CSVs, reports backlash (the forward/backward gap) per servo
   - `angle_to_pulse_lut.m` — same input/output shape as `angle_to_pulse.m`, but interpolates (`interp1`) against the table instead of a formula
   - `servo_calibration_sweep/servo_calibration_sweep.ino` — Arduino sketch that drives the sweep and logs the raw data
   - `README.md` — usage instructions and how to adopt this later if it proves worthwhile

Nothing in `Calibration_LUT/` is wired into the production pipeline. Adopting it means manually swapping two function calls once it's validated against held-out points — see `Calibration_LUT/README.md`.

## Running the calibration routine end to end

1. **Assemble the rig**: mount the 3D-printed part + protractor, camera fixed and perpendicular to the protractor face (not handheld — parallax was the #1 named failure mode in the research), one servo mounted at a time.
2. **Upload** `Calibration_LUT/servo_calibration_sweep/servo_calibration_sweep.ino` to the Arduino UNO R4 Minima. Open Serial Monitor at 9600 baud.
3. **Select the servo**: send `S1` (repeat for `S2`, `S3` as each is swapped into the rig).
4. **Sweep forward**: send `F`. The sketch holds at 11 evenly spaced pulse widths (830–2170 µs) and prompts 3× per point for the angle read off the current photo — type it, press enter, repeat.
5. **Sweep backward**: send `B` — same 11 pulse widths, descending. Don't skip this; it's the only way backlash gets caught instead of silently baked into the fit.
6. **Capture the log**: save the full Serial Monitor session to a text file, strip lines starting with `# `. What's left is bare `pulse_width_us,angle_deg` lines — save the `F` run as `cal_servo<N>_fwd.csv` and the `B` run as `cal_servo<N>_bwd.csv`.
7. Repeat steps 3–6 for all 3 servos (6 CSVs total).
8. **Fit**: run `calibrate_servos_lut.m` in MATLAB against the 3 fwd/bwd file pairs → per-servo `cal_angle`, `cal_pw`, and `backlash_deg`.
9. **Validate**: command a few angles *not* used in the fit, check the camera-measured result against a target tolerance (start at ±1–2°, tighten if the rig proves better).
10. **Decide**: adopt the lookup table (wire `Calibration_LUT/` functions into `init_scan.m`/the Simulink model) only if it measurably beats the existing linear fit on those held-out points — otherwise, the same raw sweep data can still be used to fit real `k_us`/`sgn`/`pw_home` into the existing `Kinematics/calibrate_servos.m` path.

## Key numbers from the research (condensed — full citations in `reports/Servo PWM calibration methods.md`)

- 10–15 sweep points per servo, evenly spaced, plus 2–3 held out for validation only
- Sweep forward **and** backward — required to catch backlash
- 3–5 repeats per point; record the spread, not just the mean
- Typical hobby-servo backlash: ~0.5° (metal gear) to ~1–2° (plastic gear)
- R² alone is not sufficient validation — a curved response can still post a high R²; check residuals and use held-out points
- Add a 470–1000 µF capacitor across the servo supply before sweeping — direct-PWM setup (no PCA9685) means repeated moves can brownout-reset the Arduino mid-sweep and silently corrupt data

## File reference

```
Kinematics/init_scan.m              - all params incl. k_us/sgn/pw_home (still placeholders)
Kinematics/calibrate_servos.m       - baseline linear-fit calibration (production path)
Kinematics/angle_to_pulse.m         - baseline linear angle->pulse conversion (production path)
Calibration_LUT/calibrate_servos_lut.m       - lookup-table calibration (isolated, opt-in)
Calibration_LUT/angle_to_pulse_lut.m         - lookup-table angle->pulse conversion (isolated, opt-in)
Calibration_LUT/servo_calibration_sweep/*.ino - Arduino sweep-and-log sketch
Calibration_LUT/README.md                    - usage + how to adopt
reports/Servo PWM calibration methods.md     - full research synthesis
research_notes/Servo PWM calibration methods/ - raw research notes (5 files)
```

Not covered here: mechanical design, magnetometer interface, full scan runtime, report writing. Ask if those are needed.
