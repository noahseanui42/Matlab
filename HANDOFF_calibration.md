# HANDOFF — Servo Calibration (Method & Routine)

Scope: this file covers **only** the servo PWM-to-angle calibration work — not the wider delta robot project (mechanical design, magnetometer interface, full scan runtime). Repo: `noahseanui42/Matlab`, branch `claude/zen-cray-hudl5s`.

---

## The problem this solves

Each of the 3 bicep servos will end up with its own PWM-to-angle relationship once measured on real hardware — different slope (µs/degree), different center pulse width, possibly different direction sign, and not perfectly linear across its range. `Kinematics/calibrate_servos.m` and `Kinematics/angle_to_pulse.m` already handle per-servo independence correctly (every constant is a 3-element vector, one per servo, fit/applied independently) — that part of the design was never in question. What's in progress is *measuring the real values* and deciding whether a simple straight-line fit is good enough or whether a lookup table is worth the extra work.

## Two tables — do not conflate them

- **`pulse_table.csv`** (produced by `Kinematics/Export_table.m`) — the big scan lookup table: 2541 rows, one per scan point, each with a desired joint angle + pulse width per servo. This is a pipeline *output*. Its structure doesn't change no matter which calibration approach is used.
- **Calibration table** — a small, separate, per-servo table (~10-15 rows) of measured `(pulse_width, angle)` pairs from the physical sweep rig. It's an *input* consumed inside the angle→pulse conversion step, applied once per scan point (2541 times) to produce the pulse-width column of `pulse_table.csv`. It is never merged or joined with `pulse_table.csv` — it's a function, not a second dataset sitting alongside it.

## Plan: two-phase rollout

**Phase 1 — get a real (not placeholder) `pulse_table.csv` on the existing linear path.** Fit `k_us`/`sgn`/`pw_home` from real sweep data using `Kinematics/calibrate_servos.m` (already written, already the production path), paste into `init_scan.m`, re-run the pipeline. This gets the whole system running on real hardware numbers as fast as possible, using code that's already validated end-to-end (2541/2541 valid, zero saturation, per the project handoff) — the only thing missing was real measured constants.

**Phase 2 — second pass: interpolate against the calibration table instead.** Reuse the *exact same* sweep data to build a per-servo lookup table with `Calibration_LUT/calibrate_servos_lut.m`, validate it against held-out points, and compare its residual error to Phase 1's linear fit on those same points. Only swap it into the production pipeline (`Calibration_LUT/README.md` → "Adopting this later") if it measurably beats Phase 1.

Why this order and not lookup-table-first:
- The sweep only needs to be **run once**. Both phases consume the same raw `(pulse_width, angle)` measurements — Phase 2 doesn't need new hardware data, just a different fit on data already collected in Phase 1.
- Phase 1's code path is already the one that's been validated against the full pipeline (IK → validation → pulse headroom check). Getting real numbers into that path first de-risks the rest of the project fastest.
- This staged approach produces a genuine measured comparison for the report — "linear calibration gave X° residual on held-out points, the lookup table gave Y°" — instead of an assumed improvement. The research didn't find a source that quantified this specific improvement for a protractor+camera rig, so this project's own numbers become the evidence.

## Current physical state

- 3D-printed calibration-rig part: **done**
- Printed protractor dial (marked per angle): **done**
- Rig assembly (mounting part + protractor + fixed camera): **not yet done — next step**
- Real sweep data: **not yet collected**
- `k_us` / `sgn` / `pw_home` in `Kinematics/init_scan.m`: still placeholders (`[11.67 11.67 11.67]`, `[1 -1 -1]`)

## The two calibration implementations, mapped to the phases above

1. **`Kinematics/calibrate_servos.m`** (Phase 1) — fits a single line per servo, `pw = pw_home + sgn*k_us*theta`, via `polyfit`, independently per servo, warns if R² < 0.98. Reads one CSV per servo, columns `[pulse_width_us, angle_deg]`, via `readmatrix` — no forward/backward distinction. This is what `init_scan.m` → `angle_to_pulse.m` → `Export_table.m` currently uses in production. **Untouched by the Phase 2 work below.**
2. **`Calibration_LUT/`** (Phase 2, isolated, opt-in — does not touch or call anything in `Kinematics/`):
   - `calibrate_servos_lut.m` — builds a per-servo `(angle, pulse_width)` table from **forward + backward** sweep CSVs, reports backlash (the forward/backward gap) per servo
   - `angle_to_pulse_lut.m` — same input/output shape as `angle_to_pulse.m`, but interpolates (`interp1`) against the table instead of a formula
   - `servo_calibration_sweep/servo_calibration_sweep.ino` — Arduino sketch that drives the sweep and logs the raw data
   - `README.md` — usage instructions and how to adopt this later

## Running the calibration routine end to end

**Sweep once, shared by both phases:**

1. **Assemble the rig**: mount the 3D-printed part + protractor, camera fixed and perpendicular to the protractor face (not handheld — parallax was the #1 named failure mode in the research), one servo mounted at a time.
2. **Upload** `Calibration_LUT/servo_calibration_sweep/servo_calibration_sweep.ino` to the Arduino UNO R4 Minima. Open Serial Monitor at 9600 baud.
3. **Select the servo**: send `S1` (repeat for `S2`, `S3` as each is swapped into the rig).
4. **Sweep forward**: send `F`. The sketch holds at 11 evenly spaced pulse widths (830–2170 µs) and prompts 3× per point for the angle read off the current photo — type it, press enter, repeat.
5. **Sweep backward**: send `B` — same 11 pulse widths, descending. Don't skip this; it's the only way backlash gets caught.
6. **Hold-out points**: send `P<us>` for 2–3 pulse widths *not* on the sweep grid (e.g. `P1050`, `P1450`, `P1850`) and read the angle the same way. These are the only data step 13 can use.
7. **Capture the log**: save the full Serial Monitor session to a text file (all servos can share one file) and run `out = parse_sweep_log('log.txt', 'cal_data')` from `Calibration_LUT/`. It splits on the sketch's `# BEGIN`/`# END` markers and writes `cal_servo<N>_fwd.csv`, `cal_servo<N>_bwd.csv`, the combined `cal_servo<N>.csv` (Phase 1 input) and `cal_servo<N>_holdout.csv`. No hand-stripping.
8. Repeat steps 3–6 for all 3 servos, then run step 7 once.

**Phase 1 — linear fit, get the pipeline running on real numbers:**

9. `Kinematics/calibrate_servos.m` expects one CSV per servo, no fwd/bwd split — that is the combined `cal_servo<N>.csv` from step 7 (a direction-blind linear fit is fine here — Phase 1 isn't trying to model backlash). Run `calibrate_servos({out.combined})` → get `k_us`/`sgn`/`pw_home` per servo.
10. Paste those into `Kinematics/init_scan.m`, replacing the placeholders. Re-run the pipeline (`init_scan.m` → Simulink `S01_Scan_pipe.slx`) and re-check pulse headroom / `th_lim` — real per-servo `k_us` values will likely differ from each other, unlike the placeholders.
11. `pulse_table.csv` now reflects real hardware. Bench-test before moving on.

**Phase 2 — lookup table, second pass on the same data:**

12. Run `calibrate_servos_lut({out.fwd}, {out.bwd})` against the 6 fwd/bwd CSVs (this function wants the direction split) → per-servo lookup tables + backlash figure.
13. Validate: `compare_calibrations({out.holdout}, k_us, sgn, pw_home, cal_angle, cal_pw, pw_min, pw_max)` prints per-servo RMS/max residual for both methods on the step-6 hold-out points, in degrees and in µs through the real `angle_to_pulse` / `angle_to_pulse_lut` code paths.
14. **Decide**: if Phase 2's residuals are meaningfully smaller (bigger than the repeat spread and backlash step 12 reported), wire `Calibration_LUT/`'s functions into `init_scan.m`/Simulink per its README and regenerate `pulse_table.csv` again. If not, Phase 1's result stands as final — either way, the comparison itself is worth a line in the report.

## Key numbers from the research (condensed — full citations in `reports/Servo PWM calibration methods.md`)

- 10–15 sweep points per servo, evenly spaced, plus 2–3 held out for validation only
- Sweep forward **and** backward — required to catch backlash
- 3–5 repeats per point; record the spread, not just the mean
- Typical hobby-servo backlash: ~0.5° (metal gear) to ~1–2° (plastic gear)
- R² alone is not sufficient validation — a curved response can still post a high R²; check residuals and use held-out points
- Add a 470–1000 µF capacitor across the servo supply before sweeping — direct-PWM setup (no PCA9685) means repeated moves can brownout-reset the Arduino mid-sweep and silently corrupt data
- The one measured comparison found in the research (a different project, encoder-based, not this rig): linear formula 3.13° error → 20-point lookup table 0.19° error. This project's own Phase 1 vs Phase 2 validation numbers (step 12 above) are what actually matter here — treat the research figure as a reason to expect a gain, not as this project's number.

## File reference

```
Kinematics/init_scan.m              - all params incl. k_us/sgn/pw_home (still placeholders)
Kinematics/calibrate_servos.m       - Phase 1: baseline linear-fit calibration (production path)
Kinematics/angle_to_pulse.m         - Phase 1: baseline linear angle->pulse conversion (production path)
Calibration_LUT/calibrate_servos_lut.m       - Phase 2: lookup-table calibration (isolated, opt-in)
Calibration_LUT/angle_to_pulse_lut.m         - Phase 2: lookup-table angle->pulse conversion (isolated, opt-in)
Calibration_LUT/parse_sweep_log.m            - serial log -> all CSVs (fwd, bwd, combined, holdout)
Calibration_LUT/compare_calibrations.m       - step 13: linear vs table residuals on hold-out points
Calibration_LUT/servo_calibration_sweep/*.ino - Arduino sweep-and-log sketch (shared by both phases); P<us> logs hold-out points
Calibration_LUT/README.md                    - usage + how to adopt Phase 2
reports/Servo PWM calibration methods.md     - full research synthesis
research_notes/Servo PWM calibration methods/ - raw research notes (5 files)
```

Not covered here: mechanical design, magnetometer interface, full scan runtime, report writing. Ask if those are needed.
