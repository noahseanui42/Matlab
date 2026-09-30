# Servo calibration log

## 2026-09-29: what we found

- **Arm 2 (D10) re-swept unloaded** with the ruler method, 1200–1950 µs in
  10 µs steps (`servo2_angles.xlsx`). R = 18.5 cm, shaft axis 10 cm below the
  top wood, corner reads 9 cm when flat.
  - New values in `config.h`: `centre_us` 1385 → **1350**, `us_per_deg`
    10.0481 → **9.51**. Close to a straight line (worst point 1.4° off).
  - Sweep reached ~63° at 1950 µs; 63–70° is extrapolated.
  - Possible hitch or misread between 1690 and 1700 µs (the reading jumped twice as far as usual).
- **Arm 2 has ~5 mm of play at the corner when flat (~1.5°).** Assembled, it
  sits slightly *above* flat at home, so the unloaded centre doesn't fully
  hold under load. 1° on one arm ≈ 9 mm of effector drift sideways at z ≈ −700.
- **Drift at the centre while moving down:** slight +x around z = −640, then
  −x / +y (worse) at z = −740. The direction points at arm 3 (D11) sitting
  high as it goes deeper, i.e. its `us_per_deg` is likely too small. Could
  also be arm 2's play. Arms 1 and 3 are still on the first-run values.
- **Pen higher on the −x side:** the paper plate was off level. Now levelled.

## 2026-09-30: plan

Total ≈ 2–2.5 h. Do the steps in order; each one decides the next.

### 1. Re-check after levelling (10 min)
With `delta_servo` flashed, speed 10%:
- Pen-to-paper gap at (±100, 0, −700) and (0, ±100, −700).
- Effector offset from centre (x, y in mm) at (0, 0, −640) and (0, 0, −740).

### 2. Play test on arms 1 and 3 (5 min)
At home, push each elbow gently up, release, measure the corner height; push down,
release, measure again. Record the difference for arms 1 and 3 (arm 2 = 5 mm).
- Arms 1 and 3 under ~2 mm → servo 2 is worn → **plan to replace it**.
- All about 5 mm → normal for these servos → work around it (step 5).

### 3. Ruler sweep on arm 3, D11 (30 min)
Flash `servo_sweep`, rods off arm 3, then in the Serial Monitor type `11 1200`.
- Down sweep: 1200 → 1950 every 50 µs, corner height from the top wood.
- Back up: 1950 → 1200 every 50 µs (backlash).
- Also note: R (shaft centre → corner), shaft depth below the top wood, corner reading when flat.

### 4. Ruler sweep on arm 1, D9 (30 min)
Same as step 3 with `9 1200`.

→ **Send Claude the step 1–4 numbers.** Claude updates `config.h` for arms 1 and 3.

### 5. Assembled flat check (20 min)
Rods on, all three powered through `servo_sweep`: `9 <centre>`, `10 <centre>`,
`11 <centre>` using the new values. Phone level on each bicep; nudge a pin's µs
until it reads flat, **always approaching from the same direction** (from
slightly below). Send the three final µs values → new `centre_us`.

### 6. Verify (20 min)
Reflash `delta_servo`, then repeat step 1 plus z = −550 and −790 at the centre.
Suggested targets:
- Drift from centre ≤ ±5 mm across z = −550 to −790.
- Pen gap within ±2 mm at all four points.

If it's still off after this, the next suspects are forearm rod lengths
(measure all six between ball centres, should match within ~1 mm) and
servo 2's play.

## 2026-09-30: assembled protractor sweep and refit

- **Method:** all three arms connected, flashed with the confirmed geometry
  (SB 175 / SP 75 / L_UP 177 / L_LO 625) and D9 1460 / 11.8231, D10 1350 / 9.51,
  D11 1410 / 10.5544. On-axis, probe z −740 (lead-in) then −730 → −590 in
  10 mm steps, moving up, no Home in between. Digital protractor on each bicep
  (positive = down). Raw data: `sweep_2026-09-30_up.csv`. The GUI's commanded
  angles matched the model exactly, so the geometry/build is as intended.
- **Before the refit the errors were large:** at −730, D9 +7.2°, D10 −2.4°,
  D11 +4.9° (measured − commanded), so the arms disagreed by up to ~10° and the
  platform was tilted, not just at the wrong height (≈3 mm of probe z per
  degree near the bottom).
- **Straight-line fit, measured = s × commanded + o** (−740 lead-in excluded,
  it was approached from the other direction):

  | Pin | s | o (°) | fit residual (max) | new centre_us | new us_per_deg |
  |---|---|---|---|---|---|
  | D9  | 1.203 | +0.37 | 1.0° | 1460 → **1456** | 11.8231 → **9.8315** |
  | D10 | 0.974 | −2.38 | 1.3° | 1350 → **1373** | 9.51 → **9.7664** |
  | D11 | 1.092 | +1.73 | 0.5° | 1410 → **1393** | 10.5544 → **9.6658** |

  All three now come out at ~9.7–9.8 µs/deg, consistent with each other.
  D9's first-run 11.8231 was the biggest single error. D11 was moving *too far*
  (us_per_deg too large), the opposite of what the 09-29 drift note guessed.
- **Watch:** at −590 (≈ −9°) D9 and D10 read ~2° further up than the trend;
  D10's scatter is the worst of the three, consistent with its ~5 mm play.
- **Not measured yet:** the Down pass (hysteresis/deadband) and repeatability.

### Next
Reflash `delta_servo` with the new `config.h`, then repeat the full sweep
(Up and Down) with `joint_angle_calibration_test.xlsx`. Target: every arm
within ±0.5° of commanded and the three arms agreeing with each other.
