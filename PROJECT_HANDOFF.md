# PROJECT HANDOFF — Magnetic Field Mapping Robot (ENGR489)

Full-project status. For the deep-dive on the Simulink debugging saga specifically, see `HANDOFF_S01_Scan_pipe.md` — this file is the wider picture: goals, current state, and what's left.

---

## Goals

Build a delta-robot platform that moves a magnetometer probe through a 3D volume inside a Helmholtz coil to map the magnetic field. Supervisor: Christopher Hollit.

**Hard constraint, drives most mechanical decisions:** the robot operates *inside* an active Helmholtz coil, so no ferromagnetic or magnetically-responsive material near the probe. Allowed: PLA, carbon fibre, aluminium, plastic/nylon fasteners, Igus EBRM-06 rod ends. Explicitly avoided: steel fasteners, standard bearings, any Fe/Ni/Co content — including the Arduino choice (R4 **Minima**, not WiFi, to avoid RF interference near the magnetometer).

The project splits into two halves:
- **Offline / host half** (this repo's `Kinematics/` + `Simulink/`): batch-computes every scan point's servo commands ahead of time — scan grid → inverse kinematics → validation → pulse widths → `pulse_table.csv`. **Done and validated.**
- **Runtime / hardware half** (`Runtime/`): streams that table to the physical robot during an actual scan, coordinating servo motion with magnetometer readings. **Scaffolded this session, not yet run against real hardware.**

---

## Where things actually are, right now

### Kinematics pipeline — done, validated, not blocking anything
`init_scan.m` → `S01_Scan_pipe.slx` produces `pulse_table.mat`/`.csv`, confirmed correct two independent ways (plain MATLAB and the actual Simulink model, matching to the same decimal places):

| Check | Result |
|---|---|
| Valid count | 2541 / 2541 |
| Forearm residual | 3.89e-11 |
| Peak articulation | 11.42° (against 13° limit) |
| FK round-trip `FK(IK(p))-p` | 1.08e-10 |
| Pulse headroom | 1127–1826 µs against 830/2170, zero saturation |

`L2 = 600mm` confirmed by physical measurement (was previously an open 600-vs-700 question).

### Locked geometry (`init_scan.m`) — confirmed correct against the built robot
```
sb = 175, sp = 150, L1 = 180, L2 = 600
```
Note: a project-context reference elsewhere lists `sb=450, sp=18` — that's **stale**, from an earlier design pass before the geometry was adapted/measured. `175/150` is what the robot actually is; worth correcting wherever that `450/18` figure is still being cited (report drafts, that context skill's own notes) so it doesn't resurface and cause confusion later.

### Servo wiring — now settled
Direct PWM out of **D9/D10/D11** on the Arduino UNO R4 Minima, **no PCA9685 shield**. The shield appears in an older system block diagram and in some earlier project notes, but the actual build never used it. `Runtime/scan_controller/scan_controller.ino` is written against direct pins — this is the final decision, not an open item anymore.

### Runtime scaffolding — written, untested (waiting on hardware)
- `Runtime/scan_controller/scan_controller.ino` — Arduino sketch. Serial protocol: host sends `M,<idx>,<pw1>,<pw2>,<pw3>`, Arduino moves the 3 bicep servos, settles, replies `OK,<idx>`. `H` homes to `pw_home`.
- `Runtime/run_scan.m` — host orchestrator. Reads `pulse_table.csv`, streams valid rows to the Arduino one at a time, waits for each ack, then calls `read_magnetometer()` and appends results to `scan_results.csv` incrementally (so an interrupted scan resumes rather than restarting).
- `Runtime/read_magnetometer.m` — **stub, deliberately errors if called.** The magnetometer is read separately from the Arduino (not I2C off it), so this is the one function that needs writing once the actual sensor interface is decided.

### Calibration — blocked on hardware, routine designed
`k_us` and `sgn` in `init_scan.m` are still placeholders (`[11.67 11.67 11.67]`, `[1 -1 -1]`). Waiting on a 3D-printed part before the calibration rig can be assembled. Full rig diagram + sweep procedure: **https://claude.ai/artifact/5EjhGFcuKtEpGhYCffzPR9**. `Kinematics/calibrate_servos.m` is written and ready to fit `k_us`/`sgn`/`pw_home` from measured `(pulse_width, angle)` data the moment that part arrives.

---

## To-do, roughly in the order it'll actually come up

1. **3D-printed part arrives** → assemble the calibration rig (protractor + pointer + fixed camera) per the artifact linked above.
2. **Run the sweep** on all 3 bicep servos independently, record `cal_servo1/2/3.csv`.
3. **Run `calibrate_servos.m`** → paste the real `k_us`/`sgn`/`pw_home` into `init_scan.m`. Recompute `th_lim` from the real `k_us`.
4. **Re-run the whole pipeline** (`init_scan` → Simulink) and re-check pulse headroom — real per-servo `k_us` values will likely differ from each other, unlike the identical placeholders.
5. **Decide the magnetometer interface** (what it actually connects to — USB DAQ? second serial device? direct I2C from the host?) and implement `read_magnetometer.m` against it.
6. **Bench-test `run_scan.m` + the Arduino sketch** together before a real scan — nothing in the runtime code has touched real hardware yet. Known gaps to harden once hardware exists: no timeout if the Arduino never acks (would hang indefinitely), no retry on a dropped serial line, `SETTLE_MS` (currently 400ms guess) needs tuning against real servo speed.
7. **Full scan run** once everything above checks out — ~2541 points; at whatever settle time + magnetometer read time you land on, worth estimating total scan duration before committing (your own earlier note: ~85 min at 2s/point for the full grid, ~46 min if trimmed to 6 z-planes — decide whether that trim is still worth it once real timing is known).
8. **Fix the stale `sb=450/sp=18` reference** wherever it's written down outside this repo (report drafts, the project-context skill notes) so it doesn't get pulled in as "the" parameters later.
9. Cross-check item from the original kinematics validation checklist still unresolved: **`finverse_K_args.m`** doesn't exist anywhere in this repo — confirm whether that check is still wanted, and if so, whether the file exists elsewhere or needs writing.

---

## Repo map

```
Kinematics/
  init_scan.m          — all parameters, run before anything else
  scan_grid.m           IK, validate_batch.m, angle_to_pulse.m, rod_axes_at.m  — pipeline stages
  fk_batch.m            — forward kinematics (trilateration), used for the round-trip check
  calibrate_servos.m    — fits k_us/sgn/pw_home from measured calibration data
  Export_table.m        — writes pulse_table.mat/.csv (wired to the model's StopFcn)
Simulink/
  S01_Scan_pipe.slx     — the batch pipeline model, working end to end
Runtime/
  scan_controller/scan_controller.ino  — Arduino: servo motion over serial
  run_scan.m             — host: streams pulse_table.csv, logs magnetometer readings
  read_magnetometer.m    — stub, needs real sensor interface wired in
HANDOFF_S01_Scan_pipe.md — deep-dive on the Simulink model specifically
PROJECT_HANDOFF.md       — this file
```
