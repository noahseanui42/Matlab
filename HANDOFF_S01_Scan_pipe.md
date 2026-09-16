# HANDOFF — S01_Scan_pipe Simulink model (updated)

Supersedes the original handoff doc. That doc's "CURRENT BLOCKER" is now resolved — **the model runs end to end and produces validated output.** This file is the current state and what's still open.

---

## What this is

ENGR489 capstone: a non-magnetic delta robot that moves a magnetometer probe through a 3D volume inside a Helmholtz coil to map the magnetic field. Supervisor: Christopher Hollit.

Offline batch pipeline: `scan grid → inverse kinematics → validation → angle-to-pulse → assemble → pulse_table.mat/.csv`. Runtime half (Arduino, magnetometer logging) is separate.

---

## Files

All in repo root `Kinematics/` unless noted:

| File | Role | Status |
|---|---|---|
| `init_scan.m` | All parameters. Run before the model. | `L2 = 600` confirmed by physical measurement. `k_us`/`sgn` still placeholders. |
| `scan_grid.m` | Stage 1 — serpentine XYZ grid | Validated |
| `ik_batch.m` | Stage 2 — delta IK, returns angles + discriminant | Validated |
| `validate_batch.m` | Stage 3 — four checks, bit-coded failure reasons | Validated |
| `angle_to_pulse.m` | Stage 4 — per-servo calibration, clamp, quantise | Working; output not trustworthy until `k_us`/`sgn` are real |
| `rod_axes_at.m` | Helper — nominal ball-stud axes at a reference point | Unchanged |
| `fk_batch.m` | **New.** Forward kinematics via trilateration — inverse of the IK geometry, used for the FK round-trip check | Validated, max error 1.08e-10 |
| `calibrate_servos.m` | **New.** Fits `k_us`/`sgn`/`pw_home` per servo from measured `(pulse_width, angle)` pairs | Written, not yet run against real data |
| `Export_table.m` | Stage 5 — writes .mat/.csv from StopFcn | Updated: reads `pulse_table.signals.values` (see gotcha below) |
| `Simulink/S01_Scan_pipe.slx` | The model | **Runs and produces correct output** |

---

## Locked parameters (`init_scan.m`)

```
sb = 175, sp = 150, L1 = 180, L2 = 600   <-- L2 CONFIRMED by measurement, no longer open
xr=[-100 100], yr=[-50 50], zr=[-650 -550], nx=21, ny=11, nz=11, N=2541

pw_min=[830 830 830], pw_max=[2170 2170 2170], pw_home=[1500 1500 1500]
k_us=[11.67 11.67 11.67]   <-- STILL PLACEHOLDER, calibration routine below
sgn=[1 -1 -1]              <-- STILL PLACEHOLDER
th_lim=[-45 90]            <-- recompute once k_us/sgn are real

artic_max=13, rod_tol=1e-6
```

---

## The Simulink fix — what actually worked

The original blocker was "MATLAB Function block outputs need explicit fixed sizes." The real fix took a lot longer than that framing suggested. For the next person (or future me) touching this model, here's what's actually true, learned the hard way:

1. **Every base-workspace variable a block's script uses must be an explicitly declared symbol in that block's own Symbols pane** — nothing auto-resolves just because it's referenced in the script body or exists in the workspace. Each of the 5 MATLAB Function blocks has its own independent symbol table.
2. **Use literal numbers for Size fields, not a symbolic parameter.** We spent a huge amount of time trying to get a `N` *Parameter*-scoped symbol to work in Size fields (`[N 3]` etc.) so the grid size would propagate from one place. It never reliably worked in this MATLAB version — adding a data symbol kept getting auto-synced into the function's argument list regardless of Scope setting, and removing it from the signature by hand deleted the symbol outright. **Gave up on this entirely.** Every output's Size field is now a hardcoded literal, e.g. `[2541 3]`, `[2541 1]`, `[2541 11]`. Bracket syntax matters: `[2541 3]`, not `2541,3` or `2541 3` bare.
3. **Trade-off accepted:** if the scan grid changes (`nx`/`ny`/`nz`/`N` in `init_scan.m`), all ten Size fields across the 5 blocks need manually updating to match the new total. There's no symbolic link anymore. Written down here specifically so it isn't forgotten.
4. **All non-N parameters became Input-scoped symbols wired to Constant blocks**, exactly like `xr`/`yr`/`zr` already were — not Parameters. E.g. IK block: `sb`, `sp`, `L1`, `L2` are Inputs fed by 4 Constant blocks, each Constant's Value field set to the variable name so it reads live from the base workspace. Same pattern for `nx`/`ny`/`nz` (scan grid), `sb`/`sp`/`L1`/`L2`/`nhat`/`th_lim`/`artic_max`/`rod_tol` (validate), `k_us`/`pw_home`/`sgn`/`pw_min`/`pw_max` (angle-to-pulse).
5. **`th_deg`, `disc` (IK) and `code` (validate) ended up Variable-Size** (checkbox ticked, upper bound = the same literal size) rather than plain fixed — an earlier workaround during debugging that was never gone back and cleaned up. This is why:
   - The `sat`/`code`/`pulse_table` **To Workspace** blocks had to be switched to `Save format: Structure` + `Save 2-D signals as: 3-D array (concatenate along third dimension)`, since Array format can't log a variable-size signal. `artic_log` and `rodres` are still plain Array format (their sources are fixed-size).
   - `Export_table.m` reads `pulse_table.signals.values`, not `pulse_table` directly, because of this.
   - **Not yet cleaned up:** going back and unchecking Variable Size on `th_deg`/`disc`/`code` (now that literal sizes are used everywhere, not the broken `N` symbol) would let all To Workspace blocks go back to plain Array format and simplify `Export_table.m` back to its original form. Untried since the model already works — low priority.
6. **One real bug, not a Simulink-plumbing issue:** `sb` and `sp` Constant blocks were wired to swapped ports on the Validate block. This produced *wrong but plausible-looking* geometry (rod residual off by ~11 orders of magnitude, articulation exceeding the 13° limit, every point failing validation). If a rerun ever produces numbers that don't match the reference values below, check Constant-block wiring on every block before assuming the model logic is wrong.
7. `set_param('S01_Scan_pipe', 'StopFcn', 'Export_table')` wires up automatic export on every run (Stop Time = 0, so it fires almost instantly). Equivalent to typing `Export_table` manually after a run.

---

## Validation — all checks passed

Run against the placeholder `k_us`/`sgn` (geometry-only checks, not pulse-width checks):

| Check | Result |
|---|---|
| `xyz` size | `[2541 3]` ✓ |
| Valid count | `2541 / 2541` ✓ |
| Forearm residual (`rodres`) | `3.89e-11` (expected ~1e-13 order) ✓ |
| Peak articulation | `11.42°` (matches handoff's predicted ~11.4°) ✓ |
| Pulse headroom | `1127–1826 µs` against 830/2170, zero saturation ✓ |
| **FK round-trip** `FK(IK(p)) - p` | **`1.08e-10`**, well under the `1e-9` target ✓ |

These numbers were confirmed twice, independently: once via plain MATLAB (calling `scan_grid`/`ik_batch`/`validate_batch`/`angle_to_pulse` directly at the command line) and once via the actual Simulink model — they match to the same decimal places, confirming the model computes correctly, not just that it compiles.

Not yet done: cross-check against `finverse_K_args.m` — that file doesn't exist anywhere in this repo. Either it's elsewhere on the original machine, or it was a planned-but-never-written check; worth confirming which before treating it as still open.

---

## Servo calibration — the current open item

`k_us` (µs/degree, per servo) and `sgn` (direction, per servo) are placeholders. Everything downstream of the geometry is validated and correct; only the pulse-width numbers in `pulse_table` depend on these.

**Calibration routine designed this session** (rig doesn't yet have real measured data):
1. Build a rig: pointer rigidly extends the servo horn out to a protractor fixed to the frame, camera mounted stationary and perpendicular to avoid parallax. Full diagram and step-by-step sweep procedure published as an artifact: **https://claude.ai/artifact/5EjhGFcuKtEpGhYCffzPR9**
2. Sweep each servo's pulse width across its working range (~15–20 steps), photograph the pointer position at each step, nudge slightly past the assumed `pw_min`/`pw_max` to find the real limits.
3. Record `(pulse_width_us, angle_deg)` pairs into `cal_servo1.csv`, `cal_servo2.csv`, `cal_servo3.csv` (two plain columns, no header) in the `Kinematics` folder.
4. Run `calibrate_servos.m` — fits `pw = pw_home + sgn*k_us*theta` per servo, prints residuals (flags anything over 15 µs as suspicious — likely a reading taken too near a mechanical limit), and prints the three lines to paste into `init_scan.m`.
5. Recompute `th_lim` from the new `k_us` and `pw_min`/`pw_max`.
6. Re-run `init_scan` → Simulink → re-check pulse headroom, since real (likely non-identical across the 3 servos) `k_us` values may shift where the pw columns land relative to `pw_min`/`pw_max`.

---

## Run sequence

```matlab
cd C:\Users\nuinoah\AA_Matlab\Kinematics
init_scan
```
Open `S01_Scan_pipe.slx`, press Run. `pulse_table.mat`/`pulse_table.csv` land in MATLAB's current folder via the `StopFcn` callback (or type `Export_table` manually).

---

## Known open items (unchanged from original, still open)

- `k_us`/`sgn` calibration — see above, now has a concrete plan
- Quantisation: at real `k_us` (once measured), re-check 1 µs ≈ how many degrees against the 0.5–1° servo deadband
- Diagram inconsistency: block diagram shows a PCA9685 shield, build drives D9/D10/D11 directly off an Arduino UNO R4 Minima — pick one for the report
- All structural materials non-magnetic (PLA, carbon fibre, aluminium); SIL6T/K rod ends have accepted ferromagnetic content for the first prototype only
