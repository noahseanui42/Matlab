# report_extract.md — evidence from `noahseanui42/Matlab` for the ENGR489 report

Generated 2026-10-11 from HEAD `8233655` (branch `claude/elegant-shannon-h3fw0o` = `master`).

**Inputs read.** `Report.md` (your skeleton, called "skeleton" below) and `ENGR489_Literature_Notes.md` (your sources, called "sources" below). I assumed these are the files you named `skeleton.md` and `sources.md`. I read the skeleton in full and the sources index plus all note headings. I did not read every note body.
**Repo read.** Every tracked file except the vendored `third_party/ArduinoJson` (143 files, skimmed only), all 46 commits (`git log --reverse --stat`, vendored library excluded from the stat), and the 20 scan CSV/JSON pairs (`FieldScan/data/`).

**Citation key.**
- `path:line` and `hash` are recorded evidence in the repo.
- **[D]** = derived by me from repo files. Script in `report_extract_scripts/`; the run command is in Appendix B.
- **[X]** = repo only *points* to this, the primary evidence is elsewhere.
- "rationale not recorded" = the repo gives the decision but not the reason.

---

## 0. Read this first — what this repo can and cannot support

1. **There is no `CLAUDE.md` in the repo, now or in any commit.** The repo's own notes are `FieldScan/HANDOFF.md`, `FieldScan/README.md` and `engr489-firmware/README.md`. The README/HANDOFF text cites "HANDOFF §4/§6/§9/§10/§11", a planning document that is **not in the repo**.
2. **No plots are committed.** `git log --all` contains no png/jpg/pdf/svg/fig file. Every figure is regenerable only from scripts plus data (Section 7).
3. **The calibrated firmware is not in this repo.** The robot that produced all the data ran branch `claude/amazing-lovelace-onixrv` (commits `a185b44` calibration refit, `ffe256b` upward approach), named in `pose_correction_hybrid.json:52` and `FieldScan/README.md:83-87`. This repo has only two branches (`master`, this one).
   - HEAD `delta_servo/config.h` and `robot_config.py` still hold the **older** geometry (SP 150, L_UP 180, no arm remap, first-pass servo calibration) (`config.h:14-15,35-37`).
   - `pose_correction.py:23-29` and `field_scan.py:51-54` say so explicitly.
4. **Most of the skeleton's calibration and validation numbers have no primary data or code here.**
   - Missing: joint calibration 3.4° → 0.51°, hysteresis, pen-and-ruler box tests, the 2–2.5 mm repeatability, the dipole known-shift test, drift, tilt, the 260-point full-box maps, and the 5-map repeat set.
   - They appear in the repo only as pointers to `noahseanui42/engr489-REPORT` (`analysis/`, `calibration/2026-10-01_*.md`) (`pose_correction.py:16-21`, `pose_correction_hybrid.json:27`, `FieldScan/README.md:82`, `HANDOFF.md:156`).
   - That repo is outside this session's scope, so I did not read it. Say if you want it added.
5. **All committed field data is 5×5×3 = 75-point scans of a permanent magnet, coils off, taken 2026-10-02** (Appendix A). Every `coil_current_A` is `null`. There is **no coils-on cage map** in the repo, which is consistent with your Future Work #1.
6. **AI provenance (for your `[CHECK: course policy on declaring AI tool use]`).**
   - 28 of 46 commits are authored as `Claude` (including one merge, `1151462`). 17 of those carry `Co-Authored-By` trailers (15 Opus 5.5, 2 Sonnet 5.5).
   - 7 commits are authored by you (`aea782f` plus six data commits). 11 are PR-merge commits by `noahseanui42`.
   - The firmware port, GUI patches, `field_scan.py`, `pose_correction.py`, the tests and all `FieldScan/*.m` plotting code were committed by Claude sessions. Source: `git log --format='%an'` and trailers.
7. **The skeleton names GitLab (I-B, Appendix A); the repo remote is GitHub** (`origin https://github.com/noahseanui42/Matlab`). The repo is described as public (`engr489-firmware/README.md:22`).
8. Sections 9 and 10 list where the skeleton and repo disagree. Items I verified numerically are marked [D].

---

## 1. Repo purpose and active date range

**Purpose.**
- Software for the ENGR489 delta robot (VUW capstone, supervisor Dr Christopher Hollit): a magnetometer-probe delta robot that maps a magnetic field "inside a Helmholtz coil" (`engr489-firmware/README.md:3-5`; `FieldScan/README.md:3`; `HANDOFF.md:12-14`).
- Contents:
  - Servo firmware for an Arduino UNO R4 Minima, replacing upstream stepper firmware, keeping the upstream serial protocol (`README.md:5-9`).
  - A patched upstream Python GUI (`delta_app/`).
  - A Python scan runner (`field_scan.py`) and an open-loop pose correction (`pose_correction.py`).
  - MATLAB post-processing and plotting (`FieldScan/*.m`).
  - An earlier MATLAB/Simulink planning pipeline (`Kinematics/`, `Simulink/`).
  - Host-side tests (`engr489-firmware/tests/`).
  - A `submission_snapshot/` copy of the files to submit (`f77e915`).
- v1 is **open loop and "runs outside the coil cage"**; closed loop is future work (`README.md:11-12`).
- No mechanical CAD, BOM, wiring files, report text or cost data are in the repo.

**Active date range** (author dates from `git log`, in the committer's own timezone: your commits show +1200/+1300, Claude's +0000).
- First commit `aea782f`, 2026-09-15 (20:06 +1200). Last commit `8233655`, 2026-10-02 (20:14 +1300). That is 18 calendar days.
- Commits fall on only four dates: 09-15 (1), 09-27 (18), 10-01 (12), 10-02 (15), 46 in total (`git log --format=%ad`). There is a **12-day gap** between 09-15 and 09-27.
- Servo calibration measurements are dated 2026-09-26 (`engr489-firmware/README.md:75,211-212`, commit `41c19f1`).
- All scan data was recorded on 2026-10-02, 01:30–19:12 local time (meta `started`/`finished`, Appendix A).
- The Simulink model was created 2026-09-15 03:57Z by `nuinoah`, R2025b (unzipped `S01_Scan_pipe.slx`, `metadata/coreProperties.xml`).

---

## 2. Timeline of key changes

34 non-merge commits are listed. The 12 merges (`db42f6e b8a4ff1 bfec46f 2df7e42 48e65b4 2830a40 02b48d4 00667b9 29e37e8 40e44bf 8233655` are PRs #1–#8, #10–#12; `1151462` is a plain merge of `claude/gallant-planck-stiamj`) are omitted. There is no PR #9 in this history.

| Hash | Date | Change |
|---|---|---|
| `aea782f` | 2026-09-15 | "matlab final take": `Kinematics/` (scan_grid, ik_batch, validate_batch, angle_to_pulse, init_scan, rod_axes_at, Export_table) and `Simulink/S01_Scan_pipe.slx` |
| `631bc99` | 2026-09-27 | Servo firmware (`delta_servo/`), patched upstream GUI (`delta_app/`), host tests, README. Ported from `grzesiek2201/Delta-Robot` @ `48a8038` (GPL-3.0). Vendored ArduinoJson (143 files). Limits/geometry were `TODO(calibrate)` placeholders |
| `b54197e` | 09-27 | GUI: label text invisible in macOS Dark Mode |
| `fb57f34` | 09-27 | GUI: widen default window (1100×700 clipped) |
| `06c56ea` | 09-27 | GUI: size window to the actual screen |
| `082d636` | 09-27 | GUI: shrink 3D plot to fit 1440×900 |
| `84d8f29` | 09-27 | GUI: move Program/Available COMs into File menu (macOS bare top-level commands not rendered) |
| `35b36d6` | 09-27 | GUI: same menu fix in Program Creator popup |
| `41c19f1` | 09-27 | Real calibration data "measured 2026-09-26" (single-arm); replaces placeholders; reach shrinks to ≈ z −561…−692 mm |
| `b0576f3` | 09-27 | Angle limits corrected to −20/70° with the full 3-arm assembly (single-arm numbers were falsely tight); reach −550…−793 mm |
| `62bde4c` | 09-27 | `ELBOW_JOINT_WIDTH` 45 → 35 mm (plot-only) |
| `1bf77ce` | 2026-10-01 | `FieldScan/`: MATLAB scan via serial + Phidget 1044 over MATLAB's Python interface, plots, demo scan |
| `c33024f` | 10-01 | `FieldScan/HANDOFF.md` |
| `f11cac4` | 10-01 | `compare_runs.m` (magnet repeatability test) |
| `c917370` | 10-01 | Delta-app data logging + `compile_scan.m`/`make_program.m` (later removed in `ebd4e61`) |
| `af9497a` | 10-01 | `field_scan.py` (Python scan runner) + host tests |
| `ebd4e61` | 10-01 | Consolidation: `field_scan.py` runs scans, MATLAB only post-processes; speed sent as JSON int; `compare_runs.m` `gradient()` axis order fixed; delta-app data log removed |
| `8217320` | 10-01 | Position correction `--correction hybrid` (`pose_correction.py` + JSON); 14 new tests, "all 31 pass" (commit message) |
| `b8aac57` | 10-01 | `field_scan`: continuous background read of the 20 ms status stream (stale `en=0`/`mv=0` lines) |
| `708ebf8` | 10-01 | Print Bx/By/Bz per point; "sensor saturates per axis (about 2 G on the bench)" |
| `dfdcdc3` | 10-01 | Metadata geometry taken from the calibrated JSON, not `robot_config.py` |
| `caa2555` | 10-01 | Settle 0.5 s → 5 s ("probe was still swinging") |
| `5685767` | 10-02 02:39 +1300 | First magnet data: demo, dry run, magcheck, settle-0.5 s runs |
| `5675bed` | 10-02 02:59 +1300 | Magnet run, settle 5 s |
| `afa21dc` | 10-02 03:27 +1300 | Magnet moved close; "off ×2, hybrid ×2" (per message; the files `mag_off_1_031036` are off ×1) |
| `83c4556` | 10-02 04:02 +0000 | `plot_field_layers.m` (heat map + direction map); "tested in Octave 8.4" |
| `6cf6a37` | 10-02 | `plot_field_layers`: stacked 3D map |
| `8ed65b4` | 10-02 | `plot_field_layers`: side views |
| `c701768` | 10-02 | `plot_field_arrows3d.m` |
| `93a5220` | 10-02 17:40 +1300 | No-magnet background scan (dry run 1) |
| `eb08ddd` | 10-02 18:23 +1300 | Strong-magnet scan + `hybridcheck` |
| `88dc20b` | 10-02 19:27 +1300 | Strong magnet: hybrid, off repeat, hybrid repeat |
| `5dab932` | 10-02 06:35 +0000 | Plot scripts resolve `data/…` relative to FieldScan |
| `f77e915` | 10-02 06:59 +0000 | `submission_snapshot/` (unmodified copies; excludes demo/check/aborted/crashed scans and HANDOFF). Includes 3 committed `__pycache__` .pyc files |

---

## 3. Design decisions

| # | Decision | Chosen | Rejected / alternative | Reason recorded in repo | Source |
|---|---|---|---|---|---|
| D1 | Robot type | Delta robot, hobby servos | (original upstream: stepper delta) | Gantry→delta change **not in repo**. Servos vs steppers: rationale not recorded | `README.md:5-9` |
| D2 | Controller | Arduino UNO R4 **Minima**, 3 servos on D9/D10/D11, separate 6 V AA pack | R4 **WiFi** | "to avoid RF near the sensor" | `HANDOFF.md:14-17`; `config.h:20,34-38` |
| D3 | Magnetometer path | Phidget 1044 on PC USB, **not** via the Arduino | – | Not recorded (HANDOFF only states it) | `HANDOFF.md:24-25` |
| D4 | Phidget access from MATLAB | MATLAB `py.` interface to the `Phidget22` pip package | Phidget22 MATLAB/C library | "so the user doesn't need Xcode" | `HANDOFF.md:48-49` |
| D5 | Who runs scans | Python `field_scan.py`; MATLAB only post-processes; GUI used for set-up and must be closed | MATLAB `run_field_scan.m` (kept as "fallback"); delta-app logging + `compile_scan.m` (removed `ebd4e61`) | Python owns port + sensor, "so the GUI's 50-point program cap does not apply"; only one program can hold the serial port. Why the delta-app logging was dropped: rationale not recorded | `field_scan.py:4-8`; `HANDOFF.md:78-101`; `FieldScan/README.md:12-14,131-136` |
| D6 | Control stack | Keep upstream GUI and **byte-identical serial protocol** | – | "so the upstream Python GUI drives this robot with only the patches". A from-scratch stack with a layered safety design is **not in the repo** (nor in any commit) | `README.md:5-9` |
| D7 | Code structure | `kinematics`/`protocol`/`motion` without Arduino headers; only `.ino` touches `Servo`/`Serial`/`millis` | – | Host-testable with plain `g++` | `README.md:49-51`; `config.h:4-6` |
| D8 | Unreachable moves | IK returns false with **no partial angle write**; move rejected, error latch `e=1`; scan logs NaN | (upstream relied on NaN comparisons) | Guards "instead of relying on NaN comparisons" (HANDOFF §6.2, not in repo). Why reject rather than clamp: rationale not recorded | `kinematics.cpp:4-7,80-83`; `field_scan.py:371-376` |
| D9 | Joint vs linear moves | Scan uses joint moves (`i=0`) | Linear mode | "Linear moves can stop early near the workspace edge (`e=2`)" | `HANDOFF.md:50`; `README.md:253-254`; `motion.cpp:261-266` |
| D10 | Joint limits | −20°/70° on all arms from **full 3-arm** assembly | Single-arm pass (−16…−25 / 26.5…29.5°); placeholder ±(−30,80) | Single-arm numbers false: with the other arms slack the plate sags/tilts into an early collision | `README.md:73-95,228-231`; `config.h:22-31`; `b0576f3` |
| D11 | Servo write after `attach()` | `attach()` then `writeMicroseconds()` | Write before attach | R4 Servo library ignores writes before attach and starts at 1500 µs (checked against `ArduinoCore-renesas` source) | `README.md:152-167`; `delta_servo.ino:41-50` |
| D12 | End of scan | Robot left **enabled** | Disable | "disabling lets the arms drop" | `field_scan.py:17,399-405`; `FieldScan/README.md:50-51` |
| D13 | Settle time | 5 s after the move (including the rise) | 0.5 s | "probe was still swinging when sampling started" | `caa2555`; `field_scan.py:63` |
| D14 | Status stream handling | Background reader thread; accept `mv=0` only after seeing `mv=1` (or 1 s) | Read on demand | Old `en=0`/`mv=0` lines backed up in the Arduino caused "Robot did not enable" and false arrivals | `b8aac57`; `field_scan.py:103-128,196-227`; `test_field_scan.py:99-123` |
| D15 | Speed encoding | JSON **int** | float (`"v":2.0`) | ArduinoJson reads a float as 0 → default speed, so `--speed` was silently ignored | `ebd4e61`; `HANDOFF.md:94-98`; `test_field_scan.py:56-64` |
| D16 | Position correction | Open-loop, pose-dependent, in Python (`send = FK(IK(p) − Δθ)`); firmware unchanged; geometry stored *with* the coefficients | Reading geometry from `robot_config.py`; closed-loop pot feedback | "the model only holds for the geometry it was fitted with". Choice of *hybrid* over other models: only the structure (v1 centring + mid-box x term; corners identical to v1) is in the repo; **why hybrid was selected is not recorded here** [X] | `pose_correction.py:9-29`; `pose_correction_hybrid.json:27`; `README.md:305-309` |
| D17 | Closed loop | Deferred ("future work") | – | Rationale not recorded in repo | `README.md:11-12,305-309` |
| D18 | Mapping convention | `SB`/`SP` = equilateral **side** lengths through joint centres; forearm 625 mm | 600/700 mm | "confirmed with the user" | `README.md:53-60` |
| D19 | Baseline handling | Subtract a coils/magnet-off scan, matched **by position** (0.1 mm) | Index-wise subtraction | So a skipped point doesn't shift the rest | `FieldScan/README.md:105-107`; `plot_field_map.m:22-23` |
| D20 | Repeatability metric | σ of \|B\| over runs, noise floor subtracted in quadrature, ÷ \|∇\|B\|\| → mm; weak-gradient points (< 0.1 × median gradient) set NaN | – | Documented as an *upper bound* (includes field drift between runs) | `compare_runs.m:11-18,76,93-97,113-114` |
| D21 | Reach reporting | Report on-axis z ≈ −550…−793 mm and 89% of a 25 mm grid reachable, "treat −550..−793 as the real scan volume" | Planned −450…−750 mm | Based on −20° floor | `README.md:97-112` |
| D22 | MATLAB/Simulink planning pipeline | Offline pulse table: grid → IK → validate → angle→pulse → export | – | Role not stated; later FieldScan "only calls `scan_grid.m`" | `HANDOFF.md:141-143`; `init_scan.m:1` |

---

## 4. Quantitative results and parameters

### 4.1 Geometry and kinematics

| Value | Source | Note |
|---|---|---|
| SB = 175 mm | `config.h:13`; `robot_config.py:11`; JSON `:29` | unchanged in calibrated geometry |
| SP = **150** mm at HEAD; **75** mm calibrated | `config.h:14`; `robot_config.py:12`; `pose_correction_hybrid.json:30` | Calibrated value is not in firmware here |
| L_UP = **180** mm at HEAD; **177** mm calibrated | `config.h:15`; `robot_config.py:13`; JSON `:31` | |
| L_LO = 625 mm | `config.h:16`; JSON `:32`; `README.md:59-60` | |
| Arm remap `GEOM_TO_PHYS = [2,0,1]` (calibrated only) | JSON `:33-37`; `field_scan.py:51` | |
| Arm slots at −90°, 30°, 150°; D9 = arm 1 on −y, arms 1→2→3 CCW from above | `pose_correction.py:63`; `config.h:18`; `README.md:61-64` | |
| TCP offset (0, 0, −21) mm | `robot_config.py:28`; `README.md:65-67`; `scan_config.m:8`; `field_scan.py:60` | firmware has no TCP; Python subtracts it (`field_scan.py:201`) |
| `Z_MAX` −300; `Z_LIMIT_DEFAULT` −800 mm | `robot_config.py:26-27` | GUI guards |
| Joint limits −20° to +70°, all arms | `config.h:35-37`; `robot_config.py:24`; JSON `:38-51` | |
| Servo pulse clamp 830–2170 µs | `config.h:39`; `motion.cpp:292-304` | |
| On-axis reach z ≈ −550 to −793 mm; 1501/1690 reachable on 25 mm grid x,y ∈ ±150, z ∈ −790…−550 | `README.md:101-106` | Stated for **HEAD geometry (SP 150, L_UP 180)**. [D] reproduced exactly: −549.5…−793.5 mm, 1501/1690 (`reachability.py`) |
| Calibrated geometry (SP 75, L_UP 177): on-axis −540.0…−787.5 mm; **1414/1690 = 83.7%** | [D] `reachability.py` | Effector-centre z (TCP 0). Probe z (TCP −21): −561.0…−808.5 mm; 1397/1690 |
| `zHome(0)` = −608.2 mm (HEAD geometry); −597.2 mm (calibrated) | [D] by formula from `kinematics.cpp:91-95` | Skeleton's −597.2 is the calibrated value |
| IK singularity guard \|G−E\| < 1e-6; discriminant < 0 → unreachable; first root in limits (`+` then `−`) | `kinematics.cpp:24,32-52` | |
| Jacobian `J = S⁻¹D`; FK = Newton, ≤ 50 iterations, tol 1e-10 | `pose_correction.py:102-122` | |
| Upstream `calculateFPK` z-nudge (±0.01 mm when two elbow heights coincide) biases GUI FK by up to ≈ 0.17 mm for equal-angle poses (every x = 0 point); round-trip test tolerance 0.2 mm | `README.md:278-284`; `test_kinematics.py:85,106-108` | |
| Servo supply: "separate 6 V AA pack"; "4×AA supply sagging" | `HANDOFF.md:16-17`; `README.md:258-259` | skeleton says "6 V bench supply" |

### 4.2 Servo calibration

| Value | Source |
|---|---|
| HEAD `CAL` (centre µs / µs per °): D9 1460 / 11.8231; D10 1385 / 10.0481; D11 1410 / 10.5544; dir +1 | `config.h:35-37`; `README.md:87-91` |
| `trim_us` −60, −135, −110; centre = 1520 + trim | `config.h:32,35-37` |
| Calibrated refit (centre / µs per °): D9 1456 / 9.8315; D10 1373 / 9.7664; D11 1393 / 9.6658 | `pose_correction_hybrid.json:52` (text string only); [X] firmware branch `a185b44` |
| Single-arm pass limits −16…−25° / 26.5…29.5° (superseded) | `README.md:75-80` |
| Calibration procedure: 5-point linear fit at 1200/1360/1520/1680/1840 µs with an angle finder; limit = tighter of mechanical limit and ±(650/us_per_deg) | `README.md:223-235` |

### 4.3 Firmware motion and protocol

| Value | Source |
|---|---|
| Servo/planner tick 20 ms; status stream every 20 ms; `rxBusy` idle timeout 1000 ms | `config.h:42-44` |
| Speed = 5 mm/s per GUI step (v 1–10 → 5–50 mm/s); default v = 2 | `config.h:45-46`; `FieldScan/README.md:62` |
| Joint-speed cap 90°/s; minimum move 200 ms; max 50 program points; program not looped | `config.h:47-50` |
| Move duration `T = max(dist/speed, max Δθ/90°/s, 200 ms)` | `motion.cpp:71-81` |
| Smoothstep `s = 3u² − 2u³`; linear moves re-solve IK every tick | `motion.cpp:255,261-266` |
| Frame buffer 128 bytes; baud 115200 configured but ignored by the R4's USB CDC | `protocol.h:37`; `README.md:118` |
| Error latch `e` 0–5 (ok / unreachable / linear path left workspace / not allowed / JSON error / µs clamp) | `README.md:141-143` |

### 4.4 Scan software defaults (`field_scan.py`)

| Value | Source |
|---|---|
| speed_v 2; move timeout 30 s (fixed); settle 5.0 s; park (0,0,−650); n_avg 20; sample spacing 0.02 s | `field_scan.py:61-64,69-70` |
| Default grid x ±50, y ±50, z −700…−600, 5/5/5 (125 pts) | `field_scan.py:65-67`; `FieldScan/README.md:61` |
| Sensor serial 302277; data interval = max(min interval, 20 ms); attach timeout 5000 ms; first-sample timeout 3 s | `field_scan.py:68,239-247` |
| Enable wait ≤ 3 s; stale-line flush 100 ms after a command; trust `mv=0` after 1 s with no `mv=1` | `field_scan.py:187-192,205,221` |
| CSV: 19 columns (first 12 shared with MATLAB scan); `err` 0 ok / 1 unreachable / 5 clamp | `field_scan.py:44-49`; `FieldScan/README.md:122-129` |
| MATLAB fallback defaults: settle **0.5 s**, grid z −700…−600, 12 columns | `scan_config.m:11,17-19`; `run_field_scan.m:49` |
| Serial port `/dev/cu.usbmodem14101`, DTR True | `robot_config.py:33-34` |

### 4.5 Position correction

| Value | Source |
|---|---|
| β = 5.387484e-07 (cx), 5.387484e-07 (cy), 2.677471e-07 (cx_mid) | `pose_correction_hybrid.json:8-12` |
| Model `Δθ = β_x f_cx + β_y f_cy + β_mid f_cx (1−(y/150)²)`, `f = Jᵀ(−x,0,0)` etc. **in degrees** | `pose_correction.py:9-11,129-131` |
| **Units (your [CHECK]):** β multiplies degree-valued features and `predict_offset_deg` returns degrees; `compensate` converts with `np.radians` | `pose_correction.py:129-131,145-148,154`. Reference outputs in degrees: `test_pose_correction.py:20-28` |
| Predicted shortfall at (50,150,−650): D9 1.362°, D10 0.373°, D11 −1.978° (matches the report-repo reference values) | `test_pose_correction.py:21`; [D] `correction_magnitude.py`. So "~0.5° per servo" is not what the code gives at the box edge |
| Valid box x ±50, y ±150, z −700…−600 mm | JSON `:13-26` |
| Stated: uncorrected shortfall ≈ 13% of distance to centre, ≈ 19 mm rms at corners | `pose_correction.py:3-5`; `FieldScan/README.md:73-74` [X] |
| Stated: corrected 19 → 2.3 mm rms; box 100 × 299 mm at z −650; repeatability 2–2.5 mm; ≈ 5 mm residual at box centre, z −650, from platform tilt | `pose_correction.py:20-21`; `FieldScan/README.md:78-82` [X] |
| Fit data: v1 centring on protractor runs 2, 5, 6 + mid-box x term on pen-and-ruler box/midpoint runs 10–19 (2026-10-01) | JSON `:27` [X] |
| Correction size, scan grid corners (±50, ±50): 12.05 mm at (50,50,−675), 12.93 mm at (50,−50,−675); logged sent−target max 12.9 mm | [D] `correction_magnitude.py`, `scan_stats.py` on `…hybrid_182828.csv` |
| Correction size at R2-box corners (±50, ±150): 17.7 mm (z −600), 21.8 mm (−650), 26.6 mm (−700). At (50,0,−650): 9.85 mm | [D] `correction_magnitude.py` |
| Test references on-axis (platform z, TCP 0): −600 → 0.89307°, −650 → 16.59158°, −700 → 32.43681°, −750 → 50.504961° | `test_pose_correction.py:39` |

### 4.6 Sensor

| Value | Source |
|---|---|
| Phidget 1044_0 (`HANDOFF.md:24`), reported name "PhidgetSpatial Precision 3/3/3 High Resolution", serial 302277, range **±5.6 G** on all axes (from `getMaxMagneticField`) | e.g. `FieldScan/data/2026-10-02_dryrun1_no_magnet.meta.json:45-52`; `field_scan.py:254` |
| ±8 G was only the 1044_1 spec, unconfirmed; expected Earth field ≈ 0.55–0.6 G | `HANDOFF.md:26-31` |
| "Sensor saturates per axis (about 2 G on the bench)" | `708ebf8` (conflicts with 5.6 G; unresolved in repo) |
| Demo Biot–Savart centre field 1.4986 G vs Helmholtz formula 1.4985 G; demo R = 0.30 m, 50 A-turns | `HANDOFF.md:66-67`; `make_demo_scan.m:14-15` |

### 4.7 Tests

| Value | Source |
|---|---|
| 35 pytest items pass at HEAD (config 3, field_scan 9, kinematics 6, pose_correction 14, ik_parity 1, motion 1, protocol 1) | [D] my run on a scratch copy |
| C++ runners behind the motion/protocol items: 11 and 10 cases | `motion_test_runner.cpp:63-320`; `protocol_test_runner.cpp:86-196` |
| Earlier counts: "All 17 tests pass" (HANDOFF); "all 31 pass" (`8217320`) | `HANDOFF.md:98`; `8217320` |
| `arduino-cli compile` was **never run** on the real target; a stand-in `Arduino.h`/`Servo.h` compiled with 0 warnings (`-Wall -Wextra`) | `README.md:293-303` |

### 4.8 Quantities derived from the CSVs [D]

Scripts: `report_extract_scripts/scan_stats.py`, `compare_runs_replica.py`, `insitu_correction_check.py`.

| Quantity | Value |
|---|---|
| No-magnet background \|B\| (75 pts) | 71.2–72.9 µT, median 72.0; mean B = (0.55, −17.8, 69.8) µT (sensor axes) |
| Sensor scatter (SD of 20 samples, pooled over 5 dry-run1 scans) | median 70 nT, max 426 nT (the skeleton's 76 / 430 not reproduced exactly) |
| Scan duration, 75 pts, settle 5 s | 829–862 s = 13.8–14.4 min; 11.1–11.5 s/point |
| Scan duration, 75 pts, settle 0.5 s | 490 s; 6.5 s/point |
| Settle 0.5 s vs 5 s (`settle_0p5s/mag_off_1_…022713` vs `mag_off_1_…024004`; identical notes "magnet fixed", placement assumed equal, not recorded) | \|ΔB\| median 0.82 µT, max 1.71 µT |
| Strong magnet, magnet-only \|B\| | 5.4–24.1 µT (off); at (+50,0,−675) 23.7 / 24.1 µT, at (−50,0,−675) 7.3 µT |
| Off vs off2 / hybrid vs hybrid2 vector difference | \|ΔB\| median 0.29 µT, max 1.14 / 1.15 µT |
| `compare_runs` method (replica), off ×2 | σ_B median 135 nT (max 630), noise floor median 18.8 nT, σ_pos median 1.16 mm, p95 3.66, max 5.63; n = 2 runs |
| `compare_runs` method (replica), hybrid ×2 | σ_B median 136 nT (max 520), σ_pos median 0.98 mm, p95 4.04, max 7.08; n = 2 runs |
| In-situ correction check (mean off → mean hybrid, predicted ΔB = ∇B·(sent−target)) | n = 225, k = 1.13, r = 0.93, rms residual 279 nT; run-to-run rms difference 234 / 243 nT (off / hybrid). Single pairs: k 1.11–1.13, r = 0.90 |
| Exact-zero averaged components (whole 20-sample window = 0, SD = 0) | 9 of 225 in the no-magnet scan, all on the x-axis; 0 of 225 in each of the four magnet scans |

### 4.9 Earlier MATLAB planning pipeline (2026-09-15) — superseded

`init_scan.m:4-7,10-12,15-16,19-23,26-28`:
- sb 175, sp 150, L1 180, **L2 600** ("600 vs 700 still unresolved").
- Scan volume x ±100, y ±50, z −650…−550; grid 21 × 11 × 11 = 2541 points.
- Servo pulse 830–2170 µs, home 1500 µs, 11.67 µs/°, `sgn = [1 −1 −1]` ("CONFIRM THESE").
- Rod-end articulation limit 13° (`artic_max`, labelled "SIL6T/K"); `th_lim [−45 90]`.

---

## 5. Test procedures and methods actually used

**In the repo (code or data exists):**
1. **Host unit tests (pytest + g++).**
   - Kinematics: FK/IK round trip over a grid, on-axis regression values, unreachable cases (`test_kinematics.py`).
   - C++/Python IK parity (`test_ik_parity.py`).
   - Protocol framing and motion planner, as C++ runners driven by pytest (`test_protocol.py`, `test_motion.py`).
   - `config.h` ↔ `robot_config.py` sync (`test_config_sync.py`).
   - `field_scan.py` against `FakeLink`/`FakeMag`, including replay of a backed-up serial stream (`test_field_scan.py`).
   - Pose correction vs the report repo's reference values (`test_pose_correction.py:20-28`).
2. **Simulated scan** (`field_scan.py --simulate`): `demo_20261002_013031` (125 pts, settle 0, `sensor.name = "simulated"`).
3. **Bench dry run:** `dryrun_20261002_014611`, 27 pts, 3×3×3, x,y ±25, z −675…−625, settle 0.5 s, hybrid. This is the `HANDOFF.md:127` "next step 1" grid.
4. **Magnet checks:** `magcheck_*` (2 pts at (±50, 0, −650)) and `hybridcheck_20261002_175834` (2 pts at (50,0,−675/−650)) to place the magnet and sanity-check the correction.
5. **Settle-time comparison:** 0.5 s vs 5 s on the same magnet placement (Section 4.8).
6. **Correction on/off alternation:** with the strong magnet, off → hybrid → off2 → hybrid2 (18:00, 18:29, 18:44, 18:58), after a no-magnet baseline at 17:22. Intended protocol: `HANDOFF.md:125-133`.
7. **Analysis methods in code:**
   - Baseline subtraction by position (`plot_field_map.m:22-33`).
   - `compare_runs` repeatability (σ_|B| ÷ gradient).
   - Sensor→robot rotation hook, default identity (`scan_config.m:28-30`).
   - Plot scripts "tested in Octave 8.4" (`83c4556`, `c701768`).
8. **Bring-up checklist** (6 steps: servo power off, DTR, 3-point program upload, calibration, support arms then power, jog ±15 mm and compare with the 3D plot) is written down (`README.md:240-259`). It is marked "the user runs this"; no results are recorded other than the macOS fixes (Section 6).

**Referenced but not in the repo [X]:** protractor joint sweeps (runs 2, 5, 6), pen-and-ruler box and midpoint tests (runs 10–19), leave-run-out validation, backlash/hysteresis measurements, the dipole known-shift test, and drift/tilt analysis. Evidence: `pose_correction_hybrid.json:27`, `pose_correction.py:16-21`. The methods named in the skeleton (II-H test ladder, IV-A, IV-B–D, IV-F) have no scripts or data here.

**Never run, per the repo's own notes:**
- `HANDOFF.md:69-76`: "none of the `.m` files has been run in MATLAB", and `compare_runs.m` "Not run in MATLAB" (`:123`). No later commit records a MATLAB run; the plot scripts were exercised in Octave 8.4.
- `arduino-cli compile` on the real board (`README.md:293-303`).

---

## 6. Problems hit and workarounds

| Problem | Workaround | Source |
|---|---|---|
| No data from R4 until DTR asserted | `SERIAL_DTR = True`; `ser.dtr` set before `open()` | `README.md:184-189`; `robot_config.py:34`; `field_scan.py:120` |
| GUI labels invisible in macOS Dark Mode | `option_add` fg/bg defaults | `b54197e`; `README.md:190-194` |
| macOS menu bar drops bare top-level commands | Moved into File cascade (main + Program Creator) | `84d8f29`, `35b36d6` |
| GUI window clipped on 1440×900 | Size from screen, 3D plot 6→4.8 in | `fb57f34`, `06c56ea`, `082d636` |
| R4 `Servo` ignores writes before `attach()`; starts at 1500 µs | Write after attach; 1500 µs is inside the safe range | `README.md:152-167` |
| Single-arm calibration gave falsely tight limits (plate sag) | Re-measure with all three arms enabled: −20/70° | `b0576f3` |
| Shallow end of planned −450…−750 mm unreachable | Treat −550…−793 mm as the real volume; "worth re-probing whether −20° is the safe floor" | `README.md:97-112` |
| Upstream FK nudges z by ±0.01 mm on equal angles (up to ≈ 0.17 mm bias) | Test tolerance loosened to 0.2 mm; IK unaffected | `README.md:278-284` |
| float32 (firmware) vs float64 (Python) IK differ where `(G−E)` is small | Parity test skips those points | `README.md:285-291` |
| Float `"v"` read as 0 → `--speed` ignored | Send int; regression test | `ebd4e61`; `test_field_scan.py:56-64` |
| Backed-up 20 ms status stream: stale `en=0`, early `mv=0` | Reader thread, `flush_input()`, `seen_moving` guard, 3 s enable window | `b8aac57`; `test_field_scan.py:99-132` |
| `compare_runs.m` errored on grids with nx ≠ ny (`gradient()` axis order), `prctile` toolbox dependency | `[Gy,Gx,Gz] = gradient(V,ys,xs,zs)`; local `pct95` | `ebd4e61`; `compare_runs.m:88-91,140-146` |
| Probe still swinging at 0.5 s | Settle 5 s | `caa2555` |
| Sensor saturation on bench (≈ 2 G per axis) while placing the magnet | Print per-axis Bx/By/Bz | `708ebf8` |
| Metadata recorded older `robot_config` geometry | Read geometry from the correction JSON | `dfdcdc3` |
| Scan geometry mismatch (this branch's `robot_config.py` vs calibrated firmware) | Warning at scan start; correction carries its own geometry | `field_scan.py:314-322`; `HANDOFF.md:160-167` |
| 1044 not detected on the user's M2 MacBook Air via USB-C adapter (unresolved) | Used another Mac ("Rochelle's MacBook Pro", works) | `HANDOFF.md:36-41` |
| Authoring container could not reach `downloads.arduino.cc` (403), so the board package was unavailable | Compiled `.ino` against stand-in headers instead; real compile left to the user | `README.md:293-303` |
| Zero-valued windows on the x-axis in the no-magnet scan (9 of 225 components) | **Not handled:** `read_avg` only drops samples that raise exceptions, so zeros are averaged in. Not documented in the repo | `field_scan.py:266-281`; [D] `compare_runs_replica.py` |
| `numpy` imported by `pose_correction.py` but absent from `requirements.txt` (arrives via matplotlib/pandas) | Not addressed | `requirements.txt:1-4`; `pose_correction.py:35` |
| Two 0.5 s `mag_off` scans aborted (3 rows) or crashed (0 done, `finished` null); cause not recorded | Left out of `submission_snapshot` | `settle_0p5s/*015453*.meta.json`, `*015519*.meta.json`; `f77e915` |

---

## 7. Figures: which scripts generate which plots, from which data

**No figure images are committed.** All figures are generated on demand.

| Script | Figures | Intended inputs | Committed data it can run on |
|---|---|---|---|
| `plot_field_map.m` | (1) 3D vectors coloured by \|B\|; (2) \|B\| slices through the centre; (3) % deviation from centre on the middle z plane, the "Helmholtz uniformity" plot (`:10-13`) | Coils-on CSV minus coils-off baseline (`FieldScan/README.md:96`) | Any scan; only magnet scans exist. `make_demo_scan.m` writes synthetic Helmholtz data to `data/demo_coils_on.csv`/`demo_baseline.csv` (not committed) |
| `plot_field_layers.m` | (1) heat map per z layer; (2) direction map (arrows + Bz colour); (3) side views x-z or y-z; (4) stacked 3D map. `Save` writes PNGs | Magnet scan minus `no_magnet` scan (`:5-8`, README `:97-101`) | `2026-10-02_dryrun1_magnet_*` minus `2026-10-02_dryrun1_no_magnet.csv`; or `mag_off_1_*` |
| `plot_field_arrows3d.m` | One 3D arrow figure, colour per z layer, arrow length ∝ \|B\| | Same | Same |
| `compare_runs.m` | (1) σ_B per point vs noise floor; (2) σ_pos 3D scatter on the grid (not a heat map); (3) σ_pos histogram (`:117-137`) | ≥ 2 magnet runs on one grid + no-magnet noise file (`README.md:109-115`) | `…magnet_*180000` + `…off2…` (+ the two hybrids), noise `…no_magnet.csv` |
| `make_demo_scan.m` | None; writes synthetic CSVs | – | – |
| `deltagui.py`, `joint_angle_plot.py`, `gyro data.py` | Live GUI plots (upstream) | Serial stream | Not report figures |

**Skeleton figures with no script or data in the repo:**
- Fig. 2/3 workspace sweep: only the README's numbers and my `reachability.py`.
- Fig. 6 correction vectors: no plotting script; `PoseCorrection.compensate` could compute them.
- Fig. 7, 8, 10, 13, 14 and the 260-point map of Fig. 11: no code or data.
- Fig. 9: my `insitu_correction_check.py` reproduces the numbers but draws nothing.
- Fig. 12: the nearest is `compare_runs` figure 2.
- Fig. 1, 4, 5: CAD, photo, wiring; no files.

---

## 8. External code, libraries and papers

| Item | Where used | In sources? |
|---|---|---|
| **`grzesiek2201/Delta-Robot`** (Jakub Wrona, Grzegorz Radziwiłko), GPL-3.0 @ `48a8038`; the old stepper firmware's header additionally says "FOR NON-COMMERCIAL USE ONLY" (`engr489-firmware/README.md:16-23`) | Ported: `delta_servo/*` (IK, `zHome`, protocol, planner), `delta_app/*` (GUI, `deltarobot.py`, `calculateworkarea.py`, etc.). `COPYING` in `delta_servo/` and `delta_app/`; "Derived from…" headers | **NO** (your skeleton cites it as [9]; no entry in the sources index) |
| ArduinoJson v7.2.1 (MIT) | Vendored `third_party/ArduinoJson`; `protocol.cpp`, host tests | **NO** |
| Arduino `Servo` (ArduinoCore-renesas `src/renesas/Servo.cpp`) | `delta_servo.ino`; behaviour checked in `README.md:152-161` | **NO** |
| Phidget22 Python package + drivers + Control Panel; Phidget 1044_0 | `field_scan.py:233-263`, `mag_open.m` | **NO** (skeleton's [14] datasheet not in the sources index) |
| pyserial, matplotlib, pandas, Phidget22 (`requirements.txt:1-4`); numpy (undeclared); tkinter | `field_scan.py`, GUI, `pose_correction.py` | **NO** |
| pytest, g++; `arduino-cli` (attempted) | `tests/`; `README.md:293-303` | **NO** |
| MATLAB (Simulink R2025b, `serialport`, `scatteredInterpolant`, `pyenv`/`py.`), Octave 8.4 | `FieldScan/`, `Kinematics/`, `Simulink/` | **NO** |
| Williams delta kinematics (E cosθ + F sinθ + G = 0, half-angle substitution) | Not cited anywhere in the repo. [D] `kinematics.cpp:64-76` matches the E/F/G terms in your Williams note term-for-term | Williams is in sources (`williams2016delta`) but the repo never cites it |
| Biot–Savart / Helmholtz formula | `make_demo_scan.m:31-43`; `HANDOFF.md:66-67`; no citation | not attributable |
| Futaba S3306MG | **Not named anywhere in the repo.** The servo model comes only from your sources | in sources (`futaba2010s3306mg`) |
| Rod end "SIL6T/K" (13° limit) | `init_scan.m:26` | Your sources use "SI6T/K" (`nzminiaturebearings2026female`). The `L` suffix normally means left-hand thread; check which part is fitted |
| Papers/standards (ISO 9283, Tira-Thompson, Bentaleb, Yang, MRI-robot reviews…) | **None cited in the repo** | in sources, not used by repo text |

Also in the skeleton but absent from the sources index: Wilson 2025 [18], ISO 230-2 [26], Renaudin et al. 2010, Nguyen 2022, S. Wang et al. 2019 (dipole localisation), Gosselin 1990 (only a "gap lead" in the notes). The skeleton uses numbers [1]–[30]; the sources use cite keys with no number mapping.

---

## 9. Mapping: skeleton section → evidence in this repo

Status: ✔ evidence in repo · ◐ partial/indirect · ✘ nothing in repo · ⚠ conflicts with the skeleton.

| Skeleton | Evidence | Status |
|---|---|---|
| I opening, I-A (cage, Wilson 2.7%/0.6%) | Project context only (`HANDOFF.md:12-14`); no cage data | ✘ |
| I-B final design | Delta, Arduino kinematics + servo drive, Python scans, MATLAB maps (`FieldScan/README.md:6-10`) ✔. GitLab ⚠ (GitHub). Gantry history, $500 budget: ✘ | ◐ ⚠ |
| I-B key findings | Joint 3.4° → 0.51°: ✘. Compliance ~13% / ~19 mm: ◐ [X] (`pose_correction.py:3-5`). 2.3 mm rms: ◐ [X] (`FieldScan/README.md:78-80`). Magnetometer check r = 0.93: ✔ reproduced [D]. Five maps 180 nT / 1.5 mm: ✘ (replica: 135 nT / 1.16 mm, n = 2). Dipole fit ≤ 1.3 mm: ✘. 33 mm offset / 4° rotation: ✘ | ◐ |
| I-C R1–R6 | R2 box = JSON valid box ✔ (`JSON:13-26`). R3 grid scan with per-point log ✔ (`field_scan.py:366-390`). R1 hard rule ✔ (`HANDOFF.md:14-15`). R4/R5 numbers: ◐ [X]. R6: ✘ | ◐ |
| I-D SDGs | ✘ | ✘ |
| II-A Helmholtz theory | Demo Biot–Savart only (`make_demo_scan.m`) | ✘ |
| II-B/C mapping systems, MRI robots | ✘ | ✘ |
| II-D delta robots | Upstream kinematics ported; no citations | ◐ |
| II-E calibration literature | ✘ | ✘ |
| II-F metrics | Repeatability definition in `compare_runs.m:11-18` (σ ÷ gradient) ✔. ISO 9283 not used in code | ◐ |
| II-G contributions | Compliance correction ✔ (`pose_correction.py`); magnetometer-based position check ✔ numbers [D], no script | ◐ |
| II-H tools | Arduino/Phidget ✔; MATLAB ✔; Python ✔; C++ firmware ✔; pytest ✔ but **35 not 27** ⚠. SolidWorks: ✘. GitLab: ⚠ | ◐ ⚠ |
| III-A architecture/interfaces | Host↔Arduino frame `<mode><json><#>` ✔ (`README.md:114-131`); status every 20 ms ✔; Phidget 20 ms interval ✔; "no signal wiring down the arms" rationale not recorded | ✔ |
| III-A Table IV | Design-response rows: rationale not recorded | ✘ |
| III-B geometry | Parameters ✔ (HEAD vs calibrated). **Reach/89% mix ⚠:** −540…−788 matches calibrated geometry, 1501/1690 is the HEAD-geometry figure ([D] calibrated = 1414/1690). Sweep script ✘. "Old 450/18/180/600 not used anywhere" ⚠: 600 is still in `init_scan.m:7`, 180/150 in `config.h`. Joint limits' origin ✔ (`README.md:73-95`) – fills your `[TODO: what sets them]` | ◐ ⚠ |
| III-C kinematics | IK ✔ (`kinematics.cpp:59-89`; guards ✔; first-root rule ✔). Newton FK ✔, Jacobian ✔ (`pose_correction.py:102-122`). z_home ✔ (−597.2 is the calibrated value [D]; HEAD −608.2) | ✔ |
| III-D mechanical | Only `ELBOW_JOINT_WIDTH` 35 mm and TCP −21 mm. Materials: "PLA, carbon fibre, aluminium, nylon only" ✔ (`HANDOFF.md:14-15`). Rod end 13° ✔ (`init_scan.m:26`); torque check, BOM, servo model: ✘ | ✘ |
| III-E electronics | R4 Minima vs WiFi ✔ (rationale ✔). D9–D11 ✔. 830–2170 µs clamp ✔. Separate USB ✔. **6 V "bench supply" ⚠** (repo: AA pack). E-stop: ✘. "Unreachable rejected, never clamped" ✔ behaviour, rationale not recorded | ◐ ⚠ |
| III-F firmware | Duration formula, 90°/s, smoothstep, 200 ms ✔ (`motion.cpp:71-81,255`); arm remap and upward approach ✘ at HEAD (only named in `JSON:52`, `HANDOFF.md:160-167`); backlash numbers ✘; from-scratch stack ✘ | ◐ |
| III-G calibration/correction | Eq. 6 pulse mapping ✔ (`motion.cpp:292-304`). Table VI values ✔ (`JSON:52`). β ✔ (`JSON:9-11`; **units = degrees** ✔). Model structure ✔. v1/v2/hybrid comparison, LOO errors ✘ [X]. Fig. 6 "12.8 mm at the corners" ⚠ see below | ◐ ⚠ |
| III-H scan software | Sequence, serpentine, park (0,0,−650), settle 5 s, 20 × 20 ms ✔; per-row flush ✔; NaN for rejected ✔; metadata ✔ (config, sensor serial/range, geometry, correction, done/aborted). **Not in code:** tilt/gyro capture, 1044 reconnection, move-length-scaled timeouts (fixed 30 s), firmware *version* (only a text string), 260-point unattended scans | ◐ ⚠ |
| III-I post-processing | Background subtraction ✔, layer maps ✔, 3D arrows ✔, repeatability ÷ gradient ✔. Python dipole fit ✘, Rodrigues tilt correction ✘ | ◐ |
| III-J sustainability | GPL attribution ✔ (`README.md:14-23`); masses, Wh: ✘ | ◐ |
| IV-A metrics table | Only map repeatability and sensor scatter have repo support | ◐ |
| IV-B joint calibration/hysteresis | ✘ (README: calibration procedure only) | ✘ |
| IV-C kinematic model | 27 tests ⚠ (35 [D]). Six 10° step ratios ✘ | ◐ ⚠ |
| IV-D position accuracy/drift | Numbers quoted only via `FieldScan/README.md:78-82` [X] | ◐ |
| IV-E in-situ check | **Data ✔ (four maps, alternating), numbers reproduced [D]:** k 1.13, r 0.93, 225 values, 279 nT residual (skeleton 280), run-to-run 234/243 nT (skeleton 230). Grid 5×5×3 ✔. Magnet 24 µT at (50,0,−675) ✔ (23.7–24.1 [D]), 6 µT at x = −50 (7.3 [D]). Analysis script is not in the repo | ✔ |
| IV-F dipole known-shift | ✘ | ✘ |
| IV-G field map / repeatability | 260-point map ✘; "five maps, 70 points, two layers, 350 points, 180 nT, 1.5 mm" ✘ — the repo's repeat data are 2 + 2 runs of 75 points, 3 layers (replica 135 nT / 1.16 mm). Table VII "5 × 70" and IV-H "75-point scan 13.8 min" are inconsistent with each other; 75 pts ✔ and 13.8 min ✔ (829 s) | ⚠ |
| IV-H measurement chain | Background 71.2–72.9 µT ✔ [D]. Settle 820 nT / 1700 nT ✔ [D]. Scan time 13.8 min ✔; "14 s/point" vs repo 11.1–11.5 s/pt (75-pt grid) ⚠. Median scatter 76 nT vs 70 nT [D] ⚠. Zeros "0.8% of readings … treated as missing" ⚠: not handled in code; data shows 9/225 zero windows (x-axis, no-magnet scan). Drift, tilt, Earth field 55.4 µT: ✘ | ◐ ⚠ |
| IV-I requirements | R2/R3 ✔ in code; others [X] | ◐ |
| IV-J/K/L critical analysis, limitations, sustainability | Limitations match repo "future work" (pot feedback `README.md:305-309`; box-limited validation) | ◐ |
| V Conclusion/Future work | "Map the cage": no coils-on data ✔ consistent | ✔ |

**Conflicts to resolve in the text:**
- **Fig. 6 "up to 12.8 mm at the corners".** True for the ±50 × ±50 scan grid (12.05–12.93 mm at z −675 [D]); but at the R2-box corners (±50, ±150) the model moves the target 17.7–26.6 mm [D]. Define "corner".
- **Sensor range.** The skeleton's `[TODO: range]` can be filled from the meta files: ±5.6 G. HANDOFF's ±8 G and the "≈ 2 G saturation" in `708ebf8` conflict with it.
- **Tests.** 27 (skeleton) vs 35 (my run).
- **Reach.** Quote one geometry consistently (Section 4.1).
- **Servo supply.** AA pack (repo and your own notes: "4×AA pack") vs bench supply (skeleton).
- **Hosting.** GitHub vs GitLab.

---

## 10. Superseded values

| Parameter | Earlier | Later | Evidence |
|---|---|---|---|
| Joint limits | Placeholder ±(−30, 80)° → single-arm −16…−25 / 26.5…29.5° | **−20/70°** (all arms) | `631bc99` msg; `README.md:75-85`; `b0576f3` |
| On-axis reach (HEAD geometry) | Planned z −450…−750 mm → ≈ −561…−692 mm (single-arm) | **−550…−793 mm** | `631bc99` msg; `41c19f1`; `b0576f3`; `README.md:99-106` |
| Reach (calibrated geometry) | – | −540…−788 mm [D], not in repo | `reachability.py` |
| SP / L_UP | 150 / 180 mm (`config.h:14-15`) | 75 / 177 mm (JSON `:30-31`) | HEAD firmware is *not* updated |
| Servo calibration | HEAD: 1460/11.8231, 1385/10.0481, 1410/10.5544 | 1456/9.8315, 1373/9.7664, 1393/9.6658 | `config.h:35-37` → `pose_correction_hybrid.json:52` |
| Arm mapping | Identity D9/D10/D11 | `GEOM_TO_PHYS [2,0,1]` | `config.h:18-20` → `JSON:33-37` |
| Forearm length | 600 mm (`init_scan.m:7`, "600 vs 700 unresolved") | 625 mm | `README.md:59-60`; `HANDOFF.md:141-143` |
| `ELBOW_JOINT_WIDTH` | 45 mm placeholder | 35 mm | `62bde4c` |
| Scan volume | `init_scan.m`: x ±100, y ±50, z −650…−550, 2541 pts | Default 125 pts (x,y ±50, z −700…−600) → data 75 pts (z −675…−625) → validated box x ±50, y ±150, z −700…−600 | `init_scan.m:10-16`; `field_scan.py:65-67`; meta; JSON `:13-26` |
| Settle time | 0.5 s (`scan_config.m:11`, first data) | 5.0 s | `caa2555`; `field_scan.py:63`. The MATLAB fallback still says 0.5 s |
| Speed encoding | JSON float | int | `ebd4e61` |
| Scan approach | MATLAB scan; delta-app program + `compile_scan.m` | Python `field_scan.py` | `c917370` → `ebd4e61` |
| `run_field_scan.m` arrival logic | Immediate `mv==0` return, 12 columns | `field_scan.py` waits for `mv` 1→0, 19 columns | `delta_move.m:27-30` vs `field_scan.py:196-227` |
| Metadata geometry | `robot_config.py` (older) | Calibrated JSON | `dfdcdc3` |
| Sensor range assumption | ±8 G (1044_1 spec) | ±5.6 G (device-reported) | `HANDOFF.md:26-29` → meta `:45-52` |
| Test count | 17 | 31 → 35 | `HANDOFF.md:98`; `8217320`; [D] |
| Correction branch status | "not merged" | merged (PRs #10, #11) | `HANDOFF.md:152` → `29e37e8`, `40e44bf` |
| MATLAB run status | "not run in MATLAB" | plot scripts run in Octave 8.4 only | `HANDOFF.md:69` → `83c4556` |
| Correction model | v1 centring | hybrid (v1 + mid-box x term) | `JSON:27` |
| MATLAB IK root | `ik_batch.m` uses the `−` root only | Firmware tries `+` then `−`. [D] same result at (0,0,−650): `+` root is 160.6°, outside limits | `ik_batch.m:38`; `kinematics.cpp:42-52` |

---

## Appendix A — scan data inventory (`FieldScan/data/`)

All 75-point scans: x,y ∈ {−50,−25,0,25,50}, z ∈ {−675,−650,−625}, speed 2, n_avg 20, sensor serial 302277, range ±5.6 G, coil current null. `t_end` is the last `t_s` in the CSV.

| File (prefix) | Note (from meta) | Settle | Corr. | Started → finished (local, 2026-10-02) | t_end | In snapshot |
|---|---|---|---|---|---|---|
| `2026-10-02_dryrun1_no_magnet` | no magnet, coils off | 5 | off | 17:22:21 → 17:36:12 | 831 s | ✔ |
| `…dryrun1_magnet_20261002_180000` | strong magnet, matches no-magnet | 5 | off | 18:00:00 → 18:13:49 | 829 s | ✔ |
| `…dryrun1_magnet_hybrid_…182828` | hybrid correction | 5 | hybrid | 18:29:41 → 18:44:02 | 860 s | ✔ |
| `…dryrun1_magnet_off2_…184415` | repeat | 5 | off | 18:44:15 → 18:58:05 | 830 s | ✔ |
| `…dryrun1_magnet_hybrid2_…185818` | hybrid repeat | 5 | hybrid | 18:58:18 → 19:12:40 | 862 s | ✔ |
| `mag_off_1_20261002_024004` | magnet fixed | 5 | off | 02:40:21 → 02:54:10 | 828 s | ✔ |
| `mag_off_1_20261002_031036` | magnet close (1.1 G at +50,0,−650); max raw \|B\| 186.5 µT [D] | 5 | off | 03:10:52 → 03:24:43 | 831 s | ✔ |
| `settle_0p5s/mag_off_1_…022713` | magnet fixed | 0.5 | off | 02:27:32 → 02:35:42 | 490 s | ✔ |
| `settle_0p5s/mag_off_1_…015453` | aborted after 3 pts | 0.5 | off | 01:54:53 → 01:55:09 | 15 s | – |
| `settle_0p5s/mag_off_2_…015519` | crashed: 0 pts done, no `finished` | 0.5 | off | 01:55:19 → – | 19 s (4 rows) | – |
| `magcheck_…015819`, `…020554` | 2 pts at (±50,0,−650) | 0.5 | off | 01:58 / 02:06 | – | – |
| `hybridcheck_…175834` | 2 pts at (50,0,−675/−650) | 5 | hybrid | 17:58:52 → 17:59:15 | 23 s | – |
| `dryrun_…014611` | 27 pts, x,y ±25 | 0.5 | hybrid | 01:46:38 → 01:48:37 | 119 s | – |
| `demo_…013031` | simulated sensor, 125 pts | 0 | hybrid | 01:30:32 → 01:30:45 | 13 s | – |

The meta `note` fields are the only record of intent. Nothing records the magnet type, orientation or placement, apart from "1.1 G at +50,0,−650" in one note. "Max raw \|B\|" in the `031036` row includes Earth and background field.

## Appendix B — derived-number scripts

`report_extract_scripts/` (all read the repo's own files; run with `PYTHONDONTWRITEBYTECODE=1 python <file>`; need `numpy`):

| Script | Produces |
|---|---|
| `reachability.py` | On-axis reach and reachable fraction for HEAD and calibrated geometry |
| `scan_stats.py` | Per-scan \|B\|, scatter, durations, sent−target offsets, vector differences |
| `compare_runs_replica.py` | Python re-implementation of `compare_runs.m`; exact-zero counts |
| `insitu_correction_check.py` | k, r, residual for the correction on/off check |
| `correction_magnitude.py` | Predicted shortfall and \|send−target\| from `pose_correction.py` |

`reachability.py` also prints the `zHome(0)` values (−608.2 / −597.2 mm), evaluating `kinematics.cpp:91-95` for the two geometries.
