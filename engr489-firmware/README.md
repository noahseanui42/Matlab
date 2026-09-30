# engr489-firmware

Servo firmware and GUI for the ENGR489 delta robot (VUW capstone, supervised
by Dr Christopher Hollit): a magnetometer-probe delta robot that maps the
magnetic field inside a Helmholtz coil. This replaces the original stepper
firmware from `grzesiek2201/Delta-Robot` with firmware for 3 hobby servos on
an Arduino UNO R4 Minima, while keeping the **exact same serial protocol**,
so the upstream Python GUI drives this robot with only the patches described
below.

v1 is **open loop** and runs outside the coil cage. Closed loop (pot
feedback) is future work — see [Out of scope](#out-of-scope-future-work).

## Licence and attribution

The original firmware and GUI (`grzesiek2201/Delta-Robot`, commit
`48a8038`, by Jakub Wrona and Grzegorz Radziwiłko) are **GPL-3.0**; the old
stepper firmware's header additionally says "FOR NON-COMMERCIAL USE ONLY".
`COPYING` is kept alongside every vendored/derived file
(`delta_servo/COPYING`, `delta_app/COPYING`), and every ported or derived
file carries a `Derived from grzesiek2201/Delta-Robot (GPL-3.0) @ 48a8038`
header. **This repository is public, so the GPL-3.0 terms apply to anyone
who can see it.**

`third_party/ArduinoJson` vendors ArduinoJson v7.2.1 (MIT licence, see
`third_party/ArduinoJson/LICENSE.txt`), used by `protocol.cpp` and by the
host tests.

## Repo layout

```
engr489-firmware/
├── delta_servo/          Arduino sketch (folder name = sketch name)
│   ├── delta_servo.ino    Arduino glue only: setup/loop, Servo, Serial, millis
│   ├── config.h           geometry, pins, calibration, limits, speeds
│   ├── kinematics.h/.cpp  IK + home-z. No Arduino headers (host-testable)
│   ├── protocol.h/.cpp    frame parser + dispatch. No Arduino headers
│   ├── motion.h/.cpp      planner, program runner. No Arduino headers
│   └── COPYING
├── delta_app/             upstream DeltaApp/src, patched
│   ├── robot_config.py    single source for Python geometry/limits
│   ├── ... upstream files ...
│   └── COPYING
├── third_party/ArduinoJson  vendored (MIT) for host tests + firmware build
├── tests/                 host-side tests (pytest + g++)
└── README.md
```

`kinematics`, `protocol` and `motion` compile with plain `g++` (no Arduino
headers) so they're unit-testable on the host; only `delta_servo.ino` may
touch `Servo`, `Serial` or `millis()`.

## Architecture

```
┌───────────────────────── Mac (Python, delta_app/) ─────────────────────────┐
│                                                                            │
│  main.py ──► deltagui.py (Tkinter GUI)                                     │
│               • jog / go-to-point / program list / wait times              │
│               • start, stop, enable, port field                            │
│               • reads status stream → 3D plot, angle readouts              │
│                     │                          ▲                           │
│                     ▼                          │                           │
│             deltarobot.py  (IK / FPK, workspace checks)                    │
│                     ▲                                                      │
│                     │ SB, SP, L_UP, L_LO, ANGLE_LIMITS_DEG, Z_MAX,         │
│              robot_config.py   TCP_DEFAULT, SERIAL_PORT_DEFAULT, DTR       │
└─────────────────────┬──────────────────────────▲───────────────────────────┘
                      │ USB CDC ("115200")       │ status line every 20 ms
                      │ <mode><{json}><#> frames │ (paused while rxBusy):
                      │ 2 move, 1 program point, │ {"deg":[..],"mv","run",
                      │ 3 wait, 0 start, 8 enable│  "en","e"}  + "OK" replies
┌─────────────────────▼──────────────────────────┴───────────────────────────┐
│        Arduino UNO R4 Minima  (delta_servo/)                               │
│                                                                            │
│  delta_servo.ino  loop():                                                  │
│    1. drain Serial ──► protocol.cpp (frame parser)                         │
│                          callbacks: onMove, onProgramPoint, onFunc(wait),  │
│                          onStart, onEnable, onJsonError                    │
│                               │                                            │
│                               ▼                                            │
│         motion.cpp  start of each move: ik(target) → θ1 (fail → e=1)       │
│           (manual move: on receipt; program point: when it's reached)      │
│                                                                            │
│    2. every TICK_MS (20 ms): motion.tick()                                 │
│         DISABLED / IDLE → MOVING → DWELL → next program point              │
│         smoothstep s(u) = 3u² − 2u³                                        │
│           joint mode:  θ = θ0 + s·(θ1 − θ0)        (no IK per tick)         │
│           linear mode: p = xyz0 + s·(xyz1 − xyz0) → ik(p) every tick       │
│                        (IK fails mid-path → stop, e=2)                     │
│         kinematics.cpp: closed-form IK, CAL[i].min_deg/max_deg limits      │
│       then writeServos():                                                  │
│         angleToUs: us = centre_us + dir·us_per_deg·θ                       │
│                    clamped to US_MIN..US_MAX = 830–2170 µs (hit → e=5)     │
│         Servo.writeMicroseconds ──► D9 / D10 / D11 (only when enabled)     │
│                                                                            │
│    3. every STREAM_MS (20 ms), unless rxBusy: printStatusLine()            │
│                                                                            │
│  config.h: SB/SP/L_UP/L_LO, CAL[] per-servo table, US_MIN/MAX, timing,     │
│            speeds (kept in sync with robot_config.py by test_config_sync)  │
└────────────────────────────────────────────────────────────────────────────┘
                      │ PWM pulses
                      ▼
        servo 1 (D9)   servo 2 (D10)   servo 3 (D11)
                      │
          delta arms → effector plate → probe (TCP 21 mm below)
```

The firmware works in effector-centre coordinates only; the GUI applies the
TCP offset before sending a move.

## Assumptions made (HANDOFF §4 answers)

These were confirmed with the user before implementation:

1. **Geometry convention:** `SB`/`SP` are equilateral-triangle **side**
   lengths through the joint centres (not centre-to-axis radii). `SB` = 175 mm
   (bicep pivots, 50.5 mm inradius, measured); `SP` = **75 mm**, the inner
   triangle through the forearm ball joints on the 150 mm platform (43.3 mm
   circumradius), not the 150 mm outline.
2. **Forearm length:** **625 mm** (not the 600/700 mm the planning doc had
   flagged as unresolved).
3. **Servo-to-arm mapping:** D9 = arm 1 (on the −y axis), D10 = arm 2,
   D11 = arm 3, arms 1→2→3 CCW viewed from above. **Verify this at bring-up
   step 4** — swapped pins/`dir` show up as an arm that doesn't match the
   GUI's 3D plot when jogging.
4. **Probe TCP offset:** `(0, 0, −21)` mm — centred, 21 mm below the
   effector's ball-joint-axis centre. Lives in `delta_app/robot_config.py`;
   the firmware itself has no concept of TCP (see protocol notes below).
5. **GUI host:** macOS, confirmed during bring-up. `robot_config.SERIAL_PORT_DEFAULT`
   is set to `/dev/cu.usbmodem14101` and `SERIAL_DTR = True` (the R4's native
   USB CDC needed DTR asserted before any data reached the host — see the
   [macOS bring-up fixes](#macos-bring-up-fixes-found-during-testing) below).
   Re-check the port string if you plug into a different USB port/cable.
6. **Angle limits are now real, calibrated values**, in two passes:

   - **Pass 1 (single-arm, 2026-09-26):** each arm driven individually, rest
     of the linkage resting flat, gave per-arm limits around −16° to
     −25°/26.5° to 29.5°. **This turned out to be falsely tight** — driving
     one arm while the other two are slack lets the effector plate sag/tilt
     out of level, creating an early collision with the (unsupported) plate
     that doesn't happen in real coordinated operation.
   - **Pass 2 (full 3-arm assembly, confirmed same day):** all three servos
     enabled and jogged together, plate properly held level throughout —
     the real usable range turned out to be **−20° to 70°** on all three
     arms, described as a safe margin rather than the absolute mechanical
     stop (there may be more room past this).

   | Arm | Pin | centre_us | us_per_deg | dir | min_deg | max_deg |
   |---|---|---|---|---|---|---|
   | 1 | D9 | 1460 | 11.8231 | +1 | −20.0 | 70.0 |
   | 2 | D10 | 1350\* | 9.51\* | +1 | −20.0 | 70.0 |
   | 3 | D11 | 1410 | 10.5544 | +1 | −20.0 | 70.0 |

   \* Arm 2 re-swept 2026-09-29 with the ruler method (was 1385 / 10.0481);
   data and conversion in `tools/servo_sweep/servo2_angles.xlsx`.

   `centre_us`/`us_per_deg`/`dir` are still per-arm (from the single-arm
   pass — that part of the measurement isn't affected by plate sag); only
   `min_deg`/`max_deg` came from the full-assembly recheck.

### Reachability finding

With the corrected full-assembly limits (−20° to 70°), the reachable
envelope is **much closer to the project's z ≈ −450 to −750 mm scan
target** than either earlier estimate. On-axis (x=y=0), the workspace now
spans roughly **z≈−540mm to z≈−788mm** — covering the entire deep half of
the target range and then some, though still about 100mm short at the
shallow end (−450 to −550mm remains unreachable; θ=−20° bottoms out around
z=−550mm). Off-axis, a broad grid (x,y∈[−150,150]mm, z∈[−790,−550]mm, 25mm
steps) has ~89% of points reachable (1501/1690).

If the shallow 100mm matters for the coil measurement plan, it's worth
re-probing whether −20° really is the safe floor or was itself set with
some margin to spare — otherwise, treat −540..−788mm as the real scan
volume for this build. `tests/test_kinematics.py`'s regression values and
grid are scoped to this range.

### Pen-holder end effector (positioning accuracy test)

For bench testing positioning accuracy without the coil, the magnetometer
probe is swapped for a 3D-printed pen holder that clamps a whiteboard
marker, so a scan can mark each commanded point on a sheet of paper and the
marks can be measured against the intended grid.

- The holder's clamp bore is deliberately oversized for the marker
  currently on hand, so a second printed adapter/insert can take up the
  slack if the marker is swapped later — the bore itself doesn't need to
  change between pens.
- **TCP offset not yet updated for this tool.** `TCP_DEFAULT = (0, 0, −21)`
  mm in `delta_app/robot_config.py` (HANDOFF §4 Q4, above) is the probe's
  tip offset below the effector's ball-joint-axis centre, not the pen's.
  The pen holder almost certainly has a different tip offset, and using the
  probe's TCP with the pen will shift every marked point on the paper by
  the difference between the two. Before trusting the marking test's
  results, measure the pen holder's own offset (ball-joint-axis centre down
  to the marker tip, with the marker seated as it will be for the test) and
  either swap in a pen-specific TCP constant or update `TCP_DEFAULT` for
  the duration of the test.
- Since this runs on the bench outside the coil, the pen holder isn't bound
  by the project's non-magnetic-material rule for parts used inside the
  coil — normal fasteners are fine here.
- **Base plate orientation found rotated 120° from the IK model's
  assumption.** Running the 9-point marking test with the reference paper
  edge aligned to physical Arm 1 produced marks rotated a clean 120° from
  their targets (arms are 120° apart, so this is a discrete mismatch, not a
  calibration drift) — physical Arm 3 (D11) turned out to be the one
  actually sitting at the position `kinematics.cpp`'s closed-form IK calls
  "-y axis" (index 0), not physical Arm 1 (D9). The base plate's extra
  mounting-hole options (built in for flexibility, same reasoning as the
  pen holder's oversized bore above) meant the arms were bolted on walked
  120° around from the layout `ik()`'s formulas assume.

  Fixed in software rather than by re-bolting the plate or re-wiring pins:
  `delta_servo/config.h` and `delta_app/robot_config.py` each define
  `GEOM_TO_PHYS = [2, 0, 1]`, mapping IK's geometric slot 0/1/2 (-y axis,
  +120°, +240°) to the physical arm/pin that's actually there. `ik()`
  (`kinematics.cpp`) and the GUI's `calculateIPK`/`calculateFPK`
  (`deltarobot.py`) all apply this mapping consistently, so a physical
  servo's calibration (`CAL[]`/`ANGLE_LIMITS_DEG`, measured per unit) stays
  indexed by its own pin regardless of which geometric role it plays — only
  the geometry-to-pin correspondence changed, nothing was recalibrated.
  `tests/test_config_sync.py` enforces the two `GEOM_TO_PHYS` arrays staying
  identical. If the base plate is ever physically re-bolted to match the
  model's original assumption, this should revert to `[0, 1, 2]`.

## Serial protocol (must match the GUI byte-for-byte)

Every token is wrapped in `<` and `>`; a one-character **mode** token is
optionally followed by a JSON payload token, and `<#>` ends a transaction.
115200 baud is configured but the R4's USB CDC ignores it.

| Mode | Meaning | Firmware action | Reply |
|---|---|---|---|
| `2` | Manual move / jog / Home | Plan a move to `c` (effector centre, mm) | none |
| `1` | Upload program point `n` | Store point; `program_len = n+1` | `OK` (no newline) |
| `3` | Wait time (ms) after point `pt_no` | Store dwell | `OK` |
| `4` | Wait for input | Parsed and stored; not acted on in v1 | `OK` |
| `5` | Set output | Parsed and stored; not acted on in v1, **no pin is ever written** | `OK` |
| `0` | Start/stop the program | Start/stop per `{"start": bool}` | none |
| `8` | Enable/disable | `enable:0` = ENABLE, `1` = DISABLE (inverted, from the stepper active-low convention) | none |
| `6` | "Calibrate" button | Ignored | none |
| `7` | Gripper | Ignored, no pin written | none |
| `9` | SD interpolation | Ignored | none |

Firmware streams one status line every 20ms, always — including while
disabled and at boot, since the GUI only ever sends a command right after
parsing a `deg` line:

```
{"deg":[t1,t2,t3],"mv":0,"run":0,"en":0,"e":0}
```

`e` (0-5: ok / unreachable / linear-path-left-workspace / command-not-allowed
/ JSON-parse-error / µs-clamp-hit) is a single shared latch that clears the
moment the **next** move is accepted, wherever it was set (protocol-level
JSON errors and motion-level move errors share one field, per the GUI's
expectations — see `Motion::setError()`).

Streaming pauses whenever a message is mid-transit (`rxBusy()`, cleared on
`<#>` or a 1000ms idle timeout) so a status line can never land inside a
program upload's `OK` response window; motion keeps running underneath.
Nothing but `OK` and status lines ever goes to `Serial`.

## R4 Servo library finding (checked against source, not assumed)

The handoff asked whether the R4's Servo library (`ArduinoCore-renesas`,
`src/renesas/Servo.cpp`) honours `writeMicroseconds()` called *before*
`attach()`. Checked directly against that source:

- `writeMicroseconds()` no-ops until `attach()` has assigned a servo slot
  (`servoIndex`), so calling it before `attach()` **has no effect at all**.
- `attach()` itself unconditionally calls `writeMicroseconds(DEFAULT_PULSE_WIDTH)`
  (1500µs) as part of claiming the slot, before returning.

So `delta_servo.ino`'s `onEnable()` calls `attach()` **then**
`writeMicroseconds()` immediately after, to overwrite that default as fast
as possible. There's a sub-millisecond window at the 1500µs default, but
that's inside the 830-2170µs safe range, so it isn't a hazard — just
expect a very brief settle before the commanded pose takes hold.

## Enable/disable and "snap to flat"

`Motion`'s constructor sets θ=(0,0,0), xyz=(0,0,zHome(0)) as the boot
default; `enable()`/`disable()` never modify commanded θ/xyz themselves —
only `state()`. So: the very first enable after power-up "snaps to flat"
only because that's what the boot default happens to be, and every later
disable→enable cycle snaps back to whatever was last commanded (not to
flat), because θ/xyz were never touched while disabled. Both behaviours
described in HANDOFF §6.4 fall out of this one rule.

## macOS bring-up fixes found during testing

None of these were anticipated in the original plan — found live while
actually running the GUI on the macOS host (§4 Q5):

- **No data arrived until `SERIAL_DTR = True`.** The R4's native USB CDC
  needed DTR asserted before the OS-level connection was actually "live",
  even though the sketch itself never checks `Serial`/DTR. Symptom: the GUI
  connected with no error, but every `readline()` came back empty
  (`JSONDecodeError('Expecting value: line 1 column 1 (char 0)')` spamming
  the terminal).
- **Every label was invisible (white-on-white) in macOS Dark Mode.** Classic
  `tk.Label`/`tk.Button` widgets that set `bg="White"` without an explicit
  `fg` inherit the system text colour, which resolves to white in Dark
  Mode. Fixed with `root.option_add('*Foreground', 'black')` /
  `option_add('*Background', 'white')` right after creating the root window.
- **"Program" and "Available COMs" menu items didn't exist on macOS at
  all** (not even greyed out) — they were bare top-level
  `tk.Menu.add_command()` calls; macOS's native menu bar only renders
  cascade (dropdown) menus, silently dropping bare top-level commands.
  Moved into the "File" cascade. The Program Creator popup's own "Open" /
  "Save" / "Save as" had the identical bug and got the identical fix.
- **The window (1100x700, later 1500x850) clipped on a 1440x900 laptop
  screen**, and macOS's "Enter Full Screen" doesn't resize classic Tk
  windows (it just moves the same-sized window into its own Space), so
  fullscreening didn't help. Fixed two ways together: size the window to
  `min(1500, screen_w-60) x min(850, screen_h-100)` at launch instead of a
  fixed constant, and shrink the 3D plot from 6x6in to 4.8x4.8in so the
  layout's actual content fits a 900px-tall screen in the first place.

## Calibration procedure

`config.h`/`robot_config.py` currently hold real values measured on
2026-09-26 (see the table under [Assumptions made](#assumptions-made-handoff-4-answers)),
gathered with a separate host-side calibration tool (`python -m
host.calibrate`, writing `trim_us`/`sign`/`us_per_deg`/limits — not part of
this repo). **`centre_us`/`us_per_deg`/`dir` came from driving each arm
individually; `min_deg`/`max_deg` came from a second pass with all three
arms enabled and jogged together** — do the angle-limit part with the full
assembly, not a single arm, or you'll get a falsely tight number (see the
reachability note above for why). To recalibrate, or to use the
originally-planned `servo_calibration_v3.ino` instead, for each servo in
turn on D9/D10/D11:

1. **Centre** — the µs value where the bicep is level → `config.h`'s
   `CAL[i].centre_us`.
2. **`us_per_deg` and `dir`** — a linear fit through 5 points (e.g. 1200,
   1360, 1520, 1680, 1840 µs), each measured with an angle finder on the
   bicep.
3. **Angle limits — with all three arms enabled and linked together**,
   plate held level, jog toward each extreme and note the real interference
   point (the bicep rising into the plate slot, or the lower limit) — not a
   single arm with the others slack.
4. Enter all values in **both** `delta_servo/config.h`'s `CAL[i]` and
   `delta_app/robot_config.py`'s `ANGLE_LIMITS_DEG` — `tests/test_config_sync.py`
   fails the build if they disagree. Angle limits = the tighter of the
   mechanical limits and ±(650/`us_per_deg`).
5. Re-run `pytest tests/` after updating both files, and recheck the
   reachable z-range (see the reachability finding above) with the real
   numbers in place.

## Hardware bring-up checklist

**The user runs this; it can't be done from here.**

1. Servo power **OFF**, USB connected. Flash `delta_servo`. Open the GUI →
   Connect → Online. The angle label should update. If nothing arrives, set
   `robot_config.SERIAL_DTR = True`.
2. Still with servo power off: Enable motors, then Move to (0, 0, −600) at
   10%. The 3D plot should animate there. Upload a 3-point program with a
   1000ms wait — there must be no "Program did not upload correctly". Start
   it; the plot should follow.
3. Calibrate each servo (see above).
4. Reflash `delta_servo`. Support the arms near flat, **then** switch servo
   power on and click Enable. Set speed to 10%, press Home, then jog ±15mm
   in x, y and z. Compare the real pose with the GUI's 3D plot — an arm
   that doesn't match means the wrong pin order or the wrong `dir`.
5. Sweep z down to −750, then out to the x/y edges. Use Joint mode for
   large moves; Linear moves can stop early near the workspace edge (`e=2`).
6. If the stream stops or the board resets while the servos move, suspect
   the 4×AA supply sagging.

## Tests

```
pip install pytest
python3 -m pytest tests/ -v
```

Covers (HANDOFF §9): Python kinematics (FK/IK round trip + on-axis
regression), C++↔Python IK parity (via `tests/ik_cli`), the protocol frame
parser (10 cases incl. numeric-string coercion, oversized-frame handling,
the 1000ms `rxBusy` timeout), the motion planner/program runner (11 cases
incl. smoothstep, the T-duration formula, pending-slot "newest wins",
dwell, stop, and the µs-clamp), and config.h/robot_config.py sync.

Two notable, documented findings from writing these tests (not bugs
introduced by this port):

- `deltarobot.py`'s `calculateFPK()` nudges z by ±0.01mm whenever two
  computed elbow heights coincide (its own pre-existing div-by-zero guard)
  — this measurably biases FK output (up to ~0.17mm, growing with depth) for
  any point with two equal joint angles (e.g. every point on the x=0
  plane). `IK` itself has no such nudge, so real runtime accuracy is
  unaffected; the FK/IK round-trip test tolerance is loosened to 0.2mm to
  account for it rather than hide it in a tighter "passing" number.
- Near configurations where `kinematics.cpp`'s IK denominator `(G-E)` is
  small relative to `E`/`G`'s own magnitude, the solve is ill-conditioned:
  float32 (firmware) and float64 (`deltarobot.py`) round to meaningfully
  different angles there even though each is internally consistent. This
  is a property of the shared closed-form solution, not a porting bug; the
  parity test skips comparison at those specific points rather than
  loosening its tolerance enough to mask an unrelated mismatch elsewhere.

**`arduino-cli compile` was not run against the real target.** This
container's network allows `github.com` and the Go module proxy (used to
build `arduino-cli` itself from source), but blocks `downloads.arduino.cc`,
which is where the `arduino:renesas_uno` board package and library index
are served — `arduino-cli core update-index` fails with 403 there. As a
substitute, `delta_servo.ino` was compiled (0 warnings, `-Wall -Wextra`)
against a minimal hand-written `Arduino.h`/`Servo.h` stand-in exposing just
`Serial`, `millis()` and `Servo::attach/writeMicroseconds/detach`, to catch
API-usage mistakes; this is **not** a substitute for a real
`arduino-cli compile --fqbn arduino:renesas_uno:minima delta_servo` (or an
Arduino IDE build), which the user should run before flashing.

## Out of scope (future work)

- Pot feedback on A0-A2 streamed as measured `deg` (needs a C++ FK port so
  xyz stays known after closed-loop corrections).
- Closed-loop proportional pulse trim per servo.
- A scan script: grid in coil coordinates → mode-`2` moves → wait for
  `mv==0` → dwell → read the magnetometer → CSV (needs the base-to-coil
  transform).
- Mode `5` (set output) as a magnetometer trigger.
- Reading calibration from EEPROM instead of `config.h`.
- A `tools/sim_robot` pty-bridge simulator (HANDOFF §11, optional stretch)
  so the GUI can be exercised with no hardware attached — not built here.
