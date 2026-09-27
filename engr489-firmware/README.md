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

## Assumptions made (HANDOFF §4 answers)

These were confirmed with the user before implementation:

1. **Geometry convention:** `SB`/`SP` are equilateral-triangle **side**
   lengths through the joint centres (not centre-to-axis radii).
2. **Forearm length:** **625 mm** (not the 600/700 mm the planning doc had
   flagged as unresolved).
3. **Servo-to-arm mapping:** D9 = arm 1 (on the −y axis), D10 = arm 2,
   D11 = arm 3, arms 1→2→3 CCW viewed from above. **Verify this at bring-up
   step 4** — swapped pins/`dir` show up as an arm that doesn't match the
   GUI's 3D plot when jogging.
4. **Probe TCP offset:** `(0, 0, −21)` mm — centred, 21 mm below the
   effector's ball-joint-axis centre. Lives in `delta_app/robot_config.py`;
   the firmware itself has no concept of TCP (see protocol notes below).
5. **GUI host** (OS / Python version / serial port) wasn't known yet.
   `robot_config.SERIAL_PORT_DEFAULT` is left as `"COM3"` as a placeholder —
   **update it** once the host is known. `deltagui.py`'s `iconbitmap()`
   calls are wrapped in `try/except tk.TclError` so the GUI still launches
   on macOS/Linux, which don't support `.ico` window icons.
6. Angle limits (`config.h`'s `CAL[i].min_deg/max_deg` and
   `robot_config.ANGLE_LIMITS_DEG`) are still **`TODO(calibrate)`
   placeholders** (`-30°..80°`) — see [Calibration](#calibration-procedure).

### Reachability finding — flagged loudly

**With the confirmed geometry (L_LO=625mm) and placeholder ±(-30°..80°)
joint limits, the project's stated scan target of z ≈ −450 to −750 mm is
only partly reachable.** On-axis (x=y=0), the workspace bottoms out at
z=−750mm (θ≈46.8°) but tops out around **z≈−530mm** (θ≈−27.3°, close to the
-30° limit) — **z=−450mm is not reachable** with these joint limits; it
would need the bicep angled further up than −30° allows. This will change
once real mechanical limits are measured (§10.3) and entered into `config.h`
and `robot_config.py` — recheck the reachable envelope after calibration,
before relying on the full −450..−750 mm scan range.

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

## Calibration procedure

Use the existing `servo_calibration_v3.ino` (untouched) for each servo in
turn, on D9/D10/D11:

1. **Centre** — the µs value where the bicep is level → `config.h`'s
   `CAL[i].centre_us`.
2. **`us_per_deg` and `dir`** — a linear fit through 5 points (e.g. 1200,
   1360, 1520, 1680, 1840 µs), each measured with an angle finder on the
   bicep.
3. **Mechanical min/max** — the bicep rising into the plate slot, and the
   lower limit.
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
  — this measurably biases FK output (~0.02-0.08mm) for any point with two
  equal joint angles (e.g. every point on the x=0 plane). `IK` itself has
  no such nudge, so real runtime accuracy is unaffected; the FK/IK
  round-trip test tolerance is loosened to 0.1mm to account for it rather
  than hide it in a tighter "passing" number.
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
