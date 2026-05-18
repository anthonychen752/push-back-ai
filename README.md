# 3177B — VEX AI Push Back

VEX V5 robot code for team **3177B**, targeting the **Push Back** season under the VEX AI ruleset. The robot runs an autonomous state machine driven by wheel-and-IMU odometry, pure-pursuit path following, and object detections streamed in from a Jetson Nano coprocessor over USB serial.

---

## Hardware

| Subsystem | Device | Port | Notes |
|---|---|---|---|
| Drive (left) | 3× V5 Smart Motor | `PORT1`, `PORT2`, `PORT3` | `ratio6_1` (450 RPM), forward |
| Drive (right) | 3× V5 Smart Motor | `PORT11`, `PORT12`, `PORT13` | `ratio6_1`, reversed |
| Tracking wheel | Rotation sensor on 2" omni | `PORT20` | Direct-drive, used for distance |
| Inertial | V5 Inertial Sensor | `PORT7` | Used for heading |
| Intake | V5 Smart Motor | `PORT4` | First-stage roller |
| Hood | V5 Smart Motor | `PORT5` | Scoring hood |
| Flywheel | V5 Smart Motor | `PORT6` | Defined in config but currently unused — see `BUGFIXES.md` |
| Jetson Nano | USB CDC (`/dev/serial1`) | — | Vision coprocessor; sends `AI_RECORD` packets |

Drivetrain assumed: 6-motor tank, ~12 in track width, 3.25 in drive wheels. Tune in `include/navigation.h`.

---

## Building & deploying

This is a standard VEX VS Code project (SDK pinned in `.vscode/vex_project_settings.json` to `V5_20240802_15_00_00`, slot 1).

### Recommended: VEX VS Code extension
Use the extension's **Build** and **Download** buttons. The extension regenerates `.vscode/c_cpp_properties.json` for IntelliSense — that file is **gitignored** because the include paths are user-specific. Don't commit it.

### Manual: Makefile
Provided as a fallback for command-line builds. Uses `arm-none-eabi-c++`, C++17, Cortex-A9, no RTTI/exceptions.

```bash
make            # build → bin/push-back.elf
make clean      # remove bin/
make debug      # objdump → bin/push-back.lst
```

There is no host test suite — VEX firmware can't run off the V5.

---

## Code layout

```
include/
  ai_jetson.h        Vendor protocol — DETECTION_OBJECT, AI_RECORD, ai::jetson class
  navigation.h       Header-only library: Odometry, PID, PurePursuit, JetsonComms, StateMachine
  robot-config.h     Hardware globals (motors, sensors, Brain, Competition, Controller1)
  vex.h              Convenience header pulling in V5 SDK
src/
  ai_jetson.cpp      Vendor — packet parser, CRC32, receive thread (do not modify)
  main.cpp           50 Hz autonomous loop, driver-control placeholder, main()
  robot-config.cpp   Motor / sensor instantiation
Makefile             Manual build
.vscode/             VEX project config (c_cpp_properties.json gitignored)
CLAUDE.md            Notes for AI coding assistants
BUGFIXES.md          Audit of known bugs and dead code
```

---

## Architecture

### Control loop (`src/main.cpp`)

Runs at 50 Hz (`DT = 0.02 s`). Every tick:

1. Read tracking wheel rotation (degrees) and inertial heading.
2. Update odometry (`StateMachine::odom.update`).
3. Dispatch on the current state.
4. Render debug telemetry to `Brain.Screen`.
5. `wait(20, msec)`.

State machine: **`SEARCH → COLLECT → DRIVE_TO_GOAL → SCORE → PARK`**.

| State | Behavior |
|---|---|
| `SEARCH` | Pick the nearest of three `COLLECT_ZONES`. Pure-pursuit drive there. On arrival → `COLLECT`. |
| `COLLECT` | Run intake at 70%, creep forward at 10%. Increment a counter on every Jetson detection. After `COLLECT_TARGET` (=3) → `DRIVE_TO_GOAL`. ⚠️ See `BUGFIXES.md` #7. |
| `DRIVE_TO_GOAL` | Pure-pursuit to the closer of `SCORE_LEFT` / `SCORE_RIGHT`. On arrival → `SCORE`. |
| `SCORE` | Back up 0.5 s, run intake at 100% for 2 s to dump balls → `PARK`. |
| `PARK` | Pure-pursuit to `PARK_POS`, stop, wait 5 s, return. |

Field waypoints (`COLLECT_ZONES`, `SCORE_LEFT`, etc.) are seeded in `StateMachine`'s constructor at `navigation.h:471`. Update them for the actual field layout.

### Odometry (`navigation.h`, `Odometry`)

Fuses two sources:
- **Heading** comes directly from `Inertial.heading(degrees)`. The IMU is the source of truth for orientation; tracking-wheel differential heading is not used.
- **Distance** comes from a single 2" omni tracking wheel via `Δdegrees × π·diameter / 360`.

Each update integrates `dist · cos/sin(theta)` into `(x, y)`. Coordinates are **inches**.

### PID (`navigation.h`, `PID`)

Generic position-form PID with anti-windup (integral clamp scales with `1/Ki`). Used for in-place turns (`turnToHeading` in `main.cpp`) and for heading correction inside pure pursuit. Default gains live as `TURN_*` and `DRIVE_*` macros at the top of `navigation.h` — **tune for your robot.**

### Pure pursuit (`navigation.h`, `PurePursuit`)

Curvature-based path follower for a tank drive:

1. From the loaded path, find the first waypoint at least `LOOKAHEAD_DISTANCE` (12 in) away.
2. Compute curvature `κ = 2·sin(α) / L` where `α` is the bearing to that waypoint in robot frame, `L` is straight-line distance.
3. Split the average speed across left/right wheels: `v_L = v·(1 + κ·trackHalf)`, `v_R = v·(1 − κ·trackHalf)`.
4. Slow to 40% on `Waypoint::slow == true` and to 30% within 6 in of the target.

`computeWheelVels` returns velocities in inches/sec; the caller in `main.cpp` converts to motor percent via `BASE_SPEED`.

### Jetson comms (`ai_jetson.cpp` + `JetsonComms` in `navigation.h`)

The vendor `ai::jetson` class implements VEX's standard protocol:

- A high-priority thread blocks on `getchar()` from `/dev/serial1`.
- Bytes feed a state-machine parser: sync (`AA 55 CC 33`) → length (2 B LE) → type (2 B LE) → CRC32 (4 B LE) → payload.
- On a good packet of type `MAP_PACKET_TYPE` (0x0001), the `AI_RECORD` is copied (mutex-guarded) into `last_map`.
- `get_data()` snapshots `last_map` into a caller-supplied buffer.

`JetsonComms` wraps `ai::jetson` with `init() / update() / getDetection()` for use inside the state machine.

`AI_RECORD` (from `ai_jetson.h`):
- `pos`: GPS-frame robot pose (meters, degrees) — populated by the Jetson, **independent of `Odometry`**.
- `detections[]`: up to 50 `DETECTION_OBJECT`s. Each has class ID, probability, depth, screen-space bbox, and field-coordinate `mapLocation` **in meters**.

> ⚠️ **Unit boundary:** Odometry is in inches; Jetson coordinates are in meters. Convert at the boundary if you mix them.

### Driver control (`usercontrol`)

Placeholder tank drive on `Axis3`/`Axis2` with `R1` for intake and `R2` for hood. Extend as needed.

---

## Tuning checklist

Before a competition run:

1. **Hardware** — confirm motor ports, gear ratios, and reversal flags in `src/robot-config.cpp` match the actual robot.
2. **Robot dimensions** — `DRIVE_WHEEL_DIAMETER`, `TRACKING_WHEEL_DIAMETER`, `TRACK_WIDTH` in `navigation.h`.
3. **PID gains** — `TURN_KP/KI/KD`, `DRIVE_KP/KI/KD`. Tune turn-in-place first, then translation.
4. **Pure pursuit** — `LOOKAHEAD_DISTANCE` (12 in default; smaller = tighter, more oscillation). `BASE_SPEED` (24 in/s).
5. **Field waypoints** — `COLLECT_ZONES`, `SCORE_LEFT/RIGHT`, `PARK_POS` in `StateMachine`'s constructor.
6. **Calibration** — `Inertial.calibrate()` runs in `calibrate()` at the start of autonomous; the robot must be still during this period.

---

## Known issues

See **`BUGFIXES.md`** for a full audit. Highlights:

- `JetsonComms::getDetection` ignores its index arg (one-line fix).
- `PurePursuit::driveTo` runs a PID whose output is never applied.
- `request_map()` is never called — Jetson data may be stale depending on coprocessor firmware.
- `COLLECT` increments its counter every loop tick a detection exists, not per-ball.
- `main()` falls off the end without an idle loop.

---

## License

Vendor code in `ai_jetson.{h,cpp}` is © Innovation First, MIT-licensed. The rest of the project has no license declared — treat as all-rights-reserved by team 3177B unless updated.
