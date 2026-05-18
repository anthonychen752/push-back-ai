# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

VEX V5 robot code for the VEX AI Push Back season (team 3177B). Targets the V5 Brain over USB with a Jetson Nano coprocessor providing vision/object detections via serial.

## Build & deploy

This is a standard VEX VS Code project. The repo's `Makefile` is a manual fallback (`arm-none-eabi-c++`, C++17, Cortex-A9, no RTTI/exceptions); the VEX VS Code extension is the normal build/upload path.

- Build via Makefile: `make` (artifacts under `bin/`); `make clean`; `make debug` produces an objdump listing.
- Build/upload via VS Code: use the VEX extension's Build/Download buttons. SDK version and slot are pinned in `.vscode/vex_project_settings.json` (slot 1, SDK `V5_20240802_15_00_00`).
- `.vscode/c_cpp_properties.json` is **gitignored** because include paths are user-specific — recreate it locally via the VEX extension if IntelliSense breaks. Don't commit it.
- No tests, no linter — VEX firmware projects don't run on the host.

## Architecture

The autonomous logic is a single-file state machine driven by odometry + pure pursuit, fed by Jetson detections.

- **`include/navigation.h`** is the brain. Header-only by design — all classes are defined inline so `main.cpp` is the only translation unit pulling them in. Contents:
  - `Odometry` — fuses tracking-wheel rotation (2" omni on `PORT20`) with inertial heading (`PORT7`). Heading comes from the IMU, not wheel diff.
  - `PID` — generic with anti-windup; gains live as `TURN_*` / `DRIVE_*` macros at the top of the file.
  - `PurePursuit` — curvature-based tank steering; speeds in in/s, converted to motor pct in `main.cpp` via `BASE_SPEED`.
  - `JetsonComms` — thin wrapper around `ai::jetson` exposing `update()` / `getDetection()`.
  - `StateMachine` — owns `odom`, `pp`, `jetson`, and the field-coordinate waypoints (`COLLECT_ZONES`, `SCORE_LEFT/RIGHT`, `PARK_POS`).
  - **Hardware ports, robot dimensions, and PID gains are `#define`s in this file** — change them here, not in `robot-config.cpp`.

- **`src/main.cpp`** runs the loop at 50 Hz (`DT = 0.02`). State flow: `SEARCH → COLLECT → DRIVE_TO_GOAL → SCORE → PARK`. Each iteration: update odometry, dispatch on state, render debug to `Brain.Screen`, sleep 20 ms. Driver control is a tank-drive placeholder.

- **`src/ai_jetson.cpp` + `include/ai_jetson.h`** are VEX's stock Jetson packet protocol (James Pearman, IFI MIT-licensed). Sync bytes `AA 55 CC 33`, length, type, CRC32, then `AI_RECORD` payload. A high-priority thread blocks on `getchar()` from `/dev/serial1` and feeds bytes into a parser state machine. **Do not modify** — treat as a vendor library.

- **`src/robot-config.cpp` + `include/robot-config.h`** declare hardware globals (motors, motor groups, sensors, `Brain`, `Competition`, `Controller1`). Drive is 6-motor tank (3 per side, `ratio6_1` / 450 RPM); right side reversed.

## Gotchas

- `robot-config.cpp` defines a `Flywheel` motor on `PORT6` that is **not** declared in `robot-config.h`. It's currently dead — either expose it via the header or remove it; don't reference it from new code without fixing the declaration.
- `robot-config.h` declares `extern ai::jetson jetson_comms;` but the definition in `robot-config.cpp` is commented out. The actual Jetson instance lives inside `JetsonComms` (`navigation.h`); the extern is unused. If you wire up a global Jetson, pick one path — don't have both.
- `main.cpp`'s `auto_Isolation` calls `g_state.jetson.getDetection(0, detection)` but `JetsonComms::getDetection` writes the **whole** `AI_RECORD` into a parameter declared as `AI_RECORD &` — the `index` arg is ignored. If you add per-detection logic, fix the wrapper rather than working around it.
- Coordinate units mix: `Odometry` is in **inches**, but `AI_RECORD` map locations from the Jetson are in **meters**. Convert at the boundary.
- `main()` registers competition callbacks then exits the `while (!Competition.isEnabled())` loop without doing anything — this is intentional (VEX runtime keeps the program alive via the competition callbacks), but don't "fix" it by adding a return.

## Inclusive language

Per global rules, prefer `primary/replica`, `allowlist/denylist`, `clear day/blocked day` over the legacy alternatives in any new code or comments.
