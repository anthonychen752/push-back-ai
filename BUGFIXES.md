# Bug Fixes

Audit of `src/` and `include/` as of 2026-05-16. Issues are grouped by severity and ordered by likely impact.

## Status summary

| # | Issue | Severity | Status |
|---|---|---|---|
| 1 | Dangling `extern jetson_comms` | Build/link | ✅ Fixed |
| 2 | `main()` falls off the end | Build/link (UB) | ✅ Fixed |
| 3 | Undefined `autonomousMain` | Build/link | ⚠️ Investigated — left in place |
| 4 | `JetsonComms::getDetection` ignores index | Logic | ✅ Fixed |
| 5 | `driveTo` dead PID | Logic | ✅ Fixed |
| 6 | `request_map()` never called | Logic | ✅ Fixed |
| 7 | `COLLECT` counter race | Logic | ✅ Fixed |
| 8 | Meters vs. inches unit boundary | Robustness | ✅ Helper added |
| 9 | `computeCurvature` sign flip | Robustness | ✅ Fixed |
| 10 | No wheel-velocity clamp | Robustness | ✅ Fixed |
| 11 | Heading convention split | Robustness | ⚠️ Documented — left in place |
| 12 | Undeclared `Flywheel` motor | Cleanup | ✅ Declared |
| 13 | Unused includes | Cleanup | ✅ Fixed |
| 14 | Misleading `BASE_SPEED` comment | Cleanup | ✅ Fixed |

The two remaining ⚠️ items were intentionally not changed — see notes on each issue below.

---

## Build / link issues

### 1. Dangling `extern ai::jetson jetson_comms`
- **Where:** `include/robot-config.h:17` (declaration), `src/robot-config.cpp:19` (definition commented out)
- **Problem:** The header promises a global `jetson_comms` symbol that nothing defines. Builds today only because no translation unit references it; the moment something does, the link fails with an undefined symbol. The actual Jetson instance lives inside `JetsonComms` in `navigation.h`, so the global is redundant.
- **Fix:** Delete the `extern` from `robot-config.h` and the commented-out line in `robot-config.cpp`.
- ✅ **Resolution:** Removed both lines. `JetsonComms` (instantiated inside `StateMachine`) is now the only path to Jetson data.

### 2. `main()` falls off the end without `return`
- **Where:** `src/main.cpp:310-327`
- **Problem:** After the `while (!Competition.isEnabled())` loop, `main` returns implicitly. On a freestanding ARM target that's UB, and per the VEX template `main` is supposed to keep the program alive after registering callbacks.
- **Fix:** Replace the wait-for-enable loop with the standard idle loop:
  ```cpp
  while (true) { wait(100, msec); }
  ```
- ✅ **Resolution:** `main()` now calls `vexcodeInit()` first, registers competition callbacks, then enters `while (true) { wait(100, msec); }`. Hardware init is no longer skipped.

### 3. Undefined `autonomousMain`
- **Where:** `include/vex.h:22` declares `extern int autonomousMain(void);`
- **Problem:** No definition exists. If any V5 SDK runtime path calls it, the link fails. If nothing calls it, the declaration is dead.
- **Fix:** Verify whether the SDK references it; otherwise remove the declaration.
- ⚠️ **Resolution:** Left in place. `vex.h` is part of the VEX project template — the V5 SDK runtime may reference `autonomousMain` from outside this codebase. Removing the extern would risk a link error inside the closed-source toolchain. Revisit only if the build complains.

---

## Logic bugs (compile, misbehave)

### 4. `JetsonComms::getDetection` ignores its `index` argument
- **Where:** `include/navigation.h:422`
- **Problem:** Signature suggests "give me detection N", but the body copies the entire `AI_RECORD` (header + every detection) into `out`. Caller in `main.cpp:166` passes `0` and gets back the whole record, then later reads `detection.detections[0]` — which works by accident.
- **Fix:** Pick one API and commit to it:
  ```cpp
  // Option A — return a single detection
  bool getDetection(int index, DETECTION_OBJECT &out) {
      if (index < 0 || index >= _latest.detectionCount) return false;
      out = _latest.detections[index];
      return true;
  }
  // Option B — rename
  bool getRecord(AI_RECORD &out) { ... }
  ```
- ✅ **Resolution:** Did **both**. `getRecord(AI_RECORD&)` returns the full snapshot for callers that want pose + all detections; `getDetection(int, DETECTION_OBJECT&)` now correctly indexes `_latest.detections[index]`. Also added `frameCnt()` for de-duping detections across loop ticks. `main.cpp` uses `getRecord()` since it needs both pose and detection list.

### 5. `PurePursuit::driveTo` computes wheel velocities but they're never applied
- **Where:** `include/navigation.h:345-377`, `src/main.cpp:143-153`
- **Problem:** `driveTo` runs a PID, computes `leftV`/`rightV`, stashes them in `_lastLeftV/_lastRightV` — and `main.cpp` then calls `computeWheelVels` (which uses curvature, not the PID) and ignores the stashed values. The PID branch inside `driveTo` is dead code.
- **Fix:** Either delete the velocity computation in `driveTo` (it should only return arrival status), or have the caller use `getLastLeftV/RightV()`. The cleaner option is the former — let `driveTo` be a pure "are we there yet?" check and let `computeWheelVels` own steering.
- ✅ **Resolution:** Reduced `driveTo` to a one-liner: `return distTo(targetX, targetY) < tolerance;`. Removed `_lastLeftV/_lastRightV` and the never-used `getLastLeftV/RightV()` accessors. `computeWheelVels` is now the single source of steering.

### 6. `request_map()` is never called
- **Where:** `src/ai_jetson.cpp:292`, `include/navigation.h:404` (`JetsonComms::update`)
- **Problem:** The VEX Jetson protocol is request/response — V5 sends `"AA55CC3301\r\n"` to `/dev/serial1` and the Jetson replies. `JetsonComms::update()` just reads `get_data()` without ever requesting fresh data. Depending on Jetson-side firmware this may or may not push autonomously; the documented contract is poll-driven.
- **Fix:** Call `_jetson.request_map()` inside `JetsonComms::update()`, rate-limited to ~10–20 Hz so we don't spam the serial link:
  ```cpp
  int update() {
      if (!_connected) return 0;
      static uint32_t lastReq = 0;
      uint32_t now = vex::timer::system();
      if (now - lastReq > 50) {  // 20 Hz
          _jetson.request_map();
          lastReq = now;
      }
      ...
  }
  ```
- ✅ **Resolution:** Implemented as suggested, but using a per-instance `_lastReqMs` field instead of `static` so multiple `JetsonComms` instances would each rate-limit independently. `auto_Isolation` now calls `g_state.jetson.update()` every loop iteration (50 Hz), and `update()` itself polls `request_map()` at 20 Hz.

### 7. Collect counter races every loop tick
- **Where:** `src/main.cpp:157-177`
- **Problem:** `collectCount++` runs every iteration the camera sees *any* object. At 50 Hz loop and ~10 Hz detection rate, `COLLECT_TARGET = 3` triggers in ~60 ms — long before a ball is actually intaken. The state machine then jumps straight to `DRIVE_TO_GOAL` with an empty intake.
- **Fix:** Gate on something physical:
  - A new `framecnt` from `AI_RECORD::pos.framecnt` since the last increment, **and**
  - An intake-current spike or a separate ball-detection sensor confirming a ball was actually loaded.
- ✅ **Resolution:** Gated on `framecnt` change via the new `JetsonComms::frameCnt()`. Snapshots the current frame on entry to `COLLECT` (so the first arriving frame is fresh) and only increments `collectCount` when both `record.detectionCount > 0` and `frame != lastCountedFrame`. Also added a `COLLECT_TIMEOUT_MS = 4000` safety so the state can't stall forever if the Jetson goes silent. **Note:** still vision-only — adding an intake-current or limit-switch confirmation is a follow-up when that hardware is wired up.

---

## Smaller logic / API issues

### 8. Unit mismatch: Jetson meters vs. odom inches
- **Where:** `include/ai_jetson.h:54` (Jetson `MAP_DETECTION` is meters), `include/navigation.h` (Odometry is inches)
- **Problem:** Currently only the debug print mixes them, with the label "m" — readable, no functional bug. But any future code that drives toward a Jetson-detected object will be off by 39.37×.
- **Fix:** Convert at the boundary. Add a helper:
  ```cpp
  constexpr double M_TO_IN = 39.3700787;
  ```
  Apply it whenever a Jetson coordinate enters odom-frame logic.
- ✅ **Resolution:** Added `#define METERS_TO_INCHES 39.3700787` next to the math constants in `navigation.h`. No call sites currently consume Jetson coordinates as nav targets, so no conversions were inserted yet — the helper is in place for when that logic is added.

### 9. `computeCurvature` redundant sign manipulation
- **Where:** `include/navigation.h:277-300`
- **Problem:** Computes `kappa = 2*sin(alpha)/L` (correct sign already), then overwrites the sign using a separately recomputed `relAngle`. The double calc loses precision near the ±180° boundary.
- **Fix:** Trust the math:
  ```cpp
  double alpha = atan2(dy, dx) - robotTheta;
  // normalize alpha to -PI..PI in radians directly, no degree round-trip
  while (alpha >  PI) alpha -= TWO_PI;
  while (alpha < -PI) alpha += TWO_PI;
  return 2.0 * sin(alpha) / L;
  ```
- ✅ **Resolution:** Applied verbatim. `computeCurvature` is now ~10 lines shorter and avoids the deg → rad → deg round-trip.

### 10. Wheel-velocity-to-percent has no clamp
- **Where:** `src/main.cpp:153, 207`
- **Problem:** `setDriveVel(lv / BASE_SPEED * 100.0, ...)` can exceed ±100 when `kappa * trackHalf > 0` adds to the average speed. The motor silently clamps, but the asymmetry between left and right gets distorted (e.g., requested 110/40 → applied 100/40, sharper turn than intended).
- **Fix:** Clamp the pair while preserving the ratio:
  ```cpp
  double scale = std::max(1.0, std::max(fabs(lvPct), fabs(rvPct)) / 100.0);
  lvPct /= scale; rvPct /= scale;
  ```
- ✅ **Resolution:** Pushed the clamp into `setDriveVel` itself so every caller (autonomous, driver control, in-place turns) benefits without per-site changes:
  ```cpp
  double maxMag = fmax(fabs(leftPct), fabs(rightPct));
  if (maxMag > 100.0) {
      double scale = 100.0 / maxMag;
      leftPct *= scale;
      rightPct *= scale;
  }
  ```

### 11. Heading convention split (0..360 vs -180..180)
- **Where:** `include/navigation.h:100` stores raw `Inertial.heading()` (0..360); `src/main.cpp:51 normalizeAngle` assumes -180..180; pure-pursuit's `normalizeAngle` handles either.
- **Problem:** No bug today because every consumer normalizes its own deltas, but a future caller doing arithmetic on `_odom.theta()` directly will get burned at the 0/360 wrap.
- **Fix:** Pick one convention and document it on `Odometry::theta()`. Recommend -180..180 — normalize once in `update()`.
- ⚠️ **Resolution:** Left in place. Switching to -180..180 would change the meaning of every consumer's `_odom.theta()` reads (e.g., `turnToHeading` targets), and the existing PID gains were tuned against 0..360 angle differences. Documented this as a known sharp edge in CLAUDE.md instead. Revisit only if a new caller actually trips it.

---

## Cosmetic / dead code

### 12. Undeclared `Flywheel` motor
- **Where:** `src/robot-config.cpp:34`
- **Problem:** `motor Flywheel(PORT6, ...)` is defined but not declared in `robot-config.h`, so nothing can use it. Dead device init that still consumes a port assignment.
- **Fix:** Either expose it via the header (if you're going to use it) or delete the line.
- ✅ **Resolution:** Added `extern motor Flywheel;` to `robot-config.h`. Kept the definition since the team may be wiring up a flywheel mechanism this season — easier to consume from `usercontrol`/auton now without reopening the header.

### 13. Unused includes
- **Where:** `include/navigation.h:14`
- **Problem:** `<vector>` and `<algorithm>` aren't used.
- **Fix:** Drop them.
- ✅ **Resolution:** Removed `<vector>` and `<algorithm>`. Added `<cstring>` for `memset` (used in the new `JetsonComms::init()` and `auto_Isolation()` zero-initialization of `record`).

### 14. Misleading `BASE_SPEED` comment
- **Where:** `include/navigation.h:43`
- **Problem:** Comment says "inches/sec max" but the macro is used as a unitless scaling factor in `main.cpp:153` (`lv / BASE_SPEED * 100.0`).
- **Fix:** Either make it actually represent in/s and convert properly with a wheel-circumference factor, or rename to `MAX_DRIVE_PCT_AT_24INS` and update the comment.
- ✅ **Resolution:** Updated the comment to "scaling factor; treated as the in/s value that maps to 100% motor". Did **not** rename — the constant is referenced from multiple call sites and the rename would touch them all without changing behavior.

---

## Fix log (2026-05-16)

### Files touched

| File | What changed |
|---|---|
| `include/robot-config.h` | Removed dangling `extern jetson_comms`. Added `extern motor Flywheel` and `void vexcodeInit(void)` declarations. |
| `src/robot-config.cpp` | Removed the commented-out `jetson_comms` line. |
| `include/navigation.h` | Dropped `<vector>`/`<algorithm>`, added `<cstring>`. New `METERS_TO_INCHES` constant. Simplified `computeCurvature`. Reduced `driveTo` to an arrival check (removed dead PID, `_lastLeftV/_lastRightV`, accessors). `JetsonComms` now polls `request_map()` at 20 Hz, exposes both `getRecord()` and a working `getDetection(int, DETECTION_OBJECT&)`, and adds `frameCnt()`. Updated misleading `BASE_SPEED` comment. |
| `src/main.cpp` | Added `<cstring>` include. `setDriveVel` now clamps proportionally. Auton loop calls `g_state.jetson.update()` and `getRecord()` each tick. `COLLECT` state gates on `frameCnt()` change with a 4 s timeout. `main()` now calls `vexcodeInit()` and runs an infinite idle loop. |
| `BUGFIXES.md` | This file — added status table and per-issue Resolution notes. |

### Files NOT touched

- `include/ai_jetson.h`, `src/ai_jetson.cpp` — vendor code (Innovation First, MIT). Treated as a black box.
- `include/vex.h` — see issue #3.
- `Makefile`, `.vscode/*` — no changes needed.

### Verification

- ❌ **Local build not run.** The ARM cross-compiler (`arm-none-eabi-c++`) is not installed on the dev machine, and the V5 SDK headers (`v5.h`, `v5_vcs.h`) ship with the VEX VS Code extension's toolchain rather than this repo. Build the project through the VEX VS Code extension on a machine that has it; if anything breaks, paste the error output and we'll iterate.
- ✅ **Static checks done in-tree:** no remaining references to removed APIs (`getLastLeftV`, `getLastRightV`, the old `getDetection(int, AI_RECORD&)`, `jetson_comms`); all new identifiers have matching declarations; includes are minimal but sufficient.

### Open follow-ups

- Issue #3 (`autonomousMain` extern) — leave alone unless the build complains.
- Issue #11 (heading convention) — leave alone unless a new caller trips it.
- Issue #7 follow-up — add intake-current or a limit-switch confirmation in `COLLECT` once that hardware is wired, to back up the vision-only frame-count gate.
- `turnToHeading` is defined but never called; consider deleting if no auton path needs it.
- The `// VEXCODE DEVICE CONFIG` banner in `main.cpp:18-24` is now slightly misleading (it talks about redeclaration that no longer happens). Cosmetic; leaving for now.
