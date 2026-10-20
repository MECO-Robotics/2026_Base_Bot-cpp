# Framework audit — 2026-10-19

Repository: `MECO-Robotics/2026_Base_Bot-cpp`.
Branch: `audit/framework-2026-10-19`.
Baseline: `a2e8a6e0d04cd354ddce0ec12b2b89486aa3fa4d` (also fetched `origin/main`).
An existing edit to `.agents/skills/frc-base-project/SKILL.md` was preserved and excluded from commits.

## Audit process and assessment

Three independent audit roles examined mechanisms/hardware IO, drive/vision/characterization, and architecture/build/testing. Findings were consolidated and challenged before implementation. Different reviewers inspected patches, followed by additional cross-layer audit passes. Review rejected speculative changes to motor inversion, shared mutable Phoenix requests, and position-command completion ordering after checking the actual contracts and pinned implementation.

Final source assessment: no further confirmed actionable defect remained in the reviewed source after the follow-up passes. This is a scoped clean source audit, not production certification or proof that the entire target architecture is implemented. Hardware, full replay, future platform execution, and real-time latency remain unverified as described below. GCC analyzer diagnostics remain a tool-analysis limitation, not a demonstrated source defect.

## Findings and resolutions

| Priority | Finding | Resolution / evidence |
|---|---|---|
| High | Nonfinite external encoder throws before stopping previous motor output; computed control overflow can saturate into a command | Validate feedback before calibration, stop on nonfinite computed voltage; original-failure regressions |
| High | Disconnected feedback permits stale closed-loop/open-loop output and false completion | Require healthy snapshots, stop on loss, prevent false `AtSetpoint`, require renewed control; preserve dashboard health updates |
| High | Reset during invalid feedback permanently corrupts position offset | Reject invalid reset before mutating reference; recovery regression |
| High | Spark later successful telemetry calls mask failed encoder reads | Capture per-thread REV error immediately after every accessor; pinned API review and both-target compilation; physical fault injection remains untested |
| Medium | MT1 accepts missing/negative single-tag ambiguity, while MT2 is incorrectly rejected by raw-tag ambiguity | Solver-specific ambiguity gate; local NetworkTables original-failure tests |
| Medium | Photon/LL malformed distances can emit nonfinite covariance | Reject nonfinite/nonpositive scales and invalid Photon ambiguity/timestamp; actual serialized Photon NT regression |
| Medium | Profile snaps moving state to zero at a close goal, violating acceleration/feedforward bound | Permit unavoidable overshoot; only snap when previous velocity can stop within one timestep; bounded-crossing regression |
| Medium | PID history carries velocity error into position control; position-only dynamic acceleration leaks into velocity control | Reset history on feedback-mode entry and restore configured velocity acceleration; original-failure regressions |
| Medium | Fixed-step explicit Euler motor model reverses numerically at valid high gearing | Exact piecewise scalar integration retaining current clamp/coast; high-ratio and startup-bound regressions plus independent RK4 comparison |
| Medium | Teleop continuous chassis speeds go into Phoenix request without discretization | WPILib discretization at existing 20 ms operator period; numerical SE(2) validation; autonomous path unchanged |
| Low | Talon reads refresh five signals twice and copy cached signal objects | Retain getter refreshes, remove duplicate native refreshes, use motor-owned const references; pinned lifetime/status review |
| Validation | Command ownership/cancellation/hold contracts lack scheduler tests | Four scheduler cases cover voltage/velocity interruption and disable, position hold, SysId lifetime and cancellation |
| Compatibility | Gradle defaults to C++20 | Select C++23 through platform defaults; verify eight generated option files and desktop/roboRIO builds; preserve pinned vendor versions |

## Build, test, simulation and analysis evidence

- `./gradlew build --max-workers=2`: desktop and roboRIO debug/release compile/link pass. Native suites pass all 25 cases in both configurations. `cpp23-build.log` captures the full C++23 rebuild; `build.log` captures the final signal-reference change and revalidation. Some unchanged tasks are cached; these logs together record fresh execution in both configurations.
- `regression-original-summary.json`: recorded original failures for the meaningful numerical/safety regressions; final suites cover their corrected behavior. Some individual red/green runs occurred while unrelated test work was in progress; final combined builds resolve that transient state.
- `compiler-options.json`: eight production/test source option files each contain exactly `-std=c++23`, with no trailing C++20 override. Host GCC 15.2 and roboRIO GCC 12.1 support this mode; GCC 12 provides partial C++23/library support. Windows/macOS builds were not run.
- `api-verification.json`: season-2026 API checker found zero errors/warnings in changed C++ files. Coverage is partial; Phoenix and PathPlanner index coverage is absent. Pinned installed headers and actual compilation remain authoritative.
- `sim-smoke.json`: initial desktop HAL/NT smoke passes all nine checks. Final runtime evidence is in `sim-smoke-final.json` (forward 2.048894 m/s, reverse −2.048987 m/s, angular 3.176113 rad/s; final disabled speeds zero); both are simulation-only, with processes stopped afterward. Poll counts are reads, not guaranteed unique NT updates. HAL/NT development servers bind wildcard addresses; the driver connects to loopback.
- `gcc-analyzer-debug.log`, `gcc-analyzer-release.log`: targeted GCC 15 `-fanalyzer` on mechanism code completes with three allocator diagnostics at O0 and three callback-copy diagnostics at O2. Independent reduction in `gcc-analyzer-reproducer.cpp` reproduces the callback warning without mechanism state/control logic. These probable library-model false positives were not suppressed or used to justify production changes; static analysis is not claimed warning-free.

## Numerical and performance analysis

- Original NEO model at gear ratio 200, output inertia .025 kg m², and 4 V has Euler `dt * decay ≈ 6.999 > 2`; a 20 s run produced negative speed instead of the positive no-load equilibrium (~.157667 rotations/s). The stable model converges in the correct direction and obeys startup current/position bounds. Test tolerances protect those observable contracts.
- `python3 artifacts/audit-2026-10-19/validate_motor_model.py`: independent 1 µs RK4 comparison over 60 fixed-seed cases spans ratios 1–200, inertia .005–.5, 1–4 motors, 10–100 A limits, positive/negative initial velocities and voltages. Maximum error: `8.1706724e-8 rad/s` velocity, `4.91978e-9 rad` position. This validates a Python transcription against an independent ODE integrator; native C++ regressions separately exercise production code.
- The analytical step uses one or two closed-form intervals instead of 20 Euler substeps. This establishes bounded work for this model, not a measured speedup. Python kernel timing in `motor-model-validation.json` is explicitly not native robot timing.
- `python3 artifacts/audit-2026-10-19/run-drive-discretization.py`: 2 m/s forward plus 3 rad/s rotation over 20 ms gives 1.19964 mm lateral error without correction. WPILib's discrete speeds integrate to the requested 40 mm forward and zero lateral displacement within numerical precision. See the standalone C++ source and output.
- Removing redundant refreshes changes ten Phoenix native signal-refresh operations to five per mechanism read: 500 to 250 operations/s at 50 Hz. This does not imply CAN traffic or latency halves. Const references also avoid copying signal strings/callbacks/maps; actual allocation savings are unmeasured.
- Autonomous feedforward requests copy two four-element force vectors per submission (at least 100 vector-buffer allocations/s and 3.2 KB/s payload at 50 Hz). No latency evidence justifies replacing safe move-owned requests with shared mutable state across the vendor thread.
- `native-control-timing.json`: the actual release `ClosedLoopVelocityConverges` test repeated 1,000 times passes 500,000 control updates in 0.541781338 s, averaging 1.08356 µs/update including process/test/HAL/DS/NT overhead. This is an amortized desktop workload, not a roboRIO worst-case or real-time deadline guarantee.

## Remaining risks and external validation

1. The application scheduler, mechanisms, and simulation remain 50 Hz / 20 ms. Phoenix native swerve defaults to 250 Hz on CAN FD and 100 Hz on CAN 2.0 (`rio`). No 4 ms application or hardware latency guarantee is established; raising rates requires coordinated timestep changes and target/bus timing measurements.
2. Replay is injected mechanism snapshots, not whole-robot deterministic log playback. The default drivetrain still uses Phoenix hardware-backed native simulation; vendor-independent drivetrain replay is absent.
3. Systemcore execution requires a supported target toolchain, vendor adapters, and hardware validation. No 2027 API migration or speculative vendor substitution was made.
4. The motor model excludes gravity loading, contact, traction, battery sag, and real device timing. Its clamp is winding current; real Talon configuration limits supply current. Strict electrical/current-limit parity is not established.
5. Real CAN faults, inversion, current limiting, physical soft limits, sensor calibration, and gain tuning require commissioned hardware. Base hardware constants remain illustrative.
6. Compiler/API/static-analysis checks are incomplete by nature. The probable GCC analyzer false positives are retained for future toolchain investigation. No memory/race sanitizer or Windows/macOS execution was claimed.

No hardware deployment, physical motion, force-push, or merge occurred.

## Commit inventory

`commits.txt` lists all implementation/build commits with full SHAs, in order. This report and evidence are committed separately afterward.
