# Java/C++ parity audit — 2026-10-21

Compared Java origin/main `4e3422f` with C++ feature baseline `453ecba`.
C++ main `6692e65` is already an ancestor; no merge was needed. Existing C++
working changes were preserved in `7baeb80`, `3d8be18`, `55b5717`, and `f1da8be`.
Work is on `audit/java-parity-2026-10-21`.

## Selective parity matrix

| Concern | Result | Integrated commits |
| --- | --- | --- |
| Mechanisms | Opt-in Phoenix velocity, position, Motion Magic and coherent rotor/remote CANcoder feedback; software defaults, safety and existing WPILib SysId retained. | `dd8d512` |
| Localization | Queued 5 ms QuestNav protocol receiver, mount/alignment/stale/restart gates and raw-packet replay. | `d30cfbb`, `9ca054b`, `fe85813` |
| Camera vision | Configurable tag count/ID gates and covariance; MT2 ambiguity exemption retained; invalid/future timestamps rejected. | `de59cd8`, `8652b4c` |
| Piece detection | Full mounted 3D ray/plane projection, fresh/no-target behavior, fixed cached field locations, injectable simulation and raw-ray replay. | `4ffecfe`, `53475e4`, `ec4be9b`, `60ab392` |
| Drive/runtime | Validation before devices; native translation SysId with measured SI data, health/age gates and stop cleanup. Native swerve timing retained. | `787a799`, `63b88e9`, `7c0935c`, `1e7f91d` |
| Models/tuning | WPILib motor-model feedforward and atomic disabled gain transactions; explicit Test-owned setpoint command, cleanup and raw transaction replay. | `5b48944`, `3c50a1e` |
| Simulation power | Once-per-period plant aggregation, supply-side terminal power, shared bus sag/recovery, reset synchronization and canonical async telemetry. | `9c1ce6f`, `c8a023a` |
| Wheel calibration | Bounded raw-gyro/coupling-corrected wheel-radius command with per-module geometry, validity checks, stop cleanup and replay. No automatic configuration changes. | `15193eb`, `c3fedda` |
| C++ advances | Single-owner Jolt physics, approximate 3D field contacts, separate low-priority NT/frame workers, typed target/measured swerve telemetry, versioned code replay and provenance retained. | preservation commits above |

Usage and model contracts: [mechanisms](mechanism-control.md),
[tuning/feedforward](mechanism-tuning.md), [QuestNav](questnav.md),
[camera filtering](vision-configuration.md), [pieces](piece-detection.md),
[drive SysId](runtime-parity.md), [power](simulation-power.md),
[wheel radius](wheel-radius-characterization.md), [replay](replay.md).

## Integrated validation

Production code was built from clean revision `c3fedda`, source SHA256
`6184c5c53630464aa79a8f1b6c8622acc932cf1e6f4f463b03dd3b5b0d399399`.
The following documentation/evidence commit does not alter production code.

| Check | Result |
| --- | --- |
| `./gradlew build prepareHeadlessSimulation --max-workers=2` | Passed; Linux desktop and roboRIO Debug/Release compiled and linked; 52 native tests passed in each configuration. |
| Portable CMake `BUILD_TESTING=ON`, build and CTest | All four suites passed in Debug and Release: world, drivetrain, arena and approximate 2026 field. |
| Python script tests | 3 passed; smoke script syntax check passed. |
| Owned actual executable: `sim_headless.py --backend jolt-planar` / `jolt-3d` | 34 checks passed in each mode: target/measured vectors, both spin directions, reversal, stopping, truth/estimate movement, reset, intake/launch, finite power, sag and recovery. |
| Actual executable code replay | 1,018 frames completed; source identity, input values and timestamps preserved; no failed/rejected recorder frames. Requested drive outputs matched. |
| Release physics benchmark | Six scenes passed the 4 ms p99 budget; highest p99 2.327 ms, zero measured misses. Hardware/compiler/body/contact details retained in JSON. No MapleSim comparison. |

Replay reports 611 differing `Drive/ModuleTargets` frames. Live targets come from
native Phoenix; replay uses the documented WPILib kinematic approximation and does
not execute Phoenix's internal optimization/firmware. These differences are
visible evidence, not silently treated as equal. All other recorded output keys
matched in this run. Raw QuestNav, piece-ray and tuning transactions also have
focused replay fixtures; the base practice container instantiates drive only.

Evidence: [build/test summary](../artifacts/parity-2026-10-21/validation.json),
[build tasks](../artifacts/parity-2026-10-21/build-summary.txt),
[planar smoke](../artifacts/parity-2026-10-21/smoke-planar.json),
[3D smoke](../artifacts/parity-2026-10-21/smoke-3d.json),
[replay comparison](../artifacts/parity-2026-10-21/replay.json), and
[benchmark](../artifacts/parity-2026-10-21/benchmark.json).
Full local console logs are `/tmp/parity-full-build.log`,
`/tmp/parity-portable-{debug,release}.log`, `/tmp/parity-smoke-{planar,3d}.log`
and `/tmp/parity-replay.log`. Smoke summaries retain checks, target/measured
outputs and power ranges; full trajectories are in the corresponding `/tmp`
JSON files. The source WPILOG remains in the ignored local `logs/` directory;
its SHA256 is recorded in the replay comparison.

CI's `runFrcUserProgramTest…GoogleTestExe` tasks were independently confirmed to
execute GoogleTest and propagate failure. `testExternalNativeDebug` instead
prepares external-launch metadata and is not test execution.
## Intentional differences and limits

- Phoenix owns native swerve control/odometry (250 Hz CAN FD, 100 Hz CAN 2.0 by the pinned default). The default rio bus is CAN 2.0; the application scheduler remains 50 Hz. Optional Java Spark/NavX drivetrains are not added.
- Native Phoenix mechanisms are opt-in. Simulation uses software control and explicit plants. Replay recomputes native requests, not motor firmware voltage or native odometry; those remain recorded inputs. Remote CANcoder feedback is supported; licensed fused/synchronized modes are not selected.
- Existing position commands retain active holding after completion. Java's automatic coast/zero compliance can release gravity-loaded joints and is deliberately not the default. Intentional compliance belongs in an explicit downstream command.
- Native PhotonPoseEstimator, PathPlanner and Phoenix replace Java custom trig solvers, controller/setpoint copies, and custom AD* logging. WPILib interpolation with C++ units needs no project-specific wrapper. WPILib SysId replaces the Java ad hoc feedforward regression.
- Piece localization uses pose at observation consumption; no capture-time pose-history compensation, object classifier, camera image synthesis or occlusion simulator is added. QuestNav supplies local alignment, not remote headset reset/ack management; physical protocol commissioning remains necessary.
- The battery model is illustrative (12 V / 20 mΩ), omits accessories, wiring topology and regenerative charging, and treats drivetrain friction as a lumped voltage loss. Winding-current limits differ from real configured supply-current limits. Approximate field contacts, suspension and mechanism parameters require measured tuning.
- Java URCL and the browser system-check UI are not duplicated. Typed canonical AdvantageScope data and owned Test-mode tuning commands are the supported workflow.
- Full robot builds/practice target Linux and Windows; Mac support remains standalone Jolt models/tests/benchmarks. This audit executes Linux locally; it does not establish fresh Windows/macOS CI results, hardware calibration, physical field acceptance or deterministic Phoenix timing.
- C++ frame logs support code replay; Java AdvantageKit logs are not interchangeable. Frame recorder overflow/failure remains explicit, and outputs retain source/build provenance. Low-priority snapshot/recording workers remain separate from control; no claim is made about the priority of vendor or WPILib internal threads.

## Review process

Independent planners, implementation agents and reviewers handled mechanisms,
vision/localization, drivetrain/runtime, tuning/models, battery power and wheel
radius. Implementations used isolated worktrees and were integrated by cherry-pick.
Review findings fixed stale cached piece projection, SysId feedback health, and
friction loss in battery accounting. A fresh completeness audit identified the
wheel-radius utility; a separate integration audit checked the combined changes.
No merge, push, deployment or hardware motion was performed. The Java original
retains its pre-existing executable-bit change and was not edited.
