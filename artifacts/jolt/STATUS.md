# Jolt implementation evidence

**Accepted:** source `c890889c59551131a1427791b51b9d5793641f6f` passed all six Windows/Linux CI jobs in [run 39472708656](https://github.com/MECO-Robotics/2026_Base_Bot-cpp/actions/runs/39472708656). Separate module planning, implementation, and review assignments are complete.

Native macOS model support was subsequently added; see [Mac validation and performance limits](macos/STATUS.md).

## Baseline

- Reviewed the seven pre-existing modified C++ files: identical ordered lexical tokens; formatting only.
- Preserved them in commit `4f2049b`.
- `./gradlew build --max-workers=2` passed: desktop and roboRIO debug/release builds; 25 native tests passed in each configuration. See `baseline-build.log`.
- Merged the audit branch into main at `6692e655b10309eb0e927a864c3c16f6b464725e`, pushed main, and created `feature/jolt-simulation`.

## Implementation contract

Initial required platforms were **Windows and Linux**, per the user's scope clarification. The later native macOS model extension is recorded in `macos/STATUS.md`. Keep WPILib 2026.2.1 and vendor versions pinned. Jolt starts at 5.3.0, commit `0373ec0dd762e4bc2f6acdb08371ee84fa23c6db`.

Modules proceed through separate scoped planning, implementation, and reviewer/test agents. The coordinator owns shared contracts and integration. Module A's dependency/world gate must pass on both platforms before dependent implementation.

| Module | Planning | Implementation | Review/test |
|---|---|---|---|
| A: build/world | complete | complete | accepted |
| B: drivetrain | complete | complete | accepted |
| C: mechanism loads/timing | complete | complete | accepted |
| D: arena/pieces | complete | complete | accepted |
| E: practice/evidence | complete | complete | accepted |

## Module A evidence

- Windows/Linux Debug/Release configure, compilation, and world tests passed in [run 39457548403](https://github.com/MECO-Robotics/2026_Base_Bot-cpp/actions/runs/39457548403).
- Expanded lifecycle regressions passed on both platforms/configurations at `f4d47c8` in [run 39457773724](https://github.com/MECO-Robotics/2026_Base_Bot-cpp/actions/runs/39457773724).
- Initial Windows failure (unqualified integer type) was corrected before dependent implementation.
- Independent reviewer verified planar wall collision, callback lifetime, and clearing pending force/torque on reset. The reset reproducer changed residual velocity from 0.4 m/s to zero.
- Local Linux release Gradle executable link passed. `roborio-task-isolation.log` records the roboRIO link dry-run with no CMake dependency; final cross-target build remains required after integration.

## Integrated numerical and runtime evidence

- Separate planning, implementation, and reviewer/test assignments were used for each module (`a_planner`/`a_implementation`/`a_review`, and equivalently B–E).
- Independent B review verified the repaired loaded motor current bound against torque balance. Independent D review verified landing/rolling resistance, intake ownership, invalid configuration, and constructor rollback. Independent C review ran all 21 mechanism/motor checks successfully.
- `FRC_SIM_PHYSICS=jolt-planar CMAKE_COMMAND=/tmp/frc-jolt-tools/bin/cmake ./gradlew build --max-workers=2` passed: all 30 native tests in Debug and Release, including the real Phoenix mixed-inversion/offset/coupling feedback test; desktop and roboRIO builds passed. The temporary CMake path is local tooling, not a repository requirement.
- Actual owned desktop executable tests passed in planar and full 3D modes (`smoke-planar.json`, `smoke-3d.json`): positive/negative spin, reversal, stopping, heading/rate integration, truth/estimate telemetry, structured poses, capture, launch, disabled-held intake cleanup, and reset.
- `smoke-planar-initial.json` preserves the first failure: steady translation medians incorrectly included acceleration samples. Time-based steady windows fixed the check without changing physics, gains, or tolerances.
- The headless launcher rejects an existing listener and checks its own random run identity before sending controls. Checking listeners rather than binding avoids false failures from recently closed TCP connections.
- The first Phoenix diagnostic smoke exposed a stale desktop executable missing the new clock heartbeat. After the stable-source build, `smoke-phoenix.json` passed all diagnostic checks.
- Final `./gradlew build --max-workers=2` passed with the default planar backend: 30 tests per native configuration and successful desktop/roboRIO builds. `final-native-build.log` captures the latest full build after the rolling-resistance optimization, configuration-readback startup fixture, and COM launch correction: all 30 tests executed and passed in both Debug and Release, with successful desktop and roboRIO builds. All four desktop application/test links rerun when either static archive changes; the reviewer separately verified the roboRIO task graph remains isolated.
- The final portable Release test run passed all three suites (`model-tests.log`).

## Performance evidence

- `benchmark-linux-initial.json` preserves a dense 3D p99 of 4.56 ms that failed the 4 ms budget; do not treat that run as passing.
- Rolling resistance previously reset full body poses to change velocity, forcing unnecessary Jolt broadphase updates. The reviewed velocity-only API preserves pose/angular motion and existing rolling/landing behavior.
- `benchmark-linux.json` is the first successful post-fix run (dense p99 2.32 ms planar / 2.15 ms 3D, zero deadline misses).
- `benchmark-linux-committed.json` binds a repeat measurement to committed source `2d33ac28613711bfe2c4dfe065913894a0a26d6d`: dense p99 **3.04 ms planar / 2.22 ms 3D**. There were 1 and 8 individual 4 ms misses respectively in 5,000 measured substeps. The criterion is p99 below 4 ms, not a hard real-time guarantee. All six scenarios passed that criterion.
- CI source `0bb2a44` passed all six release scenarios on both platforms: dense p99 1.65 ms planar / 1.23 ms 3D on Windows, 1.83 ms / 1.31 ms on Linux, zero observed deadline misses. `benchmark-ci-windows.json` and `benchmark-ci-linux.json` retain the full measurements and provenance. Hosted-runner timing does not replace intended practice-machine measurement.
- Final source `c890889c59551131a1427791b51b9d5793641f6f` also passed all six scenarios on both platforms: dense p99 **1.65 ms planar / 1.25 ms 3D on Windows**, **1.79 ms / 1.29 ms on Linux**, zero observed deadline misses. See `benchmark-final-ci-windows.json` and `benchmark-final-ci-linux.json`.
- JSON records CPU/OS, compiler/version/flags, revision, build configuration, body counts, contact-manifold counts, median/p99 and misses. These are local software performance measurements, not a MapleSim comparison or physical calibration.

## Final platform follow-up

- Source `0bb2a448d90ca811c623052cc5811a5166c4ff4e` addresses two clean-runner failures without changing pinned dependencies or motion assertions.
- Windows could not connect to the official Studica Maven host. Linux downloads the exact official Windows artifacts into a SHA256-pinned Maven cache; Windows verifies the transferred files and Gradle uses them exclusively for the Studica group. A detached offline Gradle resolution verified all six native ZIP classifiers.
- Linux Release applied Phoenix module configuration before the desktop transport initialized, producing `TxFailed`. Debug had already shut down, so overlapping tests were not the observed cause. The integration fixture now waits, while disabled, for a fresh successful firmware status before constructing Drive. Both implementation and independent reviewer runs passed; see `phoenix-startup-release.log` and `phoenix-startup-review.log`. Production constructor behavior is unchanged; executable smoke remains separate evidence.
- The previous integrated run [39462354676](https://github.com/MECO-Robotics/2026_Base_Bot-cpp/actions/runs/39462354676) passed all four portable configurations; its full robot checks failed as described above.

- Integrated run [39467958995](https://github.com/MECO-Robotics/2026_Base_Bot-cpp/actions/runs/39467958995) passed all portable gates and the complete Linux robot job (30 tests/configuration, roboRIO build, both actual executable smokes). Windows compiled successfully but failed only the final instantaneous velocity samples in the drivetrain feedback test, with stale Phoenix status while Release compilation overlapped Debug testing. `fda72bc` separates all compilation/linking from sequential test execution, without changing assertions or physics.

- Isolated run [39469439571](https://github.com/MECO-Robotics/2026_Base_Bot-cpp/actions/runs/39469439571) showed firmware status alone was insufficient: Linux Release still encountered native module configuration failures. `4e328a5` strengthens the fixture to verify a nondefault configuration write and fresh readback before module construction. Focused Release implementation and independent reviewer runs passed without configuration errors; see `phoenix-config-readiness-release.log` and `phoenix-config-readiness-review.log`. Timing-sensitive tests remain separated from compilation.

- Windows in run `39469439571` passed all 30 native tests in both configurations after compilation was isolated. The launcher then exited before startup with `0xC0000135` (missing DLL). It previously borrowed the test installation runtime; the follow-up uses the application installation, colocates resolved DLLs, normalizes Windows PATH, and emits dependency diagnostics. The interim run `39470719784` was cancelled to include this fix.

- `c30c18c` combines the reviewed runtime packaging correction with configuration readiness. Local application-runtime preparation and all 23 planar executable smoke checks passed (`application-runtime-prepare.log`, `smoke-planar-app-runtime.json`); the strengthened fixture passed all 30 tests in both local native configurations (`phoenix-config-readiness-debug.log`, `phoenix-config-readiness-release.log`).

- A final interface audit found launch rotational velocity measured from the body origin rather than the configured center of mass. The reviewed correction passes COM to Arena and uses the COM-to-muzzle lever. The existing rotated launch contract now exercises nonzero COM; an independent temporary build restoring the original formula failed exactly that assertion (`com-launch-original-regression.log`), while all portable tests passed (`com-launch-review.log`). Body-state origin/COM velocity semantics are now explicit in the public snapshot comment and configuration guide.

- Run [39471124041](https://github.com/MECO-Robotics/2026_Base_Bot-cpp/actions/runs/39471124041) passed all six jobs, including Windows runtime startup and both actual executable modes. Log inspection nevertheless found failed native module configurations during actual Linux app startup, so green motion checks alone were insufficient. `c890889` moves verified configuration readiness into the production desktop Drive constructor before native ownership; the integration fixture probe is removed. Independent source review, Release native tests, three fresh-process repeats, and the actual planar executable passed locally with no failed configurations (`production-gate-*.log`, `smoke-planar-production-gate.json`). The interim COM run `39472093597` was cancelled to validate the complete correction together.

- The retained Phoenix diagnostic backend also passed all 13 actual executable checks after the production startup correction (`smoke-phoenix-production-gate.json`, `production-gate-phoenix-smoke.log`).

- Final source `c890889` passed the complete Linux robot job in run `39472708656`: 30 native tests per configuration, roboRIO build, and both actual executable smokes (`smoke-final-ci-linux-*.json`). Independent log review found no failed module configuration / `TxFailed` / `RxTimeout` entries. The planar readiness gate retried six times over 1.07 s, demonstrating the cold-start path. Initial status-frequency/stale warnings ended 0.50 s before startup completion and 2.74 s before first enabled motion; none continued during either smoke. See `startup-final-ci-linux.log` and the linked full CI log. These remaining vendor startup warnings are not hardware timing evidence.

## Final acceptance

- All six jobs passed at source `c890889c59551131a1427791b51b9d5793641f6f` in [run 39472708656](https://github.com/MECO-Robotics/2026_Base_Bot-cpp/actions/runs/39472708656): Windows/Linux Debug/Release portable models; Windows/Linux native Debug/Release (30 tests each); both actual executable modes on each platform; Linux roboRIO build/isolation.
- Windows startup logs were clean: no failed configuration, transmit, timeout, or stale-signal warnings. Gate completion was 0.068 s planar / 0.069 s 3D. Retained reports are `smoke-final-ci-windows-*.json` and `startup-final-ci-windows.log`; corresponding Linux evidence is named similarly.
- All final CI benchmarks passed the 4 ms p99 criterion, with full source/toolchain/machine/body/contact provenance retained in `benchmark-final-ci-*.json`. The designated local development machine also passed. Re-run on each intended practice machine; hosted-runner measurements do not characterize an untested machine.
- Local final production build passed desktop and roboRIO checks (`final-production-build.log`); the actual Phoenix diagnostic smoke passed after the shared startup change.
- Final documentation audit covered build/runtime preparation, backend selection, physics configuration/units, COM conventions, mechanism loads/stops, ownership/resets, practice controls, AdvantageScope topics, launch frames, model assumptions, and remaining fidelity limits. See `docs/jolt-simulation.md` and README.

Retained text logs normalize line endings and trailing whitespace for Git; raw CI logs remain available from the linked run.

Physics defaults are illustrative assumptions, not measured robot parameters. Build, numerical simulation, real executable behavior, and physical calibration are separate evidence categories.
