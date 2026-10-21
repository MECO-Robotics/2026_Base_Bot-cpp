# 2026 Base Bot — C++

Native WPILib 2026 command-based C++ port of the reusable Java `2026_Base_Bot` architecture. Team 8324. Season behavior belongs in the downstream `2026-Rebuilt-cpp` fork.

## Build

For architecture-aware Codex assistance, use the repository skill [`frc-base-project`](.agents/skills/frc-base-project/SKILL.md), or invoke `$frc-base-project`. Test guidance lives in `AGENTS.md`.

For device configuration, commissioning, and diagnosis, use [`frc-hardware`](.agents/skills/frc-hardware/SKILL.md) or invoke `$frc-hardware`.

Install WPILib 2026.2.1 with its C++ toolchains and Java 17+. The build selects C++23 mode; the pinned roboRIO GCC 12 implements only a subset, so new code must compile on both targets. From this folder:

```sh
./gradlew build
./gradlew simulateNative
./gradlew deploy
```

`deploy` writes to the team's roboRIO; review hardware configuration before deploying. Desktop tests run with `./gradlew check`. Both desktop and roboRIO targets are enabled. Native dependencies retain the source project's pinned versions.

## IDE configuration

The workspace uses C/C++ compile-command configurations rather than the WPILib
configuration provider, so IntelliSense can use the same target-specific flags
and simulation include directories as the build. Generate the databases with:

```sh
./gradlew generateCompileCommands
```

In **C/C++: Edit Configurations (UI)**, select the compiler and C++23, then set
**Compile commands** to the desired database under
`build/TargetedCompileCommands/` (for example,
`linuxx86-64debug/compile_commands.json` for Linux desktop or
`linuxathenarelease/compile_commands.json` for roboRIO). Machine-specific
`.vscode/c_cpp_properties.json` is ignored by Git. Regenerate the databases after
changing dependencies or compiler flags. To use WPILib's toolchain selector
instead, set `C_Cpp.default.configurationProvider` to
`wpilibsuite.vscode-wpilib` in workspace settings.

## Verified simulation check

The [finalization merge notes](docs/finalization-merge.md) record the latest integration checks. The preceding [parity audit](docs/parity-audit-2026-10-21.md) records its desktop/roboRIO build, 52-test baseline, headless practice, replay and numerical evidence. These checks establish software behavior, not physical calibration or hardware acceptance.

For the regular graphical simulator use `./gradlew simulateNative`. For a repeatable Windows or Linux headless check, run these in separate terminals:

```sh
# Terminal 1: after ./gradlew build; WPILIB_ROOT can override ~/wpilib/2026
python scripts/sim_headless.py --backend jolt-planar
```

```sh
# Terminal 2: isolated Python dependencies, used only by the test driver
python3 -m venv /tmp/frc-cpp-sim-venv
/tmp/frc-cpp-sim-venv/bin/pip install 'pyntcore>=2026,<2027' 'websockets>=14,<17'
/tmp/frc-cpp-sim-venv/bin/python scripts/sim_smoke.py --output artifacts/sim-smoke.json
```

The script controls only the local desktop HAL simulator and leaves it disabled. The headless HAL server is a development tool; stop the process after testing. Example base tuning is not hardware calibration.

## Timing and replay contracts

The robot scheduler, mechanism control, and simulation advance at 20 ms (50 Hz).
Changing the robot period also requires updating mechanism/profile and simulation
steps and teleop discretization. Phoenix owns its synchronized native swerve loop:
the pinned 26.1.2 library defaults to 250 Hz on CAN FD and 100 Hz on CAN 2.0,
including the default `rio` bus. The framework does not establish a 250 Hz
application control-loop latency guarantee.

Mechanism reads are observational. A single simulation clock advances registered
plants in five 4 ms substeps per 20 ms robot period, including current limiting,
brake/coast drag, optional arm gravity, elevator load, and physical travel stops. Replay instead consumes caller-injected snapshots without
stepping physics. Its current clamp models winding current; the real Talon adapter
configures supply current, so electrical current-limit parity is not established.
Desktop code replay substitutes recorded framework inputs and reruns the scheduler,
commands, and IO control according to the recorded source; see [code replay](docs/replay.md) for the
boundary and traceability workflow. Systemcore requires a supported future
toolchain and vendor adapters; this checkout retains the pinned 2026 APIs.

Use the `MotorSimulationConfig` factory overload to select free, arm, or elevator
loads. Arm inertia includes the rotating assembly; mass, COM distance, and angle
offset define gravity, with zero angle horizontal. Elevator mass adds reflected
inertia through the configured drum radius. Model initial position and physical
stops use radians; `MotorInputs` still reports output rotations and rotations/s.
Standalone model checks call `StepSimulation()` explicitly; robot simulation
automatically registers simulated IO with the shared clock. The shared [battery model](docs/simulation-power.md) aggregates supply power once per robot period and applies WPILib battery sag to the next period; mechanism inputs report supply current and effective terminal voltage.

## Reusable code

- `base/Drive.h`: configurable four-module swerve with Phoenix TalonFX/CANcoder/Pigeon or REV SparkMax/NavX IO, field driving, heading hold, X brake, vision fusion, PathPlanner autos, wheel-force feedforwards, [translation SysId](docs/runtime-parity.md), and [wheel-radius characterization](docs/wheel-radius-characterization.md).
- `base/Mechanism.h`: motor groups, flywheel and position joint subsystems; validated hardware/gains; voltage/velocity/position commands; acceleration limits, gravity feedforward, physical-coordinate soft limits, reset offsets, external encoder calibration, digital input, real/sim/injected replay IO. Angular mechanism units are rotations; linear mechanism units are metres; rates are per second.
- `base/Vision.h`: shared localization processing with replaceable Limelight, PhotonVision, QuestNav, and injected IO, reusable observation batches, configurable tag filters and covariance.
- `base/ModelFeedforward.h` and `base/MechanismTuning.h`: [WPILib model coefficients and safe dashboard transactions](docs/mechanism-tuning.md), with disabled gain updates and an explicit Test-mode command.
- `base/QuestNav.h` and `base/PieceDetection.h`: queued 5 ms QuestNav tracking and mounted camera-ray piece localization, with raw-input code replay.
- `RobotContainer`: illustrative base drivetrain configuration and controller mappings. Base motor IDs and zero offsets are examples copied from Java, not a commissioned robot configuration.

The finalization branch integration and its conflict resolutions are recorded in
[the merge notes](docs/finalization-merge.md). Offline explicit-time mechanism
control is described in [its contract](docs/explicit-time-control.md).

## Port status

This is a native implementation, with intentional framework replacements. WPILib DataLog/NetworkTables replaces AdvantageKit; it does not read Java AdvantageKit replay logs. Phoenix remains the default swerve backend with native odometry; [SparkMax/NavX](docs/portable-swerve.md) uses WPILib kinematics and estimation behind the same drive API. Mechanisms own goals, trajectories and safety, while real Talon and Spark IO execute onboard closed loops. Simulated IO owns its controller emulator; replay reconstructs the source's control requests. See [mechanism control](docs/mechanism-control.md). The default base container instantiates only drive, as in Java; other subsystems are reusable library components.

The [adapter validation report](artifacts/adapter-architecture-validation.json)
records independent reviews, native builds/tests, executable simulation, code
replay and the injected vision dispatch benchmark.

The desktop Jolt backend supplies rigid-body field contacts and generic game pieces.
The illustrative tire, suspension, load, and piece parameters require physical calibration;
an optional approximate 2026 collision scene supports practice, while scoring rules
remain downstream. QuestNav tracking and game-piece localization are opt-in reusable components; see [QuestNav](docs/questnav.md), [camera filtering](docs/vision-configuration.md), and [piece detection](docs/piece-detection.md). Java URCL logging, custom AD* telemetry, and the Java browser system-check dashboard are intentionally not duplicated. Native PathPlanner and typed AdvantageScope telemetry remain the supported interfaces. Simulated/injected joints support calibrated external sensor callbacks; real onboard loops select coherent feedback through the motor adapter. Sensors are not constructed automatically from Java encoder enum values.

Mechanism IO must supply `connected=true` with valid feedback, including injected replay snapshots. Disconnected or nonfinite feedback stops the mechanism, and control must be requested again after feedback recovers. `AtSetpoint()` only reports success for connected, active position or velocity control.

No deployment, hardware motion, sensor calibration, or field acceptance was performed as part of this port. See `docs/port-contract.md` for provenance and verification boundaries.

The Java/C++ comparison and selective migration decisions are recorded in [the parity audit](docs/parity-audit-2026-10-21.md). Earlier numerical and framework evidence remains in [the 2026-10-19 audit report](artifacts/audit-2026-10-19/REPORT.md).

## Jolt practice simulation

Full robot practice supports Windows and Linux. Native Jolt models, tests, and
benchmarks also support macOS on Apple Silicon and Intel. Jolt 5.3.0 is source-pinned
and built by CMake. The full robot Gradle build integrates it for desktop targets
only; that workflow requires CMake 3.24+, Ninja, and the WPILib C++ compiler
(MSVC on Windows). The roboRIO does not link Jolt. Standalone Mac setup is below.

Windows CI receives the exact pinned Studica 2026.0.0 Windows archives from the
Linux gate because some Windows runners cannot reach the official Studica Maven
server. `scripts/prepare_vendor_cache.py --output build/vendor-cache` downloads
only official artifacts and validates the committed SHA256 manifest, ZIP contents,
and POM coordinates. After transfer, `--verify-only` checks the cache again;
`FRC_VENDOR_CACHE` selects that local Maven repository exclusively for Studica dependencies.
Vendordep versions remain unchanged and binary archives are not stored in Git.


Set `FRC_SIM_PHYSICS` before launching: `phoenix` retains the vendor diagnostic
plant, `jolt-planar` constrains chassis movement to X/Y/yaw, and `jolt-3d` enables
wheel suspension, ramps, pitch, roll, and tipping. Desktop simulation defaults to
`jolt-3d`. Use `FRC_SIM_PHYSICS=jolt-planar` for fast planar automation.
The headless launcher also defaults to `jolt-3d`; `--backend jolt-planar` overrides it.
Physics runs on the robot thread in five fixed 4 ms substeps;
Phoenix control and estimator updates retain their vendor timing.

For example, select 3D practice in a Linux shell with
`FRC_SIM_PHYSICS=jolt-3d ./gradlew simulateNative`. In Windows PowerShell:

```powershell
$env:FRC_SIM_PHYSICS = 'jolt-3d'
.\gradlew.bat simulateNative
```

For an owned, automatically stopped local simulator and smoke check:

```sh
python scripts/sim_headless.py --backend jolt-planar --smoke-output artifacts/smoke-planar.json
python scripts/sim_headless.py --backend jolt-3d --smoke-output artifacts/smoke-3d.json
```

Run `./gradlew prepareHeadlessSimulation` first to build/install the desktop release
runtime and resolve the pinned HAL websocket archive.
Use `.\gradlew.bat prepareHeadlessSimulation` in Windows PowerShell.
The launcher rejects occupied local ports, verifies a per-run process identity,
snapshots the application runtime libraries, and loads only the HAL websocket extension. It
controls localhost. Install `pyntcore>=2026,<2027` and `websockets>=14,<17`
in the Python environment. `WPILIB_ROOT` overrides the WPILib installation;
the launcher also searches the Gradle archive cache.

In Jolt practice, Xbox **Y** resets the whole scene and places the robot at (8, 4)
metres with zero yaw; **LB** enables intake capture while held; **RB** launches one
possessed piece on its rising edge. Existing A aim, B estimator heading reset,
and X wheel lock remain available. B changes the estimator without teleporting
the physical chassis. Practice defaults to an [approximate 2026 field](docs/2026-field-simulation.md):
perimeter, hubs, two poles per tower, plus 3D bumps and depot curbs. Trench
ground supports block beside the bumps; drive-through lanes remain open.
Set `FRC_SIM_FIELD=generic` for the original generic walls,
obstacle and 3D ramp. Neither scene implements season scoring rules.

In AdvantageScope's Swerve tab, use these canonical typed sources:

| Source | NT path |
| --- | --- |
| Measured module states | `/AdvantageScope/Drive/ModuleStates` |
| Target module states | `/AdvantageScope/Drive/ModuleTargets` |
| Measured chassis speeds | `/AdvantageScope/Drive/ChassisSpeeds` |
| Requested chassis speeds | `/AdvantageScope/Drive/RequestedChassisSpeeds` |
| Heading | `/AdvantageScope/Drive/Pose/rotation` (decoded child) |

Module states are WPILib `SwerveModuleState[]`, speeds are `ChassisSpeeds`,
and `Pose` is `Pose2d`. Speeds use m/s, angular speeds rad/s, and angles radians.
Array order follows `DriveConfig.modules`: front-left, front-right, back-left,
back-right by default. Set maximum speed to `maxSpeed` (4.572 m/s by default)
and the module-center width/length to 0.6731 m for the default configuration.
Compare targets against measurements; physical simulation truth is a separate signal.

Connect AdvantageScope to the simulator NT server. Add `/Simulation/RobotTruth`
and `/Simulation/Pieces` (`Pose3d` / `Pose3d[]`) to a 3D field view, plus
`/AdvantageScope/Drive/Pose` for the 2D estimator view. `RobotTruth` uses the
chassis body origin. Collider outlines under `/Simulation/Colliders` can be
added as trajectories. `/Simulation/Velocity` is physical world-frame velocity,
not the measured robot-relative chassis speeds.

The former `Drive/Rotation`, `SmartDashboard/Drive/Pose`,
`SmartDashboard/Drive/Speeds`, `Simulation/RobotEstimated`, `Simulation/Truth`,
and `Simulation/RequestedSpeeds` aliases are removed. Update saved layouts to
the canonical paths above. NT values and struct schemas are recorded in WPILOG
for AdvantageScope playback. Application publishers consume snapshots on a
lower-priority worker; a separate lower-priority FIFO worker serializes recorded
frames. Replay inputs are still captured every control cycle on the robot thread.
See [code replay](docs/replay.md) for rerunning robot logic and comparing new outputs.

Portable model tests and benchmarks can run separately, without WPILib or Phoenix.
On macOS, install Xcode Command Line Tools (`xcode-select --install`), CMake 3.24+,
Ninja, and Python 3 (for example, `brew install cmake ninja python`). Then use the
same commands below. The build targets the current Mac architecture; use a native
terminal and toolchain for Apple Silicon or Intel. Replace `Release` and the build
directory with `Debug` / `build/jolt-debug` for debug tests. Benchmark Release builds.
On macOS, use `python3` for the benchmark command if `python` is not installed.

The pinned Phoenix 26.1.2 vendordep has no macOS native libraries. Consequently,
`simulateNative`, `prepareHeadlessSimulation`, and the Xbox/HAL/AdvantageScope
robot practice launcher remain Windows/Linux workflows. The standalone Mac models
exercise the Jolt world, drivetrain physics, and arena/pieces; they do not run
Phoenix control/odometry or WPILib mechanism adapters.

Portable model commands:

```sh
cmake -S simulation -B build/jolt-release -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/jolt-release --parallel 2
ctest --test-dir build/jolt-release --output-on-failure
python scripts/sim_benchmark.py --build-dir build/jolt-release --output artifacts/benchmark.json --max-p99-ms 4
```

The benchmark covers empty, typical, and dense scenes in both modes with active
drivetrain voltages. It measures the complete model/arena/Jolt substep, excluding
robot controls and telemetry. Results include body count, peak added/persisted
contact manifolds, median/p99 cost, and 4 ms deadline misses; record CPU/OS,
build configuration, compiler, and revision alongside results. A p99 below 4 ms
on the intended practice machine is enforced by the benchmark wrapper, not a hardware latency
or MapleSim performance claim. CI builds/tests the portable models on Windows, Linux, Apple Silicon macOS, and
Intel macOS in Debug and Release, and benchmarks each Release target. Mac CI
reports budget failures as warnings while keeping functional tests required;
its hosted Intel runner has exceeded the dense-scene budget. Mac practice-machine
performance remains unqualified until measured on the intended machine. The
benchmark command above enforces the budget by default; `--report-only` preserves
`acceptance.passed` and miss counts while allowing a measurement-only run to exit
successfully. Model crashes and invalid output still fail the run. Full robot
smokes run on Windows/Linux. CI uploads benchmark, smoke, and test evidence.

Model parameters, units, ownership, and fidelity limits are detailed in
[the simulation configuration guide](docs/jolt-simulation.md).

Recorded revisions, review dispositions, local measurements, and platform checks
are in [the Jolt verification record](artifacts/jolt/STATUS.md).
