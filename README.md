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

## Verified simulation check

`./gradlew build --max-workers=2` passed for desktop and roboRIO in debug and release, including 25 native test cases in both configurations. A real desktop `frcUserProgram` was exercised through local HAL simulation and NetworkTables: forward/reverse motion, rotation, pose changes, and stationary disabled behavior passed. The captured result is in `artifacts/sim-smoke.json`. This verifies runtime behavior; it does not establish physical calibration or hardware acceptance.

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
Whole-robot log playback and vendor-independent drivetrain
replay are not implemented. Systemcore requires a supported future toolchain and
vendor adapters; this checkout retains the pinned 2026 APIs.

Use the `MotorSimulationConfig` factory overload to select free, arm, or elevator
loads. Arm inertia includes the rotating assembly; mass, COM distance, and angle
offset define gravity, with zero angle horizontal. Elevator mass adds reflected
inertia through the configured drum radius. Model initial position and physical
stops use radians; `MotorInputs` still reports output rotations and rotations/s.
Standalone model checks call `StepSimulation()` explicitly; robot simulation
automatically registers simulated IO with the shared clock.

## Reusable code

- `base/Drive.h`: configurable four-module TalonFX/CANcoder/Pigeon swerve, Phoenix native odometry, field driving, heading hold, X brake, vision fusion, PathPlanner autos and wheel-force feedforwards.
- `base/Mechanism.h`: motor groups, flywheel and position joint subsystems; validated hardware/gains; voltage/velocity/position commands; acceleration limits, gravity feedforward, physical-coordinate soft limits, reset offsets, external encoder calibration, digital input, real/sim/injected replay IO. Angular mechanism units are rotations; linear mechanism units are metres; rates are per second.
- `base/Vision.h`: Limelight NetworkTables and PhotonVision pose observations, timestamps, bounds/ambiguity filtering and covariance.
- `RobotContainer`: illustrative base drivetrain configuration and controller mappings. Base motor IDs and zero offsets are examples copied from Java, not a commissioned robot configuration.

## Port status

This is a native implementation, with intentional framework replacements. WPILib DataLog/NetworkTables replaces AdvantageKit; it does not read Java AdvantageKit replay logs. Phoenix's native swerve replaces Java's custom odometry/setpoint implementation and currently supports TalonFX modules, rather than the optional SparkMax/NavX drivetrain combinations. Mechanisms use software voltage PID/feedforward rather than vendor onboard motion profiles. The default base container instantiates only drive, as in Java; other subsystems are reusable library components.

The desktop Jolt backend supplies rigid-body field contacts and generic game pieces.
The illustrative tire, suspension, load, and piece parameters require physical calibration;
season field geometry and scoring rules remain downstream. QuestNav pose tracking, Java URCL logging, custom AD* telemetry, piece detection localization, and the Java browser system-check dashboard are not implemented by these adapters. These gaps must not be treated as verified parity. External joint sensors are supplied through callbacks, with explicit calibration; they are not constructed automatically from Java encoder enum values.

Mechanism IO must supply `connected=true` with valid feedback, including injected replay snapshots. Disconnected or nonfinite feedback stops the mechanism, and control must be requested again after feedback recovers. `AtSetpoint()` only reports success for connected, active position or velocity control.

No deployment, hardware motion, sensor calibration, or field acceptance was performed as part of this port. See `docs/port-contract.md` for provenance and verification boundaries.

The latest independent audit, commit list, validation evidence, numerical analysis, and remaining risks are recorded in [the 2026-10-19 audit report](artifacts/audit-2026-10-19/REPORT.md).

## Jolt practice simulation

Windows and Linux are supported. Jolt 5.3.0 is source-pinned and built by CMake
through Gradle for desktop targets only; install CMake 3.24+ and Ninja, plus the
WPILib C++ compiler (MSVC on Windows). The roboRIO does not link Jolt.

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
`jolt-planar`. Physics runs on the robot thread in five fixed 4 ms substeps;
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

Run `./gradlew prepareHeadlessSimulation` to resolve the pinned HAL websocket archive.
Use `.\gradlew.bat prepareHeadlessSimulation` in Windows PowerShell.
The launcher rejects occupied local ports, verifies a per-run process identity,
snapshots runtime libraries, and loads only the HAL websocket extension. It
controls localhost. Install `pyntcore>=2026,<2027` and `websockets>=14,<17`
in the Python environment. `WPILIB_ROOT` overrides the WPILib installation;
the launcher also searches the Gradle archive cache.

In Jolt practice, Xbox **Y** resets the whole scene and places the robot at (0, 0)
metres with zero yaw; **LB** enables intake capture while held; **RB** launches one
possessed piece on its rising edge. Existing A aim, B estimator heading reset,
and X wheel lock remain available. B changes the estimator without teleporting
the physical chassis. The arena contains reusable walls, an obstacle, pieces,
and a ramp in 3D mode, with no season scoring rules.

Connect AdvantageScope to the simulator NT server. Under `/Simulation`, add
`RobotTruth`, `RobotEstimated`, and `Pieces` to a 3D field view. These are native
WPILib `Pose3d` struct topics. `Truth` is a planar [x, y, yaw] diagnostic array;
`RobotTruth` uses the chassis body origin, while `RobotEstimated` lifts the 2D
estimator to z=0; match the viewer model's origin or offsets accordingly.
`Velocity` contains world [vx, vy, vz, yaw rate], `Slip` contains per-module
longitudinal slip, and `Held`, `IntakeActive`, `Time`, `StepSeconds`, `MaximumStepSeconds`,
and `DeadlineMisses` expose state and timing. Existing `/SmartDashboard/Drive`
telemetry remains the estimated/control view. World truth is timestamped for future
camera models; synthetic images and new camera simulation are not included.

Portable model tests and benchmarks can run separately:

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
or MapleSim performance claim. CI builds/tests Windows and Linux and uploads
benchmark, smoke, and test evidence.

Recorded revisions, review dispositions, local measurements, and platform checks
are in [the Jolt verification record](artifacts/jolt/STATUS.md).
