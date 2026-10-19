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

`./gradlew build --max-workers=2` passed for desktop and roboRIO in debug and release, including nine mechanism tests in both configurations. A real desktop `frcUserProgram` was exercised through local HAL simulation and NetworkTables: forward/reverse motion, rotation, pose changes, and stationary disabled behavior passed. The captured result is in `artifacts/sim-smoke.json`. This verifies runtime behavior; it does not establish physical calibration or hardware acceptance.

For the regular graphical simulator use `./gradlew simulateNative`. For a repeatable Linux headless check, run these in separate terminals:

```sh
# Terminal 1: after ./gradlew build; WPILIB_ROOT can override ~/wpilib/2026
./scripts/sim_headless.sh
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

Mechanism simulation advances exactly once per `Read()` by 20 ms. It integrates
the current-limited motor model analytically, including brake/coast drag; additional
reads advance time. Replay instead consumes caller-injected snapshots without
stepping physics. Its current clamp models winding current; the real Talon adapter
configures supply current, so electrical current-limit parity is not established.
Whole-robot log playback and vendor-independent drivetrain
replay are not implemented. Systemcore requires a supported future toolchain and
vendor adapters; this checkout retains the pinned 2026 APIs.

## Reusable code

- `base/Drive.h`: configurable four-module TalonFX/CANcoder/Pigeon swerve, Phoenix native odometry, field driving, heading hold, X brake, vision fusion, PathPlanner autos and wheel-force feedforwards.
- `base/Mechanism.h`: motor groups, flywheel and position joint subsystems; validated hardware/gains; voltage/velocity/position commands; acceleration limits, gravity feedforward, physical-coordinate soft limits, reset offsets, external encoder calibration, digital input, real/sim/injected replay IO. Angular mechanism units are rotations; linear mechanism units are metres; rates are per second.
- `base/Vision.h`: Limelight NetworkTables and PhotonVision pose observations, timestamps, bounds/ambiguity filtering and covariance.
- `RobotContainer`: illustrative base drivetrain configuration and controller mappings. Base motor IDs and zero offsets are examples copied from Java, not a commissioned robot configuration.

## Port status

This is a native implementation, with intentional framework replacements. WPILib DataLog/NetworkTables replaces AdvantageKit; it does not read Java AdvantageKit replay logs. Phoenix's native swerve replaces Java's custom odometry/setpoint implementation and currently supports TalonFX modules, rather than the optional SparkMax/NavX drivetrain combinations. Mechanisms use software voltage PID/feedforward rather than vendor onboard motion profiles. The default base container instantiates only drive, as in Java; other subsystems are reusable library components.

Motor simulation exercises control logic with output inertia; it does not reproduce MapleSim field contacts, fuel dynamics, or arm/elevator gravity loading. QuestNav pose tracking, Java URCL logging, custom AD* telemetry, piece detection localization, and the Java browser system-check dashboard are not implemented by these adapters. These gaps must not be treated as verified parity. External joint sensors are supplied through callbacks, with explicit calibration; they are not constructed automatically from Java encoder enum values.

Mechanism IO must supply `connected=true` with valid feedback, including injected replay snapshots. Disconnected or nonfinite feedback stops the mechanism, and control must be requested again after feedback recovers. `AtSetpoint()` only reports success for connected, active position or velocity control.

No deployment, hardware motion, sensor calibration, or field acceptance was performed as part of this port. See `docs/port-contract.md` for provenance and verification boundaries.
