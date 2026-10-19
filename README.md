# 2026 Base Bot — C++

Native WPILib 2026 command-based C++ port of the reusable Java `2026_Base_Bot` architecture. Team 8324. Season behavior belongs in the downstream `2026-Rebuilt-cpp` fork.

## Build

For architecture-aware Codex assistance, use the repository skill [`frc-base-project`](.agents/skills/frc-base-project/SKILL.md), or invoke `$frc-base-project`. Test guidance lives in `AGENTS.md`.

For device configuration, commissioning, and diagnosis, use [`frc-hardware`](.agents/skills/frc-hardware/SKILL.md) or invoke `$frc-hardware`.

Install WPILib 2026.2.1 with its C++ toolchains and Java 17+. From this folder:

```sh
./gradlew build
./gradlew simulateNative
./gradlew deploy
```

`deploy` writes to the team's roboRIO; review hardware configuration before deploying. Desktop tests run with `./gradlew check`. Both desktop and roboRIO targets are enabled. Native dependencies retain the source project's pinned versions.

## Verified simulation check

`./gradlew build --max-workers=2` passed for desktop and roboRIO in debug and release, including the native regression suite in both configurations. A real desktop `frcUserProgram` was exercised through local HAL simulation and NetworkTables: forward/reverse motion, rotation, pose changes, and stationary disabled behavior passed. The captured result is in `artifacts/sim-smoke.json`. This verifies runtime behavior; it does not establish physical calibration or hardware acceptance.

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

## Reusable code

- `base/Drive.h`: configurable four-module TalonFX/CANcoder/Pigeon swerve, Phoenix native odometry, field driving, heading hold, X brake, vision fusion, PathPlanner autos and wheel-force feedforwards.
- `base/Mechanism.h`: motor groups, flywheel and position joint subsystems; validated hardware/gains; voltage/velocity/position commands; acceleration limits, gravity feedforward, physical-coordinate soft limits, reset offsets, external encoder calibration, digital input, real/sim/injected replay IO. Angular mechanism units are rotations; linear mechanism units are metres; rates are per second.
- `base/QuestNav.h`: read-only QuestNav 2026-2.2.0 protobuf NetworkTables transport, mounting correction, bounded translation alignment, freshness validation, readiness and device diagnostics. The season container owns fusion and fallback policy.
- `base/DeterministicSimulation.h`: offline integer 250/50 Hz scheduling, bounded sensor/NT latency channels and timestamped MotorIO replay; `base/MotorInputLog.h` reads this port's versioned input logs.
- `base/ModelFeedforward.h`: opt-in DC-motor model kV/kA/kG estimates from output inertia, gearing, efficiency and gravity torque; existing PID/static gains remain unchanged.
- `base/Vision.h`: Limelight NetworkTables and PhotonVision pose observations, timestamps, bounds/ambiguity filtering and covariance.
- `RobotContainer`: illustrative base drivetrain configuration and controller mappings. Base motor IDs and zero offsets are examples copied from Java, not a commissioned robot configuration.

## Port status

This is a native implementation, with intentional framework replacements. WPILib DataLog/NetworkTables replaces AdvantageKit; it does not read Java AdvantageKit replay logs. Phoenix's native swerve replaces Java's custom odometry/setpoint implementation and currently supports TalonFX modules, rather than the optional SparkMax/NavX drivetrain combinations. Mechanisms use software voltage PID/feedforward rather than vendor onboard motion profiles. The default base container instantiates only drive, as in Java; other subsystems are reusable library components.

Motor simulation exercises control logic with output inertia; it does not reproduce MapleSim field contacts, fuel dynamics, or arm/elevator gravity loading. Read-only QuestNav tracking is implemented; headset reset-request acknowledgments and automatic Java QuestNav drift simulation remain absent. Java URCL logging, custom AD* telemetry, piece detection localization, and the Java browser system-check dashboard are not implemented by these adapters. These gaps must not be treated as verified parity. External joint sensors are supplied through callbacks, with explicit calibration; they are not constructed automatically from Java encoder enum values.

No deployment, hardware motion, sensor calibration, or field acceptance was performed as part of this port. See `docs/port-contract.md` for provenance and verification boundaries.

## Control, tuning and replay contracts

Commands remain the exclusive subsystem owners at 50 Hz. They publish fixed-size targets; one retained Phoenix request resolves heading feedback and field transforms on native odometry ticks. `DriveConfig::odometryHz=0` keeps Phoenix's 250 Hz CAN FD / 100 Hz CAN 2.0 defaults. The season explicitly selects 250 Hz on its existing CANivore bus. Disabled ticks neutral both axes and invalidate old targets; moving requests expire after 100 ms without refresh. Stationary X brake remains latched until replaced or disabled. `RunHeading` uses robot-relative translation; `Aim` uses alliance-relative field translation. Install `SetPoseResetCallback` during initialization to synchronize external localization for every AutoBuilder, Choreo or operator reset.

Native swerve physics has one 4 ms simulation notifier, stopped before drivetrain destruction. It is a wall-clock simulation, while `DeterministicSimulation` is an offline harness. See [the migration review](docs/cpp-finalization.md) for numerical estimates, replay limits and remaining gaps.

Mechanisms opt in to tuning with `EnableLiveTuning()`. Set the whole NT double array `<name>/Gains/Values` in the order published at `<name>/Gains/Fields`: kP, kI, kD, kS, kV, kA, kG, maximum velocity, maximum acceleration, tolerance. Updates are validated and accepted only while disabled; `<name>/Gains/Status` explains rejection. Physical limits are excluded. Tuning is in memory, not persisted across reboot. Source gains stay the startup defaults.

Call `EnableInputLogging(frc::DataLogManager::GetLog())` during initialization to record `<name>/MotorInputs`, including disabled/fault samples. `ReadMotorInputLog(reader, entryName)` loads an owning sample vector; pass it to `ReplayMotorIO` and advance with original FPGA microseconds. The vector must outlive replay IO, and the log must outlive a logging mechanism. Choose a replay epoch explicitly when using zero-based simulation ticks. These input logs preserve output rotations/rates before optional metres scaling; restore configuration, commands, external encoders and DriverStation chronology separately. They are neither Java AdvantageKit playback nor whole-robot replay.

## AdvantageScope

Native NT4 and WPILOG visualization uses standard WPILib structs for 2D/3D robot pose, chassis speeds, and measured/target swerve states under `/AdvantageScope/Drive`. Existing scalar mechanism telemetry remains graphable. Metadata identifies the native project without requiring AdvantageKit. See [the AdvantageScope guide](docs/advantagescope.md) for live simulation, recorded logs, an installed-parser validation command and an importable layout. This supports log viewing; it does not turn MotorIO replay into whole-robot deterministic re-execution.

## Explicit-time mechanism control

Mechanism IO reads no longer advance physics. The default scheduler retains 20 ms control and a separate 20 ms simulation callback. Manual multirate owners use `UseExternalScheduling()`, `Control(dt)`, and `AdvanceSimulation(dt)` on one thread; telemetry uses cached state. See [the explicit-time audit](docs/explicit-time-control.md) for timing, validation, concurrency and unresolved whole-robot replay/Phoenix/Systemcore boundaries.
