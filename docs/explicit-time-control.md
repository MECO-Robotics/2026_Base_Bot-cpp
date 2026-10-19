# Explicit-time control audit — 2026-10-19

This follow-up targets the existing `codex/finalize-cpp-migration` branches. The runtime remains on WPILib 2026.2.1 and Phoenix 26.1.2. It does not declare whole-robot deterministic replay complete.

## Mechanism time and execution contract

`MotorIO::Read()` is observational. `AdvanceSimulation(units::second_t)` is the only mechanism plant advance operation; hardware and injected/replay IO do not implement a plant. The simulated motor retains the existing current-limited electrical model, motor constants, drag and stiffness-dependent integration. Each positive step must be finite and at most 100 ms. Repeated sampling never integrates or changes current/position/velocity. The polling regression advances the same one-second plant chronology while sampling at 50, 100 and 250 Hz, requiring exact equality of all three fields.

`Mechanism::Control(dt)` samples feedback and computes PID, acceleration feedforward, profile motion, gravity, voltage saturation and physical limits using the supplied interval. The default scheduler callback retains 20 ms. The explicit interval must be finite, positive and at most 100 ms; invalid intervals stop before throwing. The overload accepting an enabled flag supports desktop replay/tests; on real targets that flag is also gated by DriverStation enable. External encoder nonfinite values enter the normal stop path rather than throwing from calibration.

`PublishTelemetry()` publishes cached inputs/position/target and appends input logs. It neither reads sensors nor writes actuators. The ordinary `Periodic()` performs disabled-only tuning, default control and telemetry, preserving existing topic names and gain transactions. Logging remains at the telemetry cadence; an external 250 Hz mechanism control loop does not automatically record every 4 ms input sample.

WPILib calls `SimulationPeriodic()` after each subsystem's ordinary periodic callback. It advances the mechanism by 20 ms. Direct test loops must call that simulation callback explicitly. For manual multirate scheduling, call `UseExternalScheduling()` once, then execute control and physics exactly once per interval. It suppresses both automatic steps; ordinary Periodic still services tuning/telemetry. All mechanism setters, getters, control, physics and telemetry require the **same owner thread**. This API is not a safe way to add an independent mechanism Notifier beside the command scheduler.

## Deterministic execution that is verified

`DeterministicSimulation::StepWithSensors` delivers sensors/DS/configuration events first, executes commands every fifth integer 4 ms tick, then control and physics. HAL time remains caller-owned. The native integration regression pauses HAL time, runs the actual command scheduler and VelocityCommand, uses the actual mechanism controller with a current-limited motor plant, delivers feedback after an explicit 8 ms delay, and applies DriverStation disable at 1.6 s. Two runs require identical 500-sample voltage traces and exact two-second virtual duration. All disabled samples must command zero and the scheduler must cancel its command. It is mechanism integration evidence, not a deterministic test of the season RobotContainer or native Phoenix estimator.

## 250 Hz drive audit

Only the retained Phoenix request writes module actuators; commands publish fixed-size intent. Application native consumption and telemetry handoff use try-lock, while robot-thread publications/getters may block briefly. No nested application mutex cycle was found: telemetry's state copy does not call back into Phoenix, and reset observers run after releasing the snapshot mutex. The simulation notifier is stopped and destroyed before native odometry stops and the drivetrain is destroyed. Command captures must still outlive their commands.

A new regression protects a fast disable/re-enable between native ticks. A robot-thread publication rejected while disabled invalidates the old epoch; teleop-only targets rejected by native mode gating are invalidated as well. Native disabled requests still neutral both axes; the 100 ms moving-target watchdog and intentional X-brake latch remain. These checks prove application intent behavior, not hardware stop distance or hard real-time timing.

The callback is not proven allocation-free or deadlock-free across vendor internals. Phoenix ApplyRobotSpeeds uses dynamic vectors and vendor synchronization; DriverStation and status-signal operations also have internal synchronization. No CPU improvement, hardware jitter bound, or CAN FD performance gain is claimed. Existing live dashboard tuning, QuestNav, AdvantageScope and source actuator mappings remain in place.

## Phoenix deterministic simulation and replay: open acceptance requirements

The inspected pinned `swerve/impl/SwerveDrivetrainImpl.hpp` exposes odometry Start/Stop and validity, but its Run implementation is protected and has no public single-step interface. `UpdateSimState(dt, voltage)` advances the ideal plant, not an authoritative synchronous control/DAQ/estimator tick. Pausing HAL time or manually advancing that plant is not proof that its background vendor tasks execute deterministically. This follow-up retains the working 4 ms live plant notifier rather than replacing native odometry with a superficially deterministic approximation.

Pinned `HootReplay.hpp` **does** offer Pause and StepTiming for version-compatible hoot logs, loaded before devices. It replays one CAN bus and ignores program transmits/configuration during playback. It is a possible signal-replay integration path, not a public live-controller/plant single-step API or arbitrary estimator checkpoint. No compatible recorded hoot fixture, version/license validation, or end-to-end hoot replay was established here. Existing native MotorIO logs are not hoot files.

Whole-robot reproducible replay remains incomplete. DriverStation logging and NT/WPILOG/Quest telemetry do not by themselves restore mode/joystick events, command scheduling, original configuration/tuning chronology, external joint feedback, decoded Quest frames and absolute-camera inputs, or Phoenix estimator history. A valid acceptance test must cold-start a robot with the same configuration, replay those ordered inputs at their original virtual timestamps, compare command lifecycle and actuator outputs, and reproduce reset/delayed-vision estimator behavior. Copying 10 Hz displayed poses into an estimator or substituting logged outputs would not meet that requirement. No unused partial replay framework or false completion switch was added.

## Systemcore 2027 preparation and limit

Explicit timestep/plant boundaries and private vendor drivetrain implementation reduce coupling without changing 2026 dependencies. Current 2027 WPILib documentation requires project import and describes C++ namespace/package changes and the new control system. The installed 2026 compiler/headers and pinned vendor dependency platform lists do not establish Systemcore ABI/runtime support. Do not add an untested Systemcore Gradle target or compatibility namespace alias to these 2026 builds.

A separate version-matched 2027 target must establish WPILib import, compiler/ABI, each vendor's supported Systemcore artifacts, DriverStation/mode semantics, Quest/NT schema compatibility and native simulation tests before deployment. No 2027 binary or hardware validation is claimed.

Official references: [2027 changes](https://docs.wpilib.org/en/latest/docs/yearly-overview/yearly-changelog.html), [WPILib simulation](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html), and [Phoenix swerve](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/mechanisms/swerve/swerve-overview.html). Pinned installed headers remain the API authority.
