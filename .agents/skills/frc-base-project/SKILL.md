---
name: frc-base-project
description: Implement or review command-based C++ robot behavior in MECO's base project and its season forks, preserving reusable mechanisms, IO boundaries, units, and command lifetimes. Use for this architecture, not general FRC hardware maintenance or Java robot code.
---

# FRC C++ base project

Read the repository's `AGENTS.md`, `README.md`, and relevant port-status/contract documents before changing behavior. Resolve paths below from the repository root. Inspect `build.gradle`, `vendordeps/`, and the owning configuration for actual versions and hardware values; do not substitute latest APIs or example tuning. Use season- and language-matched official documentation or FRC reference tools when available, then verify against pinned headers and compilation.

## Reuse and boundaries

- Start with `src/main/include/base/Mechanism.h` and its implementation. Reuse `Flywheel` for a motor group sharing one velocity target and `PositionJoint` for one position degree of freedom. Keep independently controlled axes separate; coordinate assemblies with commands rather than copying control loops into a new subsystem.
- Keep vendor motor access behind `MotorIO` and its factories. Preserve lazy mode selection in MakeMotorIO: mechanism simulation selects SimMotorIO without hardware adapters, and mechanism replay selects InjectedMotorIO without hardware adapters or a simulator. Phoenix native swerve uses vendor-backed simulation separately; do not prohibit its simulated controller objects or assume it shares the mechanism factory. Framework code replay substitutes versioned recorded inputs before commands/controllers run; it does not read Java AdvantageKit logs or replay vendor firmware. See `docs/replay.md`.
- Preserve Phoenix native swerve timing when adding backends. Portable module/gyro IO reports calibrated SI measurements and uses separate Spark gains; never reinterpret Phoenix rotor gains as Spark duty gains. Portable simulation uses vendor-free IO and the shared world clock. Keep vendor calls outside the short request/snapshot lock. See `docs/portable-swerve.md`.
- Keep localization acquisition behind `VisionIO`; batch observations in reusable `VisionInputs`, update each source once per tick, and retain packet order. Status reads are passive. Keep QuestNav raw replay and camera accepted-observation replay boundaries distinct; use `PieceDetectionIO` for rays rather than mixing game-piece detections into robot-pose observations.
- Put reusable control/IO fixes in the base; keep game rules, season hardware configuration, shot tables, and autonomous choices in the season fork. When working downstream, identify generic fixes suitable for upstream without moving season behavior into the base or publishing changes without task authorization.

## Units and ownership

- Trace each signal through sensor, rotor, output, and physical coordinates. Mechanism angles use output rotations; linear joints use metres; rates are per second. Field geometry uses metres and API-required angular units. Convert gains, ratios, offsets, limits, and tolerances with their coordinate system; copied Java drive gains may use wheel turns while native Phoenix uses rotor turns. Check the port contract before conversion.
- Treat `frc2::CommandPtr` as owning and move-only. Move each command into one composition; create a fresh command for another use. Captures and subsystem pointers must outlive scheduled commands; avoid references to local variables in deferred lambdas.
- Declare all affected subsystem requirements. Parallel, race, and deadline children must have disjoint requirements. For held motor actions, use commands with release/interruption cleanup; a `RunOnce` that latches output does not stop that output when a trigger is released. Make any intentional persistent setpoint and its stop path explicit.
- Mechanisms own goals, trajectories and safety; motor IO owns PID/feedforward execution and vendor units. Real Talon and Spark adapters use onboard loops. Simulation and replay emulate control inside IO without constructing hardware; do not move voltage calculations back into subsystems. Native gain configuration is disabled-only. Use `docs/mechanism-control.md` for feedback and physical units and `docs/mechanism-tuning.md` for atomic tuning requests and Test-mode command ownership.
- Keep scheduler callbacks free of deliberate transport/configuration waits, sleeps, joins, and file IO. Real motor runtime configuration belongs to the IO worker; pending/fault states inhibit output and completion requires fresh intent. Cached NT reads stay on the consumer thread; publish telemetry and serialize logs through the existing workers. Preserve short snapshot locks and deterministic control ownership rather than claiming all code is lock-free.
- Reuse existing command helpers: `VelocityCommand` and `VoltageCommand` stop on end; `PositionCommand` finishes at tolerance and leaves position control active. Account for that hold behavior when coordinating or interrupting an assembly.

## Evidence proportional to the change

Follow `AGENTS.md`'s less-is-more testing guidance. Protect distinct observable behaviors and meaningful regressions; skip new tests for documentation-only changes. For control changes, choose focused native tests or existing simulation scripts that exercise the actual concern, such as units, command cancellation, limits, or IO mode selection.

Use the build and simulation commands documented in this checkout and inspect available `scripts/` before adding tooling. For a full base-to-season port, establish a successful base native build and running desktop simulation before claiming the base is ready for downstream integration; do not impose a fresh full-port validation on an unrelated narrow edit. Report build, numerical simulation, and hardware evidence separately, including relevant unverified behavior; a passing simulation does not establish calibration, physical fidelity, or field acceptance. Deployment and hardware motion require task authorization.
