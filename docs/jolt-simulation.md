# Simulation model configuration

See the [practice quick start](../README.md#jolt-practice-simulation) for build,
backend selection, controller bindings, AdvantageScope, and headless commands.
All defaults demonstrate software behavior; replace them with measured robot
parameters before using the model to assess a physical design.

## macOS model workflow

The standalone `simulation/` CMake project supports native Apple Silicon and Intel
Mac builds. Use the [portable model commands](../README.md#jolt-practice-simulation)
after installing Xcode Command Line Tools, CMake, Ninja, and Python 3. No WPILib,
Java, vendordep libraries, or Gradle are needed for these numerical tests and
benchmarks. CI runs all three model suites in Debug/Release on each Mac architecture
and enforces the same 4 ms Release p99 benchmark gate.

Full robot/HAL practice remains Windows/Linux because the pinned Phoenix C++
libraries do not provide a macOS target. Portable Mac tests cover Jolt lifecycle,
drivetrain physics, collisions, and piece ownership/launch; they do not validate
Phoenix sensors/odometry, command scheduling, or mechanism IO adapters.

## Drivetrain

`base::DriveConfig` supplies mass (`massKg`), yaw inertia (`moi`), tire friction
(`cof`), module positions, wheel radius, drive/steer/coupling ratios, current
limits, inversion, and CANcoder offsets. `DriveConfig::simulation` supplies
bumper half dimensions, body-origin ride height, center of mass relative to that
origin, roll/pitch inertia, wheel/steer inertia, tire stiffness, and suspension.
Lengths are metres, mass kilograms, inertia kg m², and stiffness/damping use SI.
The portable `DrivePhysicsConfig` additionally exposes motor resistance, torque
constant, velocity constant, and friction voltage. Invalid/nonfinite physical
parameters throw during construction.

Planar mode fixes chassis Z, pitch, and roll. Full 3D uses four downward wheel
contact rays with spring/damper normal forces and longitudinal/lateral tire
forces capped by available friction. Motor current and wheel inertia affect
acceleration; applied forces act at contact points. The bumper is a box rigid
body. Suspension rays do not model tire volume, sidewall contact, or detailed
wheel geometry; inertia is supplied along body principal axes. This is a
simplified tire/suspension model, not a measured tire model or an articulated
robot model.

Phoenix remains responsible for native controls and estimated pose. Its desktop
transport, control, and odometry threads run asynchronously; whole-robot results
are not bitwise deterministic. Before constructing native modules, the desktop drivetrain verifies a
nondefault configuration write/readback using a temporary handle to its first
drive device. The handle is released before native ownership; normal module
configuration replaces the temporary zero output limits. A bounded 10-second
startup failure reports the device, bus, and transport status. The integration
test uses this same production startup path. Firmware status alone did not prove
configuration readiness in clean-runner tests. Initial probe transactions may retry while the vendor transport starts. A
failed native module configuration after the readiness message should be
investigated before trusting a run.

`Drive::Pose()` and `ResetPose()` access the estimator. Desktop
`SimulationTruth()` returns physical state; `ResetSimulation(pose)` places and
stops the chassis, clears wheel motion, and synchronizes sensors and estimator.
Use scene reset for a complete practice restart. A wall changes physical body
motion while wheels may continue slipping; estimator pose is never collision
clamped.

## Mechanisms

Pass `MotorSimulationConfig` to `MakeMotorIO(Mode::Sim, vendor, config, model)`.
The legacy inertia-only overload retains the free motor/flywheel load. Select
`Load::Arm` with `mass`, `armCOM`, total `outputInertia`, and
`armAngleOffset` (zero physical arm angle is horizontal). Select
`Load::Elevator` with `mass`, `drumRadius`, and base `outputInertia`; reflected
payload inertia is added automatically. Both support `initialPosition` and
optional `minimum`/`maximum` physical stops in **output-shaft radians**. Elevator
travel is shaft angle times drum radius. Controller mechanism positions remain
rotations, or metres when configured with drum circumference per output turn.

Motor reads are observational. The shared runtime advances registered simulated
plants; do not also call `StepSimulation` on an automatically registered adapter.
A standalone custom IO plant may use explicit `StepSimulation(dt)` with one
owner. Real and replay IO never register with the physics runtime. Loads include
idealized gravity, electrical drag, current limits, brake/coast, and hard travel
stops; articulated contacts, battery sag, backlash, and structural compliance
are outside this model. Mechanism gravity uses the configured joint frame and
does not automatically rotate with chassis pitch or roll.

## Arena and reusable pieces

`sim::Arena` owns its colliders and pieces and registers with the shared clock.
The clock and robot-truth provider must outlive it. Pass the robot-local center
of mass as the final constructor argument when it is offset from the body origin;
truth linear velocity is measured at the center of mass. Use `AddCollider(BodySpec)`
for static geometry, `Spawn(PieceSpec)` for spheres/boxes, and `Pieces()` for
snapshots. Piece mass, friction, restitution, dimensions, initial pose/velocity,
and rolling deceleration are configurable. Planar ground pieces are constrained;
airborne launches use 3D motion in either backend. Landing restores ground
behavior. The seed controls reproducible scene placement, not asynchronous
Phoenix scheduling.

`SetIntake` configures the default robot-relative box; `RegisterIntake` creates
additional volumes. Capture needs an active intake and free capacity. Captured
pieces leave the collision world and belong to one intake. `Launch` releases
one possessed piece from its robot-relative muzzle pose with muzzle-local
launch velocity, plus chassis center-of-mass velocity and the rotational
velocity from the center of mass to the muzzle. There is no launch when possession is empty. Connect activation and
launch commands to mechanism state in a season project; the base bindings are
illustrative controller actions.

The generic 16 m by 8 m arena is centered on the origin. It includes walls, an
obstacle, seeded pieces, and a full-3D ramp. Season geometry, scoring, camera
images, and game-specific interactions belong downstream. `Arena::Reset(seed)`
recreates pieces and clears intake state; `SimulationClock::Reset()` resets
registered plants and time. Container scene reset performs the coordinated
robot placement used by the Y button.

## Timing and evidence

`Robot::SimulationPeriodic()` is the single runtime owner: one 20 ms period,
five 4 ms substeps, all physics callbacks on the robot thread. Snapshot reads
never step the world. Telemetry/rendering consumers must not mutate the world
from other threads. Truth timestamps use simulated elapsed time, while
Phoenix status and control updates retain vendor wall-clock timing.

The portable benchmark measures model/arena/Jolt work, excluding command
execution and telemetry. Run it on each intended development/practice machine;
CI runner results alone cannot establish that machine's deadline behavior.
Review p99 and individual misses, configuration, contacts, and body counts
alongside CPU/build metadata. See [the verification record](../artifacts/jolt/STATUS.md)
for retained successful and failed evidence. Simulation acceptance does not
establish hardware calibration, CAN timing, or physical safety.
