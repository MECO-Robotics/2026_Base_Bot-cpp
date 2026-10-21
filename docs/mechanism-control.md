# Mechanism control

Software PID/feedforward remains the default for `Flywheel` and `PositionJoint`.
To use a Talon FX onboard voltage loop on real hardware, set
`MotorConfig::phoenix.enabled=true` before `MakeMotorIO`. Existing APIs and gain
units are unchanged: output rotations for flywheels, and joint units configured
by `unitsPerOutputRotation` for position joints. The adapter converts setpoints,
PID, velocity/acceleration feedforward and limits into Phoenix mechanism rotations.
Static and gravity gains remain volts. Software-profile requests include velocity
and acceleration feedforward; FOC is explicitly disabled to avoid requiring Pro.

- `SetVelocity` sends acceleration-limited setpoints to `VelocityVoltage`.
- `SetPosition` uses ordinary `MotionMagicVoltage`, with the configured gain
  limits. It does not require Dynamic Motion Magic or a Phoenix Pro license.
- `SetPositionDynamic` retains the bounded software trajectory and sends its
  positions to onboard `PositionVoltage`. This avoids synchronous CAN
  configuration in the enabled control loop and the Pro/CANivore requirement of
  Phoenix's dynamic request.
- `SetPositionUnprofiled` sends an unprofiled onboard `PositionVoltage` goal.
  Software-only adapters retain their existing position profile.

`MotorConfig::gearRatio` maps integrated rotor turns to output turns. For a
CANcoder, select `PhoenixControlConfig::Feedback::RemoteCANcoder`, set
`remoteSensorId`, and set `sensorToMechanismRatio` to sensor turns per output
turn (normally 1 for an output-mounted encoder). `sensorReversed` selects its
positive direction. Configure and retain ownership of the CANcoder separately,
including its zero offset and publication rate; it must share the Talon's CAN
bus. `rotorToSensorRatio` describes the rotor/sensor relationship. Both position
and velocity then come from the same Phoenix-selected feedback source. Fused and
synchronized licensed feedback modes are intentionally not selected.

The primary `reversed` flag sets motor direction; subsequent flags oppose the
primary through Phoenix followers. Verify motor and remote-sensor positive
feedback direction before enabling any mechanism. Numeric/native unit tests and
simulation do not establish hardware direction, calibration or stability.

Disabled, disconnected, invalid/nonfinite feedback, stop and command cancellation
all send zero voltage, overriding any native request. Physical software limits
are also configured on the primary Talon; logical `ResetPosition()` does not
move those physical limits or rewrite the sensor. Runtime native gain changes
require disabled state and stop the previous request before applying bounded
Phoenix configuration calls. Configuration failures throw rather than silently
running an unconfigured controller. Native gravity supports constant/elevator
and cosine/arm behavior; nonzero sine gravity requires the software controller.
Do not attach a separate external encoder callback to a native-controlled joint.

Desktop plants deliberately use the existing software control path and explicit
plant clock, without constructing Talon controllers. This is a model-level
fallback, not firmware simulation. Replay constructs only injected IO: logs
record whether the source used native control, and native sources recompute
`Mechanism/<name>/NativeRequest` (kind, physical target, velocity, acceleration and profile limits).
Firmware voltage is a recorded measured input, not a recomputed result. Simulation
sources retain software control during replay. This boundary supports comparing
new setpoint logic without claiming to replay Phoenix firmware.

Existing `SysIdQuasistatic` and `SysIdDynamic` already use WPILib SysId routines,
record measured applied voltage/current and mechanism units, and stop on final
completion/cancellation. They override native loops through `SetVoltage`; no
parallel characterization abstraction is needed.
