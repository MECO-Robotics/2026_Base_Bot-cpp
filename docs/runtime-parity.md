# Runtime parity decisions

Drive configuration is validated before any Phoenix or Jolt construction, including
replay. Physical dimensions, inertias, gearing and current limits must be finite
and positive; suspension damping must be positive; friction/gains must be nonnegative. Module locations must be
distinct and span a plane. CAN IDs are 0–62 and unique within device type on the
configured bus: TalonFX drive/steer identities share one namespace, CANcoders use
another, and the Pigeon may share a numeric address with either.

These checks establish usable numerical inputs, not physical calibration.
Phoenix continues to own native odometry/control timing (250 Hz on CAN FD,
100 Hz on CAN 2.0 by its pinned default); no additional drivetrain loop is added.
