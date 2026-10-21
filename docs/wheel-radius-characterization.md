# Wheel radius commissioning

`Drive::CharacterizeWheelRadius()` returns an opt-in command requiring the drive.
No default binding starts it. Clear a large level area before deliberately scheduling
it on hardware. The command slews at 0.25 rad/s² to 0.5 rad/s, allows one second
for steering to settle, then measures one full turn; a 35-second timeout bounds
motion. Completion, interruption, and invalid feedback stop the chassis.

The calculation uses raw gyro heading, signed wrapped increments, and measured
module distances from either backend divided by the configured radius to recover coupling-
corrected wheel angles. Each module uses its own distance from the chassis origin.
This supports asymmetric module locations and avoids accumulating absolute gyro
noise. It does not change configuration. Traction, geometry, gearing, and gyro
accuracy still affect physical results; repeat runs and inspect module agreement
before choosing a measured radius. This command rotates counterclockwise only.

A result is valid only after at least 2π net heading change, with fresh healthy
feedback and nonzero wheel travel. Module radius spread over 20%, implausible
radii, feedback gaps over 100 ms, or heading jumps invalidate the result. The
single aggregate `/AdvantageScope/Drive/WheelRadiusResult` JSON topic contains
`valid`, `interrupted`, `yawRadians`, and four `radiiMeters`. Failed or interrupted
runs may contain partial estimates; consult `valid` first. Publication is on the
existing low-priority telemetry worker. `Drive/WheelRadius` records the raw input
boundary and `Drive/WheelRadiusResult` records the recomputed output for code
replay. This is commissioning software, not hardware calibration evidence.
