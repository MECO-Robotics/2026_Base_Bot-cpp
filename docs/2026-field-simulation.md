# Approximate 2026 REBUILT collision scene

`FRC_SIM_FIELD=rebuilt-2026` selects this scene (desktop practice default).
`FRC_SIM_FIELD=generic` retains the original generic practice scene and portable
benchmarks keep their generic default. Both use the fixed blue-corner WPILib
frame: metres, +X downfield, +Y left, +Z up. Startup/reset remains `(8,4)`.

Dimensions are taken from the [2026 game manual, sections 5.4–5.8](https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf)
and [field dimension drawings, revision B](https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf).
The field is 651.22 × 317.69 inches. Hubs are 47-inch-square blocking boxes at
X=182.11/469.11 inches, Y=158.845 inches. Their 2 m collision height is an
approximation, without scoring openings. The perimeter is rectangular and
omits wall recesses.

Full3D adds four bump envelopes: each is 44.4 inches deep in X, 73 inches wide
in Y, and 6.513 inches high, with 15-degree slopes. Two thick rotated boxes per
bump extend below the floor. Their 22.2-inch horizontal half-profile meets at
the crest; the nominal dimensions imply a roughly 0.565-inch toe lip in this
approximation. This is a driving/contact approximation, not a fabrication
model. Planar mode omits bumps to retain flat, fast automation.

Full3D depots are open U-shaped curbs, 27 inches deep by 42 inches wide, with
3-inch-wide, 1.125-inch-high rails. Blue depot center Y is 234.85 inches and red
is 82.84 inches. There is no filled platform. Planar mode omits the low curbs.

Each tower has two distinct poles in both modes, spaced 33.75 inches center to
center, with 1.5-inch Y thickness, 3.51-inch X depth and 72.125-inch height.
Blue/red tower center Y is 147.47/170.22 inches. Pole center X=41.755 inches
(mirrored for red): controlled drawing GE-26500 page 28 places the pole back
face 40 inches from the alliance wall; half the 3.51-inch depth locates its
center. Bases, rungs and braces are omitted.

Four ground support boxes beside the trenches block both modes. Each is
47 inches deep in X, centered at the hub X, and spans Y=50.345–62.345 or
255.345–267.345 inches. Their 22.25-inch box height approximates the support
envelope shown in the field drawings. There are no overhead trench arms or
colliders spanning the drivable trench lanes.

The startup console and `SmartDashboard/Simulation/PhysicsMode` report the active
physics mode. An AdvantageScope 3D field view does not change the physics backend:
`jolt-planar` deliberately keeps the chassis flat and omits bump/depot contacts.
Start bump practice with `FRC_SIM_PHYSICS=jolt-3d ./gradlew simulateNative`.

For collision debugging, `/Simulation/Colliders/BoxN` contains static `Pose3d[]`
wireframe paths generated from the actual collider geometry, including rotated
bumps. Add these as **Trajectory** objects in AdvantageScope's 3D field view;
each path traces all twelve box edges. These are diagnostic outlines, not solid
rendered objects. Use `RobotTruth` for robot height/pitch; `/AdvantageScope/Drive/Pose` remains
a planar odometry pose.

This optional season scene contains geometry only: no scoring, game rules,
climbing attachments, camera rendering or calibrated robot model. Drive tests
check representative bump traversal and depot contact, not hardware fidelity.

Arena owns the generated static bodies; reset replaces pieces without duplicating
geometry. Focused contracts query the crest/slopes, open trench lanes, ground
supports, corrected pole positions and pole gaps,
exercise bump traversal in both directions with the default drivetrain, require
flat planar motion, and exercise low depot curb contact.

The planning, implementation, and independent review passes completed with
[Linux validation evidence](../artifacts/jolt/rebuilt-field-validation.json).
All four portable suites passed; the default chassis rose about 8 cm and
pitched about 14.7 degrees crossing the bump in either direction, then settled.
Depot contact produced about 8 mm chassis rise. Both running robot modes passed
25 smoke checks. The smoke tracking windows stay inside the neutral zone to
avoid running into hubs. Desktop Debug/Release tests and roboRIO builds passed;
Windows and Mac were not executed for this change.

Correction validation: tower placement now follows the controlled back-face
dimension, with regressions that fail for the previous shifted poles or missing
trench supports. All four portable suites and native Debug/Release tests passed;
both actual robot modes passed 30 checks, including typed collider outlines.
Those correction runs retained the then-current planar default. Desktop practice
now defaults to `jolt-3d`; `FRC_SIM_PHYSICS=jolt-planar` explicitly selects planar.
The validation JSON retains the original results and
the correction results separately.
