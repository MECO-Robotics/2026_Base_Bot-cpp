# Desktop simulation power

The simulation clock samples each registered plant's battery-side current once after its five 4 ms physics substeps. Reads never step a plant. `Robot::SimulationPeriodic` uses WPILib `BatterySim`'s default 12 V / 20 mΩ model and sets both the next period's model bus and HAL roboRIO input voltage. Scene reset restores both to 12 V. Real and replay modes do not run this path.

Simulated mechanisms clamp the requested voltage to the shared bus, retain that request as voltage recovers, and report actual terminal voltage. Current limiting reduces terminal voltage; battery current is nonnegative terminal electrical power divided by bus voltage, rather than absolute winding current. Braking dissipates energy without charging this simplified battery. Multiple motors contribute once each. Direct standalone `StepSimulation` starts with a 12 V supply and does not access HAL.

Jolt drivetrain draw includes drive and steering motors using their limited winding current, back EMF, and effective terminal voltage. Its frontend clamps controller voltages to the shared bus. The Phoenix diagnostic backend instead sums asynchronously cached controller supply-current signals; it cannot promise deterministic timing or equivalent electrical fidelity.

Canonical asynchronous telemetry is `/Simulation/BatteryVoltage` and `/Simulation/SupplyCurrent`. This is an illustrative aggregate electrical model: it omits controller losses, accessories, battery state of charge, cable voltage drops, and regeneration charging. It is software simulation evidence, not measured battery or hardware calibration.

Drivetrain friction is the existing lumped voltage-drop approximation, not a detailed Coulomb-friction motor model. Battery terminal-power accounting restores that signed drop after the winding-current solve, including when current limiting reduces the effective terminal voltage.
