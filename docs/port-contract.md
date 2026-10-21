# Port contract

| Requirement | Owner | Acceptance |
|---|---|---|
| C++ base is independent of game behavior | base repo | Review source boundaries; build native targets |
| Season repo derives from completed base commit | season fork | GitHub parent metadata and Git ancestor check |
| Preserve Java originals | migration | Source working trees unchanged |
| WPILib and vendor APIs match 2026 pinned versions | build.gradle/vendordeps | Desktop and roboRIO native compilation/link |
| SI field geometry, rotations for mechanisms, metres for linear joints | owning configs | Unit conversion and numerical regression tests |
| Motor command ownership and cleanup | commands | Native subsystem and command lifetime tests |
| Honest parity and validation claims | README | Record framework replacements and unsupported functionality |

Sources: initial Java base commit `d1f264954190b87d7cde839e1661c430cecdb8d3`, parity re-audit against Java origin/main `4e3422f` (see [parity audit](parity-audit-2026-10-21.md)); season source commit `1797a83bfeac6318178f1886da5e24ece9b8b9d1`. The source Java base's pre-existing executable-bit change to `gradlew` is unrelated and preserved.

The Java configuration owns copied CAN IDs, ratios, offsets and tuned values. Base drive constants are illustrative; season overrides own the real season configuration. No mechanical component selection or physical parameter redesign is included. Existing source tuning is retained as source data, not reclassified as a hardware measurement.

Assumptions: the requested “1016-specific” means 2026-specific; native replacements for Java-only frameworks require separate commissioning. WPILib build/test evidence covers authored code and numerical/control semantics only. Physical feasibility, field contact simulation, real sensor timing, hardware acceptance and deployment are outside the evidence obtained by this migration.

API references: [WPILib command ownership](https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html), [Phoenix swerve](https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/mechanisms/swerve/swerve-overview.html). The actual pinned vendor headers are the compile-time API authority.

Native swerve drive gains use rotor turns, unlike the Java base's wheel-turn closed loop. The base drive P/D/V defaults are divided by the drive gear ratio; static voltage and azimuth-turn steering gains are unchanged. The season Java CTRE swerve gains are already rotor-based and must be copied directly. This distinction was confirmed against the pinned Phoenix 26.1.2 `SwerveModuleConstants.hpp` and `SwerveModule.hpp` headers and a running native simulation.

## Finalization branch integration

The historical finalization branch is merged with conflict resolutions recorded in
[the merge notes](finalization-merge.md). Current Jolt clock ownership, canonical
asynchronous telemetry, native mechanism control and full frame replay take
precedence over superseded implementations from that branch. Offline explicit-time
control and legacy mechanism-log readers remain available without adding a second
robot simulation clock or duplicate live logging.
