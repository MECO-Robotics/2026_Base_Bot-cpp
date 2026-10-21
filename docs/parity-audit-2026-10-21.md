# Java/C++ parity audit

Comparison source: Java `MECO-Robotics/2026_Base_Bot` origin/main
`4e3422f` (read-only worktree `/tmp/frc-parity-java`). The original Java checkout
remains unchanged, including its pre-existing executable-bit edit to `gradlew`.
C++ implementation starts from feature revision `453ecba`, with the existing
reviewed working changes preserved in commits `7baeb80`, `3d8be18`, `55b5717`,
and `f1da8be` on `audit/java-parity-2026-10-21`.

C++ `main` (`6692e65`) is already an ancestor of the feature baseline. There are
no missing main commits to merge or cherry-pick. No merge, push, deployment, or
hardware motion is part of this audit.

## Selection and review

Independent planners compare mechanisms, vision/localization, and runtime/drive.
Separate implementation agents work in isolated worktrees. Each concern receives
an independent reviewer and focused checks before integration by cherry-pick.
Further audits check the integrated changes and the remaining omission list.
The native C++ architecture remains authoritative; Java is behavioral reference,
not a source of mandatory class-by-class translations.

## Baseline findings

| Concern | Finding and intended disposition |
| --- | --- |
| Mechanisms | Restore opt-in Phoenix onboard control and coherent feedback; keep software control and all safety checks. |
| Mechanism SysId | Already implemented using WPILib, actual applied voltage/current, and interruption stop. Review rather than duplicate. |
| Swerve | Retain Phoenix native control/odometry; 250 Hz on CAN FD, 100 Hz on CAN 2.0. Default `rio` bus does not imply 250 Hz. |
| Drive configuration | Validate finite parameters, physical dimensions, per-device-type CAN identity and nondegenerate geometry before constructing devices. |
| Drive SysId | Add native translation characterization with WPILib command ownership, measured SI data, and stop cleanup. |
| QuestNav | Complete 5 ms receiver and raw-packet replay; fix stale-gap alignment before fresh queue processing. |
| Camera vision | Add count/ID filters and configurable covariance; retain MT2 ambiguity exemption and native Photon estimation. |
| Piece localization | Add reusable detection with correct 3D ray/mount geometry and freshness; do not copy Java's stale/unchecked projection behavior. |
| Simulation power | Add shared supply aggregation and WPILib battery sag after the mechanism IO interface is frozen. |
| Tuning/feedforward | Restore coherent, opt-in tuning and a validated WPILib model utility; preserve command ownership and replay. |
| Async/replay/physics | Preserve lower-priority snapshot workers, ordered frames, single explicit physics clock, Jolt contacts and portable model support. |

Validation and final acceptance are recorded alongside the completed concern
commits, not inferred from this initial inventory. Desktop numerical checks do
not establish physical calibration or vendor firmware equivalence.
