# Jolt implementation evidence

## Baseline

- Reviewed the seven pre-existing modified C++ files: identical ordered lexical tokens; formatting only.
- Preserved them in commit `4f2049b`.
- `./gradlew build --max-workers=2` passed: desktop and roboRIO debug/release builds; 25 native tests passed in each configuration. See `baseline-build.log`.
- Merged the audit branch into main at `6692e655b10309eb0e927a864c3c16f6b464725e`, pushed main, and created `feature/jolt-simulation`.

## Implementation contract

Required platforms are **Windows and Linux**, per the user's subsequent scope clarification. Keep WPILib 2026.2.1 and vendor versions pinned. Jolt starts at 5.3.0, commit `0373ec0dd762e4bc2f6acdb08371ee84fa23c6db`.

Modules proceed through separate scoped planning, implementation, and reviewer/test agents. The coordinator owns shared contracts and integration. Module A's dependency/world gate must pass on both platforms before dependent implementation.

| Module | Planning | Implementation | Review/test |
|---|---|---|---|
| A: build/world | complete | in progress | pending |
| B: drivetrain | complete | pending A gate | pending |
| C: mechanism loads/timing | complete | pending A gate | pending |
| D: arena/pieces | complete | pending A gate | pending |
| E: practice/evidence | pending | pending integration | pending |

Physics defaults are illustrative assumptions, not measured robot parameters. Build, numerical simulation, real executable behavior, and physical calibration are separate evidence categories.
