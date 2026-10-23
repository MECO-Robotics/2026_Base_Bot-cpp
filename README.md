# 2026 Base Bot C++

Reusable command-based robot code for WPILib 2026 and Team 8324. Use this repo
to learn the base components and build shared features. Season rules, robot IDs,
and autonomous choices belong in the downstream
[`2026-Rebuilt-cpp`](https://github.com/MECO-Robotics/2026-Rebuilt-cpp) project.
Values in this base are examples, not a commissioned robot configuration.

## Quickstart

Install WPILib 2026.2.1 with its C++ toolchains and Java 17 or newer. Run
commands from the repository root.

```sh
./gradlew build
./gradlew check
./gradlew simulateNative
```

`build` builds desktop and roboRIO targets; `check` runs desktop tests;
`simulateNative` opens the WPILib simulator. On Windows, use
`.\gradlew.bat` instead of `./gradlew`.
Full robot builds and simulation are supported on Windows and Linux. The pinned
Phoenix libraries do not support macOS; the standalone Jolt model in
`simulation/` does.

If Gradle cannot find the WPILib C++ toolchain, install WPILib 2026.2.1 with
support for the desktop and roboRIO platforms.

## Use as a vendor library

The reusable code is being separated into the `frcBase` static library. The
vendor descriptor and packaging workflow produce a Maven repository for the
reusable API. The workflow builds Linux desktop, roboRIO, and Windows artifacts,
then links clean consumer projects against each target. The repository is not
publicly installable until the workflow publishes the Maven repository to
GitHub Pages.

The workflow defaults to building and validating a downloadable package. Set
its `publish` input only when the package is ready to publish. Then add
`vendor/MecoBase.json` to a season robot with Manage Vendor Libraries.

The intended consumer API uses headers such as `base/Drive.h` and
`base/Mechanism.h`. Robot entry points such as `RobotContainer` remain owned by
the season robot.

`deploy` sends code to the configured roboRIO. Check the robot configuration
and get mentor approval before deploying or moving hardware.

## Find the right module

Each module groups related implementations under `src/main/cpp/base/` and
exposes its reusable API under `src/main/include/base/`. Start with the public
header; change implementation files when you need to alter its behavior.

| Module | Use it for | Start here |
| --- | --- | --- |
| `drive/` | Swerve control, drive commands, and hardware adapters | [`Drive.h`](src/main/include/base/Drive.h), [`source`](src/main/cpp/base/drive/) |
| `mechanism/` | Flywheels, joints, motor IO, and mechanism tuning | [`Mechanism.h`](src/main/include/base/Mechanism.h), [`source`](src/main/cpp/base/mechanism/) |
| `vision/` | Pose cameras, QuestNav, and game-piece detection | [`Vision.h`](src/main/include/base/Vision.h), [`source`](src/main/cpp/base/vision/) |
| `logging/` | WPILOG, input recording, and replay | [`Replay.h`](src/main/include/base/Replay.h), [`source`](src/main/cpp/base/logging/) |
| `telemetry/` | NetworkTables and AdvantageScope publishing | [`Telemetry.h`](src/main/include/base/Telemetry.h), [`source`](src/main/cpp/base/telemetry/) |
| `simulation/` | Desktop physics models and practice field | [`Jolt simulation guide`](docs/jolt-simulation.md), [`source`](simulation/) |

[`RobotContainer.cpp`](src/main/cpp/RobotContainer.cpp) shows how reusable
components are assembled. The base container currently creates the drivetrain;
add season-specific mechanisms and controls in the season project.

Here, “module” usually means a subsystem code area. The first C++20 named
module, `meco.base.drive_target`, is an internal drive-target state store. For
robot code, include the public `base/*.h` headers; Gradle handles the named
module interface for Linux, roboRIO, and Windows.

## Making a change

1. Find the public header and existing implementation before adding a component.
2. Keep hardware access in IO adapters. Subsystems own goals and safety;
   adapters own vendor calls and device units.
3. Keep units explicit: drive geometry uses metres and radians; mechanism
   coordinates use configured output rotations or metres.
4. Run relevant desktop checks. Simulation checks software behavior, not
   hardware calibration.
5. Format C++ with `clang-format -i <files>` using
   [`.clang-format`](.clang-format). Commit focused changes after checks pass.

[`AGENTS.md`](AGENTS.md) describes focused test guidance. Avoid blocking waits
and file or device IO in command callbacks and subsystem `Periodic()` methods.

## Learn more

- [Mechanism control](docs/mechanism-control.md) and [safe tuning](docs/mechanism-tuning.md)
- [Portable swerve](docs/portable-swerve.md) and [wheel-radius characterization](docs/wheel-radius-characterization.md)
- [Vision configuration](docs/vision-configuration.md), [QuestNav](docs/questnav.md), and [piece detection](docs/piece-detection.md)
- [Replay boundaries and workflow](docs/replay.md) and [AdvantageScope topics](docs/advantagescope.md)
- [Simulation setup and limits](docs/jolt-simulation.md) and [battery model](docs/simulation-power.md)
- [Port contract](docs/port-contract.md) for parity and validation boundaries

For C++ IntelliSense, run `./gradlew generateCompileCommands` and select a
database under `build/TargetedCompileCommands/` in the C/C++ extension. See the
[WPILib C++ setup guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/cpp-setup.html)
if the toolchain is not installed.
