# Native macOS model support

The user selected **native Jolt models/tests/benchmarks first**. Full robot
Phoenix controls, sensors, WPILib mechanism adapters, Xbox/HAL practice, and
AdvantageScope robot telemetry remain Windows/Linux workflows.

Source `64c6d231bf5dc70a224e56c27825b6ae80f85bd9` passed all four Mac jobs in
[run 39491159060](https://github.com/MECO-Robotics/2026_Base_Bot-cpp/actions/runs/39491159060):
Apple Silicon (`macos-15`, arm64) and Intel (`macos-15-intel`, x86_64), each in
Debug and Release. Each job passed the world, drivetrain physics, and arena suites,
plus three benchmark CLI contracts. `ci-model-results.json` preserves direct job
links and conclusions; Release CTest logs are retained beside it. The inherited
Windows/Linux pipeline runs separately in the same workflow; Mac acceptance here
refers specifically to these four completed jobs.

## Performance measurements

All measurements use the same pinned Jolt 5.3.0 source and complete registered
model/arena/Jolt substep. Hardware, Apple Clang version, flags, body/contact counts,
revision, samples, median/p99, and misses are preserved in each JSON.

| Hardware/run | Dense planar p99 | Dense 3D p99 | 4 ms budget |
|---|---:|---:|---|
| Apple M1 (Virtual), initial | 1.05 ms | 0.77 ms | passed all scenes |
| Intel i7-8700B, initial | 10.13 ms | 8.36 ms | failed dense scenes |
| Intel i7-8700B, repeat | 7.39 ms | 8.96 ms | failed dense scenes |
| Apple M1 (Virtual), final | 1.17 ms | 1.35 ms | passed all scenes |
| Intel i7-8700B, final | 2.21 ms | 1.74 ms | passed all scenes |

Final reports (`benchmark-arm64.json`, `benchmark-intel.json`) had zero observed
4 ms misses in all six scenarios. Both earlier Intel failures are retained in
`benchmark-intel-initial.json` and `benchmark-intel-repeat.json`; they must not be
reclassified as successful qualification. Comparable body/contact counts and two
failed runs establish timing variability, but wall-time measurements alone do not
identify its cause. No physics parameters, solver settings, or thresholds were
changed to obtain the final measurements.

Mac CI requires functional tests and publishes performance qualification separately.
`--report-only` allows a completed measurement to exit successfully when its budget
fails, while preserving `acceptance.passed=false`, timings, and miss counts. CI emits
warnings and a job summary for budget failures. The default CLI and Windows/Linux
CI still enforce the 4 ms gate. Empty/incomplete/duplicate/nonfinite results and
benchmark process failures remain errors; focused CLI tests protect that boundary.

Native model portability is verified. Intended Mac practice-machine timing remains
unqualified until measured on that machine; these hosted-runner results do not
establish a reliable 4 ms Intel deadline. Full native Mac robot practice needs a
separate follow-up. See the README for Command Line Tools/CMake/Ninja/Python setup
and standalone commands. Text logs normalize whitespace for Git; raw logs remain
available from the linked jobs.
