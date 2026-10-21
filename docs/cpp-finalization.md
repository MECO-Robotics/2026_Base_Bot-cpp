> Historical finalization-branch documentation (2026-10-19). Some APIs and topics below have been superseded. See [current merge resolutions](finalization-merge.md), [replay](replay.md), and [current parity contracts](parity-audit-2026-10-21.md).

# C++ migration review — 2026-10-19

Work is isolated on `codex/finalize-cpp-migration` in both C++ repositories. Java sources and original C++ worktrees are preserved. Generic implementations belong in `2026_Base_Bot-cpp`; the season merges their commits and retains hardware mappings, gains, game behavior and assets.

## Implemented and reviewed

| Area | Result |
|---|---|
| QuestNav | Pinned 2026-2.2.0 protobuf topics decoded with bundled nanopb and WPILib Pose3d; synchronized capture timestamps, restart detection, mounting transform, alignment, field/freshness rejection, explicit readiness, battery/version diagnostics. Season integrates corrected poses and absolute fallback. |
| Control | Commands publish intent at 50 Hz. One retained native Phoenix request resolves field transforms and proportional heading feedback independently, with exclusive vendor actuator access. Native DAQ remains Phoenix-owned. |
| Lifecycle | Nonblocking control-side target acquisition, epoch invalidation across disable, disabled neutral requests, finite-value guards and 100 ms moving-target expiry. X brake retains stationary latch semantics. Threads stop before their captured objects are destroyed. Shared reset observer synchronizes external localization. |
| CAN | Existing bus names and IDs retained; season explicitly requests 250 Hz. Native CAN FD handling is reused. Actual bus capability, utilization/errors, DAQ period/failures are visible without polling the bus on the control thread. |
| Logging/replay | Versioned MotorIO input records with original FPGA timestamps, native reader and forward-only replay. Fault values remain replayable. This is a native schema, not AdvantageKit import. |
| Tuning | Opt-in transactional disabled-only NT gains, enabled for all seven season mechanisms. Model pretuning estimates kV/kA/kG without overriding source PID or commissioned gains automatically. |
| Simulation | Native Phoenix plant gets a sole 4 ms notifier. Separate deterministic integer-tick harness runs five control/physics callbacks per command callback and supports bounded explicit transport latency. Season fuel reset reseeds its RNG. |
| Planning | Pinned PathPlanner already executes LocalADStar work on its worker thread. Existing ferry calculation is bounded to 51 candidates. No redundant planner framework or unused NN worker was introduced. |
| Safety parity | Invalid shot feedback blocks automatic feed; manual volley retains normal finite sag behavior but drops invalid feedback. Diagnostic Abort consumes simultaneous queued Start. |
| Portability | Vendor hardware and estimator types are private to Drive.cpp; intent resolution, Quest alignment, replay/latency and model helpers are independently testable. Motor hardware stays behind MotorIO. 2026 toolchain/vendor pins remain unchanged. |

## Scheduling and static numerical analysis

These are source-derived estimates, not CPU benchmarks or physical measurements. C++ migration, scheduling changes and algorithmic replacements are separate effects.

- Phoenix already repeatedly applied retained swerve requests on native ticks before this change. This work does **not** turn a previously 50 Hz module controller into a newly 250 Hz module controller. The change moves the field transform and heading feedback from Commands to that existing native loop and avoids installing a new request on every command tick.
- At a nominal 250 Hz, the native period is 4 ms versus a 20 ms command period: five feedback evaluations per command update. With uniformly distributed arrivals and no jitter, feedback sampling wait drops from mean 10 ms / maximum 20 ms to mean 2 ms / maximum 4 ms. The target/joystick still updates at 50 Hz; sensor, CAN, vendor processing and OS delays are additional. Base CAN 2.0 defaults to 100 Hz, so its corresponding native figures are 10 ms and mean 5 ms.
- The 100 ms moving-request watchdog selects a zero-velocity request on the next native evaluation: nominally within 104 ms of the last publication at 250 Hz. This is not a hard real-time bound or a braking-distance guarantee. DriverStation/hardware disable remains authoritative. Explicit Stop is the normal command-end path.
- The intent packet is 152 bytes in the inspected x86-64 ABI: one copy per 250 Hz tick represents 38,000 bytes/s. Layout can differ on roboRIO/Systemcore. It contains fixed arrays; force vectors allocate four entries once at construction, then overwrite existing elements. Target consumption and snapshot publication use try-lock and retain the preceding valid sample on contention.
- Pose/speed dashboard publication goes from 50 Hz to at most 10 Hz: at most 20 calls/s instead of 100, an 80% reduction for those two existing arrays. CAN health polling is at most 1 Hz and stays on the command thread. Native telemetry callbacks copy only a fixed pose/speed/DAQ snapshot, avoiding vendor state-vector copies in each public getter.
- This is **not** an allocation-free or lock-free Phoenix pipeline. Pinned `ApplyRobotSpeeds::Apply` calls vendor kinematics returning `std::vector<SwerveModuleState>`; vendor state operations have their own mutexes. Their compiled implementation costs were not benchmarked. NetworkTables queues, DataLog and 50 Hz tuning can allocate/lock outside the application control callback.
- Model pretuning uses `kV = G*(2*pi/u)/Kv`, `kA = J*(2*pi/u)*R/(Kt*G*efficiency)`, `kG = gravityTorque*R/(Kt*G*efficiency)`, where `u` is physical units per output revolution and the DCMotor describes the whole motor group. Gravity torque can be supplied as mass*gravity*COM for an arm or mass*gravity*drumRadius for an elevator. This ideal model excludes friction, compliance, current limiting, backdrive efficiency differences and battery sag. It provides no justified PID feedback gains without a specified closed-loop model/bandwidth.
- The offline integer scheduler allocates no storage itself; callback cost is separate. Latency push is O(1), receive is O(number of due samples), and storage is fixed capacity. Replay reads are O(1); advancing is amortized O(number of samples). Full queues reject explicitly and deliveries must be nondecreasing.

## Numerical parity and intentional differences

Copied CAN IDs, offsets, ratios, mounting transforms, source feedforwards/PID and season algorithms are retained. Existing numerical and mechanism regressions remain active. Heading regression checks Java's wrapped proportional law and saturation against the native resolver. Motor pretuning is checked against WPILib DCMotor voltage and rotational/linear coordinate equivalence. This is focused numerical evidence, not a complete Java/C++ differential execution harness.

Quest alignment initializes on the first compatible absolute/Quest pair and averages up to 15 translation offsets, as Java does. Unlike Java, this adapter rejects capture skew over 30 ms and tracking age at least 120 ms, resets translation history when its yaw frame changes, and gates readiness on a successfully validated field pose. These are intentional correctness/safety differences. The protocol is read-only: local resets align the field transform rather than issuing asynchronous headset reset requests.

## Remaining gaps and limits

- No physical validation, roboRIO deployment, motor activation, CAN FD load benchmark, real-time jitter measurement, headset calibration or hardware commissioning was performed.
- Whole-robot reproducible replay is incomplete: current motor input schema does not restore commands, enable/mode changes, configuration/tuning chronology, external joint sensor callbacks, or native drivetrain estimator internals. Java AdvantageKit logs, URCL and vendor hoot imports are not implemented by this schema.
- Deterministic multirate/latency tools are offline support, not a replacement for the entire native robot runtime. Phoenix's live control and plant threads remain wall-clock scheduled. Automatic QuestNav noise/drift simulation and MapleSim field contacts are absent. Native NT fixtures verify wire compatibility without a physical headset.
- QuestNav request/response ACKs and headset-origin reset commands are absent. Existing Limelight filtering still differs from Java's MegaTag1/MegaTag2/reference-pose policy. Optional SparkMax/NavX swerve combinations, rack wall compliance, detailed CAD/gravity plants and piece-localization parity remain open.
- The native bounded diagnostic runner remains a replacement for the Java browser suite; port5806 dashboard, full CAN topology/device-configuration audit and HTML/CSV report parity are not supplied. Existing diagnostic safety and JSON reporting are retained, with CANivore health telemetry added.
- There is no NN inference implementation. Future expensive inference/collision work must use bounded immutable inputs, a cancellable worker and generation-tagged results; it must never own actuators or block the native control callback. Existing AD* already follows its library worker model. No custom unused worker was added.
- 2027 Systemcore build/runtime/vendor support has not been established. The private native implementation and portable tested control/IO contracts reduce coupling; this is preparation, not a claim that the 2026 binary targets new hardware. Validate the actual 2027 WPILib/Phoenix API and ABI before changing build targets.

## Validation evidence

See `artifacts/base-finalization-validation.json` upstream and `artifacts/finalization-validation.json` in the season fork for current build, native test and simulator results. Existing historical evidence is preserved under its original filename. All simulation scripts control only local HAL simulation and leave the robot disabled. Hardware evidence is explicitly absent.


The final checks passed 28 base tests and 51 season tests in each of debug and release, with desktop and roboRIO debug/release builds passing. Native desktop base/season drive checks, season mechanism binding/output cleanup and Choreo checks passed. Choreo endpoint error was 0.0241 m against the existing 0.4 m ideal-model acceptance bound. The disabled-only Quest fixture passed alignment, mounting/relative-motion fusion, absolute fallback, combined diagnostic connectivity, tracking loss, staleness, field rejection, version/battery reporting and stationary-output checks. It injects synthetic camera/headset measurements, not a sensor physics model.

A read-only season desktop sample reported configured odometry 250 Hz and average native period 3.98346 ms, with the cumulative failed-DAQ counter at 1. This is neither a worst-case jitter measurement nor evidence of physical CAN performance. All seven mechanism gain arrays were present and their native versioned log records were decoded from the running simulator's file. Individual JSON artifacts and build logs retain raw values. Historical evidence was not overwritten.

Hardware mapping blocks, source mechanism constants, Choreo implementation and deployed assets match season baseline `9c498ec`, except the explicit 250 Hz rate selection. Generic `src/main/include/base` and `src/main/cpp/base` implementations match upstream byte-for-byte. The pre-existing Java base `gradlew` working-tree change was preserved.

Commit inventories through the validated source revisions are stored in `artifacts/base-finalization-commits.json` upstream and `artifacts/finalization-commits.json` downstream. These separate parity, control/concurrency, telemetry/optimization, simulation, tuning/replay and evidence changes. Later evidence-only commits do not change the validated runtime.


## AdvantageScope follow-up

Standard WPILib struct topics now identify Pose2d, planar Pose3d, ChassisSpeeds and measured/target SwerveModuleState arrays. The season adds Pose3d arrays for numerical floor/hopper/projectile positions. Publication is capped at 10 Hz on the robot thread; the native telemetry callback only copies fixed-size state and module arrays with a capture timestamp. Legacy topics remain intact. NT4 values and their schemas are automatically captured by DataLogManager into standard WPILOG files. See `docs/advantagescope.md` for the parser-backed check and version-matched layout workflow. This closes visualization compatibility; it does not close the separate whole-robot replay, sensor-physics or physical-validation gaps above.

## Explicit-time follow-up

The later [explicit-time audit](explicit-time-control.md) supersedes the mechanism timing assumptions in this historical review. Motor reads are now observational; explicit control and plant intervals support deterministic mechanism/scheduler/latency integration. Whole-robot replay and live Phoenix virtual-time determinism remain open and are not implied by these mechanism tests. Fresh evidence is recorded under `artifacts/time-step-*`.
