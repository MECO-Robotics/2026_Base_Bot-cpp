# Code replay and traceability

The framework records one versioned `Replay/Frames` JSON record per robot cycle
in the primary WPILOG. The file uses the AdvantageKit WPILOG header and also
contains timestamped typed `ReplayInputs` and `ReplayOutputs` entries for
AdvantageScope and other WPILOG readers. Desktop replay reads native
`Replay/Frames`, installs recorded Driver Station state and FPGA timestamps,
and reruns the existing robot mode callbacks, scheduler, commands, and IO-owned
mechanism controller emulation. It does not feed recorded actuator outputs
back to the code. New outputs go to a new WPILOG.

Desktop replay also accepts Java AdvantageKit 26.0.2 logs for the base robot's
default Drive and Driver Station boundary. It maps `DriveState/Pose`,
`DriveState/Speeds`, `DriveState/ModuleStates`, logged Driver Station state and
joystick values, and the optional `NetworkInputs/SmartDashboard/Auto Choices`
selection. The Java log does not identify the C++ drivetrain backend, so set
`FRC_DRIVE_BACKEND=phoenix` or `FRC_DRIVE_BACKEND=rev` when that differs from
the default Phoenix backend. Other Java mechanism, vision, and device IO schemas
are not mapped; a robot that requires those inputs fails replay rather than
reading live devices. AdvantageKit's Java reader can open the C++ output and
recover its typed input entries; its replay reader intentionally filters
`ReplayOutputs`, which remain present in the raw WPILOG.

AdvantageScope can open either log for normal playback; code replay is a separate
operation. WPILib NT topics are stored with an `NT:` prefix in WPILOG, so a
logged module source appears as `NT:/AdvantageScope/Drive/ModuleStates`; live
NT uses `/AdvantageScope/Drive/ModuleStates`. Select the corresponding source
in each view.

Each recorded cycle also exposes timestamp-aligned JSON snapshots at
`Replay/Inputs` and `Replay/Outputs` for direct inspection in WPILOG viewers.
These duplicate the frame's input and recomputed output maps; `Replay/Frames`
remains the authoritative replay source. `Replay/RecordingStatus` is sampled
once per second and finalized at orderly shutdown, with accepted/rejected/failed
counts and queue high-water mark.

## Run a replay

Build the current code, then run with a Python environment containing the same
2026 WPILib Python packages used by the headless smoke tools:

```sh
./gradlew prepareHeadlessSimulation
python scripts/sim_headless.py --replay-log logs/source.wpilog \
  --replay-output artifacts/replayed.wpilog \
  --replay-report artifacts/replay-comparison.json
```

On Windows use `gradlew.bat`. This workflow supports Linux and Windows robot
executables. The headless comparison helper requires a native log containing
`Replay/Frames`. A Java AdvantageKit log can be passed to the robot executable
directly with `FRC_REPLAY_LOG` and `FRC_REPLAY_OUTPUT`; Java logs need the exact
typed fields listed above. An old NT-only log without those fields cannot
reconstruct missing sensor inputs. Output files must be new:
the tool refuses to overwrite an existing log. The process exits at end of input,
without loading the HAL websocket extension or running physics. Offline replay
waits between small frame batches for the background writer, preventing a fast
replay burst from overflowing its buffers. The recorded clock stays paused during
these disk waits. A bounded EOF flush retains short runs before shutdown.

For a directly launched desktop executable, set `FRC_REPLAY_LOG` and
`FRC_REPLAY_OUTPUT` to absolute source and new output WPILOG paths. The launcher
also supplies `FRC_REPLAY_SOURCE_SHA256`. Replay is rejected on roboRIO.
The headless launcher restores the recorded drivetrain backend (older logs default
to Phoenix). When bypassing it for a portable recording, also set
`FRC_DRIVE_BACKEND=rev` before constructing the base container.

The comparison report verifies exact frame sequence, source timestamps, DS state,
and recorded inputs, then reports differences in recomputed outputs. Differences
are expected when code changes; they are evidence to inspect, not automatically
an error. `Replay/Metadata` records source metadata, source file identity/hash,
current Git revision, dirty status, and a SHA256 of source/deploy files, simulation sources, vendordeps, and
`build.gradle`. Compiler binaries, flags supplied outside that file, and environment
variables are not part of that content hash. Save both logs,
the report, and the corresponding source revision for a traceable comparison.
A dirty-tree hash identifies content but does not preserve that content for you.

## Recorded boundary

- Drive: source backend, estimated pose, measured robot-relative chassis speeds, and measured
  module states. Requested chassis motion, module targets, pose-reset requests,
  and vision-fusion requests are recomputed and logged.
- Mechanisms: motor position, velocity, applied/supply voltage, current,
  connectivity, external encoder feedback, and source control ownership. Limiting,
  commands and disconnect stops run again. Simulated-source PID/feedforward runs
  in injected IO; onboard sources reconstruct native setpoint requests. Recorded
  firmware voltage is an input, not a recomputed firmware result.
- Vision: connectivity, target yaw, and accepted timestamped pose observations
  with uncertainty. Camera acquisition, calibration, pose estimation, and
  observation rejection occur before this boundary and are not rerun.
- Autonomous chooser selection is recorded; replay selects the corresponding
  command from current code/assets and fails if a recorded command is missing.
- Digital inputs and Driver Station modes, alliance/location, and six joystick
  ports are substituted from the recording.

Vendor devices and Jolt worlds are not constructed during replay. Drivetrain
estimators, motor firmware, asynchronous control timing, and sensor fusion
are outside the replay boundary. A replayed pose reset is an output request; it
cannot alter the next recorded pose. Replayed module targets use WPILib
kinematics/optimization and do not establish bitwise parity with native Phoenix
setpoints. This supports command/controller traceability, not a counterfactual
physical trajectory after changing controls.

New external inputs must use `base::replay::ProcessInputs(key, liveReader)`;
new outputs can use `RecordOutput`. Output records retain the last value per key
within a cycle; they are not an ordered trace of every write or vision request.
Keys must be stable and unique. The first
read in a cycle captures the input; subsequent reads return that same snapshot.
Missing input keys, incompatible schemas, or non-increasing timestamps fail
instead of silently consulting live hardware. Nonfinite mechanism feedback uses
JSON null and retains its fault behavior; unbounded vision uncertainty also uses
null. Call the replay initializer before constructing any hardware adapters.
User-created device objects or uninstrumented NT/configuration reads need the
same boundary before a downstream subsystem can claim replay support.

## Control and publishing priorities

The robot thread captures coherent input frames and control outputs, then moves
completed snapshots into a dedicated bounded FIFO recorder. A lower-priority
`Frame-Record` worker serializes JSON and appends it to WPILib's buffered DataLog;
WPILib's background writer performs disk IO. The recorder and live NT publisher
use separate workers so a slow dashboard cannot delay recording. Input capture,
validation, and brief queue synchronization still cost control-thread time.
There is no live per-cycle flush, serialization, or disk wait.

The recorder holds up to 256 pending frames and preserves accepted frames in
order without coalescing. Saturation or a serialization/append exception marks
recording invalid, emits `Replay/RecordingFault`, and rejects subsequent frames;
controls continue running. Both replay readers reject a fault-marked source.
`Replay/RecordingStatus` records counts and queue high-water at orderly shutdown.
Abrupt process termination can still lose buffered tail records. Shutdown drains
and joins the recorder before stopping the AdvantageKit-compatible WPILOG
writer. Offline replay applies writer backpressure between batches; this never
runs in live periodic callbacks.

Scheduler/commands, sensor capture, mechanism control, and fixed-step physics keep
one owner on the robot thread. Moving these stateful stages to independent
threads would change callback order and feedback timing. Phoenix retains its
vendor-owned asynchronous control/odometry threads.

Application NT publishers receive immutable snapshots on a dedicated worker.
Both application workers use Linux nice 10 or Windows below-normal thread
priority. Pending live
updates coalesce by key and the queue is bounded. The worker never reads motors,
estimators, or physics objects. Drive and simulation samples retain their capture
timestamps. Dashboard lag can skip intermediate live samples; it cannot coalesce
`Replay/Frames`, which uses a separate logging path. WPILib/vendor internal NT
transport and sendable infrastructure retain their own implementation threads;
this policy covers application publishers, not a blanket OS priority guarantee
for every library thread. The WPILib disk writer's priority is not configured by
this framework.

Motor input frames include `configuration` (0 Ready, 1 Pending, 2 Fault), recording
the asynchronous hardware configuration boundary. Replay substitutes this state
before control and tuning consume it, so pending/fault inhibition and the need
for a fresh post-configuration request are reproduced without a worker or device.
`asynchronousConfiguration` records source control ownership and
`configurationFailure` (0 none, 1 gains, 2 brake, 3 coast) preserves recovery
operation restrictions. Older frames without these fields retain synchronous
Ready behavior. The firmware
configuration transaction itself remains outside replay, like onboard PID.
