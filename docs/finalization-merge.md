# Finalization branch merge

Integrates `codex/finalize-cpp-migration` (`601b841`) into main (`1804b01`).
The branches diverged at `a2e8a6e`; this is a real merge with conflict resolution,
not a fast-forward or a replacement of current main with the older branch.

## Retained additions

- Explicit mechanism `Control(dt[, enabled])`, `UseExternalScheduling`, and
  observational telemetry; the normal scheduler still controls at 20 ms.
- Offline deterministic 4 ms scheduling, bounded delayed-sensor delivery and
  timestamped `ReplayMotorIO`. `MotorInputLog` remains a standalone legacy codec,
  not a second live recording pipeline.
- Validated optional odometry frequency, autonomous PID gains and GUI configuration;
  defaults remain unchanged. `RunHeading` applies bounded robot-relative heading
  control at the caller's rate, not a new native control-thread loop.
- Pose-reset observers for estimator, explicit physical and world-clock resets,
  invoked after sensor/estimator synchronization and outside locks.
- Standalone `DriveTarget` intent helper and its stale/disable-epoch tests. It is
  not installed as another active drivetrain request; SysId and replay retain
  current native request semantics.
- Asynchronous visualizer metadata and the installed AdvantageScope parser/layout
  tool, adapted to canonical physical truth and without duplicate drive topics.
- Historical branch build/parser/simulation artifacts, with historical docs marked
  and current explicit-time/visualization instructions updated.

## Conflict resolutions

Current native Phoenix mechanism control, software safety, full frame replay,
transactional tuning, model feedforward, Jolt world ownership, load/battery models,
canonical asynchronous telemetry, and 5 ms raw-packet QuestNav remain authoritative.
The older branch's Phoenix simulation Notifier is not restored: it would advance
physics separately from the Jolt owner. Its alternative QuestNav class/10 ms
transport, live gain arrays, input-log writer and drive topic aliases are
superseded by current implementations. The current PathPlanner weak-owner proxy
is retained rather than restoring unsafe global controller ownership.

[Explicit-time control](explicit-time-control.md), [visualization](advantagescope.md),
[code replay](replay.md), and [parity audit](parity-audit-2026-10-21.md) describe the
current boundaries. Independent scoped planning, implementation and review were
used for mechanism and drivetrain conflict resolutions. Fresh validation results
are recorded below. No hardware deployment or push is
part of this operation.

## Final validation

- Linux desktop and roboRIO Debug/Release compilation and linking passed.
- All **64 native tests** passed in each configuration; the focused merge gate
  passed 33 tests. Debug was also run directly after detecting a cached filtered
  task; the final build executed both complete suites after the last adjustment.
- The fresh actual 3D robot executable passed all **34 smoke checks**.
- The installed AdvantageScope 26.0.0 parser accepted the fresh recording and
  generated a layout using canonical measured speed, module states and physical
  truth. The historical branch sample also passed parser validation.
- Code replay completed **1,024 frames**, preserving inputs/clock/source identity,
  with zero rejected or failed recorder frames. Only the existing native-versus-
  WPILib `Drive/ModuleTargets` boundary differed (613 frames); no new simulation-
  reset-only output remains. Physics is not constructed or stepped during replay.

[Validation evidence](../artifacts/finalization-merge-validation.json) records the
source hash `b3f427e2d3dba6333dd47320f7020bf989777d954dccd753306aeb76106100d3`.
The build reports `dirty=true` at parent revision `1804b01` because validation
ran against the resolved merge working tree before committing. The hash matches
all resolved production/test/simulation/dependency files and build configuration.
Historical artifacts keep their original revisions; they are not substituted for
these fresh results. Hardware, fresh Windows/macOS execution, GUI rendering and
Phoenix firmware determinism remain outside this validation.
