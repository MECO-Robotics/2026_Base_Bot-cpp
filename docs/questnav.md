# QuestNav localization

`base::QuestNav` consumes QuestNav **2026-2.2.0** `frameData` protobuf packets
from `/<config.name>/frameData`. Native NT subscriptions request **5 ms** delivery,
retain duplicates and drain the timestamped queue once per scheduler tick. This
requests transport frequency; it does not change the 20 ms command scheduler or
guarantee the headset's publication rate.

Supply the measured robot-to-headset `Transform3d` and a consumer (normally
`Drive::AddVision`). Once fresh tracking is available, call
`AlignToField(robotPose)` to establish the local tracking-space to blue-origin
field transform. This is local alignment, not the remote QuestNav POSE_RESET
command. Tracking loss, stale receive gaps and headset restarts invalidate the
alignment; an application must deliberately align again. Bounds and maximum Z
are configurable gates. Illustrative covariance defaults require tuning.

Replay records the raw queued protobuf bytes and robot receive timestamps, then
reruns parsing, mount correction, gates and alignment logic. Replay constructs no
NT subscriber. Simulation may publish the same protocol topic; no headset physics
or synthetic tracking is supplied.

`QuestNavFixtures.h` contains golden packets generated independently using the
QuestNav Java 2026-2.2.0 generated `Data.ProtobufQuestNavFrameData`, Quickbuf 1.4
and WPILib 2026.2.1. They encode headset poses (2,3,1.2,yaw90) and
(3,3,1.2,yaw180). The native decoder delegates nested WPILib Pose3d decoding to
WPILib nanopb and tolerates unknown outer protobuf fields. This adapter was tested
with protocol fixtures, not commissioned against a physical headset.

QuestNav acquisition also implements `VisionIO` via `MakeQuestNavVisionIO` and can
share one `VisionSystem` with camera localization. The existing `QuestNav` facade
keeps local alignment/status APIs and one subsystem registration. Status reads
are passive snapshots from the latest tick; `AlignToField` updates aligned status
immediately on acceptance.
