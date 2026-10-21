# Game-piece detection

`PhotonPieceDetection` returns an optional field-space intersection with a
configured horizontal target-height plane. It uses Photon target yaw/pitch,
full robot and camera mount rotation, and camera position. Parallel/upward rays,
nonfinite geometry, negative or excessive range, future/stale observations and
no-target observations clear the result. It does not infer range from image area.

A source callback returning optional `PieceRay` enables deterministic simulation
without constructing a camera. Native Photon mode drains unread results and uses
the newest result, retaining a measurement only until its freshness deadline when
no new frame arrives. An explicit no-target frame clears it immediately.
Replay records raw rays/clock and recomputes projection using robot pose; no
camera is constructed. Camera calibration, object pipeline and target height are
installation inputs. This is planar target localization, not object classification
or an occlusion/image simulator.
