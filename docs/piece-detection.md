# Game-piece detection

`PhotonPieceDetection` returns an optional field-space intersection with a
configured horizontal target-height plane. It uses Photon target yaw/pitch,
full robot and camera mount rotation, and camera position. Parallel/upward rays,
nonfinite geometry, negative or excessive range, future/stale observations and
no-target observations clear the result. It does not infer range from image area.

A source callback returning optional `PieceRay` enables deterministic simulation
without constructing a camera. Native Photon mode drains unread results and uses
the newest result, retaining a measurement only until its freshness deadline when
no new frame arrives. Its field intersection remains fixed while retained. An explicit no-target frame clears it immediately.
Replay records raw rays/clock and recomputes projection using robot pose; no
camera is constructed. Camera calibration, object pipeline and target height are
installation inputs. This is planar target localization, not object classification
or an occlusion/image simulator.

Projection uses robot pose when a new observation is consumed, without capture-time
pose history compensation. Camera latency during robot motion can therefore bias
the initial intersection; this component does not provide motion compensation.

`PieceDetectionIO::ReadRay()` separates acquisition from projection. The public
header contains no Photon types. `MakePhotonPieceDetectionIO` supplies native
Photon acquisition; `InjectedPieceDetectionIO` supplies typed rays. Pass either
adapter to the `PhotonPieceDetection` constructor; the existing callback
constructor remains supported. Replay substitutes the same recorded ray input
before the adapter is called, so it never acquires live camera data.
