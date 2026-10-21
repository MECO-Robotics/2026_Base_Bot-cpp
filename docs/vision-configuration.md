# Localization configuration

Both Limelight and PhotonVision accept `VisionConfig` minimum tag count,
optional allowed tag IDs and minimum count of allowed IDs. Whitelisting gates
observations; it does not alter an already solved camera pose. Empty ID lists
retain the original behavior. Ambiguity gates remain active for single-tag solves;
MegaTag2 retains its heading-assisted exemption and infinite yaw uncertainty.

Covariance remains distance squared / tag count, multiplied by configurable
linear/angular baselines and a per-camera factor. MegaTag2 has an additional
linear factor, default 1 to preserve prior C++ behavior (Java's .5 is an optional
tuning choice). These are estimator uncertainties, not measured accuracy claims.
Replay retains the existing accepted-observation boundary: it reruns downstream
localization/control, not camera solving or these upstream acceptance filters.
