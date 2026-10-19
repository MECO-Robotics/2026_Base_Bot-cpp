#include "base/Vision.h"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <frc/RobotController.h>
#include <networktables/NetworkTableInstance.h>
namespace base {
Vision::Vision(VisionConfig config, std::function<void(const VisionObservation&)> consumer,
               std::function<frc::Rotation2d()> heading)
    : config_(std::move(config)), consumer_(std::move(consumer)), heading_(std::move(heading)),
      table_(nt::NetworkTableInstance::GetDefault().GetTable(config_.name)),
      poses_(table_->GetDoubleArrayTopic(config_.megatag2 ? "botpose_orb_wpiblue" : "botpose_wpiblue").Subscribe({})),
      heartbeat_(table_->GetDoubleTopic("hb").Subscribe(0)),
      targetYaw_(table_->GetDoubleTopic("tx").Subscribe(0)),
      orientation_(table_->GetDoubleArrayTopic("robot_orientation_set").Publish()),
      cameraPose_(table_->GetDoubleArrayTopic("camerapose_robotspace_set").Publish()) {
  if (config_.megatag2 && !heading_) throw std::invalid_argument("MegaTag 2 requires heading supplier");
  const auto& t = config_.robotToCamera;
  cameraPose_.Set(std::array<double, 6>{t.X().value(), t.Y().value(), t.Z().value(),
      units::degree_t{t.Rotation().X()}.value(), units::degree_t{t.Rotation().Y()}.value(),
      units::degree_t{t.Rotation().Z()}.value()});
}
bool Vision::Connected() const {
  const auto changed = heartbeat_.GetLastChange();
  return changed > 0 && frc::RobotController::GetFPGATime() - changed < 250000;
}
double Vision::TargetYawDegrees() const { return targetYaw_.Get(); }
void Vision::Periodic() {
  if (config_.megatag2) orientation_.Set(std::array<double, 6>{heading_().Degrees().value(), 0, 0, 0, 0, 0});
  for (const auto& sample : poses_.ReadQueue()) {
    const auto& v = sample.value;
    if (v.size() < 11) continue;
    bool finite = true;
    for (double value : v) finite &= std::isfinite(value);
    if (!finite || v[7] < 1 || v[9] <= 0 || v[6] < 0 || std::abs(v[2]) > config_.maxZ ||
        v[0] < 0 || v[0] > config_.fieldLength || v[1] < 0 || v[1] > config_.fieldWidth) continue;
    // MT1 needs usable single-tag ambiguity (raw fiducial element 17).
    // MT2 resolves this ambiguity using the supplied robot heading.
    if (!config_.megatag2 && v[7] == 1 &&
        (v.size() <= 17 || v[17] < 0 || v[17] > config_.maxSingleTagAmbiguity)) continue;
    const double scale = v[9] * v[9] / v[7];
    if (!std::isfinite(scale) || scale <= 0) continue;
    VisionObservation observation{
      frc::Pose2d{units::meter_t{v[0]}, units::meter_t{v[1]}, frc::Rotation2d{units::degree_t{v[5]}}},
      units::second_t{sample.time * 1e-6 - v[6] * 1e-3},
      {0.02 * scale, 0.02 * scale, config_.megatag2 ? std::numeric_limits<double>::infinity() : 0.06 * scale}};
    if (consumer_) consumer_(observation);
  }
}
}

namespace base {
PhotonVision::PhotonVision(VisionConfig config, frc::AprilTagFieldLayout layout,
                          std::function<void(const VisionObservation&)> consumer)
    : config_(std::move(config)), camera_(config_.name),
      estimator_(std::move(layout), config_.robotToCamera), consumer_(std::move(consumer)) {}
void PhotonVision::Periodic() {
  for (const auto& result : camera_.GetAllUnreadResults()) {
    auto estimate = estimator_.EstimateCoprocMultiTagPose(result);
    if (!estimate) estimate = estimator_.EstimateLowestAmbiguityPose(result);
    if (!estimate || estimate->targetsUsed.empty()) continue;
    const auto& pose = estimate->estimatedPose;
    if (!std::isfinite(pose.X().value()) || !std::isfinite(pose.Y().value()) ||
        !std::isfinite(pose.Z().value()) || !std::isfinite(pose.Rotation().Z().value()) ||
        std::abs(pose.Z().value()) > config_.maxZ || pose.X().value() < 0 ||
        pose.Y().value() < 0 || pose.X().value() > config_.fieldLength ||
        pose.Y().value() > config_.fieldWidth) continue;
    if (!std::isfinite(estimate->timestamp.value())) continue;
    if (estimate->targetsUsed.size() == 1) {
      const double ambiguity = estimate->targetsUsed[0].GetPoseAmbiguity();
      if (!std::isfinite(ambiguity) || ambiguity < 0 ||
          ambiguity > config_.maxSingleTagAmbiguity) continue;
    }
    double distance = 0;
    for (const auto& target : estimate->targetsUsed) distance += target.GetBestCameraToTarget().Translation().Norm().value();
    distance /= estimate->targetsUsed.size();
    const double scale = distance * distance / estimate->targetsUsed.size();
    if (!std::isfinite(scale) || scale <= 0) continue;
    if (consumer_) consumer_({pose.ToPose2d(), estimate->timestamp, {0.02 * scale, 0.02 * scale, 0.06 * scale}});
  }
}
}
