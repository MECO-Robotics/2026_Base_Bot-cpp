#include "base/PieceDetection.h"
#include "base/Replay.h"
#include <frc/RobotController.h>
#include <cmath>
#include <numbers>
#include <stdexcept>
namespace base {
PhotonPieceDetection::PhotonPieceDetection(PieceDetectionConfig config,std::function<frc::Pose3d()> robotPose,Source source)
    :config_(std::move(config)),robotPose_(std::move(robotPose)),source_(std::move(source)) {
  if(config_.name.empty()||!robotPose_||!std::isfinite(config_.targetHeight.value())||
      !std::isfinite(config_.maxDistance.value())||config_.maxDistance<=units::meter_t{0}||
      !std::isfinite(config_.freshness.value())||config_.freshness<=units::second_t{0})
    throw std::invalid_argument("Invalid piece detection configuration");
  const auto q=config_.robotToCamera.Rotation().GetQuaternion();
  for(double v:{config_.robotToCamera.X().value(),config_.robotToCamera.Y().value(),config_.robotToCamera.Z().value(),q.W(),q.X(),q.Y(),q.Z()})
    if(!std::isfinite(v))throw std::invalid_argument("Invalid piece camera mount");
  if(!source_&&!replay::Active())camera_=std::make_unique<photon::PhotonCamera>(config_.name);
}
void PhotonPieceDetection::Periodic() {
  const auto input=replay::ProcessInputs("PieceDetection/"+config_.name,[this] {
    if(source_)lastRay_=source_();
    else for(const auto &result:camera_->GetAllUnreadResults()) {
      lastRay_.reset();
      if(result.HasTargets()) {
        const auto &t=result.GetBestTarget();
        lastRay_=PieceRay{result.GetTimestamp(),t.GetYaw(),t.GetPitch()};
      }
    }
    wpi::json value={{"now",frc::RobotController::GetFPGATime()*1e-6},{"ray",nullptr}};
    if(lastRay_&&std::isfinite(lastRay_->timestamp.value())&&std::isfinite(lastRay_->yawDegrees)&&std::isfinite(lastRay_->pitchDegrees))
      value["ray"]={lastRay_->timestamp.value(),lastRay_->yawDegrees,lastRay_->pitchDegrees};
    return value;
  });
  target_.reset();
  if(input.at("ray").is_null())return;
  const auto &r=input.at("ray");
  const double age=input.at("now").get<double>()-r.at(0).get<double>();
  if(age<0||age>config_.freshness.value())return;
  const double yaw=-r.at(1).get<double>()*std::numbers::pi/180;
  const double pitch=r.at(2).get<double>()*std::numbers::pi/180;
  const auto camera=robotPose_().TransformBy(config_.robotToCamera);
  const auto direction=frc::Translation3d{units::meter_t{std::cos(pitch)*std::cos(yaw)},
      units::meter_t{std::cos(pitch)*std::sin(yaw)},units::meter_t{std::sin(pitch)}}.RotateBy(camera.Rotation());
  if(!std::isfinite(direction.Z().value())||std::abs(direction.Z().value())<1e-9)return;
  const double distance=(config_.targetHeight-camera.Z()).value()/direction.Z().value();
  if(!std::isfinite(distance)||distance<=0||distance>config_.maxDistance.value())return;
  const auto hit=camera.Translation()+direction*distance;
  if(std::isfinite(hit.X().value())&&std::isfinite(hit.Y().value())&&std::isfinite(hit.Z().value()))target_=hit;
}
}
