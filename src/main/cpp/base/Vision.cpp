#include "base/Vision.h"
#include "base/Replay.h"
#include "base/Telemetry.h"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <frc/RobotController.h>
#include <networktables/NetworkTableInstance.h>
namespace base
{
  namespace {
    wpi::json Encode(const VisionObservation &o) {
      auto deviations=wpi::json::array();
      for(double value:o.stdDevs) deviations.push_back(std::isfinite(value)?wpi::json(value):wpi::json(nullptr));
      return {{"pose",{o.pose.X().value(),o.pose.Y().value(),o.pose.Rotation().Radians().value()}},
              {"timestamp",o.timestamp.value()},{"stdDevs",deviations}};
    }
    VisionObservation Decode(const wpi::json &o) {
      const auto &p=o.at("pose");const auto &d=o.at("stdDevs");
      if(p.size()!=3||d.size()!=3) throw std::runtime_error("Invalid replay vision observation");
      std::array<double,3> deviations;
      for(size_t i=0;i<3;++i) deviations[i]=d.at(i).is_null()?std::numeric_limits<double>::infinity():d.at(i).get<double>();
      return {frc::Pose2d{units::meter_t{p.at(0).get<double>()},units::meter_t{p.at(1).get<double>()},
               frc::Rotation2d{units::radian_t{p.at(2).get<double>()}}},units::second_t{o.at("timestamp").get<double>()},deviations};
    }
  }
  Vision::Vision(VisionConfig config, std::function<void(const VisionObservation &)> consumer,
                 std::function<frc::Rotation2d()> heading)
      : config_(std::move(config)), consumer_(std::move(consumer)), heading_(std::move(heading))
  {
    if (config_.megatag2 && !heading_)
      throw std::invalid_argument("MegaTag 2 requires heading supplier");
    if(replay::Active()) return;
    table_=nt::NetworkTableInstance::GetDefault().GetTable(config_.name);
    poses_=table_->GetDoubleArrayTopic(config_.megatag2?"botpose_orb_wpiblue":"botpose_wpiblue").Subscribe({});
    heartbeat_=table_->GetDoubleTopic("hb").Subscribe(0);
    targetYaw_=table_->GetDoubleTopic("tx").Subscribe(0);
    orientation_=std::make_shared<nt::DoubleArrayPublisher>(table_->GetDoubleArrayTopic("robot_orientation_set").Publish());
    cameraPose_=std::make_shared<nt::DoubleArrayPublisher>(table_->GetDoubleArrayTopic("camerapose_robotspace_set").Publish());
    const auto &t = config_.robotToCamera;
    const auto cameraPose=std::array<double, 6>{t.X().value(), t.Y().value(), t.Z().value(),
                                          units::degree_t{t.Rotation().X()}.value(), units::degree_t{t.Rotation().Y()}.value(),
                                          units::degree_t{t.Rotation().Z()}.value()};
    telemetry::Post("VisionConfig/"+config_.name,[publisher=cameraPose_,cameraPose]{publisher->Set(cameraPose);});
  }
  std::pair<bool,double> Vision::Status() const {
    const auto status=replay::ProcessInputs("Vision/"+config_.name+"/Status",[this] {
      const auto changed=heartbeat_.GetLastChange();
      const auto yaw=targetYaw_.Get();
      return wpi::json{{"connected",changed>0&&frc::RobotController::GetFPGATime()-changed<250000},
                       {"yaw",std::isfinite(yaw)?wpi::json(yaw):wpi::json(nullptr)}};
    });
    return {status.at("connected").get<bool>(),status.at("yaw").is_null()?std::numeric_limits<double>::quiet_NaN():status.at("yaw").get<double>()};
  }
  bool Vision::Connected() const { return Status().first; }
  double Vision::TargetYawDegrees() const { return Status().second; }
  void Vision::Periodic()
  {
    (void)Status();
    if (config_.megatag2) {
      const auto orientation=std::array<double,6>{heading_().Degrees().value(),0,0,0,0,0};
      replay::RecordOutput("Vision/"+config_.name+"/Orientation",orientation);
      if(!replay::Active()) telemetry::Post("VisionOrientation/"+config_.name,[publisher=orientation_,orientation]{publisher->Set(orientation);});
    }
    const auto observations=replay::ProcessInputs("Vision/"+config_.name+"/Observations",[this] {
    auto accepted=wpi::json::array();
    for (const auto &sample : poses_.ReadQueue())
    {
      const auto &v = sample.value;
      if (v.size() < 11)
        continue;
      bool finite = true;
      for (double value : v)
        finite &= std::isfinite(value);
      if (!finite || v[7] < 1 || v[9] <= 0 || v[6] < 0 || std::abs(v[2]) > config_.maxZ ||
          v[0] < 0 || v[0] > config_.fieldLength || v[1] < 0 || v[1] > config_.fieldWidth)
        continue;
      // MT1 needs usable single-tag ambiguity (raw fiducial element 17).
      // MT2 resolves this ambiguity using the supplied robot heading.
      if (!config_.megatag2 && v[7] == 1 &&
          (v.size() <= 17 || v[17] < 0 || v[17] > config_.maxSingleTagAmbiguity))
        continue;
      const double scale = v[9] * v[9] / v[7];
      if (!std::isfinite(scale) || scale <= 0)
        continue;
      VisionObservation observation{
          frc::Pose2d{units::meter_t{v[0]}, units::meter_t{v[1]}, frc::Rotation2d{units::degree_t{v[5]}}},
          units::second_t{sample.time * 1e-6 - v[6] * 1e-3},
          {0.02 * scale, 0.02 * scale, config_.megatag2 ? std::numeric_limits<double>::infinity() : 0.06 * scale}};
      accepted.push_back(Encode(observation));
    }
    return accepted;
    });
    for(const auto &observation:observations) if(consumer_) consumer_(Decode(observation));
  }
}

namespace base
{
  PhotonVision::PhotonVision(VisionConfig config, frc::AprilTagFieldLayout layout,
                             std::function<void(const VisionObservation &)> consumer)
      : config_(std::move(config)), consumer_(std::move(consumer)) {
    if(!replay::Active()) {
      camera_=std::make_unique<photon::PhotonCamera>(config_.name);
      estimator_=std::make_unique<photon::PhotonPoseEstimator>(std::move(layout),config_.robotToCamera);
    }
  }
  bool PhotonVision::Connected() {
    return replay::ProcessInputs("PhotonVision/"+config_.name+"/Connected",[this]{return wpi::json(camera_->IsConnected());}).get<bool>();
  }
  void PhotonVision::Periodic()
  {
    (void)Connected();
    const auto observations=replay::ProcessInputs("PhotonVision/"+config_.name+"/Observations",[this] {
    auto accepted=wpi::json::array();
    for (const auto &result : camera_->GetAllUnreadResults())
    {
      auto estimate = estimator_->EstimateCoprocMultiTagPose(result);
      if (!estimate)
        estimate = estimator_->EstimateLowestAmbiguityPose(result);
      if (!estimate || estimate->targetsUsed.empty())
        continue;
      const auto &pose = estimate->estimatedPose;
      if (!std::isfinite(pose.X().value()) || !std::isfinite(pose.Y().value()) ||
          !std::isfinite(pose.Z().value()) || !std::isfinite(pose.Rotation().Z().value()) ||
          std::abs(pose.Z().value()) > config_.maxZ || pose.X().value() < 0 ||
          pose.Y().value() < 0 || pose.X().value() > config_.fieldLength ||
          pose.Y().value() > config_.fieldWidth)
        continue;
      if (!std::isfinite(estimate->timestamp.value()))
        continue;
      if (estimate->targetsUsed.size() == 1)
      {
        const double ambiguity = estimate->targetsUsed[0].GetPoseAmbiguity();
        if (!std::isfinite(ambiguity) || ambiguity < 0 ||
            ambiguity > config_.maxSingleTagAmbiguity)
          continue;
      }
      double distance = 0;
      for (const auto &target : estimate->targetsUsed)
        distance += target.GetBestCameraToTarget().Translation().Norm().value();
      distance /= estimate->targetsUsed.size();
      const double scale = distance * distance / estimate->targetsUsed.size();
      if (!std::isfinite(scale) || scale <= 0)
        continue;
      accepted.push_back(Encode({pose.ToPose2d(), estimate->timestamp, {0.02*scale,0.02*scale,0.06*scale}}));
    }
    return accepted;
    });
    for(const auto &observation:observations) if(consumer_) consumer_(Decode(observation));
  }
}
