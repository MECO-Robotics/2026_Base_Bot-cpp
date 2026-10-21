#pragma once
#include <functional>
#include <utility>
#include <memory>
#include <string>
#include <array>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform3d.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <frc2/command/SubsystemBase.h>
namespace base {
struct VisionObservation {
  frc::Pose2d pose;
  units::second_t timestamp;
  std::array<double, 3> stdDevs;
};
struct VisionConfig {
  std::string name = "limelight";
  frc::Transform3d robotToCamera;
  double fieldLength = 17.548;
  double fieldWidth = 8.052;
  double maxZ = 0.75;
  double maxSingleTagAmbiguity = 0.3;
  bool megatag2 = false;
};
// Consumes Limelight blue-origin poses. Camera calibration and field layout remain deployment inputs.
class Vision : public frc2::SubsystemBase {
 public:
  Vision(VisionConfig config, std::function<void(const VisionObservation&)> consumer,
         std::function<frc::Rotation2d()> heading = {});
  void Periodic() override;
  bool Connected() const;
  double TargetYawDegrees() const;
 private:
  std::pair<bool,double> Status() const;
  VisionConfig config_;
  std::function<void(const VisionObservation&)> consumer_;
  std::function<frc::Rotation2d()> heading_;
  std::shared_ptr<nt::NetworkTable> table_;
  nt::DoubleArraySubscriber poses_;
  nt::DoubleSubscriber heartbeat_;
  nt::DoubleSubscriber targetYaw_;
  std::shared_ptr<nt::DoubleArrayPublisher> orientation_;
  std::shared_ptr<nt::DoubleArrayPublisher> cameraPose_;
};
}

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
namespace base {
class PhotonVision : public frc2::SubsystemBase {
 public:
  PhotonVision(VisionConfig config, frc::AprilTagFieldLayout layout,
               std::function<void(const VisionObservation&)> consumer);
  void Periodic() override;
  bool Connected();
 private:
  VisionConfig config_;
  std::unique_ptr<photon::PhotonCamera> camera_;
  std::unique_ptr<photon::PhotonPoseEstimator> estimator_;
  std::function<void(const VisionObservation&)> consumer_;
};
}
