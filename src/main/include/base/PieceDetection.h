#pragma once
#include <functional>
#include <optional>
#include <memory>
#include <string>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose3d.h>
#include <photon/PhotonCamera.h>
namespace base {
struct PieceRay {
  units::second_t timestamp;
  double yawDegrees{},pitchDegrees{}; // Photon: right-positive yaw, up-positive pitch.
};
struct PieceDetectionConfig {
  std::string name="piece-camera";
  frc::Transform3d robotToCamera;
  units::meter_t targetHeight{0};
  units::meter_t maxDistance{10};
  units::second_t freshness{.15};
};
// Horizontal target-plane intersection; no area-derived range assumption.
class PhotonPieceDetection : public frc2::SubsystemBase {
 public:
  using Source=std::function<std::optional<PieceRay>()>;
  PhotonPieceDetection(PieceDetectionConfig config,std::function<frc::Pose3d()> robotPose,Source source={});
  void Periodic() override;
  std::optional<frc::Translation3d> Target() const {return target_;}
 private:
  PieceDetectionConfig config_;
  std::function<frc::Pose3d()> robotPose_;
  Source source_;
  std::unique_ptr<photon::PhotonCamera> camera_;
  std::optional<frc::Translation3d> target_;
  std::optional<PieceRay> lastRay_;
};
}
