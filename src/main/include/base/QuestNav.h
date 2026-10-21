#pragma once
#include "base/Vision.h"
#include <optional>
#include <networktables/RawTopic.h>
#include <frc/geometry/Pose3d.h>
namespace base {
struct QuestNavConfig {
  std::string name="QuestNav";
  frc::Transform3d robotToQuest;
  // Illustrative estimator uncertainties; measure/tune for the installed headset.
  std::array<double,3> stdDevs{.1,.1,.05};
  double fieldLength=17.548,fieldWidth=8.052,maxZ=.75;
  double freshnessSeconds=.12;
};
// Native QuestNav 2026-2.2.0 frameData receiver. NT receives at 5 ms; Periodic
// drains the timestamped queue on the scheduler thread. No headset reset command.
class QuestNav : public frc2::SubsystemBase {
 public:
  static constexpr double kSubscriptionPeriod=.005;
  static constexpr std::string_view kFrameType="proto:questnav.protos.data.ProtobufQuestNavFrameData";
  QuestNav(QuestNavConfig config,std::function<void(const VisionObservation&)> consumer);
  void Periodic() override;
  bool Connected() const {return connected_;}
  bool Tracking() const {return connected_&&tracking_;}
  bool Aligned() const {return Tracking()&&aligned_;}
  // Local robot-field alignment using the most recent fresh tracked sample.
  // Returns false until Periodic has received one. Re-align after tracking loss,
  // staleness or headset restart. This does not send QuestNav's POSE_RESET command.
  bool AlignToField(const frc::Pose2d &robotPose);
 private:
  QuestNavConfig config_;
  std::function<void(const VisionObservation&)> consumer_;
  nt::RawSubscriber frames_;
  std::optional<frc::Pose3d> rawRobot_;
  frc::Pose3d rawAnchor_,fieldAnchor_;
  int64_t lastReceive_{},lastTimestamp_{};
  double lastAppTimestamp_{};
  uint64_t lastFrame_{};
  bool haveFrame_{},connected_{},tracking_{},aligned_{};
};
}
