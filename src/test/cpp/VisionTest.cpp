#include "base/Vision.h"
#include <gtest/gtest.h>
#include <networktables/NetworkTableInstance.h>
#include <vector>

namespace {
std::vector<double> SingleTagPose(double ambiguity) {
  return {1, 2, 0, 0, 0, 30, 20, 1, 0, 2, 1,
          1, 0, 0, 1, 2, 2, ambiguity};
}
}

TEST(VisionLimelight, SingleTagRequiresUsableAmbiguity) {
  base::VisionConfig config;
  config.name = "test-limelight-single-tag";
  int observations = 0;
  base::Vision vision(config, [&](const auto&) { ++observations; });
  auto publisher = nt::NetworkTableInstance::GetDefault().GetTable(config.name)
      ->GetDoubleArrayTopic("botpose_wpiblue").Publish();
  auto publish = [&](const std::vector<double>& pose) {
    publisher.Set(pose);
    vision.Periodic();
  };
  publish(SingleTagPose(config.maxSingleTagAmbiguity));
  ASSERT_EQ(observations, 1);
  publish(SingleTagPose(config.maxSingleTagAmbiguity + .01));
  publish(SingleTagPose(-1));
  auto truncated = SingleTagPose(0);
  truncated.resize(11);
  publish(truncated);
  EXPECT_EQ(observations, 1);
}

TEST(VisionLimelight, MegaTag2DoesNotUseSingleTagAmbiguity) {
  base::VisionConfig config;
  config.name = "test-limelight-megatag2";
  config.megatag2 = true;
  int observations = 0;
  base::Vision vision(config, [&](const auto&) { ++observations; },
                      [] { return frc::Rotation2d{}; });
  auto publisher = nt::NetworkTableInstance::GetDefault().GetTable(config.name)
      ->GetDoubleArrayTopic("botpose_orb_wpiblue").Publish();
  publisher.Set(SingleTagPose(.9));
  vision.Periodic();
  EXPECT_EQ(observations, 1);
}
