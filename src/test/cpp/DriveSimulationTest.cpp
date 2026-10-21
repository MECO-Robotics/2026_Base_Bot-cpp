#include "base/Drive.h"
#include <gtest/gtest.h>
#include <frc/simulation/DriverStationSim.h>
#include <ctre/phoenix6/unmanaged/Unmanaged.hpp>
#include <chrono>
#include <cmath>
#include <thread>

static void Case_DriveSimulationTest_DriveSimulation_PhoenixFeedbackTracksPhysicalMotionAndResetBoundaries() {
  auto& clock=base::sim::GetSimulationRuntime();
  if(!clock.World()) GTEST_SKIP()<<"Phoenix diagnostic backend has no Jolt world";
  // Full3D needs physical ground for tire contact; planar mode did not.
  base::sim::BodySpec floor;
  floor.halfExtents={20,20,.1};floor.pose.position.z=-.1;
  struct Ground {
    base::sim::SimulationWorld &world;
    base::sim::BodyHandle body;
    ~Ground() { world.DestroyBody(body); }
  } ground{*clock.World(),clock.World()->CreateBody(floor)};
  frc::sim::DriverStationSim::SetDsAttached(true);
  frc::sim::DriverStationSim::SetEnabled(false);
  frc::sim::DriverStationSim::NotifyNewData();
  frc::sim::DriverStationSim::SetEnabled(true);
  frc::sim::DriverStationSim::NotifyNewData();
  struct DisableOnExit {
    ~DisableOnExit() {
      ctre::phoenix::unmanaged::FeedEnable(0);
      frc::sim::DriverStationSim::SetEnabled(false);
      frc::sim::DriverStationSim::NotifyNewData();
    }
  } disable;
  base::DriveConfig config;config.couplingRatio=3.0;
  for(size_t i=0;i<4;++i) {
    config.modules[i].offsetTurns=.07*double(i+1);
    config.modules[i].driveInverted=(i==0||i==3);
    config.modules[i].steerInverted=(i==1||i==3);
    config.modules[i].encoderInverted=(i==2||i==3);
  }
  base::Drive drive(config);
  drive.ResetSimulation(frc::Pose2d{});
  // Phoenix's control/status threads run on wall time. This bounded integration
  // interval exercises real feedback, rather than asserting setpoint storage.
  for(int n=0;n<120;++n) {
    ctre::phoenix::unmanaged::FeedEnable(100);
    drive.RunVelocity(frc::ChassisSpeeds{units::meters_per_second_t{.8},units::meters_per_second_t{.3},units::radians_per_second_t{.3}});
    clock.StepRobotPeriod();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  auto truth=drive.SimulationTruth();ASSERT_TRUE(truth.has_value());
  auto estimate=drive.Pose();auto speeds=drive.Speeds();
  EXPECT_GT(truth->body.pose.position.x,.5);
  EXPECT_GT(estimate.X().value(),.5);
  EXPECT_NEAR(estimate.X().value(),truth->body.pose.position.x,.35);
  EXPECT_NEAR(estimate.Y().value(),truth->body.pose.position.y,.35);
  EXPECT_TRUE(std::isfinite(speeds.vx.value())&&std::isfinite(speeds.vy.value())&&std::isfinite(speeds.omega.value()));
  EXPECT_GT(speeds.vx.value(),.25);
  EXPECT_GT(truth->body.angularVelocity.z,.05);
  // Estimator resets must never teleport physical truth.
  drive.ResetPose(frc::Pose2d{units::meter_t{10},units::meter_t{5},frc::Rotation2d{units::degree_t{90}}});
  auto unchanged=drive.SimulationTruth();ASSERT_TRUE(unchanged.has_value());
  EXPECT_DOUBLE_EQ(unchanged->body.pose.position.x,truth->body.pose.position.x);
  EXPECT_DOUBLE_EQ(unchanged->body.pose.position.y,truth->body.pose.position.y);
  EXPECT_NEAR(drive.Pose().X().value(),10,1e-6);
  drive.Stop();
  drive.ResetSimulation(frc::Pose2d{units::meter_t{2},units::meter_t{3},frc::Rotation2d{units::degree_t{30}}});
  auto placed=drive.SimulationTruth();ASSERT_TRUE(placed.has_value());
  EXPECT_NEAR(placed->body.pose.position.x,2,1e-6);
  EXPECT_NEAR(placed->body.pose.position.y,3,1e-6);
  EXPECT_DOUBLE_EQ(placed->body.linearVelocity.x,0);
  EXPECT_NEAR(drive.Pose().X().value(),2,1e-6);
  EXPECT_NEAR(drive.Pose().Rotation().Degrees().value(),30,1e-6);
}

TEST(DriveSimulation, PhoenixFeedbackTracksPhysicalMotionAndResetBoundaries) {
  Case_DriveSimulationTest_DriveSimulation_PhoenixFeedbackTracksPhysicalMotionAndResetBoundaries();
}
