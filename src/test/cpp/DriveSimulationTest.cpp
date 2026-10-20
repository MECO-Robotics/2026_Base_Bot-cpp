#include "base/Drive.h"
#include <gtest/gtest.h>
#include <frc/simulation/DriverStationSim.h>
#include <ctre/phoenix6/unmanaged/Unmanaged.hpp>
#include <chrono>
#include <cmath>
#include <thread>
#include <string>
#include <iostream>

TEST(DriveSimulation, PhoenixFeedbackTracksPhysicalMotionAndResetBoundaries) {
  auto& clock=base::sim::GetSimulationRuntime();
  if(!clock.World()) GTEST_SKIP()<<"Phoenix diagnostic backend has no Jolt world";
  frc::sim::DriverStationSim::SetDsAttached(true);
  frc::sim::DriverStationSim::SetEnabled(false);
  frc::sim::DriverStationSim::NotifyNewData();
  // Device status frames can arrive before configuration transactions work.
  // Exercise the same configuration transport that module construction uses,
  // and verify a nondefault value returned by a fresh configuration readback.
  ctre::phoenix6::hardware::TalonFX startupProbe{60,"rio"};
  startupProbe.GetSimState().SetSupplyVoltage(units::volt_t{12});
  const auto startupTime=std::chrono::steady_clock::now();
  const auto readyDeadline=startupTime+std::chrono::seconds(10);
  ctre::phoenix6::configs::MotorOutputConfigs requested;
  requested.PeakForwardDutyCycle=0;
  requested.PeakReverseDutyCycle=0;
  bool ready=false;
  std::string transactionStatus;
  int attempts=0;
  do {
    ++attempts;
    auto applied=startupProbe.GetConfigurator().Apply(requested,units::second_t{.1});
    transactionStatus=applied.GetName();
    if(applied.IsOK()) {
      ctre::phoenix6::configs::MotorOutputConfigs actual;
      auto refreshed=startupProbe.GetConfigurator().Refresh(actual,units::second_t{.1});
      transactionStatus=refreshed.GetName();
      ready=refreshed.IsOK()&&std::abs((actual.PeakForwardDutyCycle-requested.PeakForwardDutyCycle).value())<1e-6&&std::abs((actual.PeakReverseDutyCycle-requested.PeakReverseDutyCycle).value())<1e-6;
    }
  } while(!ready&&std::chrono::steady_clock::now()<readyDeadline);
  ASSERT_TRUE(ready)<<"Phoenix desktop configuration transport did not complete verified apply/readback: "<<transactionStatus;
  std::cout<<"[DriveSimulation] Verified configuration transport after "<<std::chrono::duration<double>(std::chrono::steady_clock::now()-startupTime).count()<<" s, "<<attempts<<" attempt(s); constructing modules"<<std::endl;
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
  std::cout<<"[DriveSimulation] Module construction completed at "<<std::chrono::duration<double>(std::chrono::steady_clock::now()-startupTime).count()<<" s"<<std::endl;
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
