#include "base/Mechanism.h"
#include <gtest/gtest.h>
#include <limits>
#include <frc/simulation/DriverStationSim.h>
#include <hal/HAL.h>
#include <frc/smartdashboard/SmartDashboard.h>

TEST(MechanismConfig, RejectsInvalidMappingAndLimits) {
  base::MotorConfig valid{{1,2},{false,true},2,40,""};
  EXPECT_NO_THROW(valid.Validate());
  valid.ids[1]=1;
  EXPECT_THROW(valid.Validate(),std::invalid_argument);
  base::Gains gains;
  gains.minimum=1; gains.maximum=0;
  EXPECT_THROW(gains.Validate(true),std::invalid_argument);
}
TEST(MechanismEncoder, AppliesDirectionBeforeOffsetAndScale) {
  base::EncoderCalibration encoder{2,.25,true};
  EXPECT_DOUBLE_EQ(encoder.Apply(.5),-.5);
}
TEST(MechanismControl, DisabledAndSoftLimitsPreventVoltage) {
  HAL_Initialize(500,0);
  frc::sim::DriverStationSim::SetEnabled(true);
  frc::sim::DriverStationSim::NotifyNewData();
  auto io=std::make_unique<base::InjectedMotorIO>(); auto* observed=io.get();
  observed->inputs.connected=true;
  base::Gains gains; gains.minimum=0; gains.maximum=1; gains.p=5;
  base::PositionJoint joint("LimitTest",std::move(io),gains);
  observed->inputs.position=1;
  joint.SetVoltage(4); joint.Periodic();
  EXPECT_DOUBLE_EQ(observed->requestedVoltage,0);
  observed->inputs.position=.5;
  joint.SetVoltage(4); joint.Periodic();
  EXPECT_DOUBLE_EQ(observed->requestedVoltage,4);
  frc::sim::DriverStationSim::SetEnabled(false);
  frc::sim::DriverStationSim::NotifyNewData();
  joint.Periodic();
  EXPECT_DOUBLE_EQ(observed->requestedVoltage,0);
}
TEST(MechanismReference, ResetRetainsPhysicalSoftLimits) {
  HAL_Initialize(500,0);
  auto io=std::make_unique<base::InjectedMotorIO>(); auto* observed=io.get();
  observed->inputs.connected=true;
  base::Gains gains; gains.minimum=0; gains.maximum=1;
  base::PositionJoint joint("ResetTest",std::move(io),gains);
  observed->inputs.position=.75; joint.Periodic(); joint.ResetPosition();
  EXPECT_DOUBLE_EQ(joint.Position(),0);
  joint.SetPosition(5);
  EXPECT_DOUBLE_EQ(joint.DesiredPosition(),.25);
}
TEST(MechanismIO, ReplayDoesNotConstructHardwareAndSimulationMoves) {
  base::MotorConfig config{{1},{false},2,40,""};
  auto replay=base::MakeMotorIO(base::Mode::Replay,base::Vendor::TalonFX,config);
  ASSERT_NE(dynamic_cast<base::InjectedMotorIO*>(replay.get()),nullptr);
  auto sim=base::MakeMotorIO(base::Mode::Sim,base::Vendor::SparkMax,config);
  sim->SetVoltage(4);
  for(int n=0;n<20;++n) sim->Read();
  EXPECT_GT(sim->Read().position,0);
  EXPECT_GT(sim->Read().velocity,0);
}
TEST(MechanismSimulation, ClosedLoopVelocityConverges) {
  HAL_Initialize(500,0);
  frc::sim::DriverStationSim::SetEnabled(true);
  frc::sim::DriverStationSim::NotifyNewData();
  base::MotorConfig config{{1},{false},1,40,""};
  base::Gains gains; gains.p=.8; gains.v=.127; gains.maxVelocity=50; gains.maxAcceleration=100;
  base::Flywheel wheel("SimControl",base::MakeMotorIO(base::Mode::Sim,base::Vendor::SparkMax,config),gains);
  wheel.SetVelocity(20);
  for(int n=0;n<500;++n) wheel.Periodic();
  EXPECT_NEAR(wheel.Velocity(),20,1);
  wheel.Stop();
  frc::sim::DriverStationSim::SetEnabled(false);
  frc::sim::DriverStationSim::NotifyNewData();
}
TEST(MechanismJoint, VelocityModeUsesVelocityErrorAndPhysicalLimits) {
  HAL_Initialize(500,0);
  frc::sim::DriverStationSim::SetEnabled(true);
  frc::sim::DriverStationSim::NotifyNewData();
  auto io=std::make_unique<base::InjectedMotorIO>(); auto* observed=io.get();
  observed->inputs.connected=true;
  base::Gains gains; gains.minimum=0; gains.maximum=1; gains.p=5;
  base::PositionJoint joint("JointVelocity",std::move(io),gains);
  observed->inputs.position=.5; joint.Periodic();
  EXPECT_NO_THROW(joint.SetVelocity(.5)); joint.Periodic();
  EXPECT_TRUE(joint.VelocityControlActive());
  EXPECT_GT(observed->requestedVoltage,0);
  observed->inputs.position=1; joint.Periodic();
  EXPECT_DOUBLE_EQ(observed->requestedVoltage,0);
  frc::sim::DriverStationSim::SetEnabled(false);
  frc::sim::DriverStationSim::NotifyNewData();
}
TEST(MechanismProfile, ReversingDeceleratesBeforeChangingDirection) {
  HAL_Initialize(500,0);
  frc::sim::DriverStationSim::SetEnabled(true);
  frc::sim::DriverStationSim::NotifyNewData();
  auto io=std::make_unique<base::InjectedMotorIO>(); auto* observed=io.get();
  observed->inputs.connected=true;
  base::Gains gains; gains.p=100; gains.minimum=0; gains.maximum=1;
  base::PositionJoint joint("ProfileReverse",std::move(io),gains);
  observed->inputs.position=.5; observed->inputs.velocity=.9;
  joint.Periodic(); joint.SetPosition(.499); joint.Periodic();
  // The profile is still moving positively while slowing down; it must not snap backward.
  EXPECT_GT(observed->requestedVoltage,0);
  frc::sim::DriverStationSim::SetEnabled(false);
  frc::sim::DriverStationSim::NotifyNewData();
}
TEST(MechanismFlywheel, VelocityTargetHasNoJointProfileSpeedCap) {
  auto io=std::make_unique<base::InjectedMotorIO>();
  base::Flywheel wheel("FlywheelTarget",std::move(io),base::Gains{});
  wheel.SetVelocity(30);
  EXPECT_DOUBLE_EQ(wheel.VelocitySetpoint(),30);
}

TEST(MechanismSafety, InvalidExternalFeedbackStopsPreviousOutput) {
  HAL_Initialize(500,0);
  frc::sim::DriverStationSim::SetEnabled(true);
  frc::sim::DriverStationSim::NotifyNewData();
  auto io=std::make_unique<base::InjectedMotorIO>(); auto* observed=io.get();
  observed->inputs.connected=true;
  double sensor=.5;
  base::PositionJoint joint("InvalidEncoder",std::move(io),base::Gains{},1,
                            base::Gravity::Constant,[&sensor]{return sensor;});
  for(double invalid:{std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::infinity()}) {
    sensor=.5; joint.SetVoltage(4); joint.Periodic();
    ASSERT_DOUBLE_EQ(observed->requestedVoltage,4);
    sensor=invalid;
    EXPECT_NO_THROW(joint.Periodic());
    EXPECT_DOUBLE_EQ(observed->requestedVoltage,0);
    joint.SetPosition(.5);
    EXPECT_FALSE(joint.AtSetpoint());
    const double offset=joint.PositionOffset();
    EXPECT_NO_THROW(joint.ResetPosition());
    EXPECT_DOUBLE_EQ(joint.PositionOffset(),offset);
    sensor=.5; joint.Periodic();
    EXPECT_TRUE(std::isfinite(joint.Position()));
    EXPECT_DOUBLE_EQ(joint.PositionOffset(),offset);
    EXPECT_DOUBLE_EQ(observed->requestedVoltage,0);
  }
  frc::sim::DriverStationSim::SetEnabled(false);
  frc::sim::DriverStationSim::NotifyNewData();
}
TEST(MechanismSafety, OverflowingControlOutputStopsInsteadOfSaturating) {
  HAL_Initialize(500,0);
  frc::sim::DriverStationSim::SetEnabled(true);
  frc::sim::DriverStationSim::NotifyNewData();
  auto io=std::make_unique<base::InjectedMotorIO>(); auto* observed=io.get();
  observed->inputs.connected=true;
  base::Gains gains; gains.p=std::numeric_limits<double>::max();
  gains.maxAcceleration=1000;
  base::Flywheel wheel("OverflowControl",std::move(io),gains);
  wheel.SetVoltage(4); wheel.Periodic();
  ASSERT_DOUBLE_EQ(observed->requestedVoltage,4);
  wheel.SetVelocity(10); wheel.Periodic();
  EXPECT_DOUBLE_EQ(observed->requestedVoltage,0);
  wheel.Periodic();
  EXPECT_DOUBLE_EQ(observed->requestedVoltage,0);
  frc::sim::DriverStationSim::SetEnabled(false);
  frc::sim::DriverStationSim::NotifyNewData();
}

TEST(MechanismProfile, CloseTargetDoesNotViolateAccelerationAtCrossing) {
  HAL_Initialize(500,0);
  frc::sim::DriverStationSim::SetEnabled(true);
  frc::sim::DriverStationSim::NotifyNewData();
  auto io=std::make_unique<base::InjectedMotorIO>(); auto* observed=io.get();
  observed->inputs.connected=true;
  base::Gains gains; gains.a=1; gains.minimum=0; gains.maximum=2;
  base::PositionJoint joint("ProfileCloseTarget",std::move(io),gains);
  observed->inputs.position=.5; observed->inputs.velocity=.9;
  joint.Periodic(); joint.SetPosition(.501);
  // With only kA enabled, requested volts numerically equal profile acceleration.
  // The nearby target requires overshoot before the initial speed can be stopped.
  for(int n=0;n<200;++n) {
    joint.Periodic();
    EXPECT_LE(std::abs(observed->requestedVoltage),1+1e-12);
  }
  // Verify that allowing overshoot still converges to the requested position.
  gains.a=0; gains.p=1; joint.SetGains(gains); joint.Periodic();
  EXPECT_NEAR(observed->requestedVoltage,.001,1e-9);
  frc::sim::DriverStationSim::SetEnabled(false);
  frc::sim::DriverStationSim::NotifyNewData();
}

TEST(MechanismSafety, DisconnectedFeedbackStopsAndCannotReportAtSetpoint) {
  HAL_Initialize(500,0);
  frc::sim::DriverStationSim::SetEnabled(true);
  frc::sim::DriverStationSim::NotifyNewData();
  auto io=std::make_unique<base::InjectedMotorIO>(); auto* observed=io.get();
  observed->inputs.connected=true; observed->inputs.position=.5;
  base::PositionJoint joint("DisconnectedFeedback",std::move(io),base::Gains{});
  joint.Periodic(); joint.SetPosition(.5);
  ASSERT_TRUE(joint.AtSetpoint());
  observed->inputs.connected=false; joint.Periodic();
  EXPECT_FALSE(frc::SmartDashboard::GetBoolean("DisconnectedFeedback/Connected",true));
  EXPECT_FALSE(joint.AtSetpoint());
  joint.SetVoltage(4); joint.Periodic();
  EXPECT_DOUBLE_EQ(observed->requestedVoltage,0);
  EXPECT_FALSE(joint.AtSetpoint());
  observed->inputs.connected=true; joint.Periodic();
  EXPECT_TRUE(frc::SmartDashboard::GetBoolean("DisconnectedFeedback/Connected",false));
  EXPECT_DOUBLE_EQ(observed->requestedVoltage,0);
  EXPECT_FALSE(joint.AtSetpoint());
  joint.SetVoltage(4); joint.Periodic();
  EXPECT_DOUBLE_EQ(observed->requestedVoltage,4);
  joint.Stop();
  frc::sim::DriverStationSim::SetEnabled(false);
  frc::sim::DriverStationSim::NotifyNewData();
}
