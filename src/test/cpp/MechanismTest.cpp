#include "base/Mechanism.h"
#include <array>
#include <limits>
#include <gtest/gtest.h>
#include <frc/simulation/DriverStationSim.h>
#include <hal/HAL.h>
#include <networktables/NetworkTableInstance.h>

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
  for(int n=0;n<20;++n) sim->AdvanceSimulation(units::second_t{.02});
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
  for(int n=0;n<500;++n) { wheel.Periodic(); wheel.SimulationPeriodic(); }
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

TEST(MechanismTuning, AppliesAtomicValidatedGainsOnlyWhileDisabled) {
  HAL_Initialize(500,0);
  frc::sim::DriverStationSim::SetEnabled(false);
  frc::sim::DriverStationSim::NotifyNewData();
  auto io=std::make_unique<base::InjectedMotorIO>(); auto* observed=io.get();
  base::Flywheel wheel("TuningTest",std::move(io),base::Gains{});
  wheel.EnableLiveTuning();
  auto entry=nt::NetworkTableInstance::GetDefault().GetTable("TuningTest/Gains")->GetEntry("Values");
  entry.SetDoubleArray(std::array<double,10>{2,0,0,.1,.2,.3,.4,10,20,.01});
  wheel.Periodic();
  EXPECT_DOUBLE_EQ(wheel.GetGains().p,2);
  EXPECT_DOUBLE_EQ(wheel.GetGains().a,.3);
  EXPECT_DOUBLE_EQ(observed->requestedVoltage,0);
  entry.SetDoubleArray(std::array<double,10>{9,0,0,.1,.2,.3,.4,10,-1,.01});
  wheel.Periodic();
  EXPECT_DOUBLE_EQ(wheel.GetGains().p,2);
  EXPECT_DOUBLE_EQ(wheel.GetGains().maxAcceleration,20);
  frc::sim::DriverStationSim::SetEnabled(true);
  frc::sim::DriverStationSim::NotifyNewData();
  entry.SetDoubleArray(std::array<double,10>{5,0,0,.1,.2,.3,.4,10,20,.01});
  wheel.Periodic();
  EXPECT_DOUBLE_EQ(wheel.GetGains().p,2);
  frc::sim::DriverStationSim::SetEnabled(false);
  frc::sim::DriverStationSim::NotifyNewData();
  wheel.EnableLiveTuning(false);
  wheel.Periodic();
  EXPECT_DOUBLE_EQ(wheel.GetGains().p,2);
}

TEST(MechanismTuning, UnchangedDashboardPreservesDynamicProfileConstraints) {
  HAL_Initialize(500,0);
  frc::sim::DriverStationSim::SetEnabled(false);
  frc::sim::DriverStationSim::NotifyNewData();
  auto io=std::make_unique<base::InjectedMotorIO>(); auto* observed=io.get();
  observed->inputs.position=.5;
  base::Gains gains; gains.p=100;
  base::PositionJoint joint("DynamicTuningTest",std::move(io),gains);
  joint.Periodic();
  joint.EnableLiveTuning();
  joint.SetPositionDynamic(.8,1,.1);
  joint.Periodic();
  frc::sim::DriverStationSim::SetEnabled(true);
  frc::sim::DriverStationSim::NotifyNewData();
  joint.Periodic();
  // First 20 ms profile step: a*dt^2 = .00004 rotations, P=100.
  EXPECT_NEAR(observed->requestedVoltage,.004,1e-10);
  joint.Stop();
  frc::sim::DriverStationSim::SetEnabled(false);
  frc::sim::DriverStationSim::NotifyNewData();
}

TEST(MechanismSimulation, HighReductionConvergesToMotorFreeSpeedWithoutNumericalOscillation) {
  base::MotorConfig config{{1},{false},105,40,""};
  auto simulation=base::MakeMotorIO(base::Mode::Sim,base::Vendor::SparkMax,config,.01);
  simulation->SetVoltage(6);
  for(int i=0;i<200;++i) simulation->AdvanceSimulation(units::second_t{.02});
  // Ideal half-voltage NEO free speed, expressed at the 105:1 output.
  // Model viscous drag shifts equilibrium by <3 ppm; 0.1% allows that drag
  // while rejecting the oscillating fixed-1ms Euler result.
  constexpr double expected=5676./60*.5/105;
  EXPECT_NEAR(simulation->Read().velocity,expected,expected*.001);
}

TEST(MechanismSimulation, PollingFrequencyDoesNotAdvancePhysics) {
  auto run=[](int hz) {
    auto io=base::MakeMotorIO(base::Mode::Sim,base::Vendor::SparkMax,{{1},{false},2,40,""});
    io->SetVoltage(4);
    EXPECT_DOUBLE_EQ(io->Read().position,0);
    for(int ms=0;ms<1000;++ms) {
      if(ms%(1000/hz)==0) {
        const auto first=io->Read();
        EXPECT_DOUBLE_EQ(io->Read().position,first.position);
        EXPECT_DOUBLE_EQ(io->Read().velocity,first.velocity);
      }
      io->AdvanceSimulation(units::second_t{.001});
    }
    return io->Read();
  };
  const auto reference=run(50);
  for(int hz:{100,250}) {
    const auto sample=run(hz);
    EXPECT_DOUBLE_EQ(sample.position,reference.position);
    EXPECT_DOUBLE_EQ(sample.velocity,reference.velocity);
    EXPECT_DOUBLE_EQ(sample.current,reference.current);
  }
  EXPECT_GT(reference.position,1);
  RecordProperty("position_rotations",reference.position);
  RecordProperty("velocity_rotations_per_second",reference.velocity);
}

TEST(MechanismControl, ExplicitTimestepScalesIntegralDerivativeAndFeedforward) {
  for(double dt:{.02,.01,.004}) {
    auto io=std::make_unique<base::InjectedMotorIO>(); auto* observed=io.get();
    base::Gains gains; gains.p=.3; gains.i=.4; gains.d=.002;
    gains.v=.2; gains.a=.1; gains.maxAcceleration=2;
    base::Flywheel wheel("TimestepTest",std::move(io),gains);
    wheel.SetVelocity(1);
    wheel.Control(units::second_t{dt},true);
    const double profiledVelocity=2*dt;
    const double expected=.3*profiledVelocity+.4*profiledVelocity*dt+
      .002*profiledVelocity/dt+.2*profiledVelocity+.1*2;
    EXPECT_NEAR(observed->requestedVoltage,expected,1e-12);
    wheel.PublishTelemetry();
    EXPECT_NEAR(observed->requestedVoltage,expected,1e-12);
    EXPECT_THROW(wheel.Control(units::second_t{0},true),std::invalid_argument);
    EXPECT_DOUBLE_EQ(observed->requestedVoltage,0);
  }
}

TEST(MechanismControl, ExternalSchedulingAvoidsDuplicateTicksAndTelemetryDoesNotControl) {
  auto io=std::make_unique<base::InjectedMotorIO>(); auto* observed=io.get();
  base::Gains gains; gains.p=1; gains.maxAcceleration=2;
  base::Flywheel wheel("ExternalTickTest",std::move(io),gains);
  wheel.UseExternalScheduling();
  wheel.SetVelocity(1);
  wheel.Control(units::second_t{.004},true);
  const double voltage=observed->requestedVoltage;
  observed->inputs.velocity=100;
  wheel.Periodic();
  EXPECT_DOUBLE_EQ(wheel.Velocity(),0);
  EXPECT_DOUBLE_EQ(observed->requestedVoltage,voltage);
  wheel.Control(units::second_t{.004},false);
  EXPECT_DOUBLE_EQ(observed->requestedVoltage,0);
}

TEST(MechanismEncoder, NonfiniteExternalFeedbackStopsWithoutEscapingCalibration) {
  auto io=std::make_unique<base::InjectedMotorIO>(); auto* observed=io.get();
  base::Gains gains; gains.minimum=0; gains.maximum=1;
  base::PositionJoint joint("ExternalFault",std::move(io),gains,1,base::Gravity::Constant,
      []{return std::numeric_limits<double>::quiet_NaN();});
  joint.SetVoltage(4);
  EXPECT_NO_THROW(joint.Control(units::second_t{.004},true));
  EXPECT_DOUBLE_EQ(observed->requestedVoltage,0);
  EXPECT_TRUE(std::isnan(joint.Position()));
}
