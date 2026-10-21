#include "base/PortableSwerve.h"
#include "base/Drive.h"
#include <frc/RobotController.h>
#include <hal/HAL.h>
#include <gtest/gtest.h>
#include <limits>
namespace {
class Module final : public base::SwerveModuleIO {
 public:
  base::SwerveModuleInputs input{0,0,0,0,0,true};
  bool stopped=true;double force{};frc::SwerveModuleState target{};
  base::SwerveModuleInputs Read()override{return input;}
  void Set(const frc::SwerveModuleState& s,double ff)override{stopped=false;target=s;force=ff;}
  void SetVoltage(double v)override{stopped=v==0;}
  void Stop()override{stopped=true;}
};
class Gyro final : public base::SwerveGyroIO {public:base::SwerveGyroInputs Read()override{return {0,true};}};
struct Rig {
  std::array<Module*,4> modules{};base::SwerveIO io;
  Rig(){for(size_t i=0;i<4;++i){auto module=std::make_unique<Module>();modules[i]=module.get();io.modules[i]=std::move(module);}io.gyro=std::make_unique<Gyro>();}
};
}
TEST(PortableSwerve, EstimatorResetAndFailStopRecoveryUseFreshInputsAndRequests) {
  HAL_Initialize(500,0);base::DriveConfig c;c.backend=base::DriveBackend::SparkMaxNavX;Rig rig;
  base::PortableSwerve drive(c,std::move(rig.io),false);
  const double now=frc::RobotController::GetFPGATime()*1e-6;
  drive.Sample(now-.02,true);drive.Update();
  drive.Velocity({units::meters_per_second_t{1},units::meters_per_second_t{0},units::radians_per_second_t{0}});
  for(auto* m:rig.modules){m->input.distance=1;m->input.velocity=1;}
  drive.Sample(now,true);drive.Update();
  EXPECT_NEAR(drive.State().pose.X().value(),1,1e-8);EXPECT_FALSE(rig.modules[0]->stopped);
  drive.ResetPose({units::meter_t{5},units::meter_t{2},frc::Rotation2d{}});
  for(auto* m:rig.modules)m->input.distance=1.2;
  drive.Sample(now+.01,false);drive.Update();
  EXPECT_NEAR(drive.State().pose.X().value(),5.2,1e-8);EXPECT_TRUE(rig.modules[0]->stopped);
  drive.Sample(now+.02,true);EXPECT_TRUE(rig.modules[0]->stopped);
  drive.Velocity({units::meters_per_second_t{1},units::meters_per_second_t{0},units::radians_per_second_t{0}});drive.Sample(now+.03,true);EXPECT_FALSE(rig.modules[0]->stopped);
  rig.modules[1]->input.healthy=false;drive.Sample(now+.04,true);EXPECT_TRUE(rig.modules[0]->stopped);
  rig.modules[1]->input.healthy=true;drive.Sample(now+.05,true);EXPECT_TRUE(rig.modules[0]->stopped);
  drive.Velocity({units::meters_per_second_t{1},units::meters_per_second_t{0},units::radians_per_second_t{0}});
  EXPECT_THROW(drive.Velocity({units::meters_per_second_t{0},units::meters_per_second_t{0},units::radians_per_second_t{std::numeric_limits<double>::quiet_NaN()}}),std::invalid_argument);
  drive.Sample(now+.06,true);EXPECT_TRUE(rig.modules[0]->stopped);
}
TEST(PortableSwerve, WheelForceAddsOnlyProjectedTorqueVoltage) {
  HAL_Initialize(500,0);base::DriveConfig c;c.backend=base::DriveBackend::SparkMaxNavX;Rig rig;base::PortableSwerve drive(c,std::move(rig.io),false);
  drive.Sample(frc::RobotController::GetFPGATime()*1e-6,true);
  drive.Velocity({units::meters_per_second_t{2},units::meters_per_second_t{0},units::radians_per_second_t{0}},{10,10,10,10},{20,20,20,20});
  drive.Sample(frc::RobotController::GetFPGATime()*1e-6,true);
  // NEO stall torque 2.6 N m at 12 V; no velocity back EMF in this term.
  EXPECT_NEAR(rig.modules[0]->force,10*c.wheelRadius/c.driveRatio*12/2.6,1e-6);
  EXPECT_NEAR(rig.modules[0]->target.speed.value(),2,1e-10);
}

TEST(PortableSwerve, ConfigurationRejectsUnsupportedBusAndInvalidAbsoluteCalibration) {
  base::DriveConfig config;config.backend=base::DriveBackend::SparkMaxNavX;
  for(auto& module:config.modules)module.encoder=0;
  config.pigeon=-1;EXPECT_NO_THROW(config.Validate());
  config.bus="canivore";EXPECT_THROW(config.Validate(),std::invalid_argument);
  config.bus="rio";config.modules[0].offsetTurns=1;EXPECT_THROW(config.Validate(),std::invalid_argument);
  config.modules[0].offsetTurns=.75;EXPECT_NO_THROW(config.Validate());
}
