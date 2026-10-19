#include "base/Drive.h"
#include <array>
#include <algorithm>
#include <cmath>
#include <frc/DriverStation.h>
#include <ctre/phoenix6/Utils.hpp>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <frc/system/plant/DCMotor.h>
using namespace units::literals;
namespace p = ctre::phoenix6;
namespace s = p::swerve;
namespace base {
Drive::Drive(DriveConfig config) : config_(std::move(config)) {
  p::configs::TalonFXConfiguration drive, steer;
  drive.CurrentLimits.WithSupplyCurrentLimit(units::ampere_t{config_.driveCurrent}).WithSupplyCurrentLimitEnable(true);
  steer.CurrentLimits.WithStatorCurrentLimit(units::ampere_t{config_.steerCurrent}).WithStatorCurrentLimitEnable(true);
  auto factory = s::SwerveModuleConstantsFactory<p::configs::TalonFXConfiguration,p::configs::TalonFXConfiguration,p::configs::CANcoderConfiguration>{}
    .WithDriveMotorGearRatio(config_.driveRatio).WithSteerMotorGearRatio(config_.steerRatio)
    .WithCouplingGearRatio(config_.couplingRatio).WithWheelRadius(units::meter_t{config_.wheelRadius})
    .WithDriveMotorGains(p::configs::Slot0Configs{}.WithKP(config_.driveKP).WithKD(config_.driveKD).WithKS(config_.driveKS).WithKV(config_.driveKV))
    .WithSteerMotorGains(p::configs::Slot0Configs{}.WithKP(config_.steerKP).WithKD(config_.steerKD).WithKS(config_.steerKS).WithKV(config_.steerKV))
    .WithDriveMotorClosedLoopOutput(s::ClosedLoopOutputType::Voltage).WithSteerMotorClosedLoopOutput(s::ClosedLoopOutputType::Voltage)
    .WithDriveMotorType(s::DriveMotorArrangement::TalonFX_Integrated).WithSteerMotorType(s::SteerMotorArrangement::TalonFX_Integrated)
    .WithFeedbackSource(s::SteerFeedbackType::RemoteCANcoder)
    .WithSlipCurrent(units::ampere_t{config_.slipCurrent}).WithSpeedAt12Volts(units::meters_per_second_t{config_.maxSpeed})
    .WithDriveMotorInitialConfigs(drive).WithSteerMotorInitialConfigs(steer)
    .WithSteerInertia(units::kilogram_square_meter_t{.004}).WithDriveInertia(units::kilogram_square_meter_t{.025})
    .WithSteerFrictionVoltage(.1_V).WithDriveFrictionVoltage(.1_V);
  auto make = [&](int i) { const auto& m=config_.modules[i]; return factory.CreateModuleConstants(m.steer,m.drive,m.encoder,units::turn_t{m.offsetTurns},units::meter_t{m.xMeters},units::meter_t{m.yMeters},m.driveInverted,m.steerInverted,m.encoderInverted); };
  native_ = std::make_unique<NativeDrive>(s::SwerveDrivetrainConstants{}.WithCANBusName(config_.bus).WithPigeon2Id(config_.pigeon),make(0),make(1),make(2),make(3));
  SetName("Drive");
}
Drive::~Drive() = default;
void Drive::Periodic() {
  if (frc::DriverStation::IsDisabled()) {
    auto a=frc::DriverStation::GetAlliance();
    native_->SetOperatorPerspectiveForward(frc::Rotation2d{a && *a==frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg});
    Stop();
  }
  auto state=native_->GetState();
  frc::SmartDashboard::PutNumberArray("Drive/Pose",std::array<double,3>{state.Pose.X().value(),state.Pose.Y().value(),state.Pose.Rotation().Degrees().value()});
  frc::SmartDashboard::PutNumberArray("Drive/Speeds",std::array<double,3>{state.Speeds.vx.value(),state.Speeds.vy.value(),state.Speeds.omega.value()});
}
void Drive::SimulationPeriodic() { native_->UpdateSimState(.02_s,frc::RobotController::GetBatteryVoltage()); }
frc::Pose2d Drive::Pose() const { return native_->GetState().Pose; }
frc::ChassisSpeeds Drive::Speeds() const { return native_->GetState().Speeds; }
void Drive::ResetPose(const frc::Pose2d& pose) { native_->ResetPose(pose); }
void Drive::AddVision(const frc::Pose2d& pose,units::second_t time,std::array<double,3> noise) { native_->AddVisionMeasurement(pose,p::utils::FPGAToCurrentTime(time),noise); }
void Drive::RunVelocity(const frc::ChassisSpeeds& speeds) {
  native_->SetControl(s::requests::ApplyRobotSpeeds{}.WithSpeeds(speeds).WithDriveRequestType(s::impl::DriveRequestType::Velocity));
}
void Drive::RunVelocity(const frc::ChassisSpeeds& speeds,const pathplanner::DriveFeedforwards& ff) {
  native_->SetControl(s::requests::ApplyRobotSpeeds{}.WithSpeeds(speeds).WithDriveRequestType(s::impl::DriveRequestType::Velocity).WithWheelForceFeedforwardsX(ff.robotRelativeForcesX).WithWheelForceFeedforwardsY(ff.robotRelativeForcesY));
}
void Drive::FieldDrive(double x,double y,double omega) {
  if (!frc::DriverStation::IsTeleopEnabled()) { Stop(); return; }
  auto alliance=frc::DriverStation::GetAlliance();
  double sign=alliance && *alliance==frc::DriverStation::Alliance::kRed ? -1 : 1;
  const auto robotSpeeds=frc::ChassisSpeeds::FromFieldRelativeSpeeds(units::meters_per_second_t{x*sign},units::meters_per_second_t{y*sign},units::radians_per_second_t{omega},Pose().Rotation());
  // Operator commands update every 20 ms; ApplyRobotSpeeds does not discretize.
  RunVelocity(frc::ChassisSpeeds::Discretize(robotSpeeds,.02_s));
}
void Drive::Stop() { RunVelocity(frc::ChassisSpeeds{}); }
void Drive::StopWithX() { native_->SetControl(s::requests::SwerveDriveBrake{}); }
void Drive::ConfigureAuto() {
  std::vector<frc::Translation2d> locations;
  for (const auto& m:config_.modules) locations.emplace_back(units::meter_t{m.xMeters},units::meter_t{m.yMeters});
  pathplanner::RobotConfig rc{units::kilogram_t{config_.massKg},units::kilogram_square_meter_t{config_.moi},
    pathplanner::ModuleConfig{units::meter_t{config_.wheelRadius},units::meters_per_second_t{config_.maxSpeed},config_.cof,frc::DCMotor::KrakenX60FOC(1).WithReduction(config_.driveRatio),units::ampere_t{config_.driveCurrent},1},locations};
  pathplanner::AutoBuilder::configure([this]{return Pose();},[this](const auto& pose){ResetPose(pose);},[this]{return Speeds();},
    [this](const frc::ChassisSpeeds& speed,const pathplanner::DriveFeedforwards& ff){RunVelocity(speed,ff);},
    std::make_shared<pathplanner::PPHolonomicDriveController>(pathplanner::PIDConstants{5,0,0},pathplanner::PIDConstants{5,0,0}),rc,
    []{auto a=frc::DriverStation::GetAlliance(); return a && *a==frc::DriverStation::Alliance::kRed;},this);
}
static double Deadband(double value) { return std::abs(value)<.1 ? 0 : std::copysign((std::abs(value)-.1)/.9,value); }
frc2::CommandPtr Drive::Joystick(std::function<double()> x,std::function<double()> y,std::function<double()> omega) {
  return frc2::cmd::RunEnd([=,this]{FieldDrive(Deadband(x())*config_.maxSpeed,Deadband(y())*config_.maxSpeed,Deadband(omega())*config_.maxAngularRate);},[this]{Stop();},{this});
}
frc2::CommandPtr Drive::Aim(std::function<double()> x,std::function<double()> y,std::function<frc::Rotation2d()> heading) {
  return frc2::cmd::RunEnd([=,this]{double error=(heading()-Pose().Rotation()).Radians().value(); FieldDrive(Deadband(x())*config_.maxSpeed,Deadband(y())*config_.maxSpeed,std::clamp(5*error,-config_.maxAngularRate,config_.maxAngularRate));},[this]{Stop();},{this});
}
}
