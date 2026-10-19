#include "base/Drive.h"
#include "base/AdvantageScope.h"
#include <ctre/phoenix6/swerve/SwerveDrivetrain.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include "base/DriveTarget.h"
#include <frc/Timer.h>
#include <frc/RobotBase.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>
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
// Vendor hardware and estimator implementation stays behind the public API.
struct Drive::NativeDrive : s::SwerveDrivetrain<p::hardware::TalonFX, p::hardware::TalonFX, p::hardware::CANcoder> {
  using Parent = s::SwerveDrivetrain<p::hardware::TalonFX, p::hardware::TalonFX, p::hardware::CANcoder>;
  using Parent::Parent;
};
struct Drive::ControlRequest final : s::requests::SwerveRequest {
  DriveTarget target;
  s::requests::ApplyRobotSpeeds velocity;
  s::requests::SwerveDriveBrake brake;
  ControlRequest() {
    velocity.DriveRequestType = s::impl::DriveRequestType::Velocity;
    velocity.WheelForceFeedforwardsX.resize(4);
    velocity.WheelForceFeedforwardsY.resize(4);
  }
  ctre::phoenix::StatusCode Apply(const ControlParameters& params,
      std::span<std::unique_ptr<s::impl::SwerveModuleImpl> const> modules) override {
    const bool enabled = frc::DriverStation::IsEnabled();
    const auto value = target.Consume(frc::Timer::GetFPGATimestamp().value(), enabled);
    if (!enabled || (value.teleopOnly && !frc::DriverStation::IsTeleopEnabled())) {
      target.Invalidate();
      for (const auto& module : modules)
        module->Apply(p::controls::NeutralOut{}, p::controls::NeutralOut{});
      return ctre::phoenix::StatusCode::OK;
    }
    if (value.brake) return brake.Apply(params, modules);
    velocity.Speeds = DriveTarget::Resolve(value, params.currentPose.Rotation());
    for (int i=0;i<4;++i) {
      velocity.WheelForceFeedforwardsX[i] = units::newton_t{value.forceX[i]};
      velocity.WheelForceFeedforwardsY[i] = units::newton_t{value.forceY[i]};
    }
    return velocity.Apply(params, modules);
  }
};
Drive::Drive(DriveConfig config) : config_(std::move(config)) {
  if (!std::isfinite(config_.odometryHz) || config_.odometryHz < 0)
    throw std::invalid_argument("Invalid odometry frequency");
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
    .WithFeedbackSource(config_.fusedEncoder ? s::SteerFeedbackType::FusedCANcoder : s::SteerFeedbackType::RemoteCANcoder)
    .WithSlipCurrent(units::ampere_t{config_.slipCurrent}).WithSpeedAt12Volts(units::meters_per_second_t{config_.speedAt12Volts > 0 ? config_.speedAt12Volts : config_.maxSpeed})
    .WithDriveMotorInitialConfigs(drive).WithSteerMotorInitialConfigs(steer)
    .WithSteerInertia(units::kilogram_square_meter_t{config_.steerInertia}).WithDriveInertia(units::kilogram_square_meter_t{config_.driveInertia})
    .WithSteerFrictionVoltage(.1_V).WithDriveFrictionVoltage(.1_V);
  auto make = [&](int i) { const auto& m=config_.modules[i]; return factory.CreateModuleConstants(m.steer,m.drive,m.encoder,units::turn_t{m.offsetTurns},units::meter_t{m.xMeters},units::meter_t{m.yMeters},m.driveInverted,m.steerInverted,m.encoderInverted); };
  native_ = std::make_unique<NativeDrive>(s::SwerveDrivetrainConstants{}.WithCANBusName(config_.bus).WithPigeon2Id(config_.pigeon),units::hertz_t{config_.odometryHz},make(0),make(1),make(2),make(3));
  control_ = std::make_unique<ControlRequest>();
  native_->SetControl(*control_);
  native_->RegisterTelemetry([this](const auto& state) {
    // Fixed-size handoff only. Skip a busy consumer instead of delaying native DAQ.
    if (std::unique_lock lock(snapshotMutex_, std::try_to_lock); lock.owns_lock()) {
      snapshot_.pose = state.Pose;
      snapshot_.speeds = state.Speeds;
      snapshot_.odometryPeriod = state.OdometryPeriod.value();
      snapshot_.failedDaqs = state.FailedDaqs;
      snapshot_.timestampUs = static_cast<int64_t>(p::utils::CurrentTimeToFPGATime(state.Timestamp).value()*1e6);
      if (state.ModuleStates.size()==4 && state.ModuleTargets.size()==4) {
        std::copy_n(state.ModuleStates.begin(),4,snapshot_.measured.begin());
        std::copy_n(state.ModuleTargets.begin(),4,snapshot_.targets.begin());
      } else {
        snapshot_.measured = {};
        snapshot_.targets = {};
      }
    }
  });
  if (frc::RobotBase::IsSimulation()) {
    simNotifier_ = std::make_unique<frc::Notifier>([this] {
      native_->UpdateSimState(.004_s, frc::RobotController::GetBatteryVoltage());
    });
    simNotifier_->SetName("DriveSimulation");
    simNotifier_->StartPeriodic(.004_s);
  }
  telemetry_ = std::make_unique<DriveTelemetry>();
  SetName("Drive");
}
Drive::~Drive() {
  if (simNotifier_) simNotifier_->Stop();
  simNotifier_.reset();
  native_->GetOdometryThread().Stop();
  native_.reset();
}
void Drive::Periodic() {
  if (frc::DriverStation::IsDisabled()) {
    auto a=frc::DriverStation::GetAlliance();
    native_->SetOperatorPerspectiveForward(frc::Rotation2d{a && *a==frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg});
    Stop();
  }
  const double now = frc::Timer::GetFPGATimestamp().value();
  if (now < nextTelemetry_) return;
  nextTelemetry_ = now + .1;
  if (now >= nextBusTelemetry_) {
    nextBusTelemetry_ = now + 1;
    const p::CANBus bus{config_.bus};
    const auto status = bus.GetStatus();
    frc::SmartDashboard::PutBoolean("Drive/CANFD", bus.IsNetworkFD());
    frc::SmartDashboard::PutBoolean("Drive/CANStatusOK", status.Status.IsOK());
    frc::SmartDashboard::PutNumber("Drive/CANUtilization", status.BusUtilization);
    frc::SmartDashboard::PutNumber("Drive/CANBusOff", status.BusOffCount);
    frc::SmartDashboard::PutNumber("Drive/CANTxFull", status.TxFullCount);
    frc::SmartDashboard::PutNumber("Drive/CANReceiveErrors", status.REC);
    frc::SmartDashboard::PutNumber("Drive/CANTransmitErrors", status.TEC);
  }
  Snapshot state;
  { std::scoped_lock lock(snapshotMutex_); state = snapshot_; }
  telemetry_->Publish(state.pose,state.speeds,state.measured,state.targets,state.timestampUs);
  frc::SmartDashboard::PutNumber("Drive/OdometryHz", native_->GetOdometryFrequency().value());
  frc::SmartDashboard::PutNumber("Drive/OdometryPeriod", state.odometryPeriod);
  frc::SmartDashboard::PutNumber("Drive/FailedDaqs", state.failedDaqs);
  frc::SmartDashboard::PutNumberArray("Drive/Pose",std::array<double,3>{state.pose.X().value(),state.pose.Y().value(),state.pose.Rotation().Degrees().value()});
  frc::SmartDashboard::PutNumberArray("Drive/Speeds",std::array<double,3>{state.speeds.vx.value(),state.speeds.vy.value(),state.speeds.omega.value()});
}
void Drive::SimulationPeriodic() {
  // The 4 ms simulation notifier is the sole native plant writer.
}
frc::Pose2d Drive::Pose() const { std::scoped_lock lock(snapshotMutex_); return snapshot_.pose; }
frc::ChassisSpeeds Drive::Speeds() const { std::scoped_lock lock(snapshotMutex_); return snapshot_.speeds; }
void Drive::ResetPose(const frc::Pose2d& pose) {
  native_->ResetPose(pose);
  { std::scoped_lock lock(snapshotMutex_); snapshot_.pose = pose; }
  if (poseResetCallback_) poseResetCallback_(pose);
}
void Drive::SetPoseResetCallback(std::function<void(const frc::Pose2d&)> callback) {
  poseResetCallback_ = std::move(callback);
}
void Drive::AddVision(const frc::Pose2d& pose,units::second_t time,std::array<double,3> noise) { native_->AddVisionMeasurement(pose,p::utils::FPGAToCurrentTime(time),noise); }
void Drive::RunVelocity(const frc::ChassisSpeeds& speeds) {
  DriveTarget::Value value;
  value.speeds = speeds;
  value.timestamp = frc::Timer::GetFPGATimestamp().value();
  control_->target.Publish(value, []{ return frc::DriverStation::IsEnabled(); });
}
void Drive::RunVelocity(const frc::ChassisSpeeds& speeds,const pathplanner::DriveFeedforwards& ff) {
  DriveTarget::Value value;
  value.speeds = speeds;
  value.timestamp = frc::Timer::GetFPGATimestamp().value();
  if (ff.robotRelativeForcesX.size() == 4 && ff.robotRelativeForcesY.size() == 4)
    for (int i=0;i<4;++i) {
      value.forceX[i] = ff.robotRelativeForcesX[i].value();
      value.forceY[i] = ff.robotRelativeForcesY[i].value();
    }
  control_->target.Publish(value, []{ return frc::DriverStation::IsEnabled(); });
}
void Drive::FieldDrive(double x,double y,double omega) {
  if (!frc::DriverStation::IsTeleopEnabled()) { Stop(); return; }
  auto alliance=frc::DriverStation::GetAlliance();
  double sign=alliance && *alliance==frc::DriverStation::Alliance::kRed ? -1 : 1;
  DriveTarget::Value value;
  value.speeds = {units::meters_per_second_t{x*sign}, units::meters_per_second_t{y*sign}, units::radians_per_second_t{omega}};
  value.fieldRelative = value.teleopOnly = true;
  value.timestamp = frc::Timer::GetFPGATimestamp().value();
  control_->target.Publish(value, []{ return frc::DriverStation::IsTeleopEnabled(); });
}
void Drive::RunHeading(double x, double y, frc::Rotation2d heading, double kp, double maxOmega) {
  DriveTarget::Value value;
  value.speeds = {units::meters_per_second_t{x}, units::meters_per_second_t{y}, units::radians_per_second_t{0}};
  value.heading = heading;
  value.headingKP = kp;
  value.maxOmega = maxOmega;
  value.headingControl = true;
  value.timestamp = frc::Timer::GetFPGATimestamp().value();
  control_->target.Publish(value, []{ return frc::DriverStation::IsEnabled(); });
}
void Drive::Stop() { RunVelocity(frc::ChassisSpeeds{}); }
void Drive::StopWithX() {
  DriveTarget::Value value;
  value.brake = true;
  value.timestamp = frc::Timer::GetFPGATimestamp().value();
  control_->target.Publish(value, []{ return frc::DriverStation::IsEnabled(); });
}
void Drive::ConfigureAuto() {
  std::vector<frc::Translation2d> locations;
  for (const auto& m:config_.modules) locations.emplace_back(units::meter_t{m.xMeters},units::meter_t{m.yMeters});
  pathplanner::RobotConfig rc{units::kilogram_t{config_.massKg},units::kilogram_square_meter_t{config_.moi},
    pathplanner::ModuleConfig{units::meter_t{config_.wheelRadius},units::meters_per_second_t{config_.maxSpeed},config_.cof,frc::DCMotor::KrakenX60FOC(1).WithReduction(config_.driveRatio),units::ampere_t{config_.driveCurrent},1},locations};
  if(config_.autoConfigFromGUI) rc=pathplanner::RobotConfig::fromGUISettings();
  pathplanner::AutoBuilder::configure([this]{return Pose();},[this](const auto& pose){ResetPose(pose);},[this]{return Speeds();},
    [this](const frc::ChassisSpeeds& speed,const pathplanner::DriveFeedforwards& ff){RunVelocity(speed,ff);},
    std::make_shared<pathplanner::PPHolonomicDriveController>(pathplanner::PIDConstants{config_.autoTranslationKP,0,0},pathplanner::PIDConstants{config_.autoRotationKP,0,0}),rc,
    []{auto a=frc::DriverStation::GetAlliance(); return a && *a==frc::DriverStation::Alliance::kRed;},this);
}
static double Deadband(double value) { return std::abs(value)<.1 ? 0 : std::copysign((std::abs(value)-.1)/.9,value); }
frc2::CommandPtr Drive::Joystick(std::function<double()> x,std::function<double()> y,std::function<double()> omega) {
  return frc2::cmd::RunEnd([=,this]{FieldDrive(Deadband(x())*config_.maxSpeed,Deadband(y())*config_.maxSpeed,Deadband(omega())*config_.maxAngularRate);},[this]{Stop();},{this});
}
frc2::CommandPtr Drive::Aim(std::function<double()> x,std::function<double()> y,std::function<frc::Rotation2d()> heading) {
  return frc2::cmd::RunEnd([=,this]{
    if (!frc::DriverStation::IsTeleopEnabled()) { Stop(); return; }
    auto alliance=frc::DriverStation::GetAlliance();
    const double sign=alliance && *alliance==frc::DriverStation::Alliance::kRed ? -1 : 1;
    DriveTarget::Value value;
    value.speeds = {units::meters_per_second_t{Deadband(x())*config_.maxSpeed*sign},
                    units::meters_per_second_t{Deadband(y())*config_.maxSpeed*sign}, units::radians_per_second_t{0}};
    value.heading = heading();
    value.headingKP = 5;
    value.maxOmega = config_.maxAngularRate;
    value.headingControl = value.fieldRelative = value.teleopOnly = true;
    value.timestamp = frc::Timer::GetFPGATimestamp().value();
    control_->target.Publish(value, []{ return frc::DriverStation::IsTeleopEnabled(); });
  },[this]{Stop();},{this});
}
}
