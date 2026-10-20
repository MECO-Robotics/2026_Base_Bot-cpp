#include "base/Drive.h"
#include <array>
#include <algorithm>
#include <cmath>
#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#ifndef __FRC_ROBORIO__
#include <ctre/phoenix6/sim/ChassisReference.hpp>
#include <chrono>
#include <iostream>
#include <stdexcept>
#endif
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
namespace base
{
#ifndef __FRC_ROBORIO__
  namespace {
    void AwaitPhoenixSimulationTransport(const DriveConfig& config) {
      // Status frames can arrive before configuration transactions work. Probe
      // the actual first drive device with the same Apply/Refresh transport used
      // by native module construction, rather than relying on status freshness.
      // This temporary handle is destroyed before NativeDrive owns its devices.
      p::hardware::TalonFX probe{config.modules[0].drive,p::CANBus{config.bus}};
      probe.GetSimState().SetSupplyVoltage(12_V);
      p::configs::MotorOutputConfigs requested;
      requested.PeakForwardDutyCycle=0;
      requested.PeakReverseDutyCycle=0;
      const auto start=std::chrono::steady_clock::now();
      const auto deadline=start+std::chrono::seconds(10);
      std::string status;
      int attempts=0;
      do {
        ++attempts;
        auto applied=probe.GetConfigurator().Apply(requested,.1_s);
        status=applied.GetName();
        if(applied.IsOK()) {
          p::configs::MotorOutputConfigs actual;
          auto refreshed=probe.GetConfigurator().Refresh(actual,.1_s);
          status=refreshed.GetName();
          if(refreshed.IsOK()&&actual.PeakForwardDutyCycle==requested.PeakForwardDutyCycle&&actual.PeakReverseDutyCycle==requested.PeakReverseDutyCycle) {
            std::cout<<"[DriveSim] Verified Phoenix configuration transport after "<<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<" s, "<<attempts<<" attempt(s)"<<std::endl;
            // Native SwerveModule applies the full DriveMotorInitialConfigs;
            // its default +/-1 duty limits replace these temporary zero limits.
            return;
          }
        }
      } while(std::chrono::steady_clock::now()<deadline);
      throw std::runtime_error("Phoenix desktop simulation configuration transport did not complete verified Apply/Refresh within 10 seconds for drive device "+std::to_string(config.modules[0].drive)+" on bus "+config.bus+": "+status);
    }
  }
  struct Drive::Simulation {
    Drive& owner;
    sim::DrivePhysics physics;
    double lastYaw{}, unwrappedYaw{};
    sim::Registration registration; // release callbacks before physics and owner
    static sim::DrivePhysicsConfig Configuration(const DriveConfig& c) {
      sim::DrivePhysicsConfig out;
      const auto& s=c.simulation;
      out.mass=c.massKg; out.inertia={s.rollInertia,s.pitchInertia,c.moi};
      out.centerOfMass={s.centerOfMassX,s.centerOfMassY,s.centerOfMassZ};
      out.bumperHalfExtents={s.bumperHalfLength,s.bumperHalfWidth,s.bumperHalfHeight};
      out.rideHeight=s.rideHeight;out.wheelRadius=c.wheelRadius;
      out.driveRatio=c.driveRatio;out.steerRatio=c.steerRatio;out.couplingRatio=c.couplingRatio;
      out.wheelInertia=s.wheelInertia;out.steerInertia=s.steerInertia;out.friction=c.cof;
      out.longitudinalStiffness=s.longitudinalStiffness;out.lateralStiffness=s.lateralStiffness;
      out.suspensionRestLength=s.suspensionRestLength;out.suspensionTravel=s.suspensionTravel;
      out.suspensionStiffness=s.suspensionStiffness;out.suspensionDamping=s.suspensionDamping;
      out.driveCurrentLimit=c.driveCurrent;out.steerCurrentLimit=c.steerCurrent;
      for(size_t i=0;i<4;++i) out.moduleLocations[i]={c.modules[i].xMeters,c.modules[i].yMeters,0};
      return out;
    }
    explicit Simulation(Drive& drive):owner(drive),physics(*sim::GetSimulationRuntime().World(),Configuration(drive.config_)) {
      WriteSensors();
      registration=sim::GetSimulationRuntime().Register({
        [this](double dt) {ReadVoltages();physics.BeforePhysics(dt);},
        [this](double dt) {physics.AfterPhysics(dt);WriteSensors();},
        [this] {Reset(frc::Pose2d{});}});
    }
    void ReadVoltages() {
      for(size_t i=0;i<4;++i) {
        auto& module=owner.native_->GetModule(i);
        physics.SetModuleVoltages(i,module.GetDriveMotor().GetSimState().GetMotorVoltage().value(),module.GetSteerMotor().GetSimState().GetMotorVoltage().value());
      }
    }
    void WriteSensors() {
      const auto truth=physics.Truth();
      for(size_t i=0;i<4;++i) {
        auto& module=owner.native_->GetModule(i);const auto& config=owner.config_.modules[i];
        auto& drive=module.GetDriveMotor().GetSimState();
        auto& steer=module.GetSteerMotor().GetSimState();
        auto& encoder=module.GetEncoder().GetSimState();
        // Match pinned Phoenix SimSwerveDrivetrain's physical coordinate convention.
        using Ref=p::sim::ChassisReference;
        drive.Orientation=config.driveInverted?Ref::Clockwise_Positive:Ref::CounterClockwise_Positive;
        steer.Orientation=config.steerInverted?Ref::Clockwise_Positive:Ref::CounterClockwise_Positive;
        encoder.Orientation=config.encoderInverted?Ref::Clockwise_Positive:Ref::CounterClockwise_Positive;
        encoder.SensorOffset=units::turn_t{config.offsetTurns};
        auto supply=frc::RobotController::GetBatteryVoltage();
        drive.SetSupplyVoltage(supply);steer.SetSupplyVoltage(supply);encoder.SetSupplyVoltage(supply);
        const auto& m=truth.modules[i];
        drive.SetRawRotorPosition(units::radian_t{m.driveRotorAngle});
        drive.SetRotorVelocity(units::radians_per_second_t{m.driveRotorVelocity});
        steer.SetRawRotorPosition(units::radian_t{m.steerRotorAngle});
        steer.SetRotorVelocity(units::radians_per_second_t{m.steerRotorVelocity});
        encoder.SetRawPosition(units::radian_t{m.steerAngle});
        encoder.SetVelocity(units::radians_per_second_t{m.steerVelocity});
      }
      const auto q=truth.body.pose.rotation;
      const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
      const double pitch=std::asin(std::clamp(2*(q.w*q.y-q.z*q.x),-1.0,1.0));
      const double roll=std::atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y));
      auto& pigeon=owner.native_->GetPigeon2().GetSimState();
      unwrappedYaw+=std::remainder(yaw-lastYaw,2*std::acos(-1.0));lastYaw=yaw;
      pigeon.SetRawYaw(units::radian_t{unwrappedYaw});pigeon.SetPitch(units::radian_t{pitch});pigeon.SetRoll(units::radian_t{roll});
      // Pigeon angular rates are in sensor/body coordinates.
      auto w=truth.body.angularVelocity;
      const sim::Vec3 u{-q.x,-q.y,-q.z};
      auto cross=[](sim::Vec3 a,sim::Vec3 b){return sim::Vec3{a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};};
      auto a=cross(u,w),b=cross(u,a);
      w={w.x+2*(q.w*a.x+b.x),w.y+2*(q.w*a.y+b.y),w.z+2*(q.w*a.z+b.z)};
      pigeon.SetAngularVelocityX(units::radians_per_second_t{w.x});
      pigeon.SetAngularVelocityY(units::radians_per_second_t{w.y});
      pigeon.SetAngularVelocityZ(units::radians_per_second_t{w.z});
    }
    void Reset(const frc::Pose2d& pose) {
      double angle=pose.Rotation().Radians().value();
      lastYaw=angle;unwrappedYaw=angle;
      physics.Reset({{pose.X().value(),pose.Y().value(),owner.config_.simulation.rideHeight},{0,0,std::sin(angle/2),std::cos(angle/2)}});
      WriteSensors();owner.native_->ResetPose(pose);
    }
  };
#endif
  Drive::Drive(DriveConfig config) : config_(std::move(config))
  {
#ifndef __FRC_ROBORIO__
    if(frc::RobotBase::IsSimulation()) AwaitPhoenixSimulationTransport(config_);
#endif
    p::configs::TalonFXConfiguration drive, steer;
    drive.CurrentLimits.WithSupplyCurrentLimit(units::ampere_t{config_.driveCurrent}).WithSupplyCurrentLimitEnable(true);
    steer.CurrentLimits.WithStatorCurrentLimit(units::ampere_t{config_.steerCurrent}).WithStatorCurrentLimitEnable(true);
    auto factory = s::SwerveModuleConstantsFactory<p::configs::TalonFXConfiguration, p::configs::TalonFXConfiguration, p::configs::CANcoderConfiguration>{}
                       .WithDriveMotorGearRatio(config_.driveRatio)
                       .WithSteerMotorGearRatio(config_.steerRatio)
                       .WithCouplingGearRatio(config_.couplingRatio)
                       .WithWheelRadius(units::meter_t{config_.wheelRadius})
                       .WithDriveMotorGains(p::configs::Slot0Configs{}.WithKP(config_.driveKP).WithKD(config_.driveKD).WithKS(config_.driveKS).WithKV(config_.driveKV))
                       .WithSteerMotorGains(p::configs::Slot0Configs{}.WithKP(config_.steerKP).WithKD(config_.steerKD).WithKS(config_.steerKS).WithKV(config_.steerKV))
                       .WithDriveMotorClosedLoopOutput(s::ClosedLoopOutputType::Voltage)
                       .WithSteerMotorClosedLoopOutput(s::ClosedLoopOutputType::Voltage)
                       .WithDriveMotorType(s::DriveMotorArrangement::TalonFX_Integrated)
                       .WithSteerMotorType(s::SteerMotorArrangement::TalonFX_Integrated)
                       .WithFeedbackSource(s::SteerFeedbackType::RemoteCANcoder)
                       .WithSlipCurrent(units::ampere_t{config_.slipCurrent})
                       .WithSpeedAt12Volts(units::meters_per_second_t{config_.maxSpeed})
                       .WithDriveMotorInitialConfigs(drive)
                       .WithSteerMotorInitialConfigs(steer)
                       .WithSteerInertia(units::kilogram_square_meter_t{.004})
                       .WithDriveInertia(units::kilogram_square_meter_t{.025})
                       .WithSteerFrictionVoltage(.1_V)
                       .WithDriveFrictionVoltage(.1_V);
    auto make = [&](int i)
    { const auto& m=config_.modules[i]; return factory.CreateModuleConstants(m.steer,m.drive,m.encoder,units::turn_t{m.offsetTurns},units::meter_t{m.xMeters},units::meter_t{m.yMeters},m.driveInverted,m.steerInverted,m.encoderInverted); };
    native_ = std::make_unique<NativeDrive>(s::SwerveDrivetrainConstants{}.WithCANBusName(config_.bus).WithPigeon2Id(config_.pigeon), make(0), make(1), make(2), make(3));
#ifndef __FRC_ROBORIO__
    if(frc::RobotBase::IsSimulation() && sim::GetSimulationRuntime().World())
      simulation_=std::make_unique<Simulation>(*this);
#endif
    SetName("Drive");
  }
  Drive::~Drive() = default;
  void Drive::Periodic()
  {
    if (frc::DriverStation::IsDisabled())
    {
      auto a = frc::DriverStation::GetAlliance();
      native_->SetOperatorPerspectiveForward(frc::Rotation2d{a && *a == frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg});
      Stop();
    }
    auto state = native_->GetState();
    frc::SmartDashboard::PutNumberArray("Drive/Pose", std::array<double, 3>{state.Pose.X().value(), state.Pose.Y().value(), state.Pose.Rotation().Degrees().value()});
    frc::SmartDashboard::PutNumberArray("Drive/Speeds", std::array<double, 3>{state.Speeds.vx.value(), state.Speeds.vy.value(), state.Speeds.omega.value()});
  }
  void Drive::SimulationPeriodic() {
#ifndef __FRC_ROBORIO__
    if(simulation_) return; // The single world clock owns all Jolt stepping.
#endif
    native_->UpdateSimState(.02_s, frc::RobotController::GetBatteryVoltage());
  }
#ifndef __FRC_ROBORIO__
  std::optional<sim::DriveTruth> Drive::SimulationTruth() const {
    if(simulation_) return simulation_->physics.Truth();
    return std::nullopt;
  }
  sim::DrivePhysics* Drive::SimulationPhysics() { return simulation_?&simulation_->physics:nullptr; }
  void Drive::ResetSimulation(const frc::Pose2d& pose) {
    if(simulation_) simulation_->Reset(pose);
  }
#endif
  frc::Pose2d Drive::Pose() const { return native_->GetState().Pose; }
  frc::ChassisSpeeds Drive::Speeds() const { return native_->GetState().Speeds; }
  void Drive::ResetPose(const frc::Pose2d &pose) { native_->ResetPose(pose); }
  void Drive::AddVision(const frc::Pose2d &pose, units::second_t time, std::array<double, 3> noise) { native_->AddVisionMeasurement(pose, p::utils::FPGAToCurrentTime(time), noise); }
  void Drive::RunVelocity(const frc::ChassisSpeeds &speeds)
  {
    requestedSpeeds_ = speeds;
    native_->SetControl(s::requests::ApplyRobotSpeeds{}.WithSpeeds(speeds).WithDriveRequestType(s::impl::DriveRequestType::Velocity));
  }
  void Drive::RunVelocity(const frc::ChassisSpeeds &speeds, const pathplanner::DriveFeedforwards &ff)
  {
    requestedSpeeds_ = speeds;
    native_->SetControl(s::requests::ApplyRobotSpeeds{}.WithSpeeds(speeds).WithDriveRequestType(s::impl::DriveRequestType::Velocity).WithWheelForceFeedforwardsX(ff.robotRelativeForcesX).WithWheelForceFeedforwardsY(ff.robotRelativeForcesY));
  }
  void Drive::FieldDrive(double x, double y, double omega)
  {
    if (!frc::DriverStation::IsTeleopEnabled())
    {
      Stop();
      return;
    }
    auto alliance = frc::DriverStation::GetAlliance();
    double sign = alliance && *alliance == frc::DriverStation::Alliance::kRed ? -1 : 1;
    const auto robotSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(units::meters_per_second_t{x * sign}, units::meters_per_second_t{y * sign}, units::radians_per_second_t{omega}, Pose().Rotation());
    // Operator commands update every 20 ms; ApplyRobotSpeeds does not discretize.
    RunVelocity(frc::ChassisSpeeds::Discretize(robotSpeeds, .02_s));
  }
  void Drive::Stop() { RunVelocity(frc::ChassisSpeeds{}); }
  void Drive::StopWithX() {
    requestedSpeeds_ = {};
    native_->SetControl(s::requests::SwerveDriveBrake{});
  }
  void Drive::ConfigureAuto()
  {
    std::vector<frc::Translation2d> locations;
    for (const auto &m : config_.modules)
      locations.emplace_back(units::meter_t{m.xMeters}, units::meter_t{m.yMeters});
    pathplanner::RobotConfig rc{units::kilogram_t{config_.massKg}, units::kilogram_square_meter_t{config_.moi},
                                pathplanner::ModuleConfig{units::meter_t{config_.wheelRadius}, units::meters_per_second_t{config_.maxSpeed}, config_.cof, frc::DCMotor::KrakenX60FOC(1).WithReduction(config_.driveRatio), units::ampere_t{config_.driveCurrent}, 1}, locations};
    pathplanner::AutoBuilder::configure([this]
                                        { return Pose(); }, [this](const auto &pose)
                                        { ResetPose(pose); }, [this]
                                        { return Speeds(); },
                                        [this](const frc::ChassisSpeeds &speed, const pathplanner::DriveFeedforwards &ff)
                                        { RunVelocity(speed, ff); },
                                        std::make_shared<pathplanner::PPHolonomicDriveController>(pathplanner::PIDConstants{5, 0, 0}, pathplanner::PIDConstants{5, 0, 0}), rc,
                                        []
                                        {auto a=frc::DriverStation::GetAlliance(); return a && *a==frc::DriverStation::Alliance::kRed; }, this);
  }
  static double Deadband(double value) { return std::abs(value) < .1 ? 0 : std::copysign((std::abs(value) - .1) / .9, value); }
  frc2::CommandPtr Drive::Joystick(std::function<double()> x, std::function<double()> y, std::function<double()> omega)
  {
    return frc2::cmd::RunEnd([=, this]
                             { FieldDrive(Deadband(x()) * config_.maxSpeed, Deadband(y()) * config_.maxSpeed, Deadband(omega()) * config_.maxAngularRate); }, [this]
                             { Stop(); }, {this});
  }
  frc2::CommandPtr Drive::Aim(std::function<double()> x, std::function<double()> y, std::function<frc::Rotation2d()> heading)
  {
    return frc2::cmd::RunEnd([=, this]
                             {double error=(heading()-Pose().Rotation()).Radians().value(); FieldDrive(Deadband(x())*config_.maxSpeed,Deadband(y())*config_.maxSpeed,std::clamp(5*error,-config_.maxAngularRate,config_.maxAngularRate)); }, [this]
                             { Stop(); }, {this});
  }
}
