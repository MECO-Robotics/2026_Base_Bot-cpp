#pragma once
#include <array>
#include <memory>
#include <string>
#ifndef __FRC_ROBORIO__
#include <optional>
#include "base/sim/DrivePhysics.h"
#endif
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <ctre/phoenix6/swerve/SwerveDrivetrain.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <pathplanner/lib/util/DriveFeedforwards.h>
namespace pathplanner { class PathFollowingController; }
namespace base {
struct ModuleConfig {
  int drive, steer, encoder;
  double offsetTurns, xMeters, yMeters;
  bool driveInverted = false, steerInverted = false, encoderInverted = false;
};
// Illustrative simulation dimensions and contact parameters; calibrate for a robot.
struct DriveSimulationConfig {
  double bumperHalfLength=.43, bumperHalfWidth=.43, bumperHalfHeight=.12;
  double rideHeight=.30, centerOfMassX=0, centerOfMassY=0, centerOfMassZ=0;
  double rollInertia=4, pitchInertia=4;
  double wheelInertia=.025, steerInertia=.004;
  double longitudinalStiffness=1800, lateralStiffness=1800;
  double suspensionRestLength=.30, suspensionTravel=.15;
  double suspensionStiffness=6000, suspensionDamping=650;
};
struct DriveConfig {
  DriveSimulationConfig simulation{};
  std::string bus = "rio";
  int pigeon = 0;
  double driveRatio = 6.122448979591837, steerRatio = 150.0 / 7.0, couplingRatio = 0;
  double wheelRadius = 0.0508, maxSpeed = 4.572, maxAngularRate = 7;
  // Phoenix native swerve drive gains use rotor turns; Java used output-wheel turns.
  // Divide the Java P/D/V gains by driveRatio; static voltage is unchanged.
  double driveKP = 1 / 6.122448979591837, driveKD = 0, driveKS = .5, driveKV = .71 / 6.122448979591837;
  double steerKP = 25, steerKD = 0, steerKS = 0, steerKV = 2.5;
  double driveCurrent = 40, steerCurrent = 40, slipCurrent = 50;
  double massKg = 74.088, moi = 6.883, cof = 1.2;
  void Validate() const;
  std::array<ModuleConfig, 4> modules{{
    {31,41,21,0,.33655,.33655}, {32,42,22,0,.33655,-.33655},
    {33,43,23,0,-.33655,.33655}, {34,44,24,0,-.33655,-.33655}}};
};
class Drive : public frc2::SubsystemBase {
 public:
  explicit Drive(DriveConfig config = {});
  ~Drive() override;
  void Periodic() override;
  void SimulationPeriodic() override;
  frc::Pose2d Pose() const;
  frc::ChassisSpeeds Speeds() const;
  frc::ChassisSpeeds RequestedSpeeds() const { return requestedSpeeds_; }
  void ResetPose(const frc::Pose2d& pose);
  void AddVision(const frc::Pose2d& pose, units::second_t time, std::array<double,3> noise);
  void RunVelocity(const frc::ChassisSpeeds& speeds);
  void RunVelocity(const frc::ChassisSpeeds& speeds, const pathplanner::DriveFeedforwards& ff);
  void FieldDrive(double x, double y, double omega);
  void Stop();
  void StopWithX();
  void ConfigureAuto();
  // WPILib translation SysId, using native Phoenix voltage requests.
  frc2::CommandPtr Characterize(bool dynamic, frc2::sysid::Direction direction);
  // Opt-in low-speed spin commissioning; reports estimates without changing config.
  frc2::CommandPtr CharacterizeWheelRadius();
  frc2::CommandPtr Joystick(std::function<double()> x, std::function<double()> y, std::function<double()> omega);
  frc2::CommandPtr Aim(std::function<double()> x, std::function<double()> y, std::function<frc::Rotation2d()> heading);
  const DriveConfig& Config() const { return config_; }
#ifndef __FRC_ROBORIO__
  std::optional<sim::DriveTruth> SimulationTruth() const;
  // Physical placement/reset also synchronizes sensor state and odometry.
  void ResetSimulation(const frc::Pose2d& pose);
  sim::DrivePhysics* SimulationPhysics();
#endif
 private:
  using NativeDrive = ctre::phoenix6::swerve::SwerveDrivetrain<ctre::phoenix6::hardware::TalonFX,ctre::phoenix6::hardware::TalonFX,ctre::phoenix6::hardware::CANcoder>;
  DriveConfig config_;
  frc::ChassisSpeeds requestedSpeeds_{};
  std::unique_ptr<NativeDrive> native_;
  // AutoBuilder stores process-global callbacks; they must not own PID
  // sendables beyond the robot/SendableRegistry lifetime.
  std::shared_ptr<pathplanner::PathFollowingController> autoController_;
  struct Telemetry;
  std::shared_ptr<Telemetry> telemetry_;
  struct Inputs {
    frc::Pose2d pose{};
    frc::ChassisSpeeds speeds{};
    std::array<frc::SwerveModuleState,4> states{};
  };
  Inputs ReadInputs() const;
  std::array<frc::SwerveModuleState,4> replayTargets_{};
  void RecordDriveOutput();
#ifndef __FRC_ROBORIO__
  struct Simulation;
  std::unique_ptr<Simulation> simulation_;
  double diagnosticSupplyCurrent_{};
  sim::Registration diagnosticPowerRegistration_;
#endif
};
}
