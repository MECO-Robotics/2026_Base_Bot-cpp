#include "RobotContainer.h"
#include <cmath>
#include <array>
#include <cstdlib>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#ifndef __FRC_ROBORIO__
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>
#include <networktables/StructArrayTopic.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/struct/Pose3dStruct.h>
#endif
using namespace units::literals;
RobotContainer::RobotContainer()
{
#ifndef __FRC_ROBORIO__
  if (drive_.SimulationTruth()) {
    auto &clock = base::sim::GetSimulationRuntime();
    const auto &sim = drive_.Config().simulation;
    arena_ = std::make_unique<base::sim::Arena>(clock, [this] {
      return drive_.SimulationTruth().value().body;
    }, true, 2026, base::sim::Vec3{sim.centerOfMassX, sim.centerOfMassY,
                                 sim.centerOfMassZ});
    base::sim::IntakeConfig intake;
    intake.localPose.position = {.65, 0, -.18};
    arena_->SetIntake(intake);
    controller_.Y().OnTrue(frc2::cmd::RunOnce([this] {
      base::sim::GetSimulationRuntime().Reset();
      drive_.ResetSimulation(frc::Pose2d{});
    }, {&drive_}).IgnoringDisable(true));
    controller_.LeftBumper().WhileTrue(frc2::cmd::RunEnd(
        [this] { arena_->SetIntakeActive(true); },
        [this] { arena_->SetIntakeActive(false); }));
    controller_.RightBumper().OnTrue(frc2::cmd::RunOnce([this] {
      arena_->Launch({{.7, 0, .6}, {}}, {6, 0, 4});
    }));
    drive_.ResetSimulation(frc::Pose2d{});
  }
#endif
  drive_.ConfigureAuto();
  drive_.SetDefaultCommand(drive_.Joystick([this]
                                           { return -controller_.GetLeftY(); }, [this]
                                           { return -controller_.GetLeftX(); }, [this]
                                           { return -controller_.GetRightX(); }));
  controller_.A().WhileTrue(drive_.Aim([this]
                                       { return -controller_.GetLeftY(); }, [this]
                                       { return -controller_.GetLeftX(); }, []
                                       { return frc::Rotation2d{}; }));
  controller_.B().OnTrue(frc2::cmd::RunOnce([this]
                                            { drive_.ResetPose(frc::Pose2d{drive_.Pose().Translation(), frc::Rotation2d{}}); }, {&drive_})
                             .IgnoringDisable(true));
  controller_.X().WhileTrue(frc2::cmd::RunEnd([this]
                                              { drive_.StopWithX(); }, [this]
                                              { drive_.Stop(); }, {&drive_}));
  autoChooser_ = pathplanner::AutoBuilder::buildAutoChooser();
  frc::SmartDashboard::PutData("Auto Choices", &autoChooser_);
}
frc2::Command *RobotContainer::GetAutonomousCommand() { return autoChooser_.GetSelected(); }

void RobotContainer::PublishSimulation()
{
#ifndef __FRC_ROBORIO__
  auto table = nt::NetworkTableInstance::GetDefault().GetTable("Simulation");
  if (const char *token = std::getenv("FRC_SIM_RUN_TOKEN"))
    table->PutString("RunToken", token);
  auto &clock = base::sim::GetSimulationRuntime();
  table->PutNumber("Time", clock.Time());
  table->PutNumber("StepSeconds", clock.Timing().seconds);
  table->PutNumber("MaximumStepSeconds", clock.Timing().maximumSeconds);
  table->PutNumber("DeadlineMisses", clock.Timing().deadlineMisses);
  table->PutString("Backend", !clock.World() ? "phoenix" :
      clock.World()->Mode() == base::sim::Backend::Planar ? "jolt-planar" : "jolt-3d");
  auto truth = drive_.SimulationTruth();
  if (!truth) return;
  auto pose3d = [](const base::sim::Pose &pose) {
    const auto &p = pose.position;
    const auto &q = pose.rotation;
    return frc::Pose3d{units::meter_t{p.x}, units::meter_t{p.y}, units::meter_t{p.z},
                      frc::Rotation3d{frc::Quaternion{q.w, q.x, q.y, q.z}}};
  };
  // Keep publishers alive: temporary publishers can disappear before NT flushes.
  static auto truthPublisher = table->GetStructTopic<frc::Pose3d>("RobotTruth").Publish();
  static auto estimatePublisher = table->GetStructTopic<frc::Pose3d>("RobotEstimated").Publish();
  static auto piecePublisher = table->GetStructArrayTopic<frc::Pose3d>("Pieces").Publish();
  truthPublisher.Set(pose3d(truth->body.pose));
  estimatePublisher.Set(frc::Pose3d{drive_.Pose()});
  std::vector<frc::Pose3d> pieces;
  if (arena_) for (const auto &piece : arena_->Pieces())
    if (!piece.held) pieces.push_back(pose3d(piece.state.pose));
  piecePublisher.Set(pieces);
  const auto &body = truth->body;
  const auto &q = body.pose.rotation;
  double yaw = std::atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z));
  table->PutNumberArray("Truth", std::array<double, 3>{body.pose.position.x, body.pose.position.y, yaw});
  table->PutNumberArray("Velocity", std::array<double, 4>{body.linearVelocity.x, body.linearVelocity.y,
                                     body.linearVelocity.z, body.angularVelocity.z});
  std::vector<double> slip;
  for (const auto &module : truth->modules) slip.push_back(module.longitudinalSlip);
  table->PutNumberArray("Slip", slip);
  const auto requested = drive_.RequestedSpeeds();
  table->PutNumberArray("RequestedSpeeds", std::array<double, 3>{requested.vx.value(), requested.vy.value(), requested.omega.value()});
  table->PutNumber("Held", arena_ ? arena_->HeldCount() : 0);
  table->PutBoolean("IntakeActive", arena_ && arena_->IntakeActive());

#endif
}
