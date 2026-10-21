#include "RobotContainer.h"
#include "base/Telemetry.h"
#include "base/Replay.h"
#include <mutex>
#include <map>
#include <stdexcept>
#include <frc/RobotController.h>
#include <cmath>
#include <array>
#include <cstdlib>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#ifndef __FRC_ROBORIO__
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>
#include <networktables/StructArrayTopic.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <iostream>
#include <frc/geometry/struct/Pose3dStruct.h>
#endif
using namespace units::literals;
class RobotContainer::AutoChooser : public frc::SendableChooser<frc2::Command*> {
 public:
  AutoChooser() {
    SetDefaultOption("None",nullptr);
    for(const auto &name:pathplanner::AutoBuilder::getAllAutoNames()) {
      frc2::CommandPtr command{std::make_unique<pathplanner::PathPlannerAuto>(name)};
      AddOption(name,command.get());
      commands_.emplace(name,std::move(command));
    }
  }
  std::string Selection() const {
    std::scoped_lock lock(m_mutex);
    const auto name=m_haveSelected ? m_selected : m_defaultChoice;
    // Match SendableChooser: a stale/unknown dashboard choice selects no command.
    return name=="None" || commands_.contains(name) ? name : "None";
  }
  frc2::Command *Resolve(const std::string &name) const {
    if(name=="None") return nullptr;
    auto found=commands_.find(name);
    if(found==commands_.end()) throw std::runtime_error("Autonomous choice missing from current code/deploy assets: "+name);
    return found->second.get();
  }
 private:
  std::map<std::string,frc2::CommandPtr> commands_;
};
RobotContainer::RobotContainer()
{
#ifndef __FRC_ROBORIO__
  if (drive_.SimulationTruth()) {
    auto &clock = base::sim::GetSimulationRuntime();
    const auto start = base::sim::Arena::kPracticeStart;
    const frc::Pose2d practicePose{units::meter_t{start.x}, units::meter_t{start.y},
                                 frc::Rotation2d{}};
    const auto &sim = drive_.Config().simulation;
    const auto scene = base::sim::ArenaSceneFromEnvironment();
    arena_ = std::make_unique<base::sim::Arena>(clock, [this] {
      return drive_.SimulationTruth().value().body;
    }, scene, 2026, base::sim::Vec3{sim.centerOfMassX, sim.centerOfMassY,
                                 sim.centerOfMassZ});
    const bool full3d = clock.World()->Mode() == base::sim::Backend::Full3D;
    const std::string mode = full3d ? "jolt-3d: bump and depot contacts enabled"
        : "jolt-planar: flat ground; use FRC_SIM_PHYSICS=jolt-3d for bump/depot contacts";
    std::cout << "[Simulation] " << mode << '\n';
    base::telemetry::Post("Scene", [scene, geometry=arena_->ColliderGeometry()] {
    nt::NetworkTableInstance::GetDefault().GetTable("Simulation")->PutString(
        "Field", scene == base::sim::ArenaScene::Rebuilt2026 ? "rebuilt-2026" : "generic");
    // Pose arrays are line paths, not robot poses: add as Trajectory in the
    // AdvantageScope 3D view. Every segment follows an actual box edge.
    static std::vector<nt::StructArrayPublisher<frc::Pose3d>> colliderPublishers;
    colliderPublishers.clear();
    auto table = nt::NetworkTableInstance::GetDefault().GetTable("Simulation/Colliders");
    constexpr std::array<int, 16> walk{0,1,3,2,0,4,5,1,5,7,3,7,6,2,6,4};
    for (const auto &box : geometry) {
      if (box.shape != base::sim::BodySpec::Shape::Box) continue;
      auto &p = box.pose.position;
      auto &q = box.pose.rotation;
      const frc::Pose3d origin{units::meter_t{p.x}, units::meter_t{p.y}, units::meter_t{p.z},
          frc::Rotation3d{frc::Quaternion{q.w,q.x,q.y,q.z}}};
      std::vector<frc::Pose3d> outline;
      for (int corner : walk) {
        outline.push_back(origin.TransformBy(frc::Transform3d{
            frc::Translation3d{units::meter_t{(corner & 1 ? 1 : -1) * box.halfExtents.x},
                               units::meter_t{(corner & 2 ? 1 : -1) * box.halfExtents.y},
                               units::meter_t{(corner & 4 ? 1 : -1) * box.halfExtents.z}},
            frc::Rotation3d{}}));
      }
      colliderPublishers.push_back(table->GetStructArrayTopic<frc::Pose3d>(
          "Box" + std::to_string(colliderPublishers.size())).Publish());
      colliderPublishers.back().Set(outline);
    }
    });
    base::sim::IntakeConfig intake;
    intake.localPose.position = {.65, 0, -.18};
    arena_->SetIntake(intake);
    controller_.Y().OnTrue(frc2::cmd::RunOnce([this, practicePose] {
      base::sim::GetSimulationRuntime().Reset();
      drive_.ResetSimulation(practicePose);
    }, {&drive_}).IgnoringDisable(true));
    controller_.LeftBumper().WhileTrue(frc2::cmd::RunEnd(
        [this] { arena_->SetIntakeActive(true); },
        [this] { arena_->SetIntakeActive(false); }));
    controller_.RightBumper().OnTrue(frc2::cmd::RunOnce([this] {
      arena_->Launch({{.7, 0, .6}, {}}, {6, 0, 4});
    }));
    drive_.ResetSimulation(practicePose);
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
  autoChooser_ = std::make_shared<AutoChooser>();
  base::telemetry::Post("AutoChoices", [chooser=autoChooser_] {
    frc::SmartDashboard::PutData("Auto Choices", chooser.get());
  });
}
frc2::Command *RobotContainer::GetAutonomousCommand() {
  const auto selection=base::replay::ProcessInputs("Auto/Selection",[this] {
    return wpi::json(autoChooser_->Selection());
  });
  return autoChooser_->Resolve(selection.get<std::string>());
}

void RobotContainer::PublishSimulation()
{
#ifndef __FRC_ROBORIO__
  auto &clock=base::sim::GetSimulationRuntime();
  const auto timestamp=frc::RobotController::GetFPGATime();
  const auto time=clock.Time();
  const auto timing=clock.Timing();
  const std::string backend=!clock.World()?"phoenix":clock.World()->Mode()==base::sim::Backend::Planar?"jolt-planar":"jolt-3d";
  const auto truth=drive_.SimulationTruth();
  const auto pieceStates=arena_?arena_->Pieces():std::vector<base::sim::PieceSnapshot>{};
  const auto held=arena_?arena_->HeldCount():0;
  const bool intakeActive=arena_&&arena_->IntakeActive();
  const char *token=std::getenv("FRC_SIM_RUN_TOKEN");
  const std::string runToken=token?token:"";
  base::telemetry::Post("Simulation",[timestamp,time,timing,backend,truth,pieceStates,held,intakeActive,runToken] {
  auto table = nt::NetworkTableInstance::GetDefault().GetTable("Simulation");
  if (!runToken.empty()) table->GetEntry("RunToken").SetString(runToken, timestamp);
  table->GetEntry("Time").SetDouble(time, timestamp);
  table->GetEntry("StepSeconds").SetDouble(timing.seconds, timestamp);
  table->GetEntry("MaximumStepSeconds").SetDouble(timing.maximumSeconds, timestamp);
  table->GetEntry("DeadlineMisses").SetDouble(timing.deadlineMisses, timestamp);
  table->GetEntry("Backend").SetString(backend, timestamp);
  if (!truth) return;
  auto pose3d = [](const base::sim::Pose &pose) {
    const auto &p = pose.position;
    const auto &q = pose.rotation;
    return frc::Pose3d{units::meter_t{p.x}, units::meter_t{p.y}, units::meter_t{p.z},
                      frc::Rotation3d{frc::Quaternion{q.w, q.x, q.y, q.z}}};
  };
  // Keep publishers alive: temporary publishers can disappear before NT flushes.
  static auto truthPublisher = table->GetStructTopic<frc::Pose3d>("RobotTruth").Publish();
  static auto piecePublisher = table->GetStructArrayTopic<frc::Pose3d>("Pieces").Publish();
  truthPublisher.Set(pose3d(truth->body.pose), timestamp);
  std::vector<frc::Pose3d> pieces;
  for (const auto &piece : pieceStates)
    if (!piece.held) pieces.push_back(pose3d(piece.state.pose));
  piecePublisher.Set(pieces, timestamp);
  const auto &body = truth->body;
  table->GetEntry("Velocity").SetDoubleArray( std::array<double, 4>{body.linearVelocity.x, body.linearVelocity.y,
                                     body.linearVelocity.z, body.angularVelocity.z}, timestamp);
  std::vector<double> slip;
  for (const auto &module : truth->modules) slip.push_back(module.longitudinalSlip);
  table->GetEntry("Slip").SetDoubleArray(slip, timestamp);
  table->GetEntry("Held").SetDouble(held, timestamp);
  table->GetEntry("IntakeActive").SetBoolean(intakeActive, timestamp);

  });
#endif
}
