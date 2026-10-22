#include "Robot.h"
#include "base/Replay.h"
#include "base/Telemetry.h"
#include "base/AdvantageScope.h"
#include "base/AdvantageKitLog.h"
#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <frc2/command/CommandScheduler.h>
#include <iostream>
#include <exception>
#ifndef __FRC_ROBORIO__
#include "base/sim/SimulationWorld.h"
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#endif
namespace {
void StopAdvantageKitLogSafely() noexcept {
  try {
    base::logging::StopAdvantageKitLog();
  } catch (const std::exception& error) {
    std::cerr << "[Logging] Failed to finalize AdvantageKit log: " << error.what() << '\n';
  } catch (...) {
    std::cerr << "[Logging] Failed to finalize AdvantageKit log with an unknown error\n";
  }
}
}
Robot::Robot() : replayInitialized_((base::replay::InitializeFromEnvironment(),true))
{

  // Attach canonical NT values/schemas and Driver Station samples to the
  // AdvantageKit-compatible primary WPILOG.
  base::logging::StartRobotDataLogSources();
  base::PublishTelemetryMetadata("2026_Base_Bot-cpp");
  frc::RobotController::SetBrownoutVoltage(units::volt_t{6});
}
Robot::~Robot() {
  base::telemetry::Shutdown();
  base::replay::ShutdownRecording();
  base::logging::StopNetworkTablesDataLog();
  StopAdvantageKitLogSafely();
}
void Robot::StartCompetition() {
  if(!base::replay::Active()){
    frc::TimedRobot::StartCompetition();
    base::telemetry::Shutdown();
    base::replay::ShutdownRecording();
    base::logging::StopNetworkTablesDataLog();
    StopAdvantageKitLogSafely();
    return;
  }
  base::replay::RunReplay([this]{RobotInit();},[this]{LoopFunc();});
  base::telemetry::Shutdown();
  base::replay::ShutdownRecording();
  base::replay::FlushOutput();
  base::logging::StopNetworkTablesDataLog();
  StopAdvantageKitLogSafely();
}
void Robot::RobotPeriodic() {
  base::replay::BeginFrame();
  frc2::CommandScheduler::GetInstance().Run();
  if(!base::replay::Active())base::replay::EndFrame();
}
void Robot::DisabledInit() { base::replay::BeginFrame(); }
void Robot::AutonomousInit()
{
  base::replay::BeginFrame();
  autonomous_ = container_.GetAutonomousCommand();
  if (autonomous_)
    autonomous_->Schedule();
}
void Robot::TeleopInit()
{
  base::replay::BeginFrame();
  if (autonomous_)
    autonomous_->Cancel();
}
void Robot::TestInit() { base::replay::BeginFrame();frc2::CommandScheduler::GetInstance().CancelAll(); }
void Robot::SimulationPeriodic()
{
  if(base::replay::Active())return;
#ifndef __FRC_ROBORIO__
  // This is the sole owner of plant time. Subsystem reads never advance physics.
  auto &clock = base::sim::GetSimulationRuntime();
  clock.StepRobotPeriod();
  const auto voltage = frc::sim::BatterySim::Calculate({units::ampere_t{clock.SupplyCurrent()}});
  clock.SetBusVoltage(voltage.value());
  frc::sim::RoboRioSim::SetVInVoltage(voltage);
  container_.PublishSimulation();
#endif
}
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
