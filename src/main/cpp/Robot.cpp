#include "Robot.h"
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <frc2/command/CommandScheduler.h>
#ifndef __FRC_ROBORIO__
#include "base/sim/SimulationWorld.h"
#endif
Robot::Robot()
{
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  frc::RobotController::SetBrownoutVoltage(units::volt_t{6});
}
void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }
void Robot::AutonomousInit()
{
  autonomous_ = container_.GetAutonomousCommand();
  if (autonomous_)
    autonomous_->Schedule();
}
void Robot::TeleopInit()
{
  if (autonomous_)
    autonomous_->Cancel();
}
void Robot::TestInit() { frc2::CommandScheduler::GetInstance().CancelAll(); }
void Robot::SimulationPeriodic()
{
#ifndef __FRC_ROBORIO__
  // This is the sole owner of plant time. Subsystem reads never advance physics.
  base::sim::GetSimulationRuntime().StepRobotPeriod();
  container_.PublishSimulation();
#endif
}
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
