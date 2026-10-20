#pragma once
#include <frc/TimedRobot.h>
#include "RobotContainer.h"
class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void TeleopInit() override;
  void TestInit() override;
  void SimulationPeriodic() override;
 private:
  RobotContainer container_;
  frc2::Command* autonomous_ = nullptr;
};
