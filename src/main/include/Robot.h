#pragma once
#include <frc/TimedRobot.h>
#include "RobotContainer.h"
class Robot : public frc::TimedRobot
{
  public:
    Robot();
    ~Robot() override;
    void StartCompetition() override;
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void DisabledInit() override;
    void TeleopInit() override;
    void TestInit() override;
    void SimulationPeriodic() override;

  private:
    bool replayInitialized_;
    RobotContainer container_;
    frc2::Command *autonomous_ = nullptr;
};
