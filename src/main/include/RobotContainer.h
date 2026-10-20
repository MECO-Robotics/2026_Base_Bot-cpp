#pragma once
#include "base/Drive.h"
#include "base/Vision.h"
#ifndef __FRC_ROBORIO__
#include "base/sim/Arena.h"
#include <memory>
#endif
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
class RobotContainer {
 public:
  RobotContainer();
  frc2::Command* GetAutonomousCommand();
  void PublishSimulation();
 private:
  base::Drive drive_;
#ifndef __FRC_ROBORIO__
  std::unique_ptr<base::sim::Arena> arena_;
#endif
  frc2::CommandXboxController controller_{0};
  frc::SendableChooser<frc2::Command*> autoChooser_;
};
