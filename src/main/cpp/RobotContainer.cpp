#include "RobotContainer.h"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
using namespace units::literals;
RobotContainer::RobotContainer()
{
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
