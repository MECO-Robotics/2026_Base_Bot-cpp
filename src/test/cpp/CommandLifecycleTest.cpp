#include "base/Characterization.h"
#include "base/Mechanism.h"
#include <frc/simulation/DriverStationSim.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>
#include <gtest/gtest.h>
#include <memory>

class CommandLifecycle : public testing::Test {
 protected:
  void SetUp() override {
    scheduler.CancelAll();
    scheduler.Enable();
    frc::sim::DriverStationSim::ResetData();
    frc::sim::DriverStationSim::SetDsAttached(true);
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();
    auto io=std::make_unique<base::InjectedMotorIO>();
    observed=io.get();
    observed->inputs.connected=true;
    observed->inputs.position=.5;
    base::Gains gains; gains.p=5;
    joint=std::make_unique<base::PositionJoint>("CommandLifecycle",std::move(io),gains);
  }
  void TearDown() override {
    scheduler.CancelAll();
    joint.reset();
    frc::sim::DriverStationSim::ResetData();
    frc::sim::DriverStationSim::NotifyNewData();
  }
  void Run(int loops=2) { for(int i=0;i<loops;++i) scheduler.Run(); }
  void VerifyHeldCommand(bool voltage) {
    auto command=voltage ? joint->VoltageCommand([]{return 4.;})
                         : joint->VelocityCommand([]{return .5;});
    scheduler.Schedule(command); Run();
    ASSERT_GT(observed->requestedVoltage,0);
    auto replacement=frc2::cmd::RunOnce([]{}, {joint.get()});
    scheduler.Schedule(replacement);
    EXPECT_FALSE(command.IsScheduled());
    EXPECT_DOUBLE_EQ(observed->requestedVoltage,0);
    Run(); scheduler.Schedule(command); Run();
    ASSERT_GT(observed->requestedVoltage,0);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData(); Run();
    EXPECT_FALSE(command.IsScheduled());
    EXPECT_DOUBLE_EQ(observed->requestedVoltage,0);
  }
  void VerifyPositionHold() {
    auto command=joint->PositionCommand([]{return .5;});
    scheduler.Schedule(command); Run();
    EXPECT_FALSE(command.IsScheduled()); ASSERT_TRUE(joint->AtSetpoint());
    observed->inputs.position=.4; Run(1);
    EXPECT_GT(observed->requestedVoltage,0); EXPECT_FALSE(joint->AtSetpoint());
  }
  void VerifySysIdCancellation() {
    auto command=base::SysIdDynamic(*joint,frc2::sysid::Direction::kForward);
    scheduler.Schedule(command); Run(4); ASSERT_GT(observed->requestedVoltage,0);
    command.Cancel(); EXPECT_DOUBLE_EQ(observed->requestedVoltage,0);
    Run(1); EXPECT_DOUBLE_EQ(observed->requestedVoltage,0);
  }
  frc2::CommandScheduler& scheduler=frc2::CommandScheduler::GetInstance();
  base::InjectedMotorIO* observed=nullptr;
  std::unique_ptr<base::PositionJoint> joint;
};

TEST_F(CommandLifecycle, VelocityCommandOwnsAndStopsOnReplacementOrDisable) { VerifyHeldCommand(false); }
TEST_F(CommandLifecycle, VoltageCommandOwnsAndStopsOnReplacementOrDisable) { VerifyHeldCommand(true); }
TEST_F(CommandLifecycle, CompletedPositionCommandRetainsHold) { VerifyPositionHold(); }
TEST_F(CommandLifecycle, SysIdCancellationStopsOutput) { VerifySysIdCancellation(); }
