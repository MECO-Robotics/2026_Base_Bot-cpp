#include "base/Drive.h"
#include "base/Mechanism.h"
#include "base/PortableSwerve.h"
#include <frc/Errors.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>
#include <gtest/gtest.h>

namespace
{
class Module final : public base::SwerveModuleIO
{
  public:
    base::SwerveModuleInputs Read() override
    {
        return {0, 0, 0, 0, 0, true};
    }
    void Set(const frc::SwerveModuleState &, double) override
    {
    }
    void SetVoltage(double) override
    {
    }
    void Stop() override
    {
    }
};
class Gyro final : public base::SwerveGyroIO
{
  public:
    base::SwerveGyroInputs Read() override
    {
        return {0, true};
    }
};
class CommandComposition : public testing::Test
{
  protected:
    void SetUp() override
    {
        auto &scheduler = frc2::CommandScheduler::GetInstance();
        scheduler.CancelAll();
        scheduler.Enable();
        frc::sim::DriverStationSim::ResetData();
        frc::sim::DriverStationSim::SetDsAttached(true);
        frc::sim::DriverStationSim::SetEnabled(true);
        frc::sim::DriverStationSim::SetAutonomous(false);
        frc::sim::DriverStationSim::SetTest(false);
        frc::sim::DriverStationSim::NotifyNewData();
        frc::DriverStation::RefreshData();

        base::DriveConfig config;
        config.backend = base::DriveBackend::SparkMaxNavX;
        base::SwerveIO io;
        for (auto &module : io.modules)
            module = std::make_unique<Module>();
        io.gyro = std::make_unique<Gyro>();
        drive = std::make_unique<base::Drive>(config, std::move(io));

        auto motor = std::make_unique<base::InjectedMotorIO>();
        motor->inputs.connected = true;
        base::Gains gains;
        gains.p = 1;
        mechanism = std::make_unique<base::Flywheel>("CompositionTest", std::move(motor), gains);
    }
    void TearDown() override
    {
        auto &scheduler = frc2::CommandScheduler::GetInstance();
        scheduler.CancelAll();
        mechanism.reset();
        drive.reset();
        frc::sim::DriverStationSim::ResetData();
        frc::sim::DriverStationSim::NotifyNewData();
    }
    std::unique_ptr<base::Drive> drive;
    std::unique_ptr<base::Flywheel> mechanism;
};
} // namespace

TEST_F(CommandComposition, ParallelDriveAndMechanismCommandsScheduleTogether)
{
    auto group = frc2::cmd::Parallel(drive->Joystick([] { return .2; }, [] { return 0.; }, [] { return 0.; }),
                                     mechanism->VelocityCommand([] { return .5; }));

    const auto &requirements = group.get()->GetRequirements();
    EXPECT_TRUE(requirements.contains(drive.get()));
    EXPECT_TRUE(requirements.contains(mechanism.get()));

    auto &scheduler = frc2::CommandScheduler::GetInstance();
    scheduler.Schedule(group);
    scheduler.Run();
    EXPECT_TRUE(group.IsScheduled());
    EXPECT_EQ(scheduler.Requiring(drive.get()), group.get());
    EXPECT_EQ(scheduler.Requiring(mechanism.get()), group.get());
}

TEST_F(CommandComposition, ParallelDriveCommandsRejectDuplicateSubsystemRequirement)
{
    auto joystick = drive->Joystick([] { return .2; }, [] { return 0.; }, [] { return 0.; });
    auto aim = drive->Aim([] { return 0.; }, [] { return 0.; }, [] { return frc::Rotation2d{}; });

    EXPECT_THROW((void)frc2::cmd::Parallel(std::move(joystick), std::move(aim)), frc::RuntimeError);
}
