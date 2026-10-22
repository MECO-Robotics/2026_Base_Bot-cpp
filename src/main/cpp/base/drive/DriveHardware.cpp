#include "base/Drive.h"
#include "base/PortableSwerve.h"
#include "base/Replay.h"
#include "base/Telemetry.h"
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <set>
#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#ifndef __FRC_ROBORIO__
#include <ctre/phoenix6/sim/ChassisReference.hpp>
#include <chrono>
#include <iostream>
#endif
#include <ctre/phoenix6/Utils.hpp>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/kinematics/struct/SwerveModuleStateStruct.h>
#include <frc/geometry/struct/Pose2dStruct.h>
#include <frc/kinematics/struct/ChassisSpeedsStruct.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>
using namespace units::literals;
namespace p = ctre::phoenix6;
namespace s = p::swerve;
namespace base
{
struct Drive::ReplayKinematics
{
    static frc::Translation2d Location(const DriveConfig &config, size_t i)
    {
        return {units::meter_t{config.modules[i].xMeters}, units::meter_t{config.modules[i].yMeters}};
    }
    explicit ReplayKinematics(const DriveConfig &config)
        : value(Location(config, 0), Location(config, 1), Location(config, 2), Location(config, 3))
    {
    }
    frc::SwerveDriveKinematics<4> value;
};

struct Drive::Telemetry
{
    nt::StructArrayPublisher<frc::SwerveModuleState> measured =
        nt::NetworkTableInstance::GetDefault()
            .GetStructArrayTopic<frc::SwerveModuleState>("/AdvantageScope/Drive/ModuleStates")
            .Publish();
    nt::StructArrayPublisher<frc::SwerveModuleState> targets =
        nt::NetworkTableInstance::GetDefault()
            .GetStructArrayTopic<frc::SwerveModuleState>("/AdvantageScope/Drive/ModuleTargets")
            .Publish();
    nt::StructPublisher<frc::Pose2d> pose =
        nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("/AdvantageScope/Drive/Pose").Publish();
    nt::StructPublisher<frc::ChassisSpeeds> speeds =
        nt::NetworkTableInstance::GetDefault()
            .GetStructTopic<frc::ChassisSpeeds>("/AdvantageScope/Drive/ChassisSpeeds")
            .Publish();
    nt::StructPublisher<frc::ChassisSpeeds> requested =
        nt::NetworkTableInstance::GetDefault()
            .GetStructTopic<frc::ChassisSpeeds>("/AdvantageScope/Drive/RequestedChassisSpeeds")
            .Publish();
};
#ifndef __FRC_ROBORIO__
namespace
{
void AwaitPhoenixSimulationTransport(const DriveConfig &config)
{
    // Probe with the native Apply/Refresh transport; status can precede configuration readiness.
    // Destroy this temporary handle before NativeDrive constructs its devices.
    p::hardware::TalonFX probe{config.modules[0].drive, p::CANBus{config.bus}};
    probe.GetSimState().SetSupplyVoltage(12_V);
    p::configs::MotorOutputConfigs requested;
    requested.PeakForwardDutyCycle = 0;
    requested.PeakReverseDutyCycle = 0;
    const auto start = std::chrono::steady_clock::now();
    const auto deadline = start + std::chrono::seconds(10);
    std::string status;
    int attempts = 0;
    do
    {
        ++attempts;
        auto applied = probe.GetConfigurator().Apply(requested, .1_s);
        status = applied.GetName();
        if (applied.IsOK())
        {
            p::configs::MotorOutputConfigs actual;
            auto refreshed = probe.GetConfigurator().Refresh(actual, .1_s);
            status = refreshed.GetName();
            if (refreshed.IsOK() && actual.PeakForwardDutyCycle == requested.PeakForwardDutyCycle &&
                actual.PeakReverseDutyCycle == requested.PeakReverseDutyCycle)
            {
                std::cout << "[DriveSim] Verified Phoenix configuration transport after "
                          << std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() << " s, "
                          << attempts << " attempt(s)" << std::endl;
                // Native SwerveModule applies the full DriveMotorInitialConfigs;
                // its default +/-1 duty limits replace these temporary zero limits.
                return;
            }
        }
    } while (std::chrono::steady_clock::now() < deadline);
    throw std::runtime_error("Phoenix desktop simulation configuration transport did not complete verified "
                             "Apply/Refresh within 10 seconds for drive device " +
                             std::to_string(config.modules[0].drive) + " on bus " + config.bus + ": " + status);
}
} // namespace
struct Drive::Simulation
{
    Drive &owner;
    sim::DrivePhysics physics;
    double lastYaw{}, unwrappedYaw{};
    double timeOrigin = frc::RobotController::GetFPGATime() * 1e-6 - .02;
    sim::Registration registration; // release callbacks before physics and owner
    static sim::DrivePhysicsConfig Configuration(const DriveConfig &c)
    {
        sim::DrivePhysicsConfig out;
        const auto &s = c.simulation;
        out.mass = c.massKg;
        out.inertia = {s.rollInertia, s.pitchInertia, c.moi};
        out.centerOfMass = {s.centerOfMassX, s.centerOfMassY, s.centerOfMassZ};
        out.bumperHalfExtents = {s.bumperHalfLength, s.bumperHalfWidth, s.bumperHalfHeight};
        out.rideHeight = s.rideHeight;
        out.wheelRadius = c.wheelRadius;
        out.driveRatio = c.driveRatio;
        out.steerRatio = c.steerRatio;
        out.couplingRatio = c.couplingRatio;
        out.wheelInertia = s.wheelInertia;
        out.steerInertia = s.steerInertia;
        out.friction = c.cof;
        out.longitudinalStiffness = s.longitudinalStiffness;
        out.lateralStiffness = s.lateralStiffness;
        out.suspensionRestLength = s.suspensionRestLength;
        out.suspensionTravel = s.suspensionTravel;
        out.suspensionStiffness = s.suspensionStiffness;
        out.suspensionDamping = s.suspensionDamping;
        out.driveCurrentLimit = c.driveCurrent;
        out.steerCurrentLimit = c.steerCurrent;
        if (c.backend == DriveBackend::SparkMaxNavX)
        {
            const auto motor = frc::DCMotor::NEO(1);
            out.motorResistance = motor.R.value();
            out.motorKt = motor.Kt.value();
            out.motorKv = motor.Kv.value();
        }
        for (size_t i = 0; i < 4; ++i)
            out.moduleLocations[i] = {c.modules[i].xMeters, c.modules[i].yMeters, 0};
        return out;
    }
    explicit Simulation(Drive &drive)
        : owner(drive), physics(*sim::GetSimulationRuntime().World(), Configuration(drive.config_))
    {
        WriteSensors();
        registration = sim::GetSimulationRuntime().Register(
            {[this](double dt) {
                 if (owner.portable_)
                     owner.portable_->Sample(timeOrigin + physics.Truth().timestamp, frc::DriverStation::IsEnabled());
                 ReadVoltages(dt);
                 physics.BeforePhysics(dt);
             },
             [this](double dt) {
                 physics.AfterPhysics(dt);
                 WriteSensors();
             },
             [this] { owner.ResetSimulation(frc::Pose2d{}); },
             [this] { return physics.SupplyCurrent(sim::GetSimulationRuntime().BusVoltage()); }});
    }
    void ReadVoltages(double dt)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            const double bus = sim::GetSimulationRuntime().BusVoltage();
            if (owner.portable_)
            {
                const auto v = owner.portable_->SimVoltages(i, dt, bus);
                physics.SetModuleVoltages(i, v[0], v[1]);
                continue;
            }
            auto &module = owner.native_->GetModule(i);
            physics.SetModuleVoltages(
                i, std::clamp(module.GetDriveMotor().GetSimState().GetMotorVoltage().value(), -bus, bus),
                std::clamp(module.GetSteerMotor().GetSimState().GetMotorVoltage().value(), -bus, bus));
        }
    }
    void WriteSensors()
    {
        const auto truth = physics.Truth();
        for (size_t i = 0; i < 4; ++i)
        {
            if (owner.portable_)
            {
                const auto &m = truth.modules[i];
                owner.portable_->SetSimInputs(i, {m.wheelAngle * owner.config_.wheelRadius,
                                                  m.wheelVelocity * owner.config_.wheelRadius, m.steerAngle,
                                                  m.steerVelocity, 0, true});
                continue;
            }
            auto &module = owner.native_->GetModule(i);
            const auto &config = owner.config_.modules[i];
            auto &drive = module.GetDriveMotor().GetSimState();
            auto &steer = module.GetSteerMotor().GetSimState();
            auto &encoder = module.GetEncoder().GetSimState();
            // Match pinned Phoenix SimSwerveDrivetrain's physical coordinate convention.
            using Ref = p::sim::ChassisReference;
            drive.Orientation = config.driveInverted ? Ref::Clockwise_Positive : Ref::CounterClockwise_Positive;
            steer.Orientation = config.steerInverted ? Ref::Clockwise_Positive : Ref::CounterClockwise_Positive;
            encoder.Orientation = config.encoderInverted ? Ref::Clockwise_Positive : Ref::CounterClockwise_Positive;
            encoder.SensorOffset = units::turn_t{config.offsetTurns};
            auto supply = frc::RobotController::GetBatteryVoltage();
            drive.SetSupplyVoltage(supply);
            steer.SetSupplyVoltage(supply);
            encoder.SetSupplyVoltage(supply);
            const auto &m = truth.modules[i];
            drive.SetRawRotorPosition(units::radian_t{m.driveRotorAngle});
            drive.SetRotorVelocity(units::radians_per_second_t{m.driveRotorVelocity});
            steer.SetRawRotorPosition(units::radian_t{m.steerRotorAngle});
            steer.SetRotorVelocity(units::radians_per_second_t{m.steerRotorVelocity});
            encoder.SetRawPosition(units::radian_t{m.steerAngle});
            encoder.SetVelocity(units::radians_per_second_t{m.steerVelocity});
        }
        const auto q = truth.body.pose.rotation;
        const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        const double pitch = std::asin(std::clamp(2 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
        const double roll = std::atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
        unwrappedYaw += std::remainder(yaw - lastYaw, 2 * std::acos(-1.0));
        lastYaw = yaw;
        if (owner.portable_)
        {
            owner.portable_->SetSimYaw(unwrappedYaw);
            return;
        }
        auto &pigeon = owner.native_->GetPigeon2().GetSimState();
        pigeon.SetRawYaw(units::radian_t{unwrappedYaw});
        pigeon.SetPitch(units::radian_t{pitch});
        pigeon.SetRoll(units::radian_t{roll});
        // Pigeon angular rates are in sensor/body coordinates.
        auto w = truth.body.angularVelocity;
        const sim::Vec3 u{-q.x, -q.y, -q.z};
        auto cross = [](sim::Vec3 a, sim::Vec3 b) {
            return sim::Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
        };
        auto a = cross(u, w), b = cross(u, a);
        w = {w.x + 2 * (q.w * a.x + b.x), w.y + 2 * (q.w * a.y + b.y), w.z + 2 * (q.w * a.z + b.z)};
        pigeon.SetAngularVelocityX(units::radians_per_second_t{w.x});
        pigeon.SetAngularVelocityY(units::radians_per_second_t{w.y});
        pigeon.SetAngularVelocityZ(units::radians_per_second_t{w.z});
    }
    void Reset(const frc::Pose2d &pose)
    {
        double angle = pose.Rotation().Radians().value();
        lastYaw = angle;
        unwrappedYaw = angle;
        timeOrigin = frc::RobotController::GetFPGATime() * 1e-6 - .02;
        physics.Reset({{pose.X().value(), pose.Y().value(), owner.config_.simulation.rideHeight},
                       {0, 0, std::sin(angle / 2), std::cos(angle / 2)}});
        WriteSensors();
        if (owner.portable_)
        {
            owner.portable_->Sample(frc::RobotController::GetFPGATime() * 1e-6, false);
            owner.portable_->Update();
            owner.portable_->ResetPose(pose);
        }
        else
            owner.native_->ResetPose(pose);
    }
};
#endif
void DriveConfig::Validate() const
{
    if (backend != DriveBackend::Phoenix && backend != DriveBackend::SparkMaxNavX)
        throw std::invalid_argument("Unsupported drivetrain backend");
    if (backend == DriveBackend::SparkMaxNavX && odometryHz != 0 && (odometryHz < 20 || odometryHz > 200))
        throw std::invalid_argument("Portable acquisition requires 20 to 200 Hz");
    auto finite = [](double value) {
        if (!std::isfinite(value))
            throw std::invalid_argument("DriveConfig requires finite values");
    };
    auto positive = [&](double value) {
        finite(value);
        if (value <= 0)
            throw std::invalid_argument("DriveConfig requires positive physical dimensions, ratios and limits");
    };
    auto nonnegative = [&](double value) {
        finite(value);
        if (value < 0)
            throw std::invalid_argument("DriveConfig requires nonnegative gains, friction and damping");
    };
    for (double v : {driveRatio,
                     steerRatio,
                     wheelRadius,
                     maxSpeed,
                     maxAngularRate,
                     driveCurrent,
                     steerCurrent,
                     slipCurrent,
                     massKg,
                     moi,
                     simulation.bumperHalfLength,
                     simulation.bumperHalfWidth,
                     simulation.bumperHalfHeight,
                     simulation.rideHeight,
                     simulation.rollInertia,
                     simulation.pitchInertia,
                     simulation.wheelInertia,
                     simulation.steerInertia,
                     simulation.longitudinalStiffness,
                     simulation.lateralStiffness,
                     simulation.suspensionRestLength,
                     simulation.suspensionTravel,
                     simulation.suspensionStiffness,
                     simulation.suspensionDamping})
        positive(v);
    for (double v : {spark.driveP, spark.driveD, spark.driveKS, spark.driveKV, spark.steerP, spark.steerD, odometryHz,
                     autoTranslationKP, autoRotationKP, cof, driveKP, driveKD, driveKS, driveKV, steerKP, steerKD,
                     steerKS, steerKV})
        nonnegative(v);
    if (backend == DriveBackend::SparkMaxNavX)
        for (double derivative : {spark.driveD, spark.steerD})
            if (!std::isfinite(derivative * 1000))
                throw std::invalid_argument("Spark derivative gain overflows millisecond normalization");
    for (double v : {couplingRatio, simulation.centerOfMassX, simulation.centerOfMassY, simulation.centerOfMassZ})
        finite(v);
    if (backend == DriveBackend::SparkMaxNavX)
        for (double current : {driveCurrent, steerCurrent})
            if (current < 1 || current > 200 || std::floor(current) != current)
                throw std::invalid_argument("Spark swerve current limits must be integer amperes in [1,200]");
    // Talons share a device type; CANcoders and Pigeon have separate type IDs.
    std::set<int> motors, encoders;
    auto id = [](int v) {
        if (v < 0 || v > 62)
            throw std::invalid_argument("DriveConfig CAN IDs must be in [0,62]");
    };
    if (backend == DriveBackend::Phoenix)
        id(pigeon);
    else if (!bus.empty() && bus != "rio")
        throw std::invalid_argument("Spark swerve supports the roboRIO CAN bus only");
    double twiceArea = 0;
    for (size_t i = 0; i < modules.size(); ++i)
    {
        const auto &m = modules[i];
        id(m.drive);
        id(m.steer);
        if (backend == DriveBackend::Phoenix)
            id(m.encoder);
        if (!motors.insert(m.drive).second || !motors.insert(m.steer).second ||
            (backend == DriveBackend::Phoenix && !encoders.insert(m.encoder).second))
            throw std::invalid_argument("DriveConfig repeats a CAN device identity on its bus");
        finite(m.offsetTurns);
        finite(m.xMeters);
        finite(m.yMeters);
        if (backend == DriveBackend::SparkMaxNavX && (m.offsetTurns < 0 || m.offsetTurns >= 1))
            throw std::invalid_argument("REV absolute zero offsets must be in [0,1) rotations");
        for (size_t j = 0; j < i; ++j)
            if (std::hypot(m.xMeters - modules[j].xMeters, m.yMeters - modules[j].yMeters) < 1e-6)
                throw std::invalid_argument("DriveConfig module locations must be distinct");
    }
    for (size_t i = 1; i + 1 < modules.size(); ++i)
        twiceArea +=
            std::abs((modules[i].xMeters - modules[0].xMeters) * (modules[i + 1].yMeters - modules[0].yMeters) -
                     (modules[i].yMeters - modules[0].yMeters) * (modules[i + 1].xMeters - modules[0].xMeters));
    if (twiceArea < 1e-9)
        throw std::invalid_argument("DriveConfig module geometry must span a plane");
}
Drive::Drive(DriveConfig config) : config_(std::move(config))
{
    config_.Validate();
    SetName("Drive");
    telemetry_ = std::make_shared<Telemetry>();
    if (replay::Active())
    {
        replayKinematics_ = std::make_unique<ReplayKinematics>(config_);
        return; // Before any Phoenix device or Jolt owner.
    }
#ifndef __FRC_ROBORIO__
    if (frc::RobotBase::IsSimulation() && config_.backend == DriveBackend::Phoenix)
        AwaitPhoenixSimulationTransport(config_);
#endif
    if (config_.backend == DriveBackend::SparkMaxNavX)
    {
        portable_ = std::make_unique<PortableSwerve>(
            config_, MakeSparkNavXSwerveIO(config_, frc::RobotBase::IsSimulation()), !frc::RobotBase::IsSimulation());
#ifndef __FRC_ROBORIO__
        if (frc::RobotBase::IsSimulation())
        {
            if (!sim::GetSimulationRuntime().World())
                throw std::invalid_argument("REV simulation requires Jolt planar or 3D physics");
            simulation_ = std::make_unique<Simulation>(*this);
        }
#endif
        return;
    }
    p::configs::TalonFXConfiguration drive, steer;
    drive.CurrentLimits.WithSupplyCurrentLimit(units::ampere_t{config_.driveCurrent})
        .WithSupplyCurrentLimitEnable(true);
    steer.CurrentLimits.WithStatorCurrentLimit(units::ampere_t{config_.steerCurrent})
        .WithStatorCurrentLimitEnable(true);
    auto factory = s::SwerveModuleConstantsFactory<p::configs::TalonFXConfiguration, p::configs::TalonFXConfiguration,
                                                   p::configs::CANcoderConfiguration>{}
                       .WithDriveMotorGearRatio(config_.driveRatio)
                       .WithSteerMotorGearRatio(config_.steerRatio)
                       .WithCouplingGearRatio(config_.couplingRatio)
                       .WithWheelRadius(units::meter_t{config_.wheelRadius})
                       .WithDriveMotorGains(p::configs::Slot0Configs{}
                                                .WithKP(config_.driveKP)
                                                .WithKD(config_.driveKD)
                                                .WithKS(config_.driveKS)
                                                .WithKV(config_.driveKV))
                       .WithSteerMotorGains(p::configs::Slot0Configs{}
                                                .WithKP(config_.steerKP)
                                                .WithKD(config_.steerKD)
                                                .WithKS(config_.steerKS)
                                                .WithKV(config_.steerKV))
                       .WithDriveMotorClosedLoopOutput(s::ClosedLoopOutputType::Voltage)
                       .WithSteerMotorClosedLoopOutput(s::ClosedLoopOutputType::Voltage)
                       .WithDriveMotorType(s::DriveMotorArrangement::TalonFX_Integrated)
                       .WithSteerMotorType(s::SteerMotorArrangement::TalonFX_Integrated)
                       .WithFeedbackSource(s::SteerFeedbackType::RemoteCANcoder)
                       .WithSlipCurrent(units::ampere_t{config_.slipCurrent})
                       .WithSpeedAt12Volts(units::meters_per_second_t{config_.maxSpeed})
                       .WithDriveMotorInitialConfigs(drive)
                       .WithSteerMotorInitialConfigs(steer)
                       .WithSteerInertia(units::kilogram_square_meter_t{.004})
                       .WithDriveInertia(units::kilogram_square_meter_t{.025})
                       .WithSteerFrictionVoltage(.1_V)
                       .WithDriveFrictionVoltage(.1_V);
    auto make = [&](int i) {
        const auto &m = config_.modules[i];
        return factory.CreateModuleConstants(m.steer, m.drive, m.encoder, units::turn_t{m.offsetTurns},
                                             units::meter_t{m.xMeters}, units::meter_t{m.yMeters}, m.driveInverted,
                                             m.steerInverted, m.encoderInverted);
    };
    native_ = std::make_unique<NativeDrive>(
        s::SwerveDrivetrainConstants{}.WithCANBusName(config_.bus).WithPigeon2Id(config_.pigeon),
        units::hertz_t{config_.odometryHz}, make(0), make(1), make(2), make(3));
#ifndef __FRC_ROBORIO__
    if (frc::RobotBase::IsSimulation() && sim::GetSimulationRuntime().World())
        simulation_ = std::make_unique<Simulation>(*this);
    else if (frc::RobotBase::IsSimulation())
        diagnosticPowerRegistration_ = sim::GetSimulationRuntime().Register(
            {{}, {}, [this] { diagnosticSupplyCurrent_ = 0; }, [this] { return diagnosticSupplyCurrent_; }});
#endif
    SetName("Drive");
}
Drive::Drive(DriveConfig config, SwerveIO io) : config_(std::move(config))
{
    if (config_.backend != DriveBackend::SparkMaxNavX)
        throw std::invalid_argument("Injected portable Drive requires SparkMaxNavX backend");
    config_.Validate();
    SetName("Drive");
    telemetry_ = std::make_shared<Telemetry>();
    if (replay::Active())
        replayKinematics_ = std::make_unique<ReplayKinematics>(config_);
    else
        portable_ = std::make_unique<PortableSwerve>(config_, std::move(io), !frc::RobotBase::IsSimulation());
}
Drive::~Drive() = default;
void Drive::Periodic()
{
    if (frc::DriverStation::IsDisabled())
    {
        if (native_)
        {
            auto a = frc::DriverStation::GetAlliance();
            native_->SetOperatorPerspectiveForward(
                frc::Rotation2d{a && *a == frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg});
        }
        Stop();
    }
    if (portable_)
    {
#ifndef __FRC_ROBORIO__
        // Injected desktop IO has no Jolt owner; acquire once at robot cadence.
        if (frc::RobotBase::IsSimulation() && !simulation_)
            portable_->Sample(frc::RobotController::GetFPGATime() * 1e-6, frc::DriverStation::IsEnabled());
#endif
        portable_->Update();
    }
    auto state = ReadInputs();
    std::array<frc::SwerveModuleState, 4> targets = replayTargets_;
    if (native_)
    {
        auto measured = native_->GetState();
        std::copy(measured.ModuleTargets.begin(), measured.ModuleTargets.end(), targets.begin());
    }
    if (portable_)
        targets = portable_->State().targets;
    const auto requested = requestedSpeeds_;
    auto timestamp = frc::RobotController::GetFPGATime();
    telemetry::Post("Drive", [telemetry = telemetry_, state, targets, requested, timestamp] {
        telemetry->measured.Set(state.states, timestamp);
        telemetry->targets.Set(targets, timestamp);
        telemetry->pose.Set(state.pose, timestamp);
        telemetry->speeds.Set(state.speeds, timestamp);
        telemetry->requested.Set(requested, timestamp);
    });
}
void Drive::SimulationPeriodic()
{
    if (replay::Active())
        return;
#ifndef __FRC_ROBORIO__
    if (simulation_)
        return; // The single world clock owns all Jolt stepping.
#endif
    if (!native_)
        return;
    native_->UpdateSimState(.02_s, frc::RobotController::GetBatteryVoltage());
#ifndef __FRC_ROBORIO__
    // Phoenix status signals are asynchronously cached; diagnostic power is an estimate.
    diagnosticSupplyCurrent_ = 0;
    for (size_t i = 0; i < 4; ++i)
    {
        auto &module = native_->GetModule(i);
        diagnosticSupplyCurrent_ += std::max(0., module.GetDriveMotor().GetSupplyCurrent().GetValue().value()) +
                                    std::max(0., module.GetSteerMotor().GetSupplyCurrent().GetValue().value());
    }
#endif
}
#ifndef __FRC_ROBORIO__
std::optional<sim::DriveTruth> Drive::SimulationTruth() const
{
    if (simulation_)
        return simulation_->physics.Truth();
    return std::nullopt;
}
sim::DrivePhysics *Drive::SimulationPhysics()
{
    return simulation_ ? &simulation_->physics : nullptr;
}
void Drive::ResetSimulation(const frc::Pose2d &pose)
{
    if (simulation_)
    {
        simulation_->Reset(pose);
        if (poseResetCallback_)
            poseResetCallback_(pose);
    }
}
#endif
void Drive::RunVelocity(const frc::ChassisSpeeds &speeds)
{
    requestedSpeeds_ = speeds;
    if (native_)
        native_->SetControl(s::requests::ApplyRobotSpeeds{}.WithSpeeds(speeds).WithDriveRequestType(
            s::impl::DriveRequestType::Velocity));
    else if (portable_)
        portable_->Velocity(speeds);
    else
    {
        auto &kinematics = replayKinematics_->value;
        auto targets = kinematics.ToSwerveModuleStates(speeds);
        kinematics.DesaturateWheelSpeeds(&targets, units::meters_per_second_t{config_.maxSpeed});
        auto measured = ReadInputs();
        for (size_t i = 0; i < 4; ++i)
        {
            targets[i].Optimize(measured.states[i].angle);
            replayTargets_[i] = targets[i];
        }
    }
    RecordDriveOutput();
}
void Drive::RunVelocity(const frc::ChassisSpeeds &speeds, const pathplanner::DriveFeedforwards &ff)
{
    requestedSpeeds_ = speeds;
    if (native_)
        native_->SetControl(s::requests::ApplyRobotSpeeds{}
                                .WithSpeeds(speeds)
                                .WithDriveRequestType(s::impl::DriveRequestType::Velocity)
                                .WithWheelForceFeedforwardsX(ff.robotRelativeForcesX)
                                .WithWheelForceFeedforwardsY(ff.robotRelativeForcesY));
    else if (portable_)
    {
        std::array<double, 4> x{}, y{};
        if (!ff.robotRelativeForcesX.empty() || !ff.robotRelativeForcesY.empty())
        {
            if (ff.robotRelativeForcesX.size() != 4 || ff.robotRelativeForcesY.size() != 4)
            {
                Stop();
                throw std::invalid_argument("Swerve force feedforwards require four module values");
            }
            for (size_t i = 0; i < 4; ++i)
            {
                x[i] = ff.robotRelativeForcesX[i].value();
                y[i] = ff.robotRelativeForcesY[i].value();
            }
        }
        portable_->Velocity(speeds, x, y);
    }
    else
        RunVelocity(speeds);
    wpi::json x = wpi::json::array(), y = wpi::json::array();
    for (auto v : ff.robotRelativeForcesX)
        x.push_back(v.value());
    for (auto v : ff.robotRelativeForcesY)
        y.push_back(v.value());
    replay::RecordOutput("Drive/FeedforwardForces", {{"x", x}, {"y", y}});
    RecordDriveOutput();
}
} // namespace base
