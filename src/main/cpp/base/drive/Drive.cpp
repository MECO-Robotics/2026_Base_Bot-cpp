#include "base/Drive.h"
#include "base/PortableSwerve.h"
#include "base/Replay.h"

#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include <ctre/phoenix6/Utils.hpp>

namespace p = ctre::phoenix6;
namespace base
{
Drive::Inputs Drive::ReadInputs() const
{
    auto j = replay::ProcessInputs("Drive", [this] {
        if (portable_)
        {
            auto s = portable_->State();
            wpi::json states = wpi::json::array();
            for (const auto &m : s.states)
            {
                states.push_back(m.speed.value());
                states.push_back(m.angle.Radians().value());
            }
            return wpi::json{{"backend", "rev"},
                             {"pose", {s.pose.X().value(), s.pose.Y().value(), s.pose.Rotation().Radians().value()}},
                             {"speeds", {s.speeds.vx.value(), s.speeds.vy.value(), s.speeds.omega.value()}},
                             {"states", states}};
        }
        auto s = native_->GetState();
        wpi::json states = wpi::json::array();
        for (const auto &m : s.ModuleStates)
        {
            states.push_back(m.speed.value());
            states.push_back(m.angle.Radians().value());
        }
        return wpi::json{{"pose", {s.Pose.X().value(), s.Pose.Y().value(), s.Pose.Rotation().Radians().value()}},
                         {"speeds", {s.Speeds.vx.value(), s.Speeds.vy.value(), s.Speeds.omega.value()}},
                         {"states", states}};
    });
    auto number = [](const wpi::json &v) {
        if (!v.is_number() || !std::isfinite(v.get<double>()))
            throw std::runtime_error("Invalid Drive replay input");
        return v.get<double>();
    };
    if (j.at("pose").size() != 3 || j.at("speeds").size() != 3 || j.at("states").size() != 8)
        throw std::runtime_error("Invalid Drive input dimensions");
    Inputs out;
    out.pose = {units::meter_t{number(j["pose"][0])}, units::meter_t{number(j["pose"][1])},
                frc::Rotation2d{units::radian_t{number(j["pose"][2])}}};
    out.speeds = {units::meters_per_second_t{number(j["speeds"][0])},
                  units::meters_per_second_t{number(j["speeds"][1])},
                  units::radians_per_second_t{number(j["speeds"][2])}};
    for (size_t i = 0; i < 4; ++i)
        out.states[i] = {units::meters_per_second_t{number(j["states"][2 * i])},
                         frc::Rotation2d{units::radian_t{number(j["states"][2 * i + 1])}}};
    return out;
}
void Drive::RecordDriveOutput()
{
    wpi::json targets = wpi::json::array();
    std::vector<frc::SwerveModuleState> states(replayTargets_.begin(), replayTargets_.end());
    if (native_)
        states = native_->GetState().ModuleTargets;
    if (portable_)
    {
        const auto s = portable_->State();
        states.assign(s.targets.begin(), s.targets.end());
    }
    for (const auto &m : states)
    {
        targets.push_back(m.speed.value());
        targets.push_back(m.angle.Radians().value());
    }
    replay::RecordOutput("Drive/RequestedChassisSpeeds",
                         {requestedSpeeds_.vx.value(), requestedSpeeds_.vy.value(), requestedSpeeds_.omega.value()});
    replay::RecordOutput("Drive/ModuleTargets", targets);
}
frc::Pose2d Drive::Pose() const
{
    return ReadInputs().pose;
}
frc::ChassisSpeeds Drive::Speeds() const
{
    return ReadInputs().speeds;
}
void Drive::ResetPose(const frc::Pose2d &pose)
{
    replay::RecordOutput("Drive/ResetPose", {pose.X().value(), pose.Y().value(), pose.Rotation().Radians().value()});
    if (native_)
        native_->ResetPose(pose);
    if (portable_)
        portable_->ResetPose(pose);
    if (poseResetCallback_)
        poseResetCallback_(pose);
}
void Drive::SetPoseResetCallback(std::function<void(const frc::Pose2d &)> callback)
{
    poseResetCallback_ = std::move(callback);
}
void Drive::AddVision(const frc::Pose2d &pose, units::second_t time, std::array<double, 3> noise)
{
    wpi::json deviations = wpi::json::array();
    for (double value : noise)
        deviations.push_back(std::isfinite(value) ? wpi::json(value) : wpi::json(nullptr));
    replay::RecordOutput("Drive/Vision",
                         {{"pose", {pose.X().value(), pose.Y().value(), pose.Rotation().Radians().value()}},
                          {"time", time.value()},
                          {"noise", deviations}});
    if (native_)
        native_->AddVisionMeasurement(pose, p::utils::FPGAToCurrentTime(time), noise);
    if (portable_)
        portable_->AddVision(pose, time, noise);
}
} // namespace base
