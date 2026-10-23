#include "base/Drive.h"
#include "base/PortableSwerve.h"
#include "base/DriveWheelRadius.h"
#include "base/Replay.h"
#include "base/Telemetry.h"
#include <frc/Timer.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/FunctionalCommand.h>
#include <ctre/phoenix6/Utils.hpp>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>
#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
namespace base
{
WheelRadiusMeasurement::WheelRadiusMeasurement(std::array<double, 4> distances) : distances_(distances)
{
    for (double distance : distances_)
        if (!std::isfinite(distance) || distance <= 0)
            throw std::invalid_argument("Invalid module distance");
}
bool WheelRadiusMeasurement::Add(double timestamp, double yaw, const std::array<double, 4> &angles, bool healthy)
{
    if (failed_)
        return false;
    if (!healthy || !std::isfinite(timestamp) || !std::isfinite(yaw))
    {
        failed_ = true;
        return false;
    }
    for (double angle : angles)
        if (!std::isfinite(angle))
        {
            failed_ = true;
            return false;
        }
    if (started_)
    {
        if (timestamp <= time_ || timestamp - time_ > .1)
        {
            failed_ = true;
            return false;
        }
        const double delta = std::remainder(yaw - lastYaw_, 2 * std::numbers::pi);
        // A .5 rad/s request cannot physically explain a multi-radian sample jump.
        if (std::abs(delta) > .2)
        {
            failed_ = true;
            return false;
        }
        yaw_ += delta;
    }
    else
    {
        initial_ = angles;
        started_ = true;
    }
    time_ = timestamp;
    lastYaw_ = yaw;
    current_ = angles;
    return true;
}
std::array<double, 4> WheelRadiusMeasurement::Radii() const
{
    std::array<double, 4> result{};
    for (size_t i = 0; i < 4; ++i)
        if (std::abs(current_[i] - initial_[i]) > 1e-6)
            result[i] = std::abs(yaw_) * distances_[i] / std::abs(current_[i] - initial_[i]);
    return result;
}
bool WheelRadiusMeasurement::Valid() const
{
    if (failed_ || !started_ || std::abs(yaw_) < 2 * std::numbers::pi)
        return false;
    const auto radii = Radii();
    const auto [low, high] = std::minmax_element(radii.begin(), radii.end());
    return std::isfinite(*high) && *low > .001 && *high < .5 && (*high - *low) / *low <= .2;
}
frc2::CommandPtr Drive::CharacterizeWheelRadius()
{
    struct State
    {
        explicit State(const DriveConfig &config)
            : measurement([&] {
                  std::array<double, 4> d{};
                  for (size_t i = 0; i < 4; ++i)
                      d[i] = std::hypot(config.modules[i].xMeters, config.modules[i].yMeters);
                  return d;
              }())
        {
        }
        WheelRadiusMeasurement measurement;
        frc::SlewRateLimiter<units::radians_per_second> limiter{units::radians_per_second_squared_t{.25}};
        double start = 0;
        bool failed = false, complete = false;
    };
    auto state = std::make_shared<State>(config_);
    return frc2::FunctionalCommand{
        [this, state] {
            Stop();
            *state = State(config_);
            state->start = frc::Timer::GetFPGATimestamp().value();
            state->limiter.Reset(units::radians_per_second_t{0});
        },
        [this, state] {
            try
            {
                const auto sample = replay::ProcessInputs("Drive/WheelRadius", [this] {
                    if (portable_)
                    {
                        const auto s = portable_->State();
                        wpi::json angles = wpi::json::array();
                        for (const auto &p : s.positions)
                            angles.push_back(p.distance.value() / config_.wheelRadius);
                        return wpi::json{{"timestamp", s.timestamp},
                                         {"healthy", s.healthy},
                                         {"rawYaw", s.rawYaw},
                                         {"angles", angles}};
                    }
                    const auto feedback = native_->GetState();
                    const double age = (ctre::phoenix6::utils::GetCurrentTime() - feedback.Timestamp).value();
                    wpi::json angles = wpi::json::array();
                    for (const auto &module : feedback.ModulePositions)
                        angles.push_back(module.distance.value() / config_.wheelRadius);
                    return wpi::json{
                        {"timestamp", feedback.Timestamp.value()},
                        {"healthy", native_->IsOdometryValid() && std::isfinite(age) && age >= 0 && age <= .1},
                        {"rawYaw", feedback.RawHeading.Radians().value()},
                        {"angles", angles}};
                });
                if (!sample.at("healthy").get<bool>())
                    throw std::runtime_error("Unhealthy wheel radius feedback");
                const double elapsed = frc::Timer::GetFPGATimestamp().value() - state->start;
                const auto angles = sample.at("angles").get<std::array<double, 4>>();
                if (!std::isfinite(sample.at("timestamp").get<double>()) ||
                    !std::isfinite(sample.at("rawYaw").get<double>()))
                    throw std::runtime_error("Nonfinite wheel radius feedback");
                for (double angle : angles)
                    if (!std::isfinite(angle))
                        throw std::runtime_error("Nonfinite wheel angle");
                if (elapsed >= 1)
                {
                    if (!state->measurement.Add(sample.at("timestamp").get<double>(), sample.at("rawYaw").get<double>(),
                                                angles, true))
                        throw std::runtime_error("Invalid wheel radius feedback");
                    state->complete = std::abs(state->measurement.YawTravel()) >= 2 * std::numbers::pi;
                }
                if (elapsed >= 35)
                    state->failed = true;
                if (state->complete || state->failed)
                    Stop();
                else
                    RunVelocity(frc::ChassisSpeeds{units::meters_per_second_t{0}, units::meters_per_second_t{0},
                                                   state->limiter.Calculate(units::radians_per_second_t{.5})});
            }
            catch (...)
            {
                state->failed = true;
                Stop();
            }
        },
        [this, state](bool interrupted) {
            Stop();
            const auto radii = state->measurement.Radii();
            const wpi::json result = {{"valid", !interrupted && !state->failed && state->measurement.Valid()},
                                      {"interrupted", interrupted},
                                      {"yawRadians", state->measurement.YawTravel()},
                                      {"radiiMeters", radii}};
            replay::RecordOutput("Drive/WheelRadiusResult", result);
            telemetry::Post("Drive/WheelRadiusResult", [result] {
                static auto publisher = nt::NetworkTableInstance::GetDefault()
                                            .GetStringTopic("/AdvantageScope/Drive/WheelRadiusResult")
                                            .Publish();
                publisher.Set(result.dump());
            });
        },
        [state] { return state->failed || state->complete; },
        {this}}
        .ToPtr();
}
} // namespace base
