#pragma once
#include <array>
#include <algorithm>
#include <frc/geometry/Rotation2d.h>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <frc/kinematics/ChassisSpeeds.h>

#if defined(MECO_USE_NAMED_MODULES)
import meco.base.drive_target;
#endif

namespace base
{
// Commands own the subsystem and publish intent; only the native control thread
// consumes it. A contended reader keeps its previous target, never waits.
class DriveTarget
{
  public:
    struct Value
    {
        frc::ChassisSpeeds speeds{};
        std::array<double, 4> forceX{}, forceY{}; // newtons, module order
        bool brake = false, fieldRelative = false, headingControl = false, teleopOnly = false;
        frc::Rotation2d heading{};
        double headingKP = 0, maxOmega = 0;
        double timestamp = 0;
        uint64_t epoch = 0;
    };
    template <class Enabled> void Publish(Value value, Enabled &&enabled)
    {
#if defined(MECO_USE_NAMED_MODULES)
        drive_target_detail::TargetValue target{value.speeds.vx.value(),
                                                value.speeds.vy.value(),
                                                value.speeds.omega.value(),
                                                value.forceX,
                                                value.forceY,
                                                value.brake,
                                                value.fieldRelative,
                                                value.headingControl,
                                                value.teleopOnly,
                                                value.heading.Radians().value(),
                                                value.headingKP,
                                                value.maxOmega,
                                                value.timestamp,
                                                0};
        target.epoch = store_.CaptureEpoch();
        if (!enabled())
        {
            store_.Invalidate();
            return;
        }
        store_.Publish(target);
#else
        // Capture the epoch before authorization: a disable between either check and
        // the write must invalidate this packet, including across a fast re-enable.
        value.epoch = epoch_.load();
        if (!enabled())
        {
            Invalidate();
            return;
        }
        std::scoped_lock lock(mutex_);
        pending_ = value;
#endif
    }
    // May be called by the robot thread even if no native tick observes disable.
    // Only the consumer mutates cached_; invalidation itself is atomic.
    inline void Invalidate()
    {
#if defined(MECO_USE_NAMED_MODULES)
        store_.Invalidate();
#else
        epoch_.fetch_add(1);
#endif
    }
    inline Value Consume(double now, bool enabled)
    {
#if defined(MECO_USE_NAMED_MODULES)
        const auto target = store_.Consume(now, enabled);
        Value value;
        value.speeds = {units::meters_per_second_t{target.vx}, units::meters_per_second_t{target.vy},
                        units::radians_per_second_t{target.omega}};
        value.forceX = target.forceX;
        value.forceY = target.forceY;
        value.brake = target.brake;
        value.fieldRelative = target.fieldRelative;
        value.headingControl = target.headingControl;
        value.teleopOnly = target.teleopOnly;
        value.heading = frc::Rotation2d{units::radian_t{target.headingRadians}};
        value.headingKP = target.headingKP;
        value.maxOmega = target.maxOmega;
        value.timestamp = target.timestamp;
        value.epoch = target.epoch;
        return value;
#else
        if (!enabled)
        {
            Invalidate();
            cached_ = {};
            return {};
        }
        if (std::unique_lock lock(mutex_, std::try_to_lock); lock.owns_lock())
            cached_ = pending_;
        const auto &s = cached_.speeds;
        bool valid = std::isfinite(now) && std::isfinite(cached_.timestamp) && now >= cached_.timestamp &&
                     (cached_.brake || now - cached_.timestamp <= .1) && cached_.epoch == epoch_.load() &&
                     std::isfinite(cached_.heading.Radians().value()) && std::isfinite(cached_.headingKP) &&
                     std::isfinite(cached_.maxOmega) && cached_.maxOmega >= 0 && std::isfinite(s.vx.value()) &&
                     std::isfinite(s.vy.value()) && std::isfinite(s.omega.value());
        for (int i = 0; i < 4; ++i)
            valid = valid && std::isfinite(cached_.forceX[i]) && std::isfinite(cached_.forceY[i]);
        return valid ? cached_ : Value{};
#endif
    }
    static frc::ChassisSpeeds Resolve(const Value &target, frc::Rotation2d measuredHeading)
    {
        if ((target.fieldRelative || target.headingControl) && !std::isfinite(measuredHeading.Radians().value()))
            return {};
        auto speeds = target.speeds;
        if (target.headingControl)
            speeds.omega = units::radians_per_second_t{
                std::clamp(target.headingKP * (target.heading - measuredHeading).Radians().value(), -target.maxOmega,
                           target.maxOmega)};
        return target.fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, measuredHeading) : speeds;
    }

  private:
#if defined(MECO_USE_NAMED_MODULES)
    drive_target_detail::TargetStore store_;
#else
    std::mutex mutex_;
    std::atomic<uint64_t> epoch_{1};
    Value pending_{}, cached_{};
#endif
};
} // namespace base
