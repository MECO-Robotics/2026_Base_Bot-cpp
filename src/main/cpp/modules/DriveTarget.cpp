module;
#include <cmath>
#include <mutex>
module meco.base.drive_target;
namespace base::drive_target_detail
{
uint64_t TargetStore::CaptureEpoch() const
{
    return epoch_.load();
}
void TargetStore::Publish(TargetValue value)
{
    std::scoped_lock lock(mutex_);
    pending_ = value;
}
void TargetStore::Invalidate()
{
    epoch_.fetch_add(1);
}
TargetValue TargetStore::Consume(double now, bool enabled)
{
    if (!enabled)
    {
        Invalidate();
        cached_ = {};
        return {};
    }
    if (std::unique_lock lock(mutex_, std::try_to_lock); lock.owns_lock())
        cached_ = pending_;
    bool valid = std::isfinite(now) && std::isfinite(cached_.timestamp) && now >= cached_.timestamp &&
                 (cached_.brake || now - cached_.timestamp <= .1) && cached_.epoch == epoch_.load() &&
                 std::isfinite(cached_.headingRadians) && std::isfinite(cached_.headingKP) &&
                 std::isfinite(cached_.maxOmega) && cached_.maxOmega >= 0 && std::isfinite(cached_.vx) &&
                 std::isfinite(cached_.vy) && std::isfinite(cached_.omega);
    for (int i = 0; i < 4; ++i)
        valid = valid && std::isfinite(cached_.forceX[i]) && std::isfinite(cached_.forceY[i]);
    return valid ? cached_ : TargetValue{};
}
} // namespace base::drive_target_detail
