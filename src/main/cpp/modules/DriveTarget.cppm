module;
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
export module meco.base.drive_target;
export namespace base::drive_target_detail
{
struct TargetValue
{
    double vx = 0, vy = 0, omega = 0;
    std::array<double, 4> forceX{}, forceY{};
    bool brake = false, fieldRelative = false, headingControl = false, teleopOnly = false;
    double headingRadians = 0, headingKP = 0, maxOmega = 0, timestamp = 0;
    uint64_t epoch = 0;
};
class TargetStore
{
  public:
    uint64_t CaptureEpoch() const;
    void Publish(TargetValue value);
    void Invalidate();
    TargetValue Consume(double now, bool enabled);

  private:
    std::mutex mutex_;
    std::atomic<uint64_t> epoch_{1};
    TargetValue pending_{}, cached_{};
};
} // namespace base::drive_target_detail
