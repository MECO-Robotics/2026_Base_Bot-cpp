#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <utility>
#include "base/Mechanism.h"

namespace base {
// Offline harness only; callbacks run synchronously on the caller's thread.
// Integer ticks avoid wall-clock jitter/drift. Commands precede control at coincident ticks.
class DeterministicSimulation {
 public:
  static constexpr std::int64_t kTickMicroseconds=4000;
  template<class Commands, class Control, class Physics>
  void Step(Commands&& commands, Control&& control, Physics&& physics) {
    StepWithSensors([](auto){}, std::forward<Commands>(commands),
                    std::forward<Control>(control), std::forward<Physics>(physics));
  }
  // Deliver delayed inputs and DS/config events before scheduling at this tick.
  // The caller owns its HAL clock; physics completes the interval [t,t+4ms).
  template<class Sensors, class Commands, class Control, class Physics>
  void StepWithSensors(Sensors&& sensors, Commands&& commands, Control&& control, Physics&& physics) {
    const auto time=tick_*kTickMicroseconds;
    sensors(time);
    if(tick_%5==0) commands(time);
    control(time);
    physics(time);
    ++tick_;
  }
  std::int64_t Tick() const { return tick_; }
 private:
  std::int64_t tick_=0;
};

// Single-threaded sensor/NT transport model. Delivery times must be nondecreasing;
// variable latency may be supplied explicitly. Full queues reject rather than allocate/drop silently.
template<class T, std::size_t Capacity>
class LatencyChannel {
  static_assert(Capacity>0);
 public:
  bool Push(std::int64_t deliveryMicroseconds, const T& value) {
    if(size_ && deliveryMicroseconds<entries_[(head_+size_-1)%Capacity].time)
      throw std::invalid_argument("Out-of-order latency delivery");
    if(size_==Capacity) return false;
    entries_[(head_+size_)%Capacity]={deliveryMicroseconds,value}; ++size_; return true;
  }
  std::optional<T> Receive(std::int64_t nowMicroseconds) {
    std::optional<T> latest;
    while(size_ && entries_[head_].time<=nowMicroseconds) {
      latest=entries_[head_].value; head_=(head_+1)%Capacity; --size_;
    }
    return latest;
  }
 private:
  struct Entry { std::int64_t time=0; T value{}; };
  std::array<Entry,Capacity> entries_{};
  std::size_t head_=0,size_=0;
};

struct MotorReplaySample { std::int64_t timestampMicroseconds; MotorInputs inputs; };
// Non-owning immutable log view: backing storage must outlive this IO. No hardware,
// plant or clock is constructed. Explicit AdvanceTo makes repeated reads idempotent.
class ReplayMotorIO final : public MotorIO {
 public:
  explicit ReplayMotorIO(std::span<const MotorReplaySample> samples) : samples_(samples) {
    for(std::size_t i=0;i<samples.size();++i)
      if(samples[i].timestampMicroseconds<0 ||
          (i && samples[i].timestampMicroseconds<samples[i-1].timestampMicroseconds))
        throw std::invalid_argument("Unordered replay samples");
  }
  void AdvanceTo(std::int64_t timestampMicroseconds) {
    if(timestampMicroseconds<time_) throw std::invalid_argument("Replay time moved backward");
    time_=timestampMicroseconds;
    while(next_<samples_.size() && samples_[next_].timestampMicroseconds<=time_)
      inputs_=samples_[next_++].inputs;
  }
  MotorInputs Read() override { return inputs_; }
  void SetVoltage(double volts) override { requestedVoltage=volts; }
  void SetBrake(bool enabled) override { brake=enabled; }
  double requestedVoltage=0;
  bool brake=true;
 private:
  std::span<const MotorReplaySample> samples_;
  std::size_t next_=0;
  std::int64_t time_=-1;
  MotorInputs inputs_{};
};
}
