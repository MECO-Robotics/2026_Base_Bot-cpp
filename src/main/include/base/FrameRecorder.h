#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string_view>
#include <wpi/json.h>
namespace base {
// Ordered recording, separate from coalescing live telemetry. The producer moves
// a completed snapshot; serialization and sinks run only on a low-priority worker.
class FrameRecorder {
 public:
  struct Stats { uint64_t written{}, rejected{}, failed{}; size_t highWater{}; };
  using Sink=std::function<void(std::string_view,int64_t)>;
  using FaultSink=std::function<void(Stats)>;
  FrameRecorder(Sink sink,FaultSink fault,size_t capacity=256);
  ~FrameRecorder();
  FrameRecorder(const FrameRecorder&)=delete;
  FrameRecorder& operator=(const FrameRecorder&)=delete;
  // Never waits for sink/disk IO. Saturation is sticky: refuse subsequent frames
  // and report an invalid recording rather than silently presenting a valid log.
  bool Submit(wpi::json snapshot,int64_t timestamp);
  Stats Statistics() const;
  // Lifecycle/offline-replay barriers only, never a live periodic callback.
  void Flush();
  void Stop();
 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
