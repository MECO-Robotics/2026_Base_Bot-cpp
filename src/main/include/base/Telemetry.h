#pragma once
#include <cstdint>
#include <functional>
#include <string>
namespace base::telemetry
{
// Best-effort live telemetry only. Capture values by value, never robot/IO pointers.
// A newer pending snapshot replaces an older snapshot with the same key.
void Post(std::string key, std::function<void()> publish);
struct Stats
{
    uint64_t coalesced{}, rejected{}, failed{};
};
Stats Statistics();
// Lifecycle/test barriers; never call Flush from a control-period callback.
void Flush();
void Shutdown();
} // namespace base::telemetry
