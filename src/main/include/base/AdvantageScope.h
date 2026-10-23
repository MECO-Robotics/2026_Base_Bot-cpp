#pragma once
#include <string_view>
#include <networktables/NetworkTableInstance.h>
namespace base
{
// Publish visualizer metadata through the low-priority snapshot worker.
void PublishTelemetryMetadata(std::string_view project,
                              nt::NetworkTableInstance instance = nt::NetworkTableInstance::GetDefault());
} // namespace base
