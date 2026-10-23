#pragma once

#include "base/AdvantageKitLogReader.h"

#include <cstddef>
#include <string>
#include <vector>

#include <wpi/json.h>

namespace base::akit
{

// Maps AdvantageKit 26.0.2 DriverStation, drive, and auto-choice inputs to native replay frames.
// Other subsystem IO and vendor-device inputs are not covered. Missing fields, unknown schemas,
// or malformed structs throw; replay never falls back to live reads. Missing auto choice maps to "None".
std::vector<wpi::json> ConvertJavaAdvantageKitFrames(const std::vector<Frame> &frames,
                                                     std::string driveBackend = "phoenix");

// Converts one cycle without materializing a second vector of frames. The
// caller supplies its sequence index so production replay can append directly.
wpi::json ConvertJavaAdvantageKitFrame(const Frame &frame, size_t sequence,
                                       const std::string &driveBackend = "phoenix");

} // namespace base::akit
