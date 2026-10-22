#pragma once

#include "base/AdvantageKitLogReader.h"

#include <cstddef>
#include <string>
#include <vector>

#include <wpi/json.h>

namespace base::akit {

// Converts the logged state needed by this base repository's default Drive
// input contract from AdvantageKit 26.0.2 into native Replay/Frames records.
// It maps DriverStation state/joysticks/alliance, DriveState/Pose, Speeds,
// ModuleStates, and the logged autonomous chooser input. Other Java subsystem
// IO and vendor-device inputs are not covered. Missing required DS or drive
// fields, unexpected record schemas, and malformed struct payloads throw
// std::runtime_error; the adapter never falls back to live robot reads.
// If the Java log has no Auto Choices dashboard input, selection is "None",
// matching the C++ base chooser's no-autonomous default.
std::vector<wpi::json> ConvertJavaAdvantageKitFrames(
    const std::vector<Frame>& frames, std::string driveBackend = "phoenix");

// Converts one cycle without materializing a second vector of frames. The
// caller supplies its sequence index so production replay can append directly.
wpi::json ConvertJavaAdvantageKitFrame(
    const Frame& frame, size_t sequence, const std::string& driveBackend = "phoenix");

}  // namespace base::akit
