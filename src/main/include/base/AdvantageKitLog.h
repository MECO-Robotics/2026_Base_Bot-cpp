#pragma once

#include <cstdint>
#include <string_view>

namespace wpi::log
{
class DataLog;
}

namespace base::logging
{

// Starts the primary WPILOG with the AdvantageKit v26 header and timestamp
// entry. The timestamp entry is created first and must be appended once for
// each cycle before that cycle's values are written.
void StartAdvantageKitLog(std::string_view directory = {}, std::string_view filename = {},
                          double flushPeriodSeconds = 0.25);
wpi::log::DataLog &GetLog();
void AppendTimestamp(int64_t timestampUs);
// Attach WPILib NetworkTables and Driver Station logging to the same log.
void StartRobotDataLogSources();
void StopNetworkTablesDataLog();
void StopAdvantageKitLog();

inline constexpr std::string_view kAdvantageKitHeader = "AdvantageKit";
inline constexpr std::string_view kAdvantageKitMetadata = R"({"source":"AdvantageKit"})";

} // namespace base::logging
