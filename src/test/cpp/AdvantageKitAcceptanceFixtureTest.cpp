#include "base/AdvantageKitLog.h"

#include <gtest/gtest.h>
#include <wpi/DataLog.h>

#include <cstdlib>
#include <chrono>
#include <filesystem>
#include <string>
#include <thread>

TEST(AdvantageKitAcceptanceFixture, WritesTypedTimestampedCppLogWhenRequested)
{
    const char *output = std::getenv("AKIT_ACCEPTANCE_CPP_OUTPUT");
    if (output == nullptr || *output == '\0')
    {
        GTEST_SKIP() << "set AKIT_ACCEPTANCE_CPP_OUTPUT to export the acceptance WPILOG";
    }

    const std::filesystem::path path(output);
    std::filesystem::create_directories(path.parent_path());
    std::filesystem::remove(path);

    base::logging::StartAdvantageKitLog(path.parent_path().string(), path.filename().string());
    auto &log = base::logging::GetLog();
    const auto metadata = base::logging::kAdvantageKitMetadata;
    const int backend = log.Start("/ReplayInputs/Drive/backend", "string", metadata);
    const int poseX = log.Start("/ReplayInputs/Drive/pose/0", "double", metadata);
    const int commandedVoltage = log.Start("/ReplayOutputs/Drive/voltage", "double", metadata);

    base::logging::AppendTimestamp(1'000'000);
    log.AppendString(backend, "rev", 1'000'000);
    log.AppendDouble(poseX, 1.25, 1'000'000);
    log.AppendDouble(commandedVoltage, 6.0, 1'000'000);
    base::logging::AppendTimestamp(1'020'000);
    log.AppendString(backend, "rev", 1'020'000);
    log.AppendDouble(poseX, 1.5, 1'020'000);
    log.AppendDouble(commandedVoltage, -2.0, 1'020'000);

    log.Flush();
    // DataLogBackgroundWriter flushes asynchronously; allow its configured
    // 250 ms period to commit the samples before the stop boundary.
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    base::logging::StopAdvantageKitLog();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ASSERT_TRUE(std::filesystem::is_regular_file(path));
    ASSERT_GT(std::filesystem::file_size(path), 0u);
}
