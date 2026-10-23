#include "base/AdvantageKitLogReader.h"

#include <gtest/gtest.h>
#include <wpi/DataLogWriter.h>
#include <array>
#include <chrono>
#include <filesystem>
#include <map>
#include <stdexcept>
#include <string>

namespace
{
std::filesystem::path TempLog(const char *suffix)
{
    return std::filesystem::temp_directory_path() /
           ("akit-reader-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + suffix);
}

TEST(AdvantageKitLogReader, GroupsTypedRecordsByTimestampAndPreservesMetadata)
{
    const auto path = TempLog(".wpilog");
    {
        std::error_code ec;
        wpi::log::DataLogWriter log(path.string(), ec, "AdvantageKit");
        ASSERT_FALSE(ec);
        const int timestamp = log.Start("/Timestamp", "int64");
        const int enabled = log.Start("/Robot/Enabled", "boolean", "{\"source\":\"AdvantageKit\"}");
        const int pose = log.Start("/Robot/Pose", "double[]");
        const int mode = log.Start("/Robot/Mode", "string");
        const int lateField = log.Start("/Robot/Late", "string");
        const int replayOutput = log.Start("/ReplayOutputs/Drive/Voltage", "double");
        const std::array<double, 3> firstPose{1.25, -2.5, 0.75};
        const std::array<double, 3> secondPose{4.0, 5.0, 6.0};
        for (const int64_t time : {100000, 120000})
        {
            log.AppendInteger(timestamp, time, time);
            if (time == 100000)
                log.AppendBoolean(enabled, true, time);
            log.AppendDoubleArray(
                pose, time == 100000 ? std::span<const double>(firstPose) : std::span<const double>(secondPose), time);
            log.AppendString(mode, time == 100000 ? "auto" : "teleop", time);
            if (time == 120000)
                log.AppendString(lateField, "appeared", time);
            log.AppendDouble(replayOutput, 12.0, time);
        }
        log.Flush();
    }

    const auto frames = base::akit::ReadLog(path.string());
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0].keyIndex, frames[1].keyIndex);
    EXPECT_EQ(frames[0].Find("Robot/Enabled"), frames[1].Find("Robot/Enabled"));
    EXPECT_NE(frames[0].Find("Robot/Mode"), frames[1].Find("Robot/Mode"));
    EXPECT_EQ(frames[0].Find("Robot/Late"), nullptr);
    ASSERT_NE(frames[1].Find("Robot/Late"), nullptr);
    EXPECT_EQ(std::get<std::string>(frames[1].Find("Robot/Late")->value), "appeared");
    EXPECT_LT(frames[0].records.size(), frames[1].records.size());
    EXPECT_EQ(frames[0].timestampUs, 100000);
    EXPECT_EQ(frames[1].timestampUs, 120000);
    for (const auto &frame : frames)
    {
        const auto *enabledRecord = frame.Find("Robot/Enabled");
        const auto *modeRecord = frame.Find("Robot/Mode");
        const auto *poseRecord = frame.Find("Robot/Pose");
        ASSERT_NE(enabledRecord, nullptr);
        ASSERT_NE(modeRecord, nullptr);
        ASSERT_NE(poseRecord, nullptr);
        EXPECT_EQ(enabledRecord->type, "boolean");
        EXPECT_EQ(enabledRecord->metadata, "{\"source\":\"AdvantageKit\"}");
        EXPECT_TRUE(std::get<bool>(enabledRecord->value));
        EXPECT_EQ(std::get<std::string>(modeRecord->value), frame.timestampUs == 100000 ? "auto" : "teleop");
        const auto &values = std::get<std::vector<double>>(poseRecord->value);
        ASSERT_EQ(values.size(), 3u);
        EXPECT_DOUBLE_EQ(values[0], frame.timestampUs == 100000 ? 1.25 : 4.0);
    }
    std::filesystem::remove(path);
}

TEST(AdvantageKitLogReader, RejectsNonAdvantageKitHeader)
{
    const auto wrongHeader = TempLog("-header.wpilog");
    {
        std::error_code ec;
        wpi::log::DataLogWriter log(wrongHeader.string(), ec, "Other");
        ASSERT_FALSE(ec);
        log.Flush();
    }
    EXPECT_THROW(base::akit::ReadLog(wrongHeader.string()), std::runtime_error);
    std::filesystem::remove(wrongHeader);
}

TEST(AdvantageKitLogReader, RejectsNonIncreasingCycleTimestamps)
{
    const auto path = TempLog("-timestamps.wpilog");
    {
        std::error_code ec;
        wpi::log::DataLogWriter log(path.string(), ec, "AdvantageKit");
        ASSERT_FALSE(ec);
        const int timestamp = log.Start("/Timestamp", "int64");
        log.AppendInteger(timestamp, 100000, 100000);
        log.AppendInteger(timestamp, 100000, 100000);
        log.Flush();
    }
    EXPECT_THROW(base::akit::ReadLog(path.string()), std::runtime_error);
    std::filesystem::remove(path);
}

TEST(AdvantageKitLogReader, PreservesCustomRawSchemaAndPayload)
{
    const auto customSchema = TempLog("-schema.wpilog");
    {
        std::error_code ec;
        wpi::log::DataLogWriter log(customSchema.string(), ec, "AdvantageKit");
        ASSERT_FALSE(ec);
        const int timestamp = log.Start("/Timestamp", "int64");
        const int structEntry = log.Start("/Robot/Pose", "struct:Pose2d");
        const std::array<uint8_t, 3> bytes{1, 2, 3};
        log.AppendInteger(timestamp, 100000, 100000);
        log.AppendRaw(structEntry, bytes, 100000);
        log.Flush();
    }
    const auto frames = base::akit::ReadLog(customSchema.string());
    ASSERT_EQ(frames.size(), 1u);
    ASSERT_EQ(frames.front().records.size(), 1u);
    const auto *pose = frames.front().Find("Robot/Pose");
    ASSERT_NE(pose, nullptr);
    EXPECT_EQ(pose->type, "struct:Pose2d");
    EXPECT_EQ(std::get<std::vector<uint8_t>>(pose->value), (std::vector<uint8_t>{1, 2, 3}));
    std::filesystem::remove(customSchema);
}
} // namespace
