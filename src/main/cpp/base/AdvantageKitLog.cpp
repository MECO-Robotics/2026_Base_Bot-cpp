#include "base/AdvantageKitLog.h"

#include <memory>
#include <algorithm>
#include <stdexcept>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>
#include <string>

#include <wpi/DataLog.h>
#include <wpi/DataLogBackgroundWriter.h>
#include <frc/DriverStation.h>
#include <networktables/NetworkTableInstance.h>

namespace base::logging {
namespace {
std::unique_ptr<wpi::log::DataLogBackgroundWriter> writer;
int timestampEntry = 0;
NT_DataLogger ntEntryLogger = 0;
NT_ConnectionDataLogger ntConnectionLogger = 0;
bool writerStopped = false;
bool robotDataLogSourcesStarted = false;
std::filesystem::path logPath;
}

void StartAdvantageKitLog(std::string_view directory,
                          std::string_view filename,
                          double flushPeriodSeconds) {
  if (writer) {
    return;
  }
  const std::filesystem::path logDirectory = directory.empty() ? "." : std::string{directory};
  std::filesystem::create_directories(logDirectory);
  std::string selectedFilename{filename};
  if (selectedFilename.empty()) {
    const auto unique = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    selectedFilename = "robot-" + std::to_string(unique) + ".wpilog";
  }
  logPath = logDirectory / selectedFilename;
  writerStopped = false;
  writer = std::make_unique<wpi::log::DataLogBackgroundWriter>(
      logDirectory.string(), selectedFilename, flushPeriodSeconds, kAdvantageKitHeader);
  // AdvantageKit's reader expects this entry to be declared before other
  // entries and uses its integer samples as cycle boundaries.
  timestampEntry = writer->Start("/Timestamp", "int64", kAdvantageKitMetadata);
  if (timestampEntry == 0) {
    writer.reset();
    throw std::runtime_error("failed to start AdvantageKit /Timestamp entry");
  }
}

wpi::log::DataLog& GetLog() {
  if (!writer) {
    StartAdvantageKitLog();
  }
  if (writerStopped) {
    throw std::logic_error("AdvantageKit log is stopped");
  }
  return *writer;
}

void AppendTimestamp(int64_t timestampUs) {
  if (!writer || timestampEntry == 0) {
    throw std::logic_error("AdvantageKit log has not been started");
  }
  writer->AppendInteger(timestampEntry, timestampUs, timestampUs);
}

void StartRobotDataLogSources() {
  if (robotDataLogSourcesStarted) return;
  auto& log = GetLog();
  auto nt = nt::NetworkTableInstance::GetDefault();
  // AdvantageKit's WPILOGReader removes the first character from every entry
  // name when loading LogTable paths. Preserve WPILib's conventional NT: path
  // by giving the stored entry a leading slash.
  ntEntryLogger = nt.StartEntryDataLog(log, "", "/NT:");
  ntConnectionLogger = nt.StartConnectionDataLog(log, "NTConnection");
  frc::DriverStation::StartDataLog(log);
  robotDataLogSourcesStarted = true;
}

void StopNetworkTablesDataLog() {
  if (ntEntryLogger != 0) {
    nt::NetworkTableInstance::StopEntryDataLog(ntEntryLogger);
    ntEntryLogger = 0;
  }
  if (ntConnectionLogger != 0) {
    nt::NetworkTableInstance::StopConnectionDataLog(ntConnectionLogger);
    ntConnectionLogger = 0;
  }
  robotDataLogSourcesStarted = false;
}

void StopAdvantageKitLog() {
  if (!writer || writerStopped) {
    return;
  }
  // DataLogBackgroundWriter::Flush() is asynchronous, and Stop() can close
  // its file before a pending flush is handled. Persist a unique marker first
  // so orderly shutdown never drops the final buffered records.
  const auto marker = "akit-stop-" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count());
  const int markerEntry = writer->Start("/Logging/ShutdownMarker", "string",
                                        kAdvantageKitMetadata);
  if (markerEntry == 0) throw std::runtime_error("failed to start log shutdown marker");
  writer->AppendString(markerEntry, marker, 0);
  writer->Flush();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  bool persisted = false;
  do {
    std::ifstream file(logPath, std::ios::binary);
    if (file) {
      file.seekg(0, std::ios::end);
      const auto end = file.tellg();
      file.seekg(std::max<std::streamoff>(0,
          static_cast<std::streamoff>(end) - 8192));
      const std::string contents((std::istreambuf_iterator<char>(file)), {});
      if (contents.find(marker) != std::string::npos) {
        persisted = true;
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  } while (std::chrono::steady_clock::now() < deadline);
  if (!persisted) throw std::runtime_error("AdvantageKit log did not persist its shutdown marker");
  writer->Stop();
  // DriverStation::StartDataLog has no stop API and retains this log for the
  // process lifetime. Keep the stopped object alive to avoid invalidating its
  // callback target during robot shutdown.
  timestampEntry = 0;
  writerStopped = true;
}

}  // namespace base::logging
