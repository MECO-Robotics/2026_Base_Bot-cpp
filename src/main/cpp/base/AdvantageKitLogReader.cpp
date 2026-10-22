#include "base/AdvantageKitLogReader.h"

#include <map>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <wpi/DataLogReader.h>
#include <wpi/MemoryBuffer.h>

namespace base::akit {
namespace {
struct Entry {
  std::string name;
  std::string type;
  std::string metadata;
};

std::string Copy(std::string_view value) { return {value.data(), value.size()}; }

Record Decode(const wpi::log::DataLogRecord& record, const Entry& entry) {
  Record result;
  result.type = entry.type;
  result.metadata = entry.metadata;
  if (entry.type == "boolean") {
    bool value{};
    if (!record.GetBoolean(&value)) throw std::runtime_error("Malformed boolean record: " + entry.name);
    result.value = value;
  } else if (entry.type == "int64") {
    int64_t value{};
    if (!record.GetInteger(&value)) throw std::runtime_error("Malformed int64 record: " + entry.name);
    result.value = value;
  } else if (entry.type == "float") {
    float value{};
    if (!record.GetFloat(&value)) throw std::runtime_error("Malformed float record: " + entry.name);
    result.value = value;
  } else if (entry.type == "double") {
    double value{};
    if (!record.GetDouble(&value)) throw std::runtime_error("Malformed double record: " + entry.name);
    result.value = value;
  } else if (entry.type == "string" || entry.type == "json") {
    std::string_view value;
    if (!record.GetString(&value)) throw std::runtime_error("Malformed string record: " + entry.name);
    result.value = Copy(value);
  } else if (entry.type == "raw") {
    const auto bytes = record.GetRaw();
    result.value = std::vector<uint8_t>(bytes.begin(), bytes.end());
  } else if (entry.type == "boolean[]") {
    std::vector<int> values;
    if (!record.GetBooleanArray(&values)) throw std::runtime_error("Malformed boolean[] record: " + entry.name);
    std::vector<bool> converted;
    converted.reserve(values.size());
    for (const int value : values) converted.push_back(value != 0);
    result.value = std::move(converted);
  } else if (entry.type == "int64[]") {
    std::vector<int64_t> value;
    if (!record.GetIntegerArray(&value)) throw std::runtime_error("Malformed int64[] record: " + entry.name);
    result.value = std::move(value);
  } else if (entry.type == "float[]") {
    std::vector<float> value;
    if (!record.GetFloatArray(&value)) throw std::runtime_error("Malformed float[] record: " + entry.name);
    result.value = std::move(value);
  } else if (entry.type == "double[]") {
    std::vector<double> value;
    if (!record.GetDoubleArray(&value)) throw std::runtime_error("Malformed double[] record: " + entry.name);
    result.value = std::move(value);
  } else if (entry.type == "string[]") {
    std::vector<std::string_view> values;
    if (!record.GetStringArray(&values)) throw std::runtime_error("Malformed string[] record: " + entry.name);
    std::vector<std::string> converted;
    converted.reserve(values.size());
    for (const auto value : values) converted.push_back(Copy(value));
    result.value = std::move(converted);
  } else {
    // AdvantageKit treats custom WPILOG schemas as raw values and keeps the
    // schema string (for example struct:Pose2d) as type metadata.
    const auto bytes = record.GetRaw();
    result.value = std::vector<uint8_t>(bytes.begin(), bytes.end());
  }
  return result;
}
}  // namespace

std::vector<Frame> ReadLog(const std::string& path) {
  auto buffer = wpi::MemoryBuffer::GetFile(path);
  if (!buffer) throw std::runtime_error("Unable to read AdvantageKit log: " + path);
  wpi::log::DataLogReader reader(std::move(*buffer));
  return ReadLog(reader);
}

std::vector<Frame> ReadLog(const wpi::log::DataLogReader& reader) {
  if (!reader.IsValid()) throw std::runtime_error("Invalid WPILOG");
  if (reader.GetExtraHeader() != "AdvantageKit")
    throw std::runtime_error("WPILOG was not produced by AdvantageKit");

  std::map<int, Entry> entries;
  auto keyIndex = std::make_shared<KeyIndex>();
  std::vector<Frame> frames;
  bool haveTimestamp = false;
  int64_t timestamp = 0;
  for (const auto& record : reader) {
    if (record.IsControl()) {
      if (record.IsStart()) {
        wpi::log::StartRecordData start;
        if (!record.GetStartData(&start)) throw std::runtime_error("Malformed WPILOG start record");
        entries[start.entry] = {Copy(start.name), Copy(start.type), Copy(start.metadata)};
      } else if (record.IsSetMetadata()) {
        wpi::log::MetadataRecordData metadata;
        if (!record.GetSetMetadataData(&metadata)) throw std::runtime_error("Malformed WPILOG metadata record");
        auto it = entries.find(metadata.entry);
        if (it != entries.end()) it->second.metadata = Copy(metadata.metadata);
      } else if (record.IsFinish()) {
        int entry{};
        if (record.GetFinishEntry(&entry)) entries.erase(entry);
      }
      continue;
    }

    const auto entryIt = entries.find(record.GetEntry());
    if (entryIt == entries.end()) continue;
    const Entry& entry = entryIt->second;
    if (entry.name == "/Timestamp") {
      int64_t next{};
      if (entry.type != "int64" || !record.GetInteger(&next))
        throw std::runtime_error("AdvantageKit /Timestamp must be an int64 record");
      if (next < 0 || (haveTimestamp && next <= timestamp))
        throw std::runtime_error("AdvantageKit /Timestamp values must be nonnegative and strictly increasing");
      timestamp = next;
      haveTimestamp = true;
      frames.push_back(Frame{timestamp, keyIndex,
                             frames.empty() ? std::vector<std::shared_ptr<const Record>>{}
                                            : frames.back().records});
      continue;
    }
    if (!haveTimestamp || record.GetTimestamp() != timestamp) continue;
    if (entry.name.empty() || entry.name.front() != '/')
      throw std::runtime_error("AdvantageKit entry name lacks leading slash: " + entry.name);
    const std::string key = entry.name.substr(1);
    if (key.starts_with("ReplayOutputs")) continue;
    const auto [slotIt, inserted] = keyIndex->try_emplace(key, keyIndex->size());
    auto& records = frames.back().records;
    if (inserted) records.resize(slotIt->second + 1);
    records[slotIt->second] = std::make_shared<const Record>(Decode(record, entry));
  }
  if (frames.empty()) throw std::runtime_error("AdvantageKit log has no /Timestamp records");
  return frames;
}
}  // namespace base::akit
