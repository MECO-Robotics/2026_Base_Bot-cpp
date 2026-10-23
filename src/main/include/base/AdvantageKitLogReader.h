#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace wpi::log
{
class DataLogReader;
}

namespace base::akit
{

// AdvantageKit's primitive schemas are decoded to native C++ values. Plain
// raw and custom schemas (such as WPILib structs) remain byte arrays; their
// schema name is retained in Record::type for a higher layer to interpret.
using Value = std::variant<bool, int64_t, float, double, std::string, std::vector<uint8_t>, std::vector<bool>,
                           std::vector<int64_t>, std::vector<float>, std::vector<double>, std::vector<std::string>>;

struct Record
{
    Value value;
    std::string type;
    std::string metadata;
};

using KeyIndex = std::map<std::string, size_t, std::less<>>;

struct Frame
{
    int64_t timestampUs{};
    // Keys omit the leading slash. Fields carry forward; ReplayOutputs entries are omitted to match
    // AdvantageKit's reader. Cycles share a key index and immutable values; late keys lack earlier slots.
    std::shared_ptr<const KeyIndex> keyIndex;
    std::vector<std::shared_ptr<const Record>> records;

    const Record *Find(std::string_view key) const
    {
        if (!keyIndex)
            return nullptr;
        const auto it = keyIndex->find(key);
        if (it == keyIndex->end() || it->second >= records.size())
            return nullptr;
        return records[it->second].get();
    }
};

// Read an AdvantageKit 26.0.2 WPILOG. Throws std::runtime_error for invalid
// logs, non-AdvantageKit headers, malformed primitive records, or logs without
// any /Timestamp records.
std::vector<Frame> ReadLog(const std::string &path);
// Decode an already-open reader without loading its source file again.
std::vector<Frame> ReadLog(const wpi::log::DataLogReader &reader);

} // namespace base::akit
