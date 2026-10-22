#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <string_view>
namespace questfixture
{
// Golden bytes produced by QuestNav's Java 2026-2.2.0 generated Data message,
// Quickbuf 1.4 and WPILib 2026.2.1 (not an encoder shared with this adapter).
// Headset poses: (2,3,1.2,yaw90), then (3,3,1.2,yaw180).
const std::string_view first = "110000000000002940080720011a450a1b09000000000000004011000000000000084019333333333333f33"
                               "f12260a2409cd3b7f669ea0e63f11000000000000000019000000000000000021cd3b7f669ea0e63f";
const std::string_view second = "1185eb51b81e052940080820011a450a1b09000000000000084011000000000000084019333333333333f3"
                                "3f12260a2409000000000000000011000000000000000019000000000000000021000000000000f03f";
inline std::vector<uint8_t> Bytes(std::string_view hex)
{
    std::vector<uint8_t> out;
    for (size_t i = 0; i < hex.size(); i += 2)
        out.push_back(std::stoul(std::string(hex.substr(i, 2)), nullptr, 16));
    return out;
}
} // namespace questfixture
