#pragma once
#include "base/Mechanism.h"
#include <rev/config/FeedForwardConfig.h>

namespace base::detail {
// REV kG and kCos select mutually exclusive firmware modes. Keep that choice in
// one adapter helper so reconfiguration cannot erase the requested gravity gain.
void ConfigureSparkGravity(rev::spark::FeedForwardConfig&, double volts,
                           double physicalScale, Gravity gravity, bool joint);
}
