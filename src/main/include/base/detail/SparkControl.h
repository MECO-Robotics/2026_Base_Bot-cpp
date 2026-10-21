#pragma once
#include "base/Mechanism.h"
#include <rev/config/FeedForwardConfig.h>
#include <rev/SparkMax.h>

namespace base::detail {
// Runtime reads never clear reset state: full robot restart owns recovery.
bool SparkDeviceConfigured(rev::spark::SparkMax&, bool& resetLatched);
// REV kG and kCos select mutually exclusive firmware modes. Keep that choice in
// one adapter helper so reconfiguration cannot erase the requested gravity gain.
void ConfigureSparkGravity(rev::spark::FeedForwardConfig&, double volts,
                           double physicalScale, Gravity gravity, bool joint);
}
