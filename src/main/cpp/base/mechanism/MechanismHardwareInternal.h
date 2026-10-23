#pragma once

#include "base/Mechanism.h"

#include <memory>

namespace base::detail
{

std::unique_ptr<MotorIO> MakeTalonMotorIO(const MotorConfig &config);
std::unique_ptr<MotorIO> MakeSparkMotorIO(const MotorConfig &config);
std::unique_ptr<MotorIO> MakeSimMotorIO(const MotorConfig &config, Vendor vendor,
                                        const MotorSimulationConfig &simulation);

} // namespace base::detail
