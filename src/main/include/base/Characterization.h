#pragma once
#include "base/Mechanism.h"
#include <frc2/command/sysid/SysIdRoutine.h>

namespace base
{
// Position/velocity are rotations and rotations/s unless linear=true (metres and metres/s).
// Each returned command retains its routine and declares the mechanism requirement.
frc2::CommandPtr SysIdQuasistatic(Mechanism &mechanism, frc2::sysid::Direction direction, bool linear = false);
frc2::CommandPtr SysIdDynamic(Mechanism &mechanism, frc2::sysid::Direction direction, bool linear = false);
} // namespace base
