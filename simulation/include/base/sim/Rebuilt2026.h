#pragma once
#include "base/sim/SimulationWorld.h"
namespace base::sim {
// Rough collision geometry only; source dimensions/approximations are documented
// in docs/2026-field-simulation.md. No trenches or game rules.
inline constexpr double kRebuiltLength = 651.22 * .0254;
inline constexpr double kRebuiltWidth = 317.69 * .0254;
std::vector<BodySpec> Rebuilt2026Colliders(Backend);
}
