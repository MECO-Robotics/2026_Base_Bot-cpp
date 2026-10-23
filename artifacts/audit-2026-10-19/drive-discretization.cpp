#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/geometry/Pose2d.h>
#include <iostream>
#include <cmath>
using namespace units::literals;
int main()
{
    const frc::ChassisSpeeds continuous{2_mps, 0_mps, 3_rad_per_s};
    const auto discrete = frc::ChassisSpeeds::Discretize(continuous, 20_ms);
    const auto integrate = [](const frc::ChassisSpeeds &s) {
        return frc::Pose2d{}.Exp({s.vx * 20_ms, s.vy * 20_ms, s.omega * 20_ms});
    };
    const auto before = integrate(continuous), after = integrate(discrete);
    std::cout.precision(16);
    std::cout << "continuous displacement=" << before.X().value() << "," << before.Y().value() << " m\n"
              << "discrete speeds=" << discrete.vx.value() << "," << discrete.vy.value() << " m/s\n"
              << "corrected displacement=" << after.X().value() << "," << after.Y().value() << " m\n";
    return std::abs(after.X().value() - .04) < 1e-12 && std::abs(after.Y().value()) < 1e-12 &&
                   std::abs(before.Y().value()) > .001
               ? 0
               : 1;
}