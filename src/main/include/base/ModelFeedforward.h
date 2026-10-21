#pragma once
#include "base/Mechanism.h"
#include <frc/system/plant/DCMotor.h>

namespace base {
// Illustrative inverse-dynamics coefficients, not measured friction/calibration.
struct ModelFeedforward {
  double v{}, a{}, gravity{};
  // Preserve PID, static friction, profiles and limits in an existing configuration.
  Gains ApplyTo(Gains gains) const;
  // Output inertia includes all reflected load; unitsPerOutputRotation is 1 for
  // rotations or a drum circumference for metres. Gearing is motor/output speed.
  static ModelFeedforward Motor(frc::DCMotor motor,
      units::kilogram_square_meter_t outputInertia, double gearing,
      double unitsPerOutputRotation=1);
  // Gravity amplitude: arm uses Cosine with physical angle zero horizontal;
  // elevator uses Constant. Neither includes gearbox losses or static friction.
  static ModelFeedforward Arm(frc::DCMotor motor,
      units::kilogram_square_meter_t outputInertia, double gearing,
      units::kilogram_t mass, units::meter_t centerOfMass);
  static ModelFeedforward Elevator(frc::DCMotor motor,
      units::kilogram_square_meter_t outputInertia, double gearing,
      units::kilogram_t mass, units::meter_t drumRadius);
};
}
