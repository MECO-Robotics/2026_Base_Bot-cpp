#include "base/ModelFeedforward.h"
#include <frc/system/plant/LinearSystemId.h>
#include <numbers>
#include <stdexcept>

namespace base
{
namespace
{
void Positive(double value)
{
    if (!std::isfinite(value) || value <= 0)
        throw std::invalid_argument("Model parameters must be finite and positive");
}
void MotorValid(const frc::DCMotor &motor)
{
    Positive(motor.R.value());
    Positive(motor.Kt.value());
    Positive(motor.Kv.value());
}
double GravityVoltage(const frc::DCMotor &motor, double gearing, double torque)
{
    return motor.Voltage(units::newton_meter_t{torque / gearing}, units::radians_per_second_t{0}).value();
}
} // namespace
Gains ModelFeedforward::ApplyTo(Gains gains) const
{
    for (double value : {v, a, gravity})
        if (!std::isfinite(value))
            throw std::invalid_argument("Nonfinite model coefficients");
    gains.v = v;
    gains.a = a;
    gains.gravity = gravity;
    return gains;
}
ModelFeedforward ModelFeedforward::Motor(frc::DCMotor motor, units::kilogram_square_meter_t inertia, double gearing,
                                         double unitsPerRotation)
{
    MotorValid(motor);
    Positive(inertia.value());
    Positive(gearing);
    Positive(unitsPerRotation);
    const auto plant = frc::LinearSystemId::DCMotorSystem(motor, inertia, gearing);
    const double scale = 2 * std::numbers::pi / unitsPerRotation;
    ModelFeedforward result{-plant.A(1, 1) / plant.B(1, 0) * scale, 1 / plant.B(1, 0) * scale, 0};
    for (double value : {result.v, result.a})
        Positive(value);
    return result;
}
ModelFeedforward ModelFeedforward::Arm(frc::DCMotor motor, units::kilogram_square_meter_t inertia, double gearing,
                                       units::kilogram_t mass, units::meter_t com)
{
    Positive(mass.value());
    Positive(com.value());
    auto result = Motor(motor, inertia, gearing);
    result.gravity = GravityVoltage(motor, gearing, mass.value() * 9.80665 * com.value());
    Positive(result.gravity);
    return result;
}
ModelFeedforward ModelFeedforward::Elevator(frc::DCMotor motor, units::kilogram_square_meter_t inertia, double gearing,
                                            units::kilogram_t mass, units::meter_t radius)
{
    Positive(mass.value());
    Positive(radius.value());
    auto result = Motor(motor, inertia, gearing, 2 * std::numbers::pi * radius.value());
    result.gravity = GravityVoltage(motor, gearing, mass.value() * 9.80665 * radius.value());
    Positive(result.gravity);
    return result;
}
} // namespace base
