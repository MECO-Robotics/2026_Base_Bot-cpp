#include "base/Characterization.h"
#include <memory>

namespace base
{
  namespace
  {
    frc2::CommandPtr MakeRoutine(Mechanism &mechanism, frc2::sysid::Direction direction, bool linear, bool dynamic)
    {
      auto routine = std::make_shared<frc2::sysid::SysIdRoutine>(
          frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt, nullptr},
          frc2::sysid::Mechanism{
              [&mechanism](units::volt_t voltage)
              { mechanism.SetVoltage(voltage.value()); },
              [&mechanism, linear](frc::sysid::SysIdRoutineLog *log)
              {
                auto motor = log->Motor(mechanism.GetName());
                motor.voltage(units::volt_t{mechanism.Inputs().voltage})
                    .current(units::ampere_t{mechanism.Inputs().current});
                if (linear)
                  motor.position(units::meter_t{mechanism.Position()})
                      .velocity(units::meters_per_second_t{mechanism.Velocity()});
                else
                  motor.position(units::turn_t{mechanism.Position()})
                      .velocity(units::turns_per_second_t{mechanism.Velocity()});
              },
              &mechanism});
      auto command = dynamic ? routine->Dynamic(direction) : routine->Quasistatic(direction);
      return std::move(command).FinallyDo([routine, &mechanism](bool)
                                          { mechanism.Stop(); });
    }
  }
  frc2::CommandPtr SysIdQuasistatic(Mechanism &mechanism, frc2::sysid::Direction direction, bool linear)
  {
    return MakeRoutine(mechanism, direction, linear, false);
  }
  frc2::CommandPtr SysIdDynamic(Mechanism &mechanism, frc2::sysid::Direction direction, bool linear)
  {
    return MakeRoutine(mechanism, direction, linear, true);
  }
}
