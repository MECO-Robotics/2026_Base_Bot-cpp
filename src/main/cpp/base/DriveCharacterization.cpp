#include "base/Drive.h"
#include "base/Replay.h"
#include <frc2/command/sysid/SysIdRoutine.h>
#include <ctre/phoenix6/swerve/SwerveRequest.hpp>
#include <cmath>
#include <stdexcept>

namespace base {
frc2::CommandPtr Drive::Characterize(bool dynamic, frc2::sysid::Direction direction) {
  auto apply=[this](units::volt_t volts) {
    requestedSpeeds_={};
    replay::RecordOutput("Drive/CharacterizationVoltage",volts.value());
    if(native_) native_->SetControl(ctre::phoenix6::swerve::requests::SysIdSwerveTranslation{}.WithVolts(volts));
  };
  auto routine=std::make_shared<frc2::sysid::SysIdRoutine>(
      frc2::sysid::Config{std::nullopt,std::nullopt,std::nullopt,nullptr},
      frc2::sysid::Mechanism{apply,[this,apply](frc::sysid::SysIdRoutineLog* log) {
        try {
        // Read measured wheel distance/speed (including native coupling correction),
        // never requested voltage or rotor-turn values labeled as SI.
        const auto inputs=replay::ProcessInputs("Drive/Characterization",[this] {
          const auto state=native_->GetState();
          wpi::json modules=wpi::json::array();
          for(size_t i=0;i<4;++i) modules.push_back({
              native_->GetModule(i).GetDriveMotor().GetMotorVoltage().GetValue().value(),
              state.ModulePositions[i].distance.value(),state.ModuleStates[i].speed.value()});
          return modules;
        });
        if(!inputs.is_array()||inputs.size()!=4) throw std::runtime_error("Invalid Drive/Characterization module inputs");
        for(size_t i=0;i<4;++i) {
          if(!inputs[i].is_array()||inputs[i].size()!=3) throw std::runtime_error("Invalid Drive/Characterization sample");
          for(const auto& value:inputs[i]) if(!value.is_number()||!std::isfinite(value.get<double>())) throw std::runtime_error("Nonfinite Drive/Characterization sample");
          log->Motor("drive-"+std::to_string(i)).voltage(units::volt_t{inputs[i][0].get<double>()})
              .position(units::meter_t{inputs[i][1].get<double>()})
              .velocity(units::meters_per_second_t{inputs[i][2].get<double>()});
        }
        } catch(...) {
          // A failed feedback/log read must not leave the test voltage latched.
          apply(units::volt_t{0});
          throw;
        }
      },this});
  auto command=dynamic?routine->Dynamic(direction):routine->Quasistatic(direction);
  return std::move(command).FinallyDo([this,routine,apply](bool) {
    apply(units::volt_t{0});
    Stop();
  });
}
}
