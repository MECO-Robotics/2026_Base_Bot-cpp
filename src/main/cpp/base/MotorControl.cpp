#include "base/Mechanism.h"
#include <algorithm>
#include <numbers>
#include <stdexcept>

namespace base {
bool MotorIO::ConfigureControl(const MotorControlConfig& config) {
  config.gains.Validate(config.positionControlled);
  if (!std::isfinite(config.unitsPerOutputRotation) || config.unitsPerOutputRotation<=0)
    throw std::invalid_argument("Invalid motor control units");
  const bool native=ConfigureNative(config.gains,config.unitsPerOutputRotation,
                                    config.gravity,config.positionControlled);
  if (native && config.externalFeedback)
    throw std::invalid_argument("Onboard control requires adapter-owned coherent feedback");
  controlConfig_=config;
  controlNative_=native;
  ResetControl();
  return native;
}
void MotorIO::ResetControl() { integral_=previousError_=0; }
MotorControlResult MotorIO::Control(const MotorControlRequest& request, const MotorInputs& inputs,
                                  double position, units::second_t timestep) {
  const double dt=timestep.value();
  const auto& setpoint=request.native;
  if (!std::isfinite(dt) || dt<=0 || dt>.1 || !inputs.connected ||
      !std::isfinite(position) || !std::isfinite(inputs.velocity) ||
      !std::isfinite(setpoint.target) || !std::isfinite(setpoint.velocity) ||
      !std::isfinite(setpoint.acceleration) || !std::isfinite(request.profiledPosition)) {
    ResetControl(); SetVoltage(0); return {};
  }
  if (controlNative_) return {SetNative(setpoint),std::nullopt};
  const auto& gains=controlConfig_.gains;
  const bool velocity=setpoint.kind==NativeControlRequest::Kind::Velocity;
  const double target=setpoint.kind==NativeControlRequest::Kind::MotionMagic ? request.profiledPosition : setpoint.target;
  const double error=target-(velocity ? inputs.velocity*controlConfig_.unitsPerOutputRotation : position);
  // This IO emulator deliberately retains the prior explicit-dt integral and
  // derivative contract. WPILib PIDController fixes its period at construction;
  // recreating it on multirate ticks would discard controller history.
  integral_=std::clamp(integral_+error*dt,-12.,12.);
  double volts=gains.p*error+gains.i*integral_+gains.d*(error-previousError_)/dt;
  previousError_=error;
  volts+=gains.v*setpoint.velocity+gains.a*setpoint.acceleration;
  if (std::abs(setpoint.velocity)>1e-9) volts+=std::copysign(gains.s,setpoint.velocity);
  if (controlConfig_.positionControlled) {
    const double angle=position*2*std::numbers::pi;
    volts+=gains.gravity*(controlConfig_.gravity==Gravity::Constant ? 1 :
        controlConfig_.gravity==Gravity::Cosine ? std::cos(angle) : std::sin(angle));
  }
  if (!std::isfinite(volts)) { ResetControl(); SetVoltage(0); return {}; }
  if (controlConfig_.positionControlled && ((position<=gains.minimum && volts<0) ||
                                           (position>=gains.maximum && volts>0))) volts=0;
  volts=std::clamp(volts,-12.,12.);
  SetVoltage(volts);
  return {true,volts};
}
}
