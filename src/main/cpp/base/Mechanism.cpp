#include "base/Mechanism.h"
#include <algorithm>
#include <numbers>
#include <set>
#include <stdexcept>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

namespace base {
namespace {
void Finite(double value) { if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite mechanism value"); }
}
void MotorConfig::Validate() const {
  if (ids.empty() || ids.size()!=reversed.size() || gearRatio<=0 || currentLimit<=0)
    throw std::invalid_argument("Invalid motor configuration");
  Finite(gearRatio);
  std::set<int> seen;
  for (int id:ids) if(id<0 || id>62 || !seen.insert(id).second)
    throw std::invalid_argument("Invalid or duplicate CAN ID");
}
void Gains::Validate(bool position) const {
  for(double value:{p,i,d,s,v,a,gravity,maxVelocity,maxAcceleration,tolerance,minimum,maximum,defaultPosition}) Finite(value);
  if(maxVelocity<=0 || maxAcceleration<=0 || tolerance<=0 ||
      (position && (minimum>=maximum || defaultPosition<minimum || defaultPosition>maximum)))
    throw std::invalid_argument("Invalid mechanism gains or limits");
}
void EncoderCalibration::Validate() const {
  Finite(unitsPerRotation); Finite(offsetRotations);
  if(unitsPerRotation<=0) throw std::invalid_argument("Invalid encoder scale");
}
double EncoderCalibration::Apply(double raw) const {
  Validate(); Finite(raw); return ((reversed ? -raw : raw)+offsetRotations)*unitsPerRotation;
}
Mechanism::Mechanism(std::string name, std::unique_ptr<MotorIO> io, Gains gains,
                     bool positionControlled, double unitsPerOutputRotation, Gravity gravity,
                     std::function<double()> externalEncoder, EncoderCalibration calibration)
    : io_(std::move(io)), gains_(gains), joint_(positionControlled), units_(unitsPerOutputRotation),
      maxVelocity_(gains.maxVelocity), maxAcceleration_(gains.maxAcceleration), gravity_(gravity),
      externalEncoder_(std::move(externalEncoder)), calibration_(calibration) {
  if(!io_ || !std::isfinite(units_) || units_<=0) throw std::invalid_argument("Invalid mechanism IO or units");
  gains_.Validate(joint_); calibration_.Validate(); SetName(name);
}
void Mechanism::SetGains(Gains gains) { gains.Validate(joint_); gains_=gains; maxVelocity_=gains.maxVelocity; maxAcceleration_=gains.maxAcceleration; }
void Mechanism::SetVelocity(double velocity) {
  Finite(velocity);
  if(control_!=Control::Velocity) profiledVelocity_=Velocity();
  target_=joint_ ? std::clamp(velocity,-gains_.maxVelocity,gains_.maxVelocity) : velocity; control_=Control::Velocity;
}
void Mechanism::SetPosition(double position) { SetPositionDynamic(position,gains_.maxVelocity,gains_.maxAcceleration); }
void Mechanism::SetPositionDynamic(double position,double maxVelocity,double maxAcceleration) {
  Finite(position); Finite(maxVelocity); Finite(maxAcceleration);
  if(!joint_ || maxVelocity<=0 || maxAcceleration<=0) throw std::invalid_argument("Invalid joint request");
  if(control_!=Control::Position) { profiledPosition_=physicalPosition_; profiledVelocity_=Velocity(); }
  target_=std::clamp(position+zeroOffset_,gains_.minimum,gains_.maximum);
  maxVelocity_=maxVelocity; maxAcceleration_=maxAcceleration; control_=Control::Position;
}
void Mechanism::SetVoltage(double volts) { Finite(volts); voltage_=std::clamp(volts,-12.,12.); control_=Control::Voltage; }
void Mechanism::Stop() { control_=Control::Stopped; integral_=0; previousError_=0; io_->SetVoltage(0); }
void Mechanism::SetBrake(bool enabled) { io_->SetBrake(enabled); }
void Mechanism::ResetPosition() {
  if(!inputs_.connected || !std::isfinite(physicalPosition_) || !std::isfinite(Velocity())) { Stop(); return; }
  zeroOffset_=physicalPosition_; target_=physicalPosition_; profiledPosition_=physicalPosition_;
  profiledVelocity_=0; integral_=0; SetBrake(true); Stop();
  if(joint_) SetPosition(0);
}
bool Mechanism::AtSetpoint() const {
  if(!inputs_.connected || !std::isfinite(physicalPosition_) || !std::isfinite(Velocity()) ||
      (control_!=Control::Position && control_!=Control::Velocity)) return false;
  return std::abs((control_==Control::Position ? physicalPosition_ : Velocity())-target_)<=gains_.tolerance;
}
void Mechanism::Periodic() {
  constexpr double dt=.02;
  inputs_=io_->Read();
  frc::SmartDashboard::PutBoolean(GetName()+"/Connected",inputs_.connected);
  if(!inputs_.connected) { Stop(); return; }
  if(externalEncoder_) {
    const double raw=externalEncoder_();
    if(!std::isfinite(raw)) { physicalPosition_=raw; Stop(); return; }
    physicalPosition_=calibration_.Apply(raw);
  } else physicalPosition_=inputs_.position*units_;
  if(!std::isfinite(physicalPosition_) || !std::isfinite(Velocity())) { Stop(); return; }
  if(frc::DriverStation::IsDisabled()) {
    integral_=0; previousError_=0;
    profiledPosition_=physicalPosition_; profiledVelocity_=Velocity();
    io_->SetVoltage(0);
    return;
  }
  double volts=0;
  if(control_==Control::Voltage) volts=voltage_;
  else if(control_==Control::Velocity || control_==Control::Position) {
    double previousVelocity=profiledVelocity_;
    if(control_==Control::Position) {
      double distance=target_-profiledPosition_;
      double desired=std::copysign(std::min(maxVelocity_,std::sqrt(2*maxAcceleration_*std::abs(distance))),distance);
      profiledVelocity_+=std::clamp(desired-profiledVelocity_,-maxAcceleration_*dt,maxAcceleration_*dt);
      double step=profiledVelocity_*dt;
      // A close goal may require overshoot: snapping a moving profile to rest
      // must not violate the acceleration bound or spike acceleration feedforward.
      if(step*distance>=0 && std::abs(step)>=std::abs(distance) &&
          std::abs(previousVelocity)<=maxAcceleration_*dt) {
        profiledPosition_=target_; profiledVelocity_=0;
      }
      else profiledPosition_+=step;
    } else profiledVelocity_+=std::clamp(target_-profiledVelocity_,-maxAcceleration_*dt,maxAcceleration_*dt);
    double error=control_==Control::Position ? profiledPosition_-physicalPosition_ : profiledVelocity_-Velocity();
    integral_=std::clamp(integral_+error*dt,-12.,12.);
    volts=gains_.p*error+gains_.i*integral_+gains_.d*(error-previousError_)/dt;
    previousError_=error;
    volts+=gains_.v*profiledVelocity_+gains_.a*(profiledVelocity_-previousVelocity)/dt;
    if(std::abs(profiledVelocity_)>1e-9) volts+=std::copysign(gains_.s,profiledVelocity_);
    if(joint_) {
      double angle=physicalPosition_*2*std::numbers::pi;
      volts+=gains_.gravity*(gravity_==Gravity::Constant ? 1 : gravity_==Gravity::Cosine ? std::cos(angle) : std::sin(angle));
    }
  }
  if(!std::isfinite(volts)) { Stop(); return; }
  if(joint_ && ((physicalPosition_<=gains_.minimum && volts<0) || (physicalPosition_>=gains_.maximum && volts>0))) volts=0;
  io_->SetVoltage(std::clamp(volts,-12.,12.));
  auto prefix=GetName()+"/";
  frc::SmartDashboard::PutNumber(prefix+"Position",Position());
  frc::SmartDashboard::PutNumber(prefix+"Velocity",Velocity());
  frc::SmartDashboard::PutNumber(prefix+"Target",joint_ ? DesiredPosition() : target_);
}
frc2::CommandPtr Mechanism::VelocityCommand(std::function<double()> supplier) {
  return frc2::cmd::RunEnd([this,supplier]{SetVelocity(supplier());},[this]{Stop();},{this});
}
frc2::CommandPtr Mechanism::PositionCommand(std::function<double()> supplier) {
  return frc2::cmd::Run([this,supplier]{SetPosition(supplier());},{this}).Until([this]{return AtSetpoint();});
}
frc2::CommandPtr Mechanism::VoltageCommand(std::function<double()> supplier) {
  return frc2::cmd::RunEnd([this,supplier]{SetVoltage(supplier());},[this]{Stop();},{this});
}
}
