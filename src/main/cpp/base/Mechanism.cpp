#include "base/Mechanism.h"
#include "base/MotorInputLog.h"
#include <array>
#include <algorithm>
#include <numbers>
#include <set>
#include <stdexcept>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <networktables/NetworkTableInstance.h>

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
void Mechanism::EnableInputLogging(wpi::log::DataLog& log) {
  inputLog_=std::make_unique<wpi::log::DoubleArrayLogEntry>(log,GetName()+"/MotorInputs",kMotorInputSchema);
}
void Mechanism::EnableLiveTuning(bool enabled) {
  liveTuning_=enabled;
  if(!enabled) return;
  const auto table=nt::NetworkTableInstance::GetDefault().GetTable(GetName()+"/Gains");
  table->GetEntry("Fields").SetString("kP,kI,kD,kS,kV,kA,kG,kMaxVelo,kMaxAccel,kTolerance");
  table->GetEntry("Values").SetDoubleArray(std::array<double,10>{gains_.p,gains_.i,gains_.d,gains_.s,gains_.v,
      gains_.a,gains_.gravity,gains_.maxVelocity,gains_.maxAcceleration,gains_.tolerance});
  table->GetEntry("Status").SetString("Disabled-only tuning ready");
}
void Mechanism::UpdateLiveTuning() {
  if(!liveTuning_) return;
  const auto table=nt::NetworkTableInstance::GetDefault().GetTable(GetName()+"/Gains");
  if(!frc::DriverStation::IsDisabled()) {
    table->GetEntry("Status").SetString("Locked while enabled");
    return;
  }
  const auto values=table->GetEntry("Values").GetDoubleArray({});
  if(values.size()!=10) { table->GetEntry("Status").SetString("Rejected: expected 10 gains"); return; }
  auto candidate=gains_;
  candidate.p=values[0]; candidate.i=values[1]; candidate.d=values[2];
  candidate.s=values[3]; candidate.v=values[4]; candidate.a=values[5]; candidate.gravity=values[6];
  candidate.maxVelocity=values[7]; candidate.maxAcceleration=values[8]; candidate.tolerance=values[9];
  try {
    candidate.Validate(joint_);
    const std::array<double,10> current{gains_.p,gains_.i,gains_.d,gains_.s,gains_.v,
        gains_.a,gains_.gravity,gains_.maxVelocity,gains_.maxAcceleration,gains_.tolerance};
    // An unchanged dashboard must not reset SetPositionDynamic's profile constraints.
    if(!std::equal(values.begin(),values.end(),current.begin())) SetGains(candidate);
    table->GetEntry("Status").SetString("Accepted while disabled");
  } catch(const std::invalid_argument&) {
    table->GetEntry("Status").SetString("Rejected: invalid gains; previous gains retained");
  }
}
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
  zeroOffset_=physicalPosition_; target_=physicalPosition_; profiledPosition_=physicalPosition_;
  profiledVelocity_=0; integral_=0; SetBrake(true); Stop();
  if(joint_) SetPosition(0);
}
bool Mechanism::AtSetpoint() const {
  return std::abs((control_==Control::Position ? physicalPosition_ : Velocity())-target_)<=gains_.tolerance;
}
void Mechanism::SimulationPeriodic() { AdvanceSimulation(units::second_t{.02}); }
void Mechanism::AdvanceSimulation(units::second_t dt) { io_->AdvanceSimulation(dt); }
void Mechanism::Periodic() {
  constexpr double dt=.02;
  UpdateLiveTuning();
  inputs_=io_->Read();
  if(inputLog_) inputLog_->Append(EncodeMotorInputs(inputs_));
  physicalPosition_=externalEncoder_ ? calibration_.Apply(externalEncoder_()) : inputs_.position*units_;
  if(!std::isfinite(physicalPosition_) || !std::isfinite(Velocity())) { Stop(); return; }
  auto publish=[this]{
  auto prefix=GetName()+"/";
  frc::SmartDashboard::PutNumber(prefix+"Position",Position());
  frc::SmartDashboard::PutNumber(prefix+"Velocity",Velocity());
  frc::SmartDashboard::PutNumber(prefix+"Voltage",inputs_.voltage);
  frc::SmartDashboard::PutNumber(prefix+"Target",joint_ ? DesiredPosition() : target_);
  frc::SmartDashboard::PutBoolean(prefix+"Connected",inputs_.connected);
  };
  if(frc::DriverStation::IsDisabled()) {
    integral_=0; previousError_=0;
    profiledPosition_=physicalPosition_; profiledVelocity_=Velocity();
    io_->SetVoltage(0);
    publish();
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
      if(step*distance>=0 && std::abs(step)>=std::abs(distance)) { profiledPosition_=target_; profiledVelocity_=0; }
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
  if(joint_ && ((physicalPosition_<=gains_.minimum && volts<0) || (physicalPosition_>=gains_.maximum && volts>0))) volts=0;
  io_->SetVoltage(std::clamp(volts,-12.,12.));
  publish();
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
