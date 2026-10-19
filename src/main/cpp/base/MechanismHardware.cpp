#include "base/Mechanism.h"
#include <algorithm>
#include <numbers>
#include <stdexcept>
#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <units/voltage.h>

namespace base {
namespace {
class TalonMotorIO final : public MotorIO {
 public:
  explicit TalonMotorIO(MotorConfig config) : config_(std::move(config)) {
    for(size_t i=0;i<config_.ids.size();++i) {
      auto motor=std::make_unique<ctre::phoenix6::hardware::TalonFX>(config_.ids[i],config_.canBus);
      ctre::phoenix6::configs::TalonFXConfiguration settings;
      settings.CurrentLimits.SupplyCurrentLimit=units::ampere_t{static_cast<double>(config_.currentLimit)};
      settings.CurrentLimits.SupplyCurrentLimitEnable=true;
      settings.MotorOutput.NeutralMode=ctre::phoenix6::signals::NeutralModeValue::Brake;
      if(!motor->GetConfigurator().Apply(settings).IsOK()) throw std::runtime_error("TalonFX configuration failed");
      motors_.push_back(std::move(motor));
    }
  }
  MotorInputs Read() override {
    auto& motor=*motors_.front();
    auto position=motor.GetPosition(); auto velocity=motor.GetVelocity();
    auto voltage=motor.GetMotorVoltage(); auto supply=motor.GetSupplyVoltage(); auto current=motor.GetSupplyCurrent();
    position.Refresh(); velocity.Refresh(); voltage.Refresh(); supply.Refresh(); current.Refresh();
    double direction=config_.reversed.front() ? -1 : 1;
    return {direction*position.GetValueAsDouble()/config_.gearRatio,
            direction*velocity.GetValueAsDouble()/config_.gearRatio,
            direction*voltage.GetValueAsDouble(),supply.GetValueAsDouble(),current.GetValueAsDouble(),
            position.GetStatus().IsOK() && velocity.GetStatus().IsOK()};
  }
  void SetVoltage(double volts) override {
    for(size_t i=0;i<motors_.size();++i) {
      bool reversed=i==0 ? config_.reversed[0] : config_.reversed[0]!=config_.reversed[i];
      motors_[i]->SetVoltage(units::volt_t{reversed ? -volts : volts});
    }
  }
  void SetBrake(bool enabled) override {
    for(auto& motor:motors_) motor->SetNeutralMode(enabled ? ctre::phoenix6::signals::NeutralModeValue::Brake : ctre::phoenix6::signals::NeutralModeValue::Coast);
  }
 private:
  MotorConfig config_;
  std::vector<std::unique_ptr<ctre::phoenix6::hardware::TalonFX>> motors_;
};
class SparkMotorIO final : public MotorIO {
 public:
  explicit SparkMotorIO(MotorConfig config) : config_(std::move(config)) {
    if(!config_.canBus.empty() && config_.canBus!="rio") throw std::invalid_argument("SparkMax requires roboRIO CAN bus");
    for(int id:config_.ids) {
      auto motor=std::make_unique<rev::spark::SparkMax>(id,rev::spark::SparkLowLevel::MotorType::kBrushless);
      rev::spark::SparkMaxConfig settings;
      settings.SmartCurrentLimit(config_.currentLimit).SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
      if(motor->Configure(settings,rev::ResetMode::kResetSafeParameters,rev::PersistMode::kPersistParameters)!=rev::REVLibError::kOk)
        throw std::runtime_error("SparkMax configuration failed");
      motors_.push_back(std::move(motor));
    }
  }
  MotorInputs Read() override {
    auto& motor=*motors_.front(); auto encoder=motor.GetEncoder();
    double direction=config_.reversed.front() ? -1 : 1;
    double position=encoder.GetPosition(); double velocity=encoder.GetVelocity();
    return {direction*position/config_.gearRatio,direction*velocity/(60*config_.gearRatio),
            direction*motor.GetAppliedOutput()*motor.GetBusVoltage(),motor.GetBusVoltage(),motor.GetOutputCurrent(),
            motor.GetLastError()==rev::REVLibError::kOk};
  }
  void SetVoltage(double volts) override {
    for(size_t i=0;i<motors_.size();++i) {
      bool reversed=i==0 ? config_.reversed[0] : config_.reversed[0]!=config_.reversed[i];
      motors_[i]->SetVoltage(units::volt_t{reversed ? -volts : volts});
    }
  }
  void SetBrake(bool enabled) override {
    rev::spark::SparkMaxConfig settings;
    settings.SetIdleMode(enabled ? rev::spark::SparkBaseConfig::IdleMode::kBrake : rev::spark::SparkBaseConfig::IdleMode::kCoast);
    for(auto& motor:motors_) motor->Configure(settings,rev::ResetMode::kNoResetSafeParameters,rev::PersistMode::kNoPersistParameters);
  }
 private:
  MotorConfig config_;
  std::vector<std::unique_ptr<rev::spark::SparkMax>> motors_;
};
// Explicit-time electrical/mechanical model for a NEO (Spark) or Kraken (Talon).
class SimMotorIO final : public MotorIO {
 public:
  SimMotorIO(MotorConfig config,Vendor vendor,double inertia)
      : config_(std::move(config)),inertia_(inertia),
        resistance_(vendor==Vendor::SparkMax ? 12./105 : 12./366),
        kt_(vendor==Vendor::SparkMax ? 2.6/105 : 7.09/366),
        kv_(vendor==Vendor::SparkMax ? 5676*2*std::numbers::pi/60/12 : 6000*2*std::numbers::pi/60/12) {
    if(!std::isfinite(inertia_) || inertia_<=0) throw std::invalid_argument("Invalid output inertia");
  }
  void AdvanceSimulation(units::second_t elapsed) override {
    const double duration=elapsed.value();
    if(!std::isfinite(duration) || duration<=0 || duration>.1)
      throw std::invalid_argument("Simulation step must be in (0,100ms]");
    // Back-EMF damping grows with gearing squared. Bound alpha*dt to .25;
    // a fixed 1 ms Euler step diverges for the 105:1 hood's output inertia.
    const double alpha=(kt_*config_.ids.size()*config_.gearRatio*config_.gearRatio/
                        (resistance_*kv_)+.0001)/inertia_;
    const double requestedSteps=std::max(std::ceil(duration/.001),std::ceil(duration*alpha/.25));
    if(!std::isfinite(requestedSteps) || requestedSteps>100000)
      throw std::invalid_argument("Motor simulation stiffness exceeds supported substep bound");
    const int steps=static_cast<int>(requestedSteps);
    const double dt=duration/steps;
    for(int step=0;step<steps;++step) {
      double rotorSpeed=velocity_*config_.gearRatio;
      double current=std::clamp((voltage_-rotorSpeed/kv_)/resistance_,
                               -static_cast<double>(config_.currentLimit),static_cast<double>(config_.currentLimit));
      if(!brake_ && std::abs(voltage_)<1e-9) current=0;
      current_=current;
      velocity_+=(kt_*current*config_.ids.size()*config_.gearRatio-.0001*velocity_)/inertia_*dt;
      position_+=velocity_*dt;
    }
  }
  MotorInputs Read() override {
    return {position_/(2*std::numbers::pi),velocity_/(2*std::numbers::pi),voltage_,12,std::abs(current_)*config_.ids.size(),true};
  }
  void SetVoltage(double volts) override { voltage_=std::clamp(volts,-12.,12.); }
  void SetBrake(bool enabled) override { brake_=enabled; }
 private:
  MotorConfig config_;
  double inertia_,resistance_,kt_,kv_,position_=0,velocity_=0,voltage_=0,current_=0;
  bool brake_=true;
};
}
std::unique_ptr<MotorIO> MakeMotorIO(Mode mode,Vendor vendor,const MotorConfig& config,double outputInertia) {
  config.Validate();
  if(mode==Mode::Replay) return std::make_unique<InjectedMotorIO>();
  if(mode==Mode::Sim) return std::make_unique<SimMotorIO>(config,vendor,outputInertia);
  if(vendor==Vendor::SparkMax) return std::make_unique<SparkMotorIO>(config);
  return std::make_unique<TalonMotorIO>(config);
}
}
