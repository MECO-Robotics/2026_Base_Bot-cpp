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
    double position=encoder.GetPosition();
    bool connected=motor.GetLastError()==rev::REVLibError::kOk;
    double velocity=encoder.GetVelocity();
    connected=(motor.GetLastError()==rev::REVLibError::kOk) && connected;
    double appliedOutput=motor.GetAppliedOutput();
    connected=(motor.GetLastError()==rev::REVLibError::kOk) && connected;
    double supplyVoltage=motor.GetBusVoltage();
    connected=(motor.GetLastError()==rev::REVLibError::kOk) && connected;
    double current=motor.GetOutputCurrent();
    connected=(motor.GetLastError()==rev::REVLibError::kOk) && connected;
    return {direction*position/config_.gearRatio,direction*velocity/(60*config_.gearRatio),
            direction*appliedOutput*supplyVoltage,supplyVoltage,current,connected};
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
// Deterministic 20 ms electrical/mechanical model for a NEO (Spark) or Kraken (Talon).
class SimMotorIO final : public MotorIO {
 public:
  SimMotorIO(MotorConfig config,Vendor vendor,double inertia)
      : config_(std::move(config)),inertia_(inertia),
        resistance_(vendor==Vendor::SparkMax ? 12./105 : 12./366),
        kt_(vendor==Vendor::SparkMax ? 2.6/105 : 7.09/366),
        kv_(vendor==Vendor::SparkMax ? 5676*2*std::numbers::pi/60/12 : 6000*2*std::numbers::pi/60/12) {
    if(!std::isfinite(inertia_) || inertia_<=0) throw std::invalid_argument("Invalid output inertia");
  }
  MotorInputs Read() override {
    // Exact integration of the scalar motor model avoids Euler instability at
    // high gearing. At constant voltage the speed is monotonic, so a current-
    // limited interval can cross into the linear region at most once.
    constexpr double dt=.02;
    const double drag=.0001/inertia_;
    const double torque=kt_*config_.ids.size()*config_.gearRatio/inertia_;
    const double emf=config_.gearRatio/kv_;
    const double limit=config_.currentLimit;
    auto advance=[&](double duration,double decay,double equilibrium) {
      const double integral=-std::expm1(-decay*duration)/decay;
      position_+=equilibrium*duration+(velocity_-equilibrium)*integral;
      velocity_+=(equilibrium-velocity_)*(-std::expm1(-decay*duration));
    };
    if(!brake_ && std::abs(voltage_)<1e-9) {
      advance(dt,drag,0);
      current_=0;
    } else {
      double remaining=dt;
      const double initialCurrent=(voltage_-emf*velocity_)/resistance_;
      if(std::abs(initialCurrent)>limit) {
        const double current=std::copysign(limit,initialCurrent);
        const double equilibrium=torque*current/drag;
        const double boundary=(voltage_-resistance_*current)/emf;
        const double fraction=(boundary-velocity_)/(velocity_-equilibrium);
        const double crossing=fraction>-1 ? -std::log1p(fraction)/drag : dt;
        const double duration=std::isfinite(crossing) && crossing>0 ? std::min(dt,crossing) : dt;
        advance(duration,drag,equilibrium);
        remaining-=duration;
      }
      if(remaining>0) {
        const double decay=torque*emf/resistance_+drag;
        advance(remaining,decay,torque*voltage_/resistance_/decay);
      }
      current_=std::clamp((voltage_-emf*velocity_)/resistance_,-limit,limit);
    }
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
