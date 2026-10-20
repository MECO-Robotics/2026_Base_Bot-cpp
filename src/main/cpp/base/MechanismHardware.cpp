#include "base/Mechanism.h"
#include <algorithm>
#include <numbers>
#include <stdexcept>
#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <units/voltage.h>
#ifndef __FRC_ROBORIO__
#include "base/sim/SimulationWorld.h"
#endif

namespace base
{
  namespace
  {
    class TalonMotorIO final : public MotorIO
    {
    public:
      explicit TalonMotorIO(MotorConfig config) : config_(std::move(config))
      {
        for (size_t i = 0; i < config_.ids.size(); ++i)
        {
          auto motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(config_.ids[i], config_.canBus);
          ctre::phoenix6::configs::TalonFXConfiguration settings;
          settings.CurrentLimits.SupplyCurrentLimit = units::ampere_t{static_cast<double>(config_.currentLimit)};
          settings.CurrentLimits.SupplyCurrentLimitEnable = true;
          settings.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
          if (!motor->GetConfigurator().Apply(settings).IsOK())
            throw std::runtime_error("TalonFX configuration failed");
          motors_.push_back(std::move(motor));
        }
      }
      MotorInputs Read() override
      {
        auto &motor = *motors_.front();
        const auto &position = motor.GetPosition();
        const auto &velocity = motor.GetVelocity();
        const auto &voltage = motor.GetMotorVoltage();
        const auto &supply = motor.GetSupplyVoltage();
        const auto &current = motor.GetSupplyCurrent();
        // Phoenix getters refresh their motor-owned cached signals by default.
        double direction = config_.reversed.front() ? -1 : 1;
        return {direction * position.GetValueAsDouble() / config_.gearRatio,
                direction * velocity.GetValueAsDouble() / config_.gearRatio,
                direction * voltage.GetValueAsDouble(), supply.GetValueAsDouble(), current.GetValueAsDouble(),
                position.GetStatus().IsOK() && velocity.GetStatus().IsOK()};
      }
      void SetVoltage(double volts) override
      {
        for (size_t i = 0; i < motors_.size(); ++i)
        {
          bool reversed = i == 0 ? config_.reversed[0] : config_.reversed[0] != config_.reversed[i];
          motors_[i]->SetVoltage(units::volt_t{reversed ? -volts : volts});
        }
      }
      void SetBrake(bool enabled) override
      {
        for (auto &motor : motors_)
          motor->SetNeutralMode(enabled ? ctre::phoenix6::signals::NeutralModeValue::Brake : ctre::phoenix6::signals::NeutralModeValue::Coast);
      }

    private:
      MotorConfig config_;
      std::vector<std::unique_ptr<ctre::phoenix6::hardware::TalonFX>> motors_;
    };
    class SparkMotorIO final : public MotorIO
    {
    public:
      explicit SparkMotorIO(MotorConfig config) : config_(std::move(config))
      {
        if (!config_.canBus.empty() && config_.canBus != "rio")
          throw std::invalid_argument("SparkMax requires roboRIO CAN bus");
        for (int id : config_.ids)
        {
          auto motor = std::make_unique<rev::spark::SparkMax>(id, rev::spark::SparkLowLevel::MotorType::kBrushless);
          rev::spark::SparkMaxConfig settings;
          settings.SmartCurrentLimit(config_.currentLimit).SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
          if (motor->Configure(settings, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters) != rev::REVLibError::kOk)
            throw std::runtime_error("SparkMax configuration failed");
          motors_.push_back(std::move(motor));
        }
      }
      MotorInputs Read() override
      {
        auto &motor = *motors_.front();
        auto encoder = motor.GetEncoder();
        double direction = config_.reversed.front() ? -1 : 1;
        double position = encoder.GetPosition();
        bool connected = motor.GetLastError() == rev::REVLibError::kOk;
        double velocity = encoder.GetVelocity();
        connected = (motor.GetLastError() == rev::REVLibError::kOk) && connected;
        double appliedOutput = motor.GetAppliedOutput();
        connected = (motor.GetLastError() == rev::REVLibError::kOk) && connected;
        double supplyVoltage = motor.GetBusVoltage();
        connected = (motor.GetLastError() == rev::REVLibError::kOk) && connected;
        double current = motor.GetOutputCurrent();
        connected = (motor.GetLastError() == rev::REVLibError::kOk) && connected;
        return {direction * position / config_.gearRatio, direction * velocity / (60 * config_.gearRatio),
                direction * appliedOutput * supplyVoltage, supplyVoltage, current, connected};
      }
      void SetVoltage(double volts) override
      {
        for (size_t i = 0; i < motors_.size(); ++i)
        {
          bool reversed = i == 0 ? config_.reversed[0] : config_.reversed[0] != config_.reversed[i];
          motors_[i]->SetVoltage(units::volt_t{reversed ? -volts : volts});
        }
      }
      void SetBrake(bool enabled) override
      {
        rev::spark::SparkMaxConfig settings;
        settings.SetIdleMode(enabled ? rev::spark::SparkBaseConfig::IdleMode::kBrake : rev::spark::SparkBaseConfig::IdleMode::kCoast);
        for (auto &motor : motors_)
          motor->Configure(settings, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters);
      }

    private:
      MotorConfig config_;
      std::vector<std::unique_ptr<rev::spark::SparkMax>> motors_;
    };
    // Output-shaft radians internally, output rotations at the MotorIO boundary.
    class SimMotorIO final : public MotorIO
    {
    public:
      SimMotorIO(MotorConfig config, Vendor vendor, MotorSimulationConfig model)
          : config_(std::move(config)), model_(model), inertia_(model.outputInertia.value()),
            resistance_(vendor == Vendor::SparkMax ? 12. / 105 : 12. / 366),
            kt_(vendor == Vendor::SparkMax ? 2.6 / 105 : 7.09 / 366),
            kv_(vendor == Vendor::SparkMax ? 5676 * 2 * std::numbers::pi / 60 / 12 : 6000 * 2 * std::numbers::pi / 60 / 12),
            position_(model.initialPosition.value())
      {
        model_.Validate();
        if (model_.load == MotorSimulationConfig::Load::Elevator)
          inertia_ += model_.mass.value() * std::pow(model_.drumRadius.value(), 2);
#ifndef __FRC_ROBORIO__
        registration_ = sim::GetSimulationRuntime().Register({
            [this](double dt) { StepSimulation(units::second_t{dt}); }, {},
            [this] { position_ = model_.initialPosition.value(); velocity_ = voltage_ = current_ = 0; }});
#endif
      }
      MotorInputs Read() override
      {
        return {position_ / (2 * std::numbers::pi), velocity_ / (2 * std::numbers::pi), voltage_, 12,
                std::abs(current_) * config_.ids.size(), true};
      }
      void StepSimulation(units::second_t duration) override
      {
        const double dt = duration.value();
        if (!std::isfinite(dt) || dt < 0 || dt > 1)
          throw std::invalid_argument("Simulation step must be finite and between zero and one second");
        // Bound nonlinear gravity error and stop penetration independently of caller cadence.
        const int steps = static_cast<int>(std::ceil(dt / .001));
        for (int n = 0; n < steps; ++n)
          Advance(dt / steps);
      }
      void SetVoltage(double volts) override
      {
        if (!std::isfinite(volts)) throw std::invalid_argument("Nonfinite simulated motor voltage");
        voltage_ = std::clamp(volts, -12., 12.);
      }
      void SetBrake(bool enabled) override { brake_ = enabled; }

    private:
      void Advance(double dt)
      {
        const double drag = .0001 / inertia_;
        const double torque = kt_ * config_.ids.size() * config_.gearRatio / inertia_;
        const double emf = config_.gearRatio / kv_;
        const double limit = config_.currentLimit;
        double load = 0;
        if (model_.load == MotorSimulationConfig::Load::Arm)
          load = -model_.mass.value() * 9.80665 * model_.armCOM.value() * std::cos(position_ + model_.armAngleOffset.value()) / inertia_;
        else if (model_.load == MotorSimulationConfig::Load::Elevator)
          load = -model_.mass.value() * 9.80665 * model_.drumRadius.value() / inertia_;
        auto advance = [&](double duration, double decay, double equilibrium)
        {
          const double integral = -std::expm1(-decay * duration) / decay;
          position_ += equilibrium * duration + (velocity_ - equilibrium) * integral;
          velocity_ += (equilibrium - velocity_) * (-std::expm1(-decay * duration));
        };
        if (!brake_ && std::abs(voltage_) < 1e-9)
        {
          advance(dt, drag, load / drag);
          current_ = 0;
        }
        else
        {
          double remaining = dt;
          // Split at every electrical region crossing. Gravity can drive a motor
          // through both current limits, so one crossing is not sufficient.
          for (int region = 0; remaining > 1e-14 && region < 4; ++region)
          {
            const double raw = (voltage_ - emf * velocity_) / resistance_;
            const double linearDecay = torque * emf / resistance_ + drag;
            const double linearEquilibrium = (torque * voltage_ / resistance_ + load) / linearDecay;
            double decay = linearDecay, equilibrium = linearEquilibrium;
            double boundary = 0;
            bool crossingPossible = false;
            const double acceleration = torque * std::clamp(raw, -limit, limit) + load - drag * velocity_;
            const bool positiveLimited = raw > limit + 1e-9 || (raw >= limit - 1e-9 && acceleration < 0);
            const bool negativeLimited = raw < -limit - 1e-9 || (raw <= -limit + 1e-9 && acceleration > 0);
            if (positiveLimited || negativeLimited)
            {
              const double current = positiveLimited ? limit : -limit;
              decay = drag;
              equilibrium = (torque * current + load) / drag;
              boundary = (voltage_ - resistance_ * current) / emf;
              crossingPossible = (boundary - velocity_) * (equilibrium - boundary) > 0;
            }
            else
            {
              const double low = (voltage_ - resistance_ * limit) / emf;
              const double high = (voltage_ + resistance_ * limit) / emf;
              boundary = linearEquilibrium < low ? low : high;
              crossingPossible = linearEquilibrium < low || linearEquilibrium > high;
            }
            double segment = remaining;
            if (crossingPossible && std::abs(equilibrium - velocity_) > 1e-14)
            {
              const double fraction = (boundary - velocity_) / (equilibrium - velocity_);
              if (fraction > 1e-12 && fraction < 1)
                segment = std::min(remaining, -std::log1p(-fraction) / decay);
            }
            advance(segment, decay, equilibrium);
            remaining -= segment;
          }
          current_ = std::clamp((voltage_ - emf * velocity_) / resistance_, -limit, limit);
        }
        if (model_.minimum && position_ < model_.minimum->value())
        {
          position_ = model_.minimum->value(); velocity_ = std::max(0., velocity_);
        }
        if (model_.maximum && position_ > model_.maximum->value())
        {
          position_ = model_.maximum->value(); velocity_ = std::min(0., velocity_);
        }
        // Stops constrain the shaft before reporting its electrical state: a
        // powered motor held against a stop draws stall/current-limited current.
        current_ = !brake_ && std::abs(voltage_) < 1e-9 ? 0 :
            std::clamp((voltage_ - emf * velocity_) / resistance_, -limit, limit);
      }
      MotorConfig config_;
      MotorSimulationConfig model_;
      double inertia_, resistance_, kt_, kv_, position_, velocity_ = 0, voltage_ = 0, current_ = 0;
      bool brake_ = true;
#ifndef __FRC_ROBORIO__
      // Destroy registration before any captured state.
      sim::Registration registration_;
#endif
    };
  }
  void MotorSimulationConfig::Validate() const
  {
    auto finite = [](double value) { return std::isfinite(value); };
    if (!finite(outputInertia.value()) || outputInertia.value() <= 0 ||
        !finite(mass.value()) || mass.value() < 0 || !finite(armCOM.value()) || armCOM.value() < 0 ||
        !finite(armAngleOffset.value()) || !finite(drumRadius.value()) || drumRadius.value() < 0 || !finite(initialPosition.value()) ||
        (minimum && !finite(minimum->value())) || (maximum && !finite(maximum->value())) ||
        (minimum && maximum && *minimum >= *maximum) ||
        (minimum && initialPosition < *minimum) || (maximum && initialPosition > *maximum) ||
        (load == Load::Arm && (mass.value() <= 0 || armCOM.value() <= 0)) ||
        (load == Load::Elevator && (mass.value() <= 0 || drumRadius.value() <= 0)) ||
        (load != Load::Free && load != Load::Arm && load != Load::Elevator))
      throw std::invalid_argument("Invalid mechanism simulation configuration");
  }
  std::unique_ptr<MotorIO> MakeMotorIO(Mode mode, Vendor vendor, const MotorConfig &config, double outputInertia)
  {
    MotorSimulationConfig simulation;
    simulation.outputInertia = units::kilogram_square_meter_t{outputInertia};
    return MakeMotorIO(mode, vendor, config, simulation);
  }
  std::unique_ptr<MotorIO> MakeMotorIO(Mode mode, Vendor vendor, const MotorConfig &config,
                                    const MotorSimulationConfig &simulation)
  {
    config.Validate();
    if (mode == Mode::Replay)
      return std::make_unique<InjectedMotorIO>();
    if (mode == Mode::Sim)
      return std::make_unique<SimMotorIO>(config, vendor, simulation);
    if (vendor == Vendor::SparkMax)
      return std::make_unique<SparkMotorIO>(config);
    return std::make_unique<TalonMotorIO>(config);
  }
}
