#include "base/Mechanism.h"
#include "base/detail/SparkControl.h"
#include "base/Replay.h"
#include <algorithm>
#include <chrono>
#include <thread>
#include <numbers>
#include <stdexcept>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <rev/SparkMax.h>
#include <frc/DriverStation.h>
#include <rev/config/SparkMaxConfig.h>
#include <units/voltage.h>
#ifndef __FRC_ROBORIO__
#include "base/sim/SimulationWorld.h"
#endif

namespace base
{
  namespace detail {
    bool SparkDeviceConfigured(rev::spark::SparkMax& motor, bool& resetLatched) {
      const auto warnings=motor.GetStickyWarnings();
      const bool warningStatus=motor.GetLastError()==rev::REVLibError::kOk;
      resetLatched=resetLatched || warnings.hasReset;
      const auto faults=motor.GetFaults();
      return warningStatus && motor.GetLastError()==rev::REVLibError::kOk &&
             faults.rawBits==0 && !resetLatched;
    }
    void ConfigureSparkPid(rev::spark::ClosedLoopConfig& settings, const Gains& gains,
                           double scale) {
      const double p=gains.p*scale/12;
      const double i=gains.i*scale/12/1000;
      const double d=gains.d*scale/12*1000;
      for(double value:{p,i,d})
        if(!std::isfinite(value)) throw std::invalid_argument("Spark PID conversion overflow");
      settings.Pid(p,i,d);
    }
    void ConfigureSparkGravity(rev::spark::FeedForwardConfig& settings, double volts,
                               double scale, Gravity gravity, bool joint) {
      // Each setter selects a mode and overwrites the shared gravity parameter.
      // Clear the prior cosine ratio explicitly when returning to constant.
      if (joint && gravity==Gravity::Cosine) settings.kCos(volts).kCosRatio(scale);
      else settings.kG(joint && gravity==Gravity::Constant ? volts : 0).kCosRatio(0);
    }
  }
  namespace
  {
    class SparkConfigurationTimeout {
     public:
      explicit SparkConfigurationTimeout(rev::spark::SparkMax& motor):motor_(motor) {
        if(motor_.SetCANTimeout(250)!=rev::REVLibError::kOk) {
          motor_.SetCANTimeout(0);
          throw std::runtime_error("Spark configuration timeout setup failed");
        }
      }
      ~SparkConfigurationTimeout() { if(!restored_) motor_.SetCANTimeout(0); }
      void Restore() {
        if(motor_.SetCANTimeout(0)!=rev::REVLibError::kOk)
          throw std::runtime_error("Spark runtime timeout restoration failed");
        restored_=true;
      }
     private:
      rev::spark::SparkMax& motor_;
      bool restored_=false;
    };
    class TalonMotorIO final : public MotorIO
    {
    public:
      explicit TalonMotorIO(MotorConfig config) : config_(std::move(config))
      {
        if (frc::DriverStation::IsEnabled()) throw std::logic_error("Motor construction requires disabled state");
        for (size_t i = 0; i < config_.ids.size(); ++i)
        {
          auto motor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(config_.ids[i], config_.canBus);
          ctre::phoenix6::configs::TalonFXConfiguration settings;
          {
            settings.MotorOutput.Inverted = config_.reversed[i] ?
                ctre::phoenix6::signals::InvertedValue::Clockwise_Positive :
                ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
            // Phoenix keeps its integrated sensor in phase with motor inversion.
            settings.Feedback.SensorToMechanismRatio=config_.gearRatio;
            if(config_.phoenix.feedback==PhoenixControlConfig::Feedback::RemoteCANcoder) {
              settings.Feedback.FeedbackRemoteSensorID=config_.phoenix.remoteSensorId;
              settings.Feedback.SensorToMechanismRatio=(config_.phoenix.sensorReversed?-1:1)*config_.phoenix.sensorToMechanismRatio;
              settings.Feedback.FeedbackSensorSource=ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
              settings.Feedback.RotorToSensorRatio=config_.phoenix.rotorToSensorRatio;
            }
          }
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
        bool feedbackHealthy=true;
        if(config_.phoenix.feedback==PhoenixControlConfig::Feedback::RemoteCANcoder) {
          const auto& invalid=motor.GetFault_RemoteSensorDataInvalid();
          feedbackHealthy=invalid.GetStatus().IsOK() && !invalid.GetValue();
        }
        // Phoenix getters refresh their motor-owned cached signals by default.
        double direction = 1;
        const double ratio=1;
        lastPosition_=position.GetValueAsDouble();
        return {direction * position.GetValueAsDouble() / ratio,
                direction * velocity.GetValueAsDouble() / ratio,
                (config_.reversed.front()?-1:1) * voltage.GetValueAsDouble(), supply.GetValueAsDouble(), current.GetValueAsDouble(),
                position.GetStatus().IsOK() && velocity.GetStatus().IsOK() && feedbackHealthy};
      }
      void SetVoltage(double volts) override
      {
        if (volts==0) {
          for(auto& motor:motors_) motor->SetVoltage(units::volt_t{0});
        } else {
          motors_.front()->SetVoltage(units::volt_t{volts});
          if (!Follow()) { for(auto& motor:motors_) motor->SetVoltage(units::volt_t{0}); }
        }
      }
      bool ConfigureNative(const Gains& gains, double scale, Gravity gravity, bool joint) override
      {
        if (frc::DriverStation::IsEnabled()) throw std::logic_error("Motor configuration requires disabled state");
        for(double value : {gains.p*scale,gains.i*scale,gains.d*scale,gains.v*scale,gains.a*scale,
            gains.minimum/scale,gains.maximum/scale,gains.maxVelocity/scale,gains.maxAcceleration/scale})
          if(!std::isfinite(value)) throw std::invalid_argument("Native control conversion overflow");
        scale_=scale; accelerationGain_=gains.a;
        gravity_=gravity; gravityGain_=joint?gains.gravity:0;
        customGravity_=gravity==Gravity::Sine || (gravity==Gravity::Cosine && scale!=1);
        ctre::phoenix6::configs::Slot0Configs slot;
        slot.kP=gains.p*scale; slot.kI=gains.i*scale; slot.kD=gains.d*scale;
        slot.kS=gains.s; slot.kV=gains.v*scale; slot.kA=gains.a*scale; slot.kG=customGravity_?0:gravityGain_;
        slot.GravityType=gravity==Gravity::Cosine ? ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine :
            ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
        auto& configurator=motors_.front()->GetConfigurator();
        ctre::phoenix6::configs::SoftwareLimitSwitchConfigs limits;
        limits.ForwardSoftLimitEnable=joint; limits.ReverseSoftLimitEnable=joint;
        limits.ForwardSoftLimitThreshold=units::turn_t{gains.maximum/scale};
        limits.ReverseSoftLimitThreshold=units::turn_t{gains.minimum/scale};
        ctre::phoenix6::configs::MotionMagicConfigs profile;
        profile.MotionMagicCruiseVelocity=units::turns_per_second_t{gains.maxVelocity/scale};
        profile.MotionMagicAcceleration=units::turns_per_second_squared_t{gains.maxAcceleration/scale};
        if(!configurator.Apply(slot).IsOK() || !configurator.Apply(limits).IsOK() || !configurator.Apply(profile).IsOK())
          throw std::runtime_error("Phoenix native control configuration failed");

        return true;
      }
      bool SetNative(const NativeControlRequest& request) override
      {

        for(double value : {request.target/scale_,request.velocity/scale_,request.acceleration/scale_,
            accelerationGain_*request.acceleration})
          if(!std::isfinite(value)) return false;
        auto& motor=*motors_.front();
        const double angle=lastPosition_*scale_*2*std::numbers::pi;
        const double gravity=customGravity_ ? gravityGain_*(gravity_==Gravity::Sine?std::sin(angle):std::cos(angle)) : 0;
        ctre::phoenix::StatusCode status;
        if(request.kind==NativeControlRequest::Kind::Velocity)
          status=motor.SetControl(ctre::phoenix6::controls::VelocityVoltage{units::turns_per_second_t{request.target/scale_}}
              .WithAcceleration(units::turns_per_second_squared_t{request.acceleration/scale_})
              .WithFeedForward(units::volt_t{gravity}).WithEnableFOC(false));
        else if(request.kind==NativeControlRequest::Kind::Position)
          status=motor.SetControl(ctre::phoenix6::controls::PositionVoltage{units::turn_t{request.target/scale_}}
              .WithVelocity(units::turns_per_second_t{request.velocity/scale_})
              .WithFeedForward(units::volt_t{accelerationGain_*request.acceleration+gravity}).WithEnableFOC(false));
        else {
          status=motor.SetControl(ctre::phoenix6::controls::MotionMagicVoltage{units::turn_t{request.target/scale_}}
              .WithFeedForward(units::volt_t{gravity}).WithEnableFOC(false));
        }
        const bool followers=Follow(); return status.IsOK() && followers;
      }
      void SetBrake(bool enabled) override
      {
        if(brake_==enabled) return;
        if(frc::DriverStation::IsEnabled())
          throw std::logic_error("Neutral mode configuration requires disabled state");
        for (auto &motor : motors_)
          if(!motor->SetNeutralMode(enabled ? ctre::phoenix6::signals::NeutralModeValue::Brake :
              ctre::phoenix6::signals::NeutralModeValue::Coast).IsOK()) {
            SetVoltage(0); throw std::runtime_error("Talon brake configuration failed");
          }
        brake_=enabled;
      }

    private:
      bool Follow() {
        bool ok=true;
        for(size_t i=1;i<motors_.size();++i)
          ok=motors_[i]->SetControl(ctre::phoenix6::controls::Follower{config_.ids.front(),config_.reversed[i] ? ctre::phoenix6::signals::MotorAlignmentValue::Opposed : ctre::phoenix6::signals::MotorAlignmentValue::Aligned}).IsOK() && ok;
        return ok;
      }
      double scale_=1, accelerationGain_=0, gravityGain_=0, lastPosition_=0;
      Gravity gravity_=Gravity::Constant;
      bool customGravity_=false, brake_=true;
      MotorConfig config_;
      std::vector<std::unique_ptr<ctre::phoenix6::hardware::TalonFX>> motors_;
    };
    class SparkMotorIO final : public MotorIO
    {
    public:
      explicit SparkMotorIO(MotorConfig config) : config_(std::move(config))
      {
        if (frc::DriverStation::IsEnabled()) throw std::logic_error("Motor construction requires disabled state");
        if (!config_.canBus.empty() && config_.canBus != "rio")
          throw std::invalid_argument("SparkMax requires roboRIO CAN bus");
        for (size_t i=0;i<config_.ids.size();++i)
        {
          if(frc::DriverStation::IsEnabled())
            throw std::logic_error("Spark startup configuration requires disabled state");
          auto motor = std::make_unique<rev::spark::SparkMax>(config_.ids[i], rev::spark::SparkLowLevel::MotorType::kBrushless);
          // Persisted signal periods may be slow or disabled. Establish bounded
          // health-frame delivery before waiting for cleared boot history.
          rev::spark::SparkMaxConfig healthSignals;
          healthSignals.signals.FaultsPeriodMs(20).FaultsAlwaysOn(true)
              .WarningsPeriodMs(20).WarningsAlwaysOn(true);
          if(motor->Configure(healthSignals,rev::ResetMode::kNoResetSafeParameters,
              rev::PersistMode::kNoPersistParameters)!=rev::REVLibError::kOk)
            throw std::runtime_error("Spark startup health signal configuration failed");
          // Clear and confirm boot history before writing configuration. A reset
          // during or after configuration must remain visible to runtime checks.
          if(motor->ClearFaults()!=rev::REVLibError::kOk)
            throw std::runtime_error("Spark startup reset acknowledgement failed");
          const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds{250};
          while(true) {
            if(frc::DriverStation::IsEnabled())
              throw std::logic_error("Spark startup acknowledgement requires disabled state");
            const auto warnings=motor->GetStickyWarnings();
            if(motor->GetLastError()!=rev::REVLibError::kOk)
              throw std::runtime_error("Spark startup warning read failed");
            if(!warnings.hasReset) break;
            if(std::chrono::steady_clock::now()>=deadline)
              throw std::runtime_error("Spark startup reset acknowledgement timed out");
            std::this_thread::sleep_for(std::chrono::milliseconds{5});
          }
          if(frc::DriverStation::IsEnabled())
            throw std::logic_error("Spark full configuration requires disabled state");
          rev::spark::SparkMaxConfig settings;
          settings.signals.FaultsPeriodMs(20).FaultsAlwaysOn(true)
              .WarningsPeriodMs(20).WarningsAlwaysOn(true);
          settings.SmartCurrentLimit(config_.currentLimit).SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
          settings.VoltageCompensation(12);
          if (i==0) settings.Inverted(config_.reversed.front());
          else settings.Follow(config_.ids.front(),config_.reversed[i]);
          settings.encoder.PositionConversionFactor(1/config_.gearRatio)
              .VelocityConversionFactor(1/(60*config_.gearRatio));
          if(config_.spark.feedback==SparkControlConfig::Feedback::AbsoluteEncoder)
            settings.absoluteEncoder.PositionConversionFactor(1/config_.spark.sensorToMechanismRatio)
                .VelocityConversionFactor(1/(60*config_.spark.sensorToMechanismRatio))
                .Inverted(config_.spark.sensorReversed).ZeroOffset(config_.spark.zeroOffset);
          settings.closedLoop.SetFeedbackSensor(config_.spark.feedback==SparkControlConfig::Feedback::AbsoluteEncoder ?
              rev::spark::FeedbackSensor::kAbsoluteEncoder : rev::spark::FeedbackSensor::kPrimaryEncoder);
          if (motor->Configure(settings, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kNoPersistParameters) != rev::REVLibError::kOk)
            throw std::runtime_error("SparkMax configuration failed");
          if(motor->SetCANTimeout(0)!=rev::REVLibError::kOk)
            throw std::runtime_error("Spark runtime timeout configuration failed");
          motors_.push_back(std::move(motor));
        }
      }
      MotorInputs Read() override
      {
        const bool configured=CheckConfigured();
        auto &motor = *motors_.front();
        const bool absolute=config_.spark.feedback==SparkControlConfig::Feedback::AbsoluteEncoder;
        double position = absolute ? motor.GetAbsoluteEncoder().GetPosition() : motor.GetEncoder().GetPosition();
        bool connected = motor.GetLastError() == rev::REVLibError::kOk;
        double velocity = absolute ? motor.GetAbsoluteEncoder().GetVelocity() : motor.GetEncoder().GetVelocity();
        connected = (motor.GetLastError() == rev::REVLibError::kOk) && connected;
        double appliedOutput = motor.GetAppliedOutput();
        connected = (motor.GetLastError() == rev::REVLibError::kOk) && connected;
        double supplyVoltage = motor.GetBusVoltage();
        connected = (motor.GetLastError() == rev::REVLibError::kOk) && connected;
        double current = motor.GetOutputCurrent();
        connected = (motor.GetLastError() == rev::REVLibError::kOk) && connected;
        lastPosition_=position;
        return {position, velocity, (config_.reversed.front()?-1:1)*appliedOutput*supplyVoltage,
                supplyVoltage, current, connected && configured};
      }
      void SetVoltage(double volts) override
      {
        if(!CheckConfigured()) { StopAll(); return; }
        // Native followers remain followers even while stopping or characterizing.
        motors_.front()->SetVoltage(units::volt_t{volts});
      }
      bool ConfigureNative(const Gains& gains, double scale, Gravity gravity, bool joint) override
      {
        if (frc::DriverStation::IsEnabled()) throw std::logic_error("Motor configuration requires disabled state");
        if(!CheckConfigured()) { StopAll(); throw std::runtime_error("Spark reset requires robot restart and full configuration"); }
        scale_=scale; gains_=gains; gravity_=gravity; joint_=joint;
        for (double value:{gains.v*scale,gains.a*scale,gains.minimum/scale,gains.maximum/scale,
                           gains.maxVelocity/scale,gains.maxAcceleration/scale})
          if(!std::isfinite(value)) throw std::invalid_argument("Spark control conversion overflow");
        rev::spark::SparkMaxConfig settings;
        // Encoder values are output rotations and rotations/s, not rotor RPM.
        // REV PID output is duty cycle. Nominal voltage compensation maps the
        // public voltage gains to duty gains. REV integral/derivative use ms;
        // feedforward's API already uses volts and acceleration uses seconds.
        detail::ConfigureSparkPid(settings.closedLoop,gains,scale);
        settings.closedLoop.feedForward.kS(gains.s).kV(gains.v*scale).kA(gains.a*scale);
        detail::ConfigureSparkGravity(settings.closedLoop.feedForward,gains.gravity,scale,gravity,joint);
        settings.closedLoop.maxMotion.CruiseVelocity(gains.maxVelocity/scale)
            .MaxAcceleration(gains.maxAcceleration/scale).AllowedProfileError(gains.tolerance/scale);
        settings.softLimit.ForwardSoftLimitEnabled(joint).ReverseSoftLimitEnabled(joint)
            .ForwardSoftLimit(gains.maximum/scale).ReverseSoftLimit(gains.minimum/scale);
        SparkConfigurationTimeout timeout{*motors_.front()};
        if (motors_.front()->Configure(settings,rev::ResetMode::kNoResetSafeParameters,
            rev::PersistMode::kNoPersistParameters)!=rev::REVLibError::kOk)
          throw std::runtime_error("Spark onboard control configuration failed");
        timeout.Restore();
        return true;
      }
      bool SetNative(const NativeControlRequest& request) override
      {
        if(!CheckConfigured()) { StopAll(); return false; }
        const double target=request.target/scale_;
        double feedforward=0;
        auto type=rev::spark::SparkLowLevel::ControlType::kPosition;
        if(request.kind==NativeControlRequest::Kind::MotionMagic)
          type=rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl;
        else if(request.kind==NativeControlRequest::Kind::Velocity) {
          type=rev::spark::SparkLowLevel::ControlType::kVelocity;
          feedforward=gains_.a*request.acceleration;
          // REV gravity feedforward is inactive in velocity mode.
          if (joint_ && gravity_!=Gravity::Sine) {
            const double position=lastPosition_*scale_*2*std::numbers::pi;
            feedforward+=gains_.gravity*(gravity_==Gravity::Constant ? 1 : std::cos(position));
          }
        } else {
          // REV basic position mode does not apply kV/kA automatically.
          feedforward=gains_.v*request.velocity+gains_.a*request.acceleration;
        }
        if(joint_ && gravity_==Gravity::Sine)
          feedforward+=gains_.gravity*std::sin(lastPosition_*scale_*2*std::numbers::pi);
        if(!std::isfinite(target) || !std::isfinite(feedforward)) return false;
        return motors_.front()->GetClosedLoopController().SetSetpoint(target,type,
            rev::spark::ClosedLoopSlot::kSlot0,feedforward,
            rev::spark::SparkClosedLoopController::ArbFFUnits::kVoltage)==rev::REVLibError::kOk;
      }
      void SetBrake(bool enabled) override
      {
        if(brake_==enabled) return;
        if(frc::DriverStation::IsEnabled())
          throw std::logic_error("Neutral mode configuration requires disabled state");
        rev::spark::SparkMaxConfig settings;
        settings.SetIdleMode(enabled ? rev::spark::SparkBaseConfig::IdleMode::kBrake : rev::spark::SparkBaseConfig::IdleMode::kCoast);
        for (auto &motor : motors_) {
          SparkConfigurationTimeout timeout{*motor};
          if(motor->Configure(settings, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters)!=rev::REVLibError::kOk) {
            StopAll(); throw std::runtime_error("Spark brake configuration failed");
          }
          timeout.Restore();
        }
        brake_=enabled;
      }
    private:
      bool CheckConfigured() {
        bool healthy=true;
        for(auto& motor:motors_) {
          healthy=detail::SparkDeviceConfigured(*motor,resetLatched_) && healthy;
        }
        return healthy && !resetLatched_;
      }
      void StopAll() { for(auto& motor:motors_) motor->SetVoltage(units::volt_t{0}); }
      double scale_=1, lastPosition_=0;
      Gains gains_;
      Gravity gravity_=Gravity::Constant;
      bool joint_=false, brake_=true, resetLatched_=false;
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
            [this](double dt) { busVoltage_ = sim::GetSimulationRuntime().BusVoltage(); StepSimulation(units::second_t{dt}); }, {},
            [this] { position_ = model_.initialPosition.value(); velocity_ = voltage_ = requestedVoltage_ = current_ = 0; busVoltage_ = 12; },
            [this] { return SupplyCurrent(); }});
#endif
      }
      MotorInputs Read() override
      {
        return {position_ / (2 * std::numbers::pi), velocity_ / (2 * std::numbers::pi), voltage_, busVoltage_,
                SupplyCurrent(), true};
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
        requestedVoltage_ = volts;
      }
      void SetBrake(bool enabled) override { brake_ = enabled; }

    private:
      double SupplyCurrent() const
      {
        return busVoltage_ > 0 ? std::max(0., voltage_ * current_) * config_.ids.size() / busVoltage_ : 0;
      }
      void Advance(double dt)
      {
        voltage_ = std::clamp(requestedVoltage_, -busVoltage_, busVoltage_);
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
        voltage_ = std::clamp(resistance_ * current_ + emf * velocity_, -std::abs(voltage_), std::abs(voltage_));
      }
      MotorConfig config_;
      MotorSimulationConfig model_;
      double inertia_, resistance_, kt_, kv_, position_, velocity_ = 0, voltage_ = 0, current_ = 0, requestedVoltage_ = 0, busVoltage_ = 12;
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
    if (mode == Mode::Replay || replay::Active())
    {
      auto injected=std::make_unique<InjectedMotorIO>();
      injected->nativeEnabled=false; // Source frames select firmware versus emulation.
      return injected;
    }
    if (mode == Mode::Sim)
      return std::make_unique<SimMotorIO>(config, vendor, simulation);

    if (vendor == Vendor::SparkMax)
      return MakeAsyncMotorIO(std::make_unique<SparkMotorIO>(config));
    return MakeAsyncMotorIO(std::make_unique<TalonMotorIO>(config));
  }
}
