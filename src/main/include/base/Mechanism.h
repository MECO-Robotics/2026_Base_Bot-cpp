#pragma once

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <units/time.h>
#include <wpi/DataLog.h>
#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

namespace base {
enum class Mode { Real, Sim, Replay };
enum class Vendor { TalonFX, SparkMax };
enum class Gravity { Constant, Cosine, Sine };
struct MotorConfig {
  std::vector<int> ids;
  std::vector<bool> reversed;
  double gearRatio = 1;
  int currentLimit = 40;
  std::string canBus;
  void Validate() const;
};
struct Gains {
  double p=0, i=0, d=0, s=0, v=0, a=0, gravity=0;
  double maxVelocity=1, maxAcceleration=1, tolerance=.01;
  double minimum=0, maximum=1, defaultPosition=0;
  void Validate(bool position) const;
};
struct EncoderCalibration {
  double unitsPerRotation=1, offsetRotations=0;
  bool reversed=false;
  double Apply(double raw) const;
  void Validate() const;
};
struct MotorInputs {
  double position=0, velocity=0, voltage=0, supplyVoltage=12, current=0;
  bool connected=false;
};
class MotorIO {
 public:
  virtual ~MotorIO() = default;
  // Sampling is observational. Only the simulation owner advances physics.
  virtual MotorInputs Read()=0;
  virtual void AdvanceSimulation(units::second_t dt) {}
  virtual void SetVoltage(double volts)=0;
  virtual void SetBrake(bool brake)=0;
};
// Factories are lazy: replay never constructs a controller or simulator.
std::unique_ptr<MotorIO> MakeMotorIO(Mode mode, Vendor vendor, const MotorConfig& config,
                                   double outputInertia=.025);
class InjectedMotorIO final : public MotorIO {
 public:
  MotorInputs inputs;
  double requestedVoltage=0;
  bool brake=true;
  MotorInputs Read() override { return inputs; }
  void SetVoltage(double volts) override { requestedVoltage=volts; }
  void SetBrake(bool enabled) override { brake=enabled; }
};
// Mechanism positions are rotations, or metres when unitsPerOutputRotation is a drum circumference.
class Mechanism : public frc2::SubsystemBase {
 public:
  Mechanism(std::string name, std::unique_ptr<MotorIO> io, Gains gains,
            bool positionControlled, double unitsPerOutputRotation=1,
            Gravity gravity=Gravity::Constant,
            std::function<double()> externalEncoder={}, EncoderCalibration calibration={});
  void Periodic() override;
  void SimulationPeriodic() override;
  void AdvanceSimulation(units::second_t dt);
  // Single-threaded owner only: scheduler/commands, explicit control, and telemetry
  // must not race. External scheduling suppresses automatic control/plant ticks.
  void UseExternalScheduling(bool enabled=true) { externalScheduling_=enabled; }
  void Control(units::second_t dt);
  void Control(units::second_t dt, bool enabled);
  void PublishTelemetry();
  void SetVelocity(double velocity);
  void SetPosition(double position);
  void SetPositionDynamic(double position, double maxVelocity, double maxAcceleration);
  void SetVoltage(double volts);
  void Stop();
  void ResetPosition();
  void SetBrake(bool enabled);
  void SetGains(Gains gains);
  // Opt-in NT array at <name>/Gains/Values; accepted only while disabled.
  // Order is published at <name>/Gains/Fields; one array is one atomic candidate.
  void EnableLiveTuning(bool enabled=true);
  // Log must outlive this mechanism. Call during initialization, off control threads.
  void EnableInputLogging(wpi::log::DataLog& log);
  const Gains& GetGains() const { return gains_; }
  bool AtSetpoint() const;
  bool VelocityControlActive() const { return control_==ControlMode::Velocity; }
  double VelocitySetpoint() const { return target_; }
  double Position() const { return physicalPosition_ - zeroOffset_; }
  double Velocity() const { return inputs_.velocity * units_; }
  double DesiredPosition() const { return target_ - zeroOffset_; }
  double PositionOffset() const { return zeroOffset_; }
  const MotorInputs& Inputs() const { return inputs_; }
  frc2::CommandPtr VelocityCommand(std::function<double()> velocity);
  frc2::CommandPtr PositionCommand(std::function<double()> position);
  frc2::CommandPtr VoltageCommand(std::function<double()> voltage);
 private:
  enum class ControlMode { Stopped, Voltage, Velocity, Position };
  std::unique_ptr<MotorIO> io_;
  Gains gains_;
  bool joint_;
  double units_, physicalPosition_=0, zeroOffset_=0, target_=0, voltage_=0;
  double profiledPosition_=0, profiledVelocity_=0, previousError_=0, integral_=0;
  double maxVelocity_, maxAcceleration_;
  Gravity gravity_;
  std::function<double()> externalEncoder_;
  EncoderCalibration calibration_;
  MotorInputs inputs_;
  std::unique_ptr<wpi::log::DoubleArrayLogEntry> inputLog_;
  bool liveTuning_=false, externalScheduling_=false;
  void UpdateLiveTuning();
  ControlMode control_=ControlMode::Stopped;
};
class Flywheel final : public Mechanism {
 public:
  Flywheel(std::string name, std::unique_ptr<MotorIO> io, Gains gains)
      : Mechanism(std::move(name), std::move(io), gains, false) {}
};
class PositionJoint final : public Mechanism {
 public:
  PositionJoint(std::string name, std::unique_ptr<MotorIO> io, Gains gains,
                double unitsPerOutputRotation=1, Gravity gravity=Gravity::Constant,
                std::function<double()> encoder={}, EncoderCalibration calibration={})
      : Mechanism(std::move(name), std::move(io), gains, true, unitsPerOutputRotation,
                  gravity, std::move(encoder), calibration) {}
};
class DigitalSensor {
 public:
  DigitalSensor(int channel, bool inverted=false) : input_(channel), inverted_(inverted) {}
  bool Get() const { return input_.Get() != inverted_; }
 private:
  frc::DigitalInput input_;
  bool inverted_;
};
}
