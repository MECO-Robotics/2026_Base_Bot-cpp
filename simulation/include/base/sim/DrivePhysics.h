#pragma once
#include "base/sim/SimulationWorld.h"
#include <array>
#include <cstddef>

namespace base::sim {
// SI throughout, including motor/wheel angles in radians. Defaults are illustrative.
struct DrivePhysicsConfig {
  double mass{74.088};
  Vec3 inertia{4.0, 4.0, 6.883}, centerOfMass{};
  Vec3 bumperHalfExtents{.43, .43, .12};
  double rideHeight{.30}, wheelRadius{.0508};
  double driveRatio{6.122448979591837}, steerRatio{150.0 / 7.0}, couplingRatio{};
  double wheelInertia{.025}, steerInertia{.004};
  double friction{1.2}, longitudinalStiffness{1800}, lateralStiffness{1800};
  double suspensionRestLength{.30}, suspensionTravel{.15};
  double suspensionStiffness{6000}, suspensionDamping{650};
  double motorResistance{12.0 / 366}, motorKt{7.09 / 366}, motorKv{55.0};
  double driveCurrentLimit{80}, steerCurrentLimit{40}, frictionVoltage{.1};
  std::array<Vec3, 4> moduleLocations{{{.33655,.33655,0},{.33655,-.33655,0},{-.33655,.33655,0},{-.33655,-.33655,0}}};
};
struct ModulePhysicsState {
  double wheelAngle{}, wheelVelocity{}, steerAngle{}, steerVelocity{};
  double driveRotorAngle{}, driveRotorVelocity{}, steerRotorAngle{}, steerRotorVelocity{};
  double longitudinalSlip{}, longitudinalForce{}, lateralForce{}, normalForce{};
  double driveCurrent{}, steerCurrent{}; // signed winding current, amperes
};
struct DriveTruth {
  BodyState body;
  std::array<ModulePhysicsState,4> modules{};
  double timestamp{};
};
// World must outlive the model. Stepping is explicit; querying never advances time.
class DrivePhysics {
public:
  DrivePhysics(SimulationWorld &, DrivePhysicsConfig = {});
  ~DrivePhysics();
  DrivePhysics(const DrivePhysics &) = delete;
  DrivePhysics &operator=(const DrivePhysics &) = delete;
  void SetModuleVoltages(std::size_t module, double drive, double steer);
  void BeforePhysics(double seconds);
  void AfterPhysics(double seconds);
  void Reset(Pose pose); // Clears wheel motion and body velocities.
  DriveTruth Truth() const;
  BodyHandle Body() const { return body_; }
  const DrivePhysicsConfig &Config() const { return config_; }
private:
  SimulationWorld &world_;
  DrivePhysicsConfig config_;
  BodyHandle body_;
  std::array<ModulePhysicsState,4> modules_{};
  std::array<std::array<double,2>,4> voltages_{};
  double time_{};
};
} // namespace base::sim
