#pragma once
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace base::sim {
struct Vec3 {
  double x{}, y{}, z{};
};
struct Quaternion {
  double x{}, y{}, z{}, w{1};
};
struct Pose {
  Vec3 position{};
  Quaternion rotation{};
};
enum class Backend { Phoenix, Planar, Full3D };
Backend BackendFromEnvironment();
struct BodyHandle {
  uint32_t index{};
  uint32_t generation{};
  bool operator==(const BodyHandle &) const = default;
};
struct BodySpec {
  enum class Shape { Box, Sphere };
  Shape shape{Shape::Box};
  Vec3 halfExtents{0.5, 0.5, 0.5};
  double radius{0.1};
  double mass{};  // zero creates a static collider
  Vec3 inertia{}; // zero selects shape-derived inertia
  Vec3 centerOfMass{};
  double friction{0.7}, restitution{};
  bool planar{};
  Pose pose{};
};
struct BodyState {
  Pose pose{};
  Vec3 linearVelocity{}, angularVelocity{};
  bool enabled{true};
};
struct RayHit {
  BodyHandle body;
  Vec3 position, normal;
  double fraction;
};
struct StepTiming {
  double seconds{}, maximumSeconds{};
  uint64_t substeps{}, deadlineMisses{};
};
struct PlantCallbacks {
  std::function<void(double)> beforePhysics;
  std::function<void(double)> afterPhysics;
  std::function<void()> reset;
};
// A registration can outlive its clock. Destroy it before callback captures
// die.
class Registration {
public:
  Registration() = default;
  ~Registration();
  Registration(Registration &&) noexcept;
  Registration &operator=(Registration &&) noexcept;
  Registration(const Registration &) = delete;
  Registration &operator=(const Registration &) = delete;

private:
  friend class SimulationClock;
  explicit Registration(std::function<void()> release);
  std::function<void()> release_;
};
// Single-threaded owner: create, query, and step on the robot thread.
// No Jolt types escape this boundary; all quantities use metres, kilograms, seconds.
class SimulationWorld {
public:
  explicit SimulationWorld(Backend mode);
  ~SimulationWorld();
  SimulationWorld(const SimulationWorld &) = delete;
  SimulationWorld &operator=(const SimulationWorld &) = delete;
  BodyHandle CreateBody(const BodySpec &);
  void DestroyBody(BodyHandle);
  BodyState ReadBody(BodyHandle) const;
  void SetBodyState(BodyHandle, const BodyState &);
  void SetEnabled(BodyHandle, bool);
  void AddForce(BodyHandle, Vec3 force, Vec3 worldPosition);
  void AddTorque(BodyHandle, Vec3 torque);
  std::optional<RayHit> Raycast(Vec3 origin, Vec3 displacement,
                                std::optional<BodyHandle> exclude = {}) const;
  void Step(double seconds);
  void Reset(); // Restore all extant bodies to their creation state.
  Backend Mode() const;
  uint32_t BodyCount() const;
  uint32_t ContactManifoldCount() const; // Added/persisted manifolds during latest step.

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
// Registry and motor stepping remain available with Phoenix (no Jolt world).
class SimulationClock;
class SimulationClock {
public:
  explicit SimulationClock(Backend backend = Backend::Phoenix);
  ~SimulationClock();
  Registration Register(PlantCallbacks);
  void StepRobotPeriod(); // exactly five 4 ms steps, before -> world -> after
  void Reset();
  double Time() const { return time_; }
  const StepTiming &Timing() const { return timing_; }
  SimulationWorld *World() { return world_.get(); }

private:
  struct Registry;
  std::shared_ptr<Registry> registry_;
  std::unique_ptr<SimulationWorld> world_;
  double time_{};
  StepTiming timing_{};
};
SimulationClock &GetSimulationRuntime();
} // namespace base::sim
