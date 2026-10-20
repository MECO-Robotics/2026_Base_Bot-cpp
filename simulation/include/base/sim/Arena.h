#pragma once
#include "base/sim/SimulationWorld.h"
#include <deque>

namespace base::sim {
using PieceId = uint64_t;
using IntakeId = uint64_t;
struct PieceSpec {
  BodySpec::Shape shape{BodySpec::Shape::Sphere};
  double radius{0.12};
  Vec3 halfExtents{0.12, 0.12, 0.12};
  double mass{0.4}, friction{0.6}, restitution{0.25};
  double rollingResistance{0.3}; // ground deceleration, metres/second squared
  Pose pose{};
  Vec3 velocity{};
  bool airborne{};
};
struct PieceSnapshot {
  PieceId id{};
  BodyState state{};
  bool held{}, airborne{};
};
struct IntakeConfig {
  Pose localPose{{0.65, 0, 0.2}, {}};
  Vec3 halfExtents{0.25, 0.35, 0.3};
  size_t capacity{1};
};
// Owns all bodies it creates. The clock and truth provider must outlive the
// arena.
class Arena {
public:
  // Truth reports body-origin pose and COM velocity; COM is local to that origin.
  Arena(SimulationClock &, std::function<BodyState()> robotTruth,
        bool practiceScene = true, uint32_t seed = 2026,
        Vec3 robotCenterOfMass = {});
  ~Arena();
  Arena(const Arena &) = delete;
  Arena &operator=(const Arena &) = delete;
  PieceId Spawn(const PieceSpec &);
  void AddCollider(const BodySpec &);
  void SetIntake(const IntakeConfig &);
  IntakeId RegisterIntake(const IntakeConfig &);
  void SetIntakeActive(bool active) { SetIntakeActive(0, active); }
  void SetIntakeActive(IntakeId, bool);
  std::optional<PieceId> Launch(Pose localMuzzle, Vec3 localVelocity) {
    return Launch(0, localMuzzle, localVelocity);
  }
  std::optional<PieceId> Launch(IntakeId, Pose localMuzzle, Vec3 localVelocity);
  std::vector<PieceSnapshot> Pieces() const;
  size_t HeldCount(IntakeId id = 0) const;
  bool IntakeActive(IntakeId id = 0) const;
  void Reset(uint32_t seed = 2026);

private:
  struct Piece {
    PieceId id;
    PieceSpec spec;
    std::optional<BodyHandle> body;
    bool airborne;
    double settled{};
  };
  BodyHandle CreatePiece(const PieceSpec &, bool airborne,
                         const BodyState * = nullptr);
  void Before(double);
  void After(double);
  void BuildScene();
  void ClearPieces();
  SimulationClock &clock_;
  SimulationWorld &world_;
  std::function<BodyState()> robotTruth_;
  Vec3 robotCenterOfMass_;
  bool practiceScene_;
  uint32_t seed_;
  PieceId nextId_{1};
  struct Intake {
    IntakeConfig config;
    bool active{};
    std::deque<PieceId> held;
  };
  std::vector<Intake> intakes_{{IntakeConfig{}, false, {}}};
  std::vector<BodyHandle> colliders_;
  std::vector<Piece> pieces_;
  Registration registration_;
};
} // namespace base::sim
