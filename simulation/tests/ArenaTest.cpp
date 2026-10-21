#include "base/sim/Arena.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace base::sim;
void Check(bool value, const char *message) {
  if (!value)
    throw std::runtime_error(message);
}
void Step(SimulationClock &c, int n) {
  for (int i = 0; i < n; ++i)
    c.StepRobotPeriod();
}
int main() {
  try {
    for (auto mode : {Backend::Planar, Backend::Full3D}) {
      SimulationClock clock(mode);
      Arena arena(clock, [] { return BodyState{}; }, false);
      BodySpec floor;
      floor.halfExtents = {10, 10, 0.1};
      floor.pose.position.z = -0.1;
      arena.AddCollider(floor);
      BodySpec wall;
      wall.halfExtents = {0.1, 2, 1};
      wall.pose.position = {1.1, 0, 1};
      arena.AddCollider(wall);
      PieceSpec p;
      p.pose.position = {0, 0, 0.12};
      p.velocity = {2, 0, 0};
      p.rollingResistance = 0;
      arena.Spawn(p);
      const auto beforeGeometryRead = clock.World()->BodyCount();
      auto geometry = arena.ColliderGeometry();
      Check(geometry.size() == 2 && geometry[1].pose.position.x == 1.1 &&
                geometry[1].halfExtents.x == .1,
            "collider snapshot includes static geometry, excludes pieces");
      geometry[1].pose.position.x = 100;
      Check(arena.ColliderGeometry()[1].pose.position.x == 1.1 &&
                clock.Time() == 0 && clock.World()->BodyCount() == beforeGeometryRead,
            "geometry reads return independent snapshots without advancing physics");
      Step(clock, 100);
      Check(arena.Pieces()[0].state.pose.position.x < 1,
            "wall must block physical piece");
      arena.Reset();
      p.shape = BodySpec::Shape::Box;
      p.velocity = {0.1, 0, 0};
      p.rollingResistance = 1;
      arena.Spawn(p);
      Step(clock, 100);
      auto state = arena.Pieces()[0].state;
      Check(std::abs(state.linearVelocity.x) < 0.02,
            "rolling resistance stops box");
      Check(state.pose.position.x >= -0.02,
            "resistance must not reverse velocity");
      arena.Reset();
      p.shape = BodySpec::Shape::Sphere;
      p.airborne = true;
      p.friction = 0;
      p.restitution = 0;
      p.pose.position = {-2, 0, 0.5};
      p.velocity = {1, 0, 0};
      auto landedId = arena.Spawn(p);
      Step(clock, 150);
      auto landed = arena.Pieces().front();
      Check(landed.id == landedId && !landed.airborne,
            "moving airborne piece becomes grounded in either mode");
      Check(std::abs(landed.state.linearVelocity.x) < 0.02,
            "ground resistance stops landed frictionless sphere");
      auto count = clock.World()->BodyCount();
      p.mass = 0;
      bool rejected = false;
      try {
        arena.Spawn(p);
      } catch (const std::invalid_argument &) {
        rejected = true;
      }
      Check(rejected && clock.World()->BodyCount() == count,
            "zero mass piece rejected without body leak");
      IntakeConfig invalid;
      invalid.localPose.position.x = std::numeric_limits<double>::quiet_NaN();
      rejected = false;
      try {
        arena.SetIntake(invalid);
      } catch (const std::invalid_argument &) {
        rejected = true;
      }
      Check(rejected, "nonfinite intake pose rejected");
      invalid.localPose.position.x = 0;
      invalid.localPose.rotation.w = 2;
      rejected = false;
      try {
        arena.RegisterIntake(invalid);
      } catch (const std::invalid_argument &) {
        rejected = true;
      }
      Check(rejected, "nonunit intake rotation rejected");
    }
    {
      SimulationClock clock(Backend::Planar);
      BodyState robot;
      robot.pose.rotation = {0, 0, std::sqrt(0.5), std::sqrt(0.5)};
      robot.linearVelocity = {1, 2, 0};
      robot.angularVelocity = {0, 0, 2};
      Arena arena(clock, [&] { return robot; }, false, 2026, {.25, -.5, .1});
      IntakeConfig intake;
      intake.localPose.position = {0, 0, 0.12};
      intake.halfExtents = {1, 1, 1};
      arena.SetIntake(intake);
      auto second = arena.RegisterIntake(intake);
      PieceSpec p;
      p.pose.position.z = 0.12;
      auto id1 = arena.Spawn(p);
      p.pose.position.x = 0.2;
      auto id2 = arena.Spawn(p);
      arena.SetIntakeActive(true);
      arena.SetIntakeActive(second, true);
      Step(clock, 1);
      Check(arena.HeldCount() == 1 && arena.HeldCount(second) == 1,
            "each intake owns one distinct piece");
      auto launched = arena.Launch(Pose{{1, 0, 1}, {}}, {3, 0, 0});
      Check(launched == id1, "launch FIFO stable id");
      Check(!arena.Launch(Pose{}, {}), "empty intake cannot launch");
      auto pieces = arena.Pieces();
      Check(!pieces[0].held && pieces[1].held && pieces[1].id == id2,
            "launch only releases owner piece");
      auto v = pieces[0].state.linearVelocity;
      Check(std::abs(v.x + .5) < 1e-5 && std::abs(v.y - 4) < 1e-5,
            "launch includes rotated velocity and muzzle offset from robot COM");
    }
    for (auto mode : {Backend::Planar, Backend::Full3D}) {
      SimulationClock clock(mode);
      Arena arena(clock, [] { return BodyState{}; });
      for (auto direction : {Vec3{20, 0, 0}, Vec3{-20, 0, 0},
                             Vec3{0, 20, 0}, Vec3{0, -20, 0}}) {
        auto hit = clock.World()->Raycast({8, 4, .9}, direction);
        Check(hit.has_value(), "practice perimeter encloses field center");
        auto expectedX = direction.x > 0 ? 16. : 0.;
        auto expectedY = direction.y > 0 ? 8. : 0.;
        Check(direction.x != 0 ? std::abs(hit->position.x - expectedX) < 1e-4
                               : std::abs(hit->position.y - expectedY) < 1e-4,
              "practice walls use blue-corner field coordinates");
      }
      auto first = arena.Pieces();
      for (const auto &piece : first) {
        auto position = piece.state.pose.position;
        Check(position.x > .12 && position.x < 15.88 &&
                  position.y > .12 && position.y < 7.88,
              "seeded pieces spawn inside positive field bounds");
      }
      arena.Reset(2026);
      auto repeat = arena.Pieces();
      Check(first.size() == repeat.size(), "reset count");
      for (size_t i = 0; i < first.size(); ++i)
        Check(first[i].id == repeat[i].id &&
                  first[i].state.pose.position.x ==
                      repeat[i].state.pose.position.x,
              "seed reset reproducibility");
      PieceSpec p;
      p.airborne = true;
      p.pose.position = {2, 2, 1};
      p.restitution = 0;
      auto id = arena.Spawn(p);
      Step(clock, 250);
      auto pieces = arena.Pieces();
      auto s = pieces.back();
      Check(s.id == id && !s.airborne,
            "airborne sphere settles into planar body preserving id");
    }
    {
      SimulationClock clock(Backend::Full3D);
      BodySpec robot;
      robot.mass = 10;
      auto external = clock.World()->CreateBody(robot);
      auto count = clock.World()->BodyCount();
      {
        Arena arena(clock, [] { return BodyState{}; });
        Check(clock.World()->BodyCount() > count, "arena creates bodies");
      }
      Check(clock.World()->BodyCount() == count,
            "arena cleans only its bodies");
      Step(clock, 1);
      clock.World()->DestroyBody(external);
    }
    std::cout << "Arena tests passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
