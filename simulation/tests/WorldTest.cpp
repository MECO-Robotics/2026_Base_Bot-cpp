#include "base/sim/SimulationWorld.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <source_location>
#include <string>
using namespace base::sim;
void Check(bool ok, std::source_location where = std::source_location::current()) {
  if (!ok)
    throw std::runtime_error("World contract failed at line " + std::to_string(where.line()));
}
int main() {
  {
    SimulationClock clock;
    int steps = 0, samples = 0;
    auto plant = clock.Register({[&](double) { ++steps; }, {}, {}, [&] { ++samples; return 30.; }});
    clock.SetBusVoltage(9);
    clock.StepRobotPeriod();
    Check(steps == 5 && samples == 1 && clock.SupplyCurrent() == 30);
    Check(clock.SupplyCurrent() == 30 && samples == 1 && clock.BusVoltage() == 9);
    plant = {};
    clock.StepRobotPeriod();
    Check(clock.SupplyCurrent() == 0 && samples == 1);
    clock.Reset();
    Check(clock.BusVoltage() == 12 && clock.SupplyCurrent() == 0);
  }

  // Normal practice is 3D; explicit planar automation remains available.
  for (const char *value : {static_cast<const char *>(nullptr), "", "jolt-planar"}) {
#ifdef _WIN32
    _putenv_s("FRC_SIM_PHYSICS", value ? value : "");
#else
    if (value) setenv("FRC_SIM_PHYSICS", value, 1);
    else unsetenv("FRC_SIM_PHYSICS");
#endif
    Check(BackendFromEnvironment() ==
          (value && std::string(value) == "jolt-planar" ? Backend::Planar : Backend::Full3D));
  }
  for (int repeat = 0; repeat < 2; ++repeat) {
    SimulationClock clock(Backend::Planar);
    auto *w = clock.World();
    BodySpec s;
    s.mass = 2;
    s.planar = true;
    s.pose.position.z = 1;
    auto h = w->CreateBody(s);
    int before = 0, after = 0;
    auto r = clock.Register({[&](double dt) {
                               Check(dt == .004);
                               ++before;
                               w->AddForce(h, {10, 0, 0}, {0, 0, 1});
                             },
                             [&](double) { ++after; },
                             {}});
    for (int i = 0; i < 50; ++i)
      clock.StepRobotPeriod();
    auto state = w->ReadBody(h);
    Check(before == 250 && after == 250 && std::abs(clock.Time() - 1) < 1e-12);
    Check(std::abs(state.linearVelocity.x - 5) < .01 &&
          std::abs(state.pose.position.z - 1) < 1e-6);
    clock.Reset();
    Check(w->ReadBody(h).linearVelocity.x == 0 && clock.Time() == 0);
    w->DestroyBody(h);
    bool stale = false;
    try {
      w->ReadBody(h);
    } catch (const std::invalid_argument &) {
      stale = true;
    }
    Check(stale);
    auto next = w->CreateBody(s);
    Check(next != h);
  }
  SimulationClock phoenix;
  int calls = 0;
  {
    auto r = phoenix.Register({[&](double) { ++calls; }, {}, {}});
    phoenix.StepRobotPeriod();
  }
  phoenix.StepRobotPeriod();
  Check(calls == 5 && phoenix.World() == nullptr);
  SimulationWorld world(Backend::Full3D);
  BodySpec floor;
  floor.halfExtents = {5, 5, .1};
  floor.pose.position.z = -.1;
  world.CreateBody(floor);
  BodySpec ball;
  ball.shape = BodySpec::Shape::Sphere;
  ball.mass = 1;
  ball.radius = .1;
  ball.pose.position.z = 1;
  auto h = world.CreateBody(ball);
  for (int i = 0; i < 500; ++i)
    world.Step(.004);
  Check(std::abs(world.ReadBody(h).pose.position.z - .1) < .02);
  Check(world.Raycast({0, 0, 2}, {0, 0, -3}, h).has_value());
  world.SetEnabled(h, false);
  Check(!world.ReadBody(h).enabled);
  world.Reset();
  Check(world.ReadBody(h).enabled);
  // Drag updates must not teleport the body or overwrite its angular motion.
  BodyState spinning{{{1, 2, 3}, {0, 0, std::sqrt(.5), std::sqrt(.5)}},
                     {4, 5, 6}, {1, 2, 3}, true};
  world.SetBodyState(h, spinning);
  const auto beforeVelocity = world.ReadBody(h);
  world.SetLinearVelocity(h, {2, 3, 4});
  const auto afterVelocity = world.ReadBody(h);
  Check(afterVelocity.linearVelocity.x == 2 &&
        afterVelocity.linearVelocity.y == 3 &&
        afterVelocity.linearVelocity.z == 4 &&
        afterVelocity.pose.position.x == beforeVelocity.pose.position.x &&
        afterVelocity.pose.position.y == beforeVelocity.pose.position.y &&
        afterVelocity.pose.position.z == beforeVelocity.pose.position.z &&
        afterVelocity.pose.rotation.x == beforeVelocity.pose.rotation.x &&
        afterVelocity.pose.rotation.y == beforeVelocity.pose.rotation.y &&
        afterVelocity.pose.rotation.z == beforeVelocity.pose.rotation.z &&
        afterVelocity.pose.rotation.w == beforeVelocity.pose.rotation.w &&
        afterVelocity.angularVelocity.x == beforeVelocity.angularVelocity.x &&
        afterVelocity.angularVelocity.y == beforeVelocity.angularVelocity.y &&
        afterVelocity.angularVelocity.z == beforeVelocity.angularVelocity.z);
  // Reset must clear pending forces, and stale registrations must not execute.
  SimulationClock lifecycle(Backend::Planar);
  auto *planar = lifecycle.World();
  BodySpec moving;
  moving.mass = 1;
  moving.planar = true;
  auto robot = planar->CreateBody(moving);
  planar->AddForce(robot, {100, 0, 0}, {});
  lifecycle.Reset();
  planar->Step(.004);
  Check(std::abs(planar->ReadBody(robot).linearVelocity.x) < 1e-9);
  Registration removed;
  int resets = 0;
  auto first = lifecycle.Register({{}, {}, [&] { removed = {}; }});
  removed = lifecycle.Register({{}, {}, [&] { ++resets; }});
  lifecycle.Reset();
  Check(resets == 0);
  Registration survivor;
  {
    SimulationClock temporary;
    survivor = temporary.Register({});
  }
  survivor = {};
  BodySpec wall;
  wall.pose.position.x = 2;
  wall.halfExtents = {.1, 5, 1};
  planar->CreateBody(wall);
  for (int i = 0; i < 500; ++i) {
    planar->AddForce(robot, {20, 0, 0}, planar->ReadBody(robot).pose.position);
    planar->Step(.004);
  }
  Check(planar->ReadBody(robot).pose.position.x < 1.42);
  bool invalid = false;
  moving.inertia = {1, 0, 1};
  try {
    planar->CreateBody(moving);
  } catch (const std::invalid_argument &) {
    invalid = true;
  }
  Check(invalid);
  std::cout << "world contracts passed\n";
}
