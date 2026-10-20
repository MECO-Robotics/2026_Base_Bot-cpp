#include "base/sim/SimulationWorld.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace base::sim;
void Check(bool ok) {
  if (!ok)
    throw std::runtime_error("World contract failed");
}
int main() {
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
  std::cout << "world contracts passed\n";
}
