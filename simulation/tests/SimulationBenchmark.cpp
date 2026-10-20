#include "base/sim/Arena.h"
#include "base/sim/DrivePhysics.h"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <vector>
#include <string>

// Measures the complete before -> Jolt -> after interval, including arena work.
int main() {
  using namespace base::sim;
  std::cout << "[\n";
  bool first = true;
  for (auto mode : {Backend::Planar, Backend::Full3D}) {
    for (auto scene : {"empty", "typical", "dense"}) {
      SimulationClock clock(mode);
      std::chrono::steady_clock::time_point start;
      auto startMonitor = clock.Register({[&](double) { start = std::chrono::steady_clock::now(); }, {}, {}});
      DrivePhysics drive(*clock.World());
      auto driveRegistration = clock.Register({
          [&](double dt) { drive.BeforePhysics(dt); },
          [&](double dt) { drive.AfterPhysics(dt); }, {}});
      Arena arena(clock, [&] { return drive.Truth().body; }, std::string(scene) != "empty");
      if (std::string(scene) == "empty") {
        BodySpec floor;
        floor.halfExtents = {100, 100, .1};
        floor.pose.position = {0, 0, -.1};
        arena.AddCollider(floor);
      }
      drive.Reset({{0, 0, .3}, {}});
      if (std::string(scene) == "dense") {
        for (int i = 0; i < 300; ++i) {
          PieceSpec piece;
          piece.pose.position = {4 + .20 * (i % 10), -1 + .20 * ((i / 10) % 10), .12 + .22 * (i / 100)};
          piece.airborne = mode == Backend::Full3D;
          arena.Spawn(piece);
        }
      }
      for (int i = 0; i < 4; ++i) drive.SetModuleVoltages(i, 3, 0);
      std::vector<double> samples;
      uint32_t peakManifolds = 0;
      uint64_t misses = 0;
      auto monitor = clock.Register({{},
          [&](double) {
            double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            samples.push_back(seconds);
            if (seconds > .004) ++misses;
            peakManifolds = std::max(peakManifolds, clock.World()->ContactManifoldCount());
          }, {}});
      for (int i = 0; i < 100; ++i) clock.StepRobotPeriod();
      samples.clear(); misses = 0; peakManifolds = 0;
      for (int i = 0; i < 1000; ++i) clock.StepRobotPeriod();
      std::sort(samples.begin(), samples.end());
      if (!first) std::cout << ",\n";
      first = false;
      std::cout << "{\"mode\":\"" << (mode == Backend::Planar ? "jolt-planar" : "jolt-3d")
                << "\",\"scene\":\"" << scene << "\",\"bodies\":" << clock.World()->BodyCount()
                << ",\"peakContactManifolds\":" << peakManifolds
                << ",\"samples\":" << samples.size() << ",\"medianSeconds\":" << samples[samples.size()/2]
                << ",\"p99Seconds\":" << samples[samples.size()*99/100]
                << ",\"deadlineMisses\":" << misses << "}";
    }
  }
  std::cout << "\n]\n";
}
