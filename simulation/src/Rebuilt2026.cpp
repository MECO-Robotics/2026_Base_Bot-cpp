#include "base/sim/Rebuilt2026.h"
#include <cmath>
namespace base::sim {
std::vector<BodySpec> Rebuilt2026Colliders(Backend mode) {
  constexpr double in = .0254;
  std::vector<BodySpec> bodies;
  auto box = [&](Vec3 position, Vec3 half, double pitch = 0) {
    BodySpec body;
    body.pose = {position, {0, std::sin(pitch / 2), 0, std::cos(pitch / 2)}};
    body.halfExtents = half;
    bodies.push_back(body);
  };
  const double l = kRebuiltLength, w = kRebuiltWidth;
  box({l/2,w/2,-.15},{l/2+.2,w/2+.2,.15});
  box({-.1,w/2,.5},{.1,w/2+.2,.5});
  box({l+.1,w/2,.5},{.1,w/2+.2,.5});
  box({l/2,-.1,.5},{l/2,.1,.5});
  box({l/2,w+.1,.5},{l/2,.1,.5});
  for (double x : {182.11*in,469.11*in}) {
    box({x,w/2,1},{23.5*in,23.5*in,1});
    // Ground supports beside each trench; the overhead lane stays open.
    for (double y : {56.345*in,261.345*in})
      box({x,y,11.125*in},{23.5*in,6*in,11.125*in});
    if (mode == Backend::Full3D) {
      constexpr double angle = 15 * 3.14159265358979323846 / 180;
      constexpr double halfProjection = 11.1*in;
      constexpr double thickness = .2;
      const double topCenter = 6.513*in - halfProjection*std::tan(angle);
      for (double y : {98.845*in,218.845*in}) {
        // The top faces meet exactly at the crest; lower volume is buried.
        for (double sign : {-1.,1.})
          box({x+sign*halfProjection-thickness*std::sin(sign*angle), y, topCenter-thickness*std::cos(angle)},
              {halfProjection/std::cos(angle),36.5*in,thickness}, sign*angle);
      }
    }
  }
  for (bool red : {false,true}) {
    const double towerX = red ? l-41.755*in : 41.755*in;
    const double towerY = (red ? 170.22 : 147.47)*in;
    for (double sign : {-1.,1.})
      box({towerX,towerY+sign*16.875*in,36.0625*in},
          {1.755*in,.75*in,36.0625*in});
    if (mode == Backend::Full3D) {
      const double centerY = (red ? 82.84 : 234.85)*in;
      const double centerX = red ? l-13.5*in : 13.5*in;
      // Open toward alliance wall: two side rails and downfield front rail.
      for (double sign : {-1.,1.})
        box({centerX,centerY+sign*19.5*in,.5625*in},
            {13.5*in,1.5*in,.5625*in});
      box({red ? l-25.5*in : 25.5*in,centerY,.5625*in},
          {1.5*in,18*in,.5625*in});
    }
  }
  return bodies;
}
}
