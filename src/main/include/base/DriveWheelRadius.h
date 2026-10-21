#pragma once
#include <array>
namespace base {
// Pure commissioning calculation; wheel angles are coupling-corrected radians.
class WheelRadiusMeasurement {
 public:
  explicit WheelRadiusMeasurement(std::array<double,4> moduleDistances);
  bool Add(double timestamp, double rawYaw, const std::array<double,4>& angles, bool healthy);
  double YawTravel() const { return yaw_; }
  bool Valid() const;
  std::array<double,4> Radii() const;
 private:
  std::array<double,4> distances_, initial_{}, current_{};
  double time_=0, lastYaw_=0, yaw_=0;
  bool started_=false, failed_=false;
};
}
