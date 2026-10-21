#include "base/Drive.h"
#include <gtest/gtest.h>
#include <limits>

TEST(DriveConfig, RejectsUnsafeConfigurationBeforeDeviceConstruction) {
  base::DriveConfig good;
  EXPECT_NO_THROW(good.Validate());
  // Different CAN device types may use the same numeric address.
  good.modules[0].encoder=good.modules[0].drive;
  good.pigeon=good.modules[0].drive;
  good.bus=""; // Phoenix default-bus alias is supported.
  EXPECT_NO_THROW(good.Validate());
  for(int fault=0;fault<6;++fault) {
    base::DriveConfig bad;
    switch(fault) {
      case 0:bad.driveRatio=0;break;
      case 1:bad.simulation.suspensionDamping=std::numeric_limits<double>::quiet_NaN();break;
      case 2:bad.modules[1].steer=bad.modules[0].drive;break;
      case 3:bad.modules[1].encoder=bad.modules[0].encoder;break;
      case 4:for(auto& m:bad.modules)m.yMeters=0;break;
      case 5:bad.pigeon=63;break;
    }
    EXPECT_THROW(base::Drive{bad},std::invalid_argument)<<fault;
  }
}
