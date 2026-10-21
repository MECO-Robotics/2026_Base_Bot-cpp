#include "base/Mechanism.h"
#include <frc/simulation/DriverStationSim.h>
#include <frc/simulation/SimDeviceSim.h>
#include <hal/HAL.h>
#include <gtest/gtest.h>

TEST(SparkAdapter, DefaultRealAdapterUsesOnboardPhysicalSetpoints) {
  HAL_Initialize(500,0);
  frc::sim::DriverStationSim::SetEnabled(false);
  frc::sim::DriverStationSim::NotifyNewData();
  base::MotorConfig config{{51},{true},7,35,"rio"};
  auto io=base::MakeMotorIO(base::Mode::Real,base::Vendor::SparkMax,config);
  base::Gains gains; gains.minimum=-2; gains.maximum=3;
  // Real adapters cannot silently select a software PID, even with the legacy
  // Phoenix option absent. Scale .5 represents metres per output revolution.
  ASSERT_TRUE(io->ConfigureControl({gains,.5,base::Gravity::Constant,true,false}));
  frc::sim::SimDeviceSim device{"SPARK MAX [51]"};
  ASSERT_NE(static_cast<HAL_SimDeviceHandle>(device),0);
  base::NativeControlRequest request{base::NativeControlRequest::Kind::Position,1.25,1,1,0,0};
  ASSERT_TRUE(io->SetNative(request));
  EXPECT_DOUBLE_EQ(device.GetDouble("Setpoint").Get(),2.5);
  request.kind=base::NativeControlRequest::Kind::Velocity; request.target=.75;
  ASSERT_TRUE(io->SetNative(request));
  EXPECT_DOUBLE_EQ(device.GetDouble("Setpoint").Get(),1.5);
  io->SetVoltage(0);
  EXPECT_THROW(io->ConfigureControl({gains,.5,base::Gravity::Constant,true,true}),std::invalid_argument);
  frc::sim::DriverStationSim::SetEnabled(true);
  frc::sim::DriverStationSim::NotifyNewData();
  EXPECT_THROW(io->ConfigureControl({gains,.5,base::Gravity::Constant,true,false}),std::logic_error);
  frc::sim::DriverStationSim::SetEnabled(false);
  frc::sim::DriverStationSim::NotifyNewData();
}
