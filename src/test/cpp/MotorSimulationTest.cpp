#include "base/Mechanism.h"
#include <cmath>
#include <numbers>
#include <gtest/gtest.h>

TEST(MotorSimulation, HighReductionConvergesWithoutNumericalReversal) {
  // No-load NEO speed at 4 V, including the model's output viscous drag.
  constexpr double ratio=200, inertia=.025, voltage=4;
  constexpr double resistance=12./105, kt=2.6/105;
  constexpr double kv=5676*2*std::numbers::pi/60/12;
  const double expected=voltage/(ratio/kv+.0001*resistance/(kt*ratio))/(2*std::numbers::pi);
  auto io=base::MakeMotorIO(base::Mode::Sim,base::Vendor::SparkMax,
                          base::MotorConfig{{1},{false},ratio,40,""},inertia);
  io->SetVoltage(voltage);
  base::MotorInputs state;
  for(int n=0;n<50;++n) {
    state=io->Read();
    EXPECT_GE(state.velocity,0);
    EXPECT_LE(state.velocity,expected+1e-9);
    EXPECT_LE(state.current,40+1e-9);
  }
  EXPECT_NEAR(state.velocity,expected,1e-6);
  io->SetVoltage(-voltage);
  for(int n=0;n<50;++n) state=io->Read();
  EXPECT_NEAR(state.velocity,-expected,1e-6);
  io->SetBrake(false); io->SetVoltage(0);
  const double coastSpeed=std::abs(io->Read().velocity);
  io->SetBrake(true);
  EXPECT_LT(std::abs(io->Read().velocity),coastSpeed*.01);
}

TEST(MotorSimulation, CurrentLimitBoundsStartupAcceleration) {
  constexpr double inertia=.025, ratio=2, kt=2.6/105;
  auto io=base::MakeMotorIO(base::Mode::Sim,base::Vendor::SparkMax,
                          base::MotorConfig{{1,2},{false,true},ratio,10,""},inertia);
  io->SetVoltage(12);
  const auto state=io->Read();
  const double acceleration=kt*10*2*ratio/inertia;
  EXPECT_GT(state.velocity,0);
  EXPECT_LE(state.velocity,acceleration*.02/(2*std::numbers::pi));
  EXPECT_LE(state.position,.5*acceleration*.02*.02/(2*std::numbers::pi));
  EXPECT_LE(state.current,20+1e-9);
}
