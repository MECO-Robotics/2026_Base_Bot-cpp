#include <gtest/gtest.h>
#include <cstdlib>
void Run_AsyncMotorIOTest();
void Run_DeterministicSimulationTest();
void Run_DriveCharacterizationTest();
void Run_DriveConfigTest();
void Run_DriveOwnershipTest();
void Run_DriveSimulationTest();
void Run_DriveTargetTest();
void Run_DriveWheelRadiusCommandTest();
void Run_DriveWheelRadiusTest();
void Run_FrameRecorderTest();
void Run_MechanismTest();
void Run_MechanismTuningReplayTest();
void Run_MechanismTuningTest();
void Run_ModelFeedforwardTest();
void Run_MotorInputLogTest();
void Run_MotorSimulationTest();
void Run_PieceDetectionTest();
void Run_PortableSwerveTest();
void Run_QuestNavTest();
void Run_ReplayTest();
void Run_SparkAdapterTest();
void Run_TelemetryTest();
void Run_VisionIOTest();
void Run_VisionTest();
void Run_CommandLifecycle();
void Run_DriveSystemBoundaryTest();

TEST(Consolidated, AsyncMotorSafety) {
  Run_AsyncMotorIOTest();
}

TEST(Consolidated, CommandLifecycle) {
  Run_CommandLifecycle();
}

TEST(Consolidated, DeterministicSimulation) {
  Run_DeterministicSimulationTest();
}

TEST(Consolidated, DriveCharacterizationAndRadius) {
  if (std::getenv("FRC_DRIVE_SYSID_CHILD") || std::getenv("FRC_DRIVE_SYSID_NATIVE_CHILD")) {
    Run_DriveCharacterizationTest(); return;
  }
  if (std::getenv("FRC_RADIUS_CHILD")) { Run_DriveWheelRadiusCommandTest(); return; }
  Run_DriveCharacterizationTest();
  Run_DriveWheelRadiusTest();
  Run_DriveWheelRadiusCommandTest();
}

TEST(Consolidated, DriveContractsAndOwnership) {
  if (std::getenv("FRC_DRIVE_OWNERSHIP_CHILD")) { Run_DriveOwnershipTest(); return; }
  Run_DriveConfigTest();
  Run_DriveOwnershipTest();
  Run_DriveTargetTest();
  Run_DriveSystemBoundaryTest();
}

TEST(Consolidated, MechanismControlAndTuning) {
  if (std::getenv("FRC_TUNING_REPLAY_CHILD")) { Run_MechanismTuningReplayTest(); return; }
  Run_MechanismTest();
  Run_MechanismTuningTest();
  Run_MechanismTuningReplayTest();
}

TEST(Consolidated, MotorAdaptersAndPlant) {
  Run_MotorSimulationTest();
  Run_ModelFeedforwardTest();
  Run_SparkAdapterTest();
}

TEST(Consolidated, PortableSwerve) {
  Run_PortableSwerveTest();
}

TEST(Consolidated, ReplayAndLogging) {
  if (std::getenv("FRC_REPLAY_TEST_CHILD")) { Run_ReplayTest(); return; }
  Run_ReplayTest();
  Run_MotorInputLogTest();
}

TEST(Consolidated, TelemetryAndRecording) {
  Run_FrameRecorderTest();
  Run_TelemetryTest();
}

TEST(Consolidated, VisionAndLocalization) {
  Run_PieceDetectionTest();
  Run_QuestNavTest();
  Run_VisionTest();
  Run_VisionIOTest();
}

TEST(Consolidated, DriveSimulation) {
  Run_DriveSimulationTest();
}
