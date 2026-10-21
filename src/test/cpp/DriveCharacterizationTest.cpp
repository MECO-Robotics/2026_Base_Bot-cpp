#include "base/Drive.h"
#include "base/Replay.h"
#include <gtest/gtest.h>
#include <frc/DataLogManager.h>
#include <frc/TimedRobot.h>
#include <wpi/DataLogWriter.h>
#include <wpi/DataLogReader.h>
#include <wpi/MemoryBuffer.h>
#include <filesystem>
#include <chrono>
#include <cstdlib>
#include <map>
namespace {
void Environment(const char* key,const std::string& value) {
#ifdef _WIN32
  _putenv_s(key,value.c_str());
#else
  if(value.empty())unsetenv(key);else setenv(key,value.c_str(),1);
#endif
}
std::string SavedValue(const char* key){const auto* value=std::getenv(key);return value?value:"";}
}
TEST(DriveCharacterization, OwnsDriveRecordsMeasuredSIAndZerosOnInterruption) {
  if(std::getenv("FRC_DRIVE_SYSID_CHILD")) {
    frc::TimedRobot notifier;
    base::replay::InitializeFromEnvironment();
    base::Drive drive;
    auto command=drive.Characterize(true,frc2::sysid::Direction::kForward);
    EXPECT_TRUE(command.get()->GetRequirements().contains(&drive));
    base::replay::RunReplay([]{},[&]{
      if(base::replay::FrameIndex()==0)command.get()->Initialize();
      command.get()->Execute();
      if(base::replay::FrameIndex()==2)command.get()->End(true);
    });
    EXPECT_FALSE(drive.SimulationTruth().has_value());
    base::replay::ShutdownRecording();base::replay::FlushOutput();frc::DataLogManager::Stop();return;
  }
  auto directory=std::filesystem::temp_directory_path()/std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  std::filesystem::create_directories(directory);
  auto source=directory/"source.wpilog",output=directory/"result.wpilog";
  wpi::json sticks=wpi::json::array();for(int i=0;i<6;++i)sticks.push_back({{"axes",wpi::json::array()},{"povs",wpi::json::array()},{"buttons",0},{"buttonCount",0},{"xbox",false},{"type",0},{"name",""}});
  wpi::json frame={{"schemaVersion",1},{"sequence",0},{"timestampUs",100000}};
  frame["ds"]={{"enabled",true},{"autonomous",false},{"test",false},{"estop",false},{"dsAttached",true},{"fmsAttached",false},{"alliance","blue"},{"location",1},{"joysticks",sticks}};
  frame["inputs"]["Drive/Characterization"]={{2.5,1.25,.75},{2.6,1.3,.8},{2.7,1.35,.85},{2.8,1.4,.9}};
  frame["inputs"]["Drive"]={{"pose",{0,0,0}},{"speeds",{0,0,0}},{"states",{0,0,0,0,0,0,0,0}}};
  frame["outputs"]=wpi::json::object();
  {std::error_code error;wpi::log::DataLogWriter log(source.string(),error);ASSERT_FALSE(error);wpi::log::StringLogEntry entry(log,"Replay/Frames");for(int i=0;i<3;++i){frame["sequence"]=i;frame["timestampUs"]=100000+i*20000;entry.Append(frame.dump(),100000+i*20000);}log.Flush();}
  auto oldSource=SavedValue("FRC_REPLAY_LOG"),oldOutput=SavedValue("FRC_REPLAY_OUTPUT");
  Environment("FRC_REPLAY_LOG",source.string());Environment("FRC_REPLAY_OUTPUT",output.string());Environment("FRC_DRIVE_SYSID_CHILD","1");
  auto result=std::system(("\""+::testing::internal::GetArgvs()[0]+"\" --gtest_filter=DriveCharacterization.*").c_str());
  Environment("FRC_REPLAY_LOG",oldSource);Environment("FRC_REPLAY_OUTPUT",oldOutput);Environment("FRC_DRIVE_SYSID_CHILD","");
  ASSERT_EQ(result,0);
  auto buffer=wpi::MemoryBuffer::GetFile(output.string());ASSERT_TRUE(buffer);
  wpi::log::DataLogReader reader(std::move(*buffer));std::map<int,std::string> names;int frames=0;bool measuredVoltage=false,measuredPosition=false,measuredVelocity=false;
  for(const auto& record:reader) {
    if(record.IsStart()){wpi::log::StartRecordData start;if(record.GetStartData(&start))names[start.entry]=std::string(start.name);continue;}
    auto name=names[record.GetEntry()];
    if(name=="Replay/Frames") {std::string_view text;if(record.GetString(&text)){auto replayed=wpi::json::parse(text);if(frames==0)EXPECT_FALSE(replayed["outputs"].contains("Drive/CharacterizationVoltage"));else EXPECT_DOUBLE_EQ(replayed["outputs"]["Drive/CharacterizationVoltage"].get<double>(),frames==1?7:0);++frames;}}
    double value;
    if(record.GetDouble(&value)) {
      if(name.find("voltage-drive-0")!=std::string::npos){EXPECT_DOUBLE_EQ(value,2.5);measuredVoltage=true;}
      if(name.find("position-drive-0")!=std::string::npos){EXPECT_DOUBLE_EQ(value,1.25);measuredPosition=true;}
      if(name.find("velocity-drive-0")!=std::string::npos){EXPECT_DOUBLE_EQ(value,.75);measuredVelocity=true;}
    }
  }
  EXPECT_EQ(frames,3);EXPECT_TRUE(measuredVoltage);EXPECT_TRUE(measuredPosition);EXPECT_TRUE(measuredVelocity);
  std::filesystem::remove_all(directory);
}
