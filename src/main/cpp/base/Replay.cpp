#include "base/Replay.h"
#include "base/FrameRecorder.h"
#include <atomic>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <wpi/DataLog.h>
#include <wpi/DataLogReader.h>
#include <wpi/MemoryBuffer.h>
#include <networktables/NetworkTableInstance.h>
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <thread>
#include <chrono>
#ifndef __FRC_ROBORIO__
#include <frc/simulation/DriverStationSim.h>
#include <frc/simulation/SimHooks.h>
#endif
#if __has_include("BuildProvenance.h")
#include "BuildProvenance.h"
#else
#define FRC_BUILD_REVISION "unknown"
#define FRC_BUILD_DIRTY "unknown"
#define FRC_BUILD_SOURCE_HASH "unknown"
#endif
namespace base::replay {
namespace {
using Json=wpi::json;
bool initialized=false,active=false,framed=false;
uint64_t index=0;
size_t cursor=0;
std::atomic<size_t> pendingReplayBytes{0};
Json frame;
std::filesystem::path outputPath;
std::vector<Json> source;
std::shared_ptr<wpi::log::StringLogEntry> frames,metadata,recordingFault;
std::unique_ptr<FrameRecorder> recorder;
FrameRecorder::Stats finalRecordingStats;
[[noreturn]] void Fail(const std::string &message){throw std::runtime_error("Replay: "+message);}
void Validate(const Json &j) {
  if(j.is_number_float()&&!std::isfinite(j.get<double>()))Fail("nonfinite number");
  if(j.is_structured())for(const auto &v:j)Validate(v);
}
Json CaptureDS() {
  Json sticks=Json::array();
  for(int stick=0;stick<6;++stick){
    Json axes=Json::array(),povs=Json::array();
    for(int i=0;i<frc::DriverStation::GetStickAxisCount(stick);++i)axes.push_back(frc::DriverStation::GetStickAxis(stick,i));
    for(int i=0;i<frc::DriverStation::GetStickPOVCount(stick);++i)povs.push_back(frc::DriverStation::GetStickPOV(stick,i));
    sticks.push_back({{"axes",axes},{"povs",povs},{"buttons",frc::DriverStation::GetStickButtons(stick)},{"buttonCount",frc::DriverStation::GetStickButtonCount(stick)},
      {"xbox",frc::DriverStation::GetJoystickIsXbox(stick)},{"type",frc::DriverStation::GetJoystickType(stick)},{"name",frc::DriverStation::GetJoystickName(stick)}});
  }
  auto alliance=frc::DriverStation::GetAlliance();
  return {{"enabled",frc::DriverStation::IsEnabled()},{"autonomous",frc::DriverStation::IsAutonomous()},{"test",frc::DriverStation::IsTest()},
    {"estop",frc::DriverStation::IsEStopped()},{"dsAttached",frc::DriverStation::IsDSAttached()},{"fmsAttached",frc::DriverStation::IsFMSAttached()},
    {"alliance",alliance?(*alliance==frc::DriverStation::Alliance::kRed?"red":"blue"):"unknown"},
    {"location",frc::DriverStation::GetLocation().value_or(1)},{"joysticks",sticks}};
}
#ifndef __FRC_ROBORIO__
void InstallDS(const Json &ds) {
  using D=frc::sim::DriverStationSim;
  D::SetEnabled(ds.at("enabled").get<bool>());D::SetAutonomous(ds.at("autonomous").get<bool>());
  D::SetTest(ds.at("test").get<bool>());D::SetEStop(ds.at("estop").get<bool>());
  D::SetDsAttached(ds.at("dsAttached").get<bool>());D::SetFmsAttached(ds.at("fmsAttached").get<bool>());
  int location=ds.at("location").get<int>();if(location<1||location>3)Fail("invalid alliance location");
  auto alliance=ds.at("alliance").get<std::string>();
  D::SetAllianceStationId(alliance=="red"?static_cast<HAL_AllianceStationID>(HAL_AllianceStationID_kRed1+location-1):alliance=="blue"?static_cast<HAL_AllianceStationID>(HAL_AllianceStationID_kBlue1+location-1):HAL_AllianceStationID_kUnknown);
  const auto &sticks=ds.at("joysticks");if(sticks.size()!=6)Fail("expected six joystick ports");
  for(int stick=0;stick<6;++stick){const auto &s=sticks.at(stick);const auto &axes=s.at("axes"),&povs=s.at("povs");
    if(axes.size()>12||povs.size()>12)Fail("joystick dimension out of range");
    D::SetJoystickAxisCount(stick,axes.size());D::SetJoystickPOVCount(stick,povs.size());
    for(size_t i=0;i<axes.size();++i)D::SetJoystickAxis(stick,i,axes.at(i).get<double>());
    for(size_t i=0;i<povs.size();++i)D::SetJoystickPOV(stick,i,povs.at(i).get<int>());
    D::SetJoystickButtons(stick,s.at("buttons").get<uint32_t>());D::SetJoystickButtonCount(stick,s.at("buttonCount").get<int>());
    D::SetJoystickIsXbox(stick,s.at("xbox").get<bool>());D::SetJoystickType(stick,s.at("type").get<int>());D::SetJoystickName(stick,s.at("name").get<std::string>());
  }
  D::NotifyNewData();frc::DriverStation::RefreshData();
}
#endif
}
void InitializeFromEnvironment() {
  if(initialized)return;
  initialized=true;
  const char *input=std::getenv("FRC_REPLAY_LOG");active=input&&*input;
  Json provenance={{"schemaVersion",1},{"buildRevision",FRC_BUILD_REVISION},{"buildDirty",std::string_view(FRC_BUILD_DIRTY)=="true"},{"buildSourceHash",FRC_BUILD_SOURCE_HASH},{"mode",active?"replay":"live"}};
  if(active){
    // RobotBase may start the default NT server before our member initialization.
    // Replay uses only process-local tables; remote chooser/tuning writes must
    // never change the recorded-input run or contend with a live simulator.
    auto nt=nt::NetworkTableInstance::GetDefault();
    nt.StopServer();nt.StopClient();nt.StartLocal();
#ifdef __FRC_ROBORIO__
    Fail("input replay is desktop-only");
#else
    auto buffer=wpi::MemoryBuffer::GetFile(input);if(!buffer)Fail("cannot read source log");
    uint64_t hash=14695981039346656037ull;for(unsigned char c:(*buffer)->GetBuffer()){hash^=c;hash*=1099511628211ull;}
    wpi::log::DataLogReader reader(std::move(*buffer));if(!reader.IsValid())Fail("invalid WPILOG");
    std::map<int,std::string> entries;int64_t last=-1;uint64_t sequence=0;Json recordingStatus;
    for(const auto &record:reader){
      if(record.IsStart()){wpi::log::StartRecordData start;if(record.GetStartData(&start))entries[start.entry]=std::string(start.name);continue;}
      const auto &name=entries[record.GetEntry()];std::string_view value;
      if(name=="Replay/RecordingFault")Fail("source recording reported overflow or serialization failure");
      if(name=="Replay/RecordingStatus"&&record.GetString(&value))recordingStatus=Json::parse(value);
      if(name=="Replay/Metadata"&&record.GetString(&value))provenance["sourceMetadata"]=Json::parse(value);
      if(name!="Replay/Frames"||!record.GetString(&value))continue;
      Json f=Json::parse(value);Validate(f);
      if(f.at("schemaVersion")!=1)Fail("unsupported frame schema");
      const auto time=f.at("timestampUs").get<int64_t>();if(time<0||time<=last)Fail("frame timestamps must strictly increase");last=time;
      if(f.at("sequence").get<uint64_t>()!=sequence++)Fail("frame sequence is not contiguous");
      if(!f.at("inputs").is_object()||!f.at("outputs").is_object()||!f.at("ds").is_object())Fail("invalid frame structure");
      source.push_back(std::move(f));
    }
    if(source.empty())Fail("source has no replay input frames");
    if(!recordingStatus.is_null()&&(recordingStatus.at("rejected")!=0||recordingStatus.at("failed")!=0||recordingStatus.at("written")!=source.size()))
      Fail("source recording status reports failed or missing frames");
    provenance["sourceFile"]=std::filesystem::absolute(input).string();provenance["sourceFileHashFnv1a64"]=std::to_string(hash);
    if(const char *sha=std::getenv("FRC_REPLAY_SOURCE_SHA256")) {
      const std::string digest(sha);
      if(digest.size()!=64||digest.find_first_not_of("0123456789abcdefABCDEF")!=std::string::npos)Fail("invalid source SHA256 metadata");
      provenance["sourceFileSha256"]=digest;
    }
    const char *output=std::getenv("FRC_REPLAY_OUTPUT");if(!output||!*output)Fail("FRC_REPLAY_OUTPUT must name a new WPILOG file");
    auto path=std::filesystem::absolute(output);outputPath=path;
    if(std::filesystem::exists(path))Fail("output already exists; refusing overwrite");
    std::filesystem::create_directories(path.parent_path());
    frc::DataLogManager::Start(path.parent_path().string(),path.filename().string(),.01);
    std::cout<<"[Replay] Loaded "<<source.size()<<" input frames; new outputs: "<<path<<std::endl;
#endif
  } else frc::DataLogManager::Start();
  auto &log=frc::DataLogManager::GetLog();
  frames=std::make_shared<wpi::log::StringLogEntry>(log,"Replay/Frames");
  recordingFault=std::make_shared<wpi::log::StringLogEntry>(log,"Replay/RecordingFault");
  recorder=std::make_unique<FrameRecorder>([entry=frames](std::string_view payload,int64_t timestamp) {
    entry->Append(payload,timestamp);
    if(active)pendingReplayBytes.fetch_add(payload.size(),std::memory_order_relaxed);
  },[entry=recordingFault](FrameRecorder::Stats status) {
    entry->Append(Json{{"rejected",status.rejected},{"failed",status.failed}}.dump());
  });
  metadata=std::make_shared<wpi::log::StringLogEntry>(log,"Replay/Metadata");metadata->Append(provenance.dump());
}
bool Active(){return active;}
bool FrameActive(){return framed;}
uint64_t FrameIndex(){return index;}
void BeginFrame(){
  if(framed)return;
  if(!initialized)InitializeFromEnvironment();
  if(active){if(cursor>=source.size())Fail("frame beyond EOF");frame=source.at(cursor);frame["outputs"]=Json::object();index=frame.at("sequence").get<uint64_t>();}
  else frame={{"schemaVersion",1},{"sequence",index},{"timestampUs",frc::RobotController::GetFPGATime()},{"ds",CaptureDS()},{"inputs",Json::object()},{"outputs",Json::object()}};
  framed=true;
}
Json ProcessInputs(std::string_view key,std::function<Json()> live){
  if(!framed){if(!active)return live();BeginFrame();}
  auto name=std::string(key);
  if(!frame["inputs"].contains(name)){if(active)Fail("missing required inputs: "+name);auto value=live();Validate(value);frame["inputs"][name]=std::move(value);}
  return frame["inputs"].at(name);
}
void RecordOutput(std::string_view key,const Json &value){if(!framed)return;Validate(value);frame["outputs"][std::string(key)]=value;}
void EndFrame(){
  if(!framed)return;
  const auto timestamp=frame.at("timestampUs").get<int64_t>();
  recorder->Submit(std::move(frame),timestamp);
  framed=false;if(active)++cursor;else ++index;
}
namespace {
void PersistMarker(std::string_view entryName){
  if(recorder)recorder->Flush();
  const auto status=recorder?recorder->Statistics():finalRecordingStats;
  if(status.rejected||status.failed)Fail("recording worker rejected or failed a frame");
  const std::string marker="replay-persisted-"+std::to_string(cursor)+"-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  wpi::log::StringLogEntry completion(frc::DataLogManager::GetLog(),entryName);completion.Append(marker);
  frc::DataLogManager::GetLog().Flush(); // Nonblocking background-writer request.
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
  do {
    std::ifstream file(outputPath,std::ios::binary);
    if(file){file.seekg(0,std::ios::end);const auto end=file.tellg();file.seekg(std::max<std::streamoff>(0,static_cast<std::streamoff>(end)-65536));std::string tail((std::istreambuf_iterator<char>(file)),{});if(tail.find(marker)!=std::string::npos){pendingReplayBytes=0;return;}}
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  } while(std::chrono::steady_clock::now()<deadline);
  Fail("output writer did not persist replay marker within five seconds");
}
}
void FlushOutput(){if(active)PersistMarker("Replay/Complete");}
void ShutdownRecording(){
  if(!recorder)return;
  recorder->Stop();
  const auto status=recorder->Statistics();
  finalRecordingStats=status;
  wpi::log::StringLogEntry summary(frc::DataLogManager::GetLog(),"Replay/RecordingStatus");
  summary.Append(Json{{"written",status.written},{"rejected",status.rejected},
      {"failed",status.failed},{"highWater",status.highWater}}.dump());
  recorder.reset();
  frc::DataLogManager::GetLog().Flush();
}
void RunReplay(const std::function<void()> &robotInit,const std::function<void()> &loop){
#ifndef __FRC_ROBORIO__
  if(!active)Fail("runner requires replay input");
  frc::sim::PauseTiming();frc::sim::RestartTiming();robotInit();
  int64_t last=0;
  while(cursor<source.size()){
    auto now=source.at(cursor).at("timestampUs").get<int64_t>();
    // TimedRobot's notifier exists but its normal notifier loop is bypassed.
    // Synchronous StepTiming waits for that unconsumed notifier indefinitely;
    // this runner owns callback ordering and invokes LoopFunc directly below.
    frc::sim::StepTimingAsync(units::second_t{(now-last)*1e-6});last=now;
    BeginFrame();InstallDS(frame.at("ds"));loop();EndFrame();
    // Offline replay can generate many seconds of data before the writer's
    // wall-clock flush period elapses. Apply backpressure BETWEEN callbacks;
    // live 20 ms controls never wait for disk and no replay frames are coalesced.
    if(cursor%32==0||pendingReplayBytes.load(std::memory_order_relaxed)>=65536)PersistMarker("Replay/Progress");
  }
  std::cout<<"[Replay] Completed "<<cursor<<" frames"<<std::endl;
#else
  Fail("desktop-only runner");
#endif
}
}
