#pragma once
#include <algorithm>
#include <array>
#include <string_view>
#include <vector>
#include <wpi/DataLog.h>
#include <wpi/DataLogReader.h>
#include "base/DeterministicSimulation.h"

namespace base {
// Versioned native schema; position/rate are OUTPUT rotations and rotations/s,
// before Mechanism's optional metres conversion. WPILib record timestamps are us.
inline constexpr std::string_view kMotorInputSchema=
    R"({"schema":"meco.motorinputs.v1","fields":["position_rot","velocity_rot_s","voltage_V","supplyVoltage_V","current_A","connected_0_1"]})";
inline std::array<double,6> EncodeMotorInputs(const MotorInputs& input) {
  return {input.position,input.velocity,input.voltage,input.supplyVoltage,input.current,input.connected ? 1. : 0.};
}
// Nonfinite sensor readings are retained to reproduce the control safety response.
// Offline only: allocations and sorting are intentionally outside control loops.
// The returned vector must outlive ReplayMotorIO's non-owning span.
inline std::vector<MotorReplaySample> ReadMotorInputLog(const wpi::log::DataLogReader& log,
                                                      std::string_view entryName) {
  if(!log.IsValid()) throw std::invalid_argument("Invalid native input log");
  std::vector<MotorReplaySample> samples;
  int entry=-1;
  for(const auto& record:log) {
    if(record.IsStart()) {
      wpi::log::StartRecordData start;
      if(record.GetStartData(&start) && start.name==entryName) {
        if(start.type!="double[]" || start.metadata!=kMotorInputSchema)
          throw std::invalid_argument("Unsupported motor input schema");
        entry=start.entry;
      } else if(record.GetStartData(&start) && start.entry==entry) entry=-1;
    } else if(record.GetEntry()==entry) {
      std::vector<double> values;
      if(!record.GetDoubleArray(&values) || values.size()!=6 ||
          (values[5]!=0 && values[5]!=1) || record.GetTimestamp()<0)
        throw std::invalid_argument("Malformed motor input record");
      samples.push_back({record.GetTimestamp(),{values[0],values[1],values[2],values[3],values[4],values[5]!=0}});
    }
  }
  std::stable_sort(samples.begin(),samples.end(),[](const auto& a,const auto& b){return a.timestampMicroseconds<b.timestampMicroseconds;});
  return samples;
}
}
