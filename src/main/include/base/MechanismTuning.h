#pragma once
#include "base/Mechanism.h"
#include <networktables/StringTopic.h>
#include <optional>

namespace base {
// Opt-in dashboard transaction endpoint. This object and mechanism must outlive
// its command; call Periodic from the normal robot thread (automatic subsystem).
class MechanismTuning final : public frc2::SubsystemBase {
 public:
  explicit MechanismTuning(Mechanism& mechanism);
  void Periodic() override;
  // Exclusively owns the mechanism and runs only while enabled in Test mode.
  // Requested setpoints are never latched across command interruption.
  frc2::CommandPtr TestCommand();
  uint64_t LastRevision() const { return revision_; }
  std::string LastStatus() const { return status_; }
 private:
  Mechanism& mechanism_;
  nt::StringSubscriber subscriber_;
  std::shared_ptr<nt::StringPublisher> publisher_;
  std::string path_,status_="idle";
  uint64_t revision_=0, configurationRevision_=0;
  bool tuning_=false, configuring_=false;
  std::optional<double> pending_;
  void Process(const std::string& request);
  void Status(std::string status, std::optional<uint64_t> revision={});
};
}
