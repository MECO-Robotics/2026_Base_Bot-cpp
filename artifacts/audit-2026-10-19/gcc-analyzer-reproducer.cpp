#include "base/Mechanism.h"
#include <frc2/command/Commands.h>
frc2::CommandPtr MakeCommand(base::Mechanism* mechanism, std::function<double()> supplier) {
  return frc2::cmd::RunEnd([mechanism,supplier]{mechanism->SetVelocity(supplier());},
                           [mechanism]{mechanism->Stop();},{mechanism});
}
