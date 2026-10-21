#include "base/AdvantageScope.h"
#include "base/Telemetry.h"
#include <WPILibVersion.h>
#include <networktables/NetworkTable.h>
namespace base {
void PublishTelemetryMetadata(std::string_view project, nt::NetworkTableInstance instance) {
  telemetry::Post("Metadata", [project=std::string(project),instance] {
    auto table=instance.GetTable("Metadata");
    table->PutString("Project",project);
    table->PutString("Language","C++");
    table->PutString("WPILib",GetWPILibVersion());
    table->PutString("Telemetry","WPILib structs / NT4 / WPILOG");
  });
}
}
