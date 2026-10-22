#include "base/MechanismTuning.h"
#include "base/Replay.h"
#include "base/Telemetry.h"
#include <frc/DriverStation.h>
#include <frc2/command/FunctionalCommand.h>
#include <networktables/NetworkTableInstance.h>
#include <wpi/json.h>

namespace base
{
namespace
{
Gains Decode(const wpi::json &j)
{
    if (!j.is_object() || j.size() != 13)
        throw std::invalid_argument("Expected complete gains object");
    return {j.at("p").get<double>(),
            j.at("i").get<double>(),
            j.at("d").get<double>(),
            j.at("s").get<double>(),
            j.at("v").get<double>(),
            j.at("a").get<double>(),
            j.at("gravity").get<double>(),
            j.at("maxVelocity").get<double>(),
            j.at("maxAcceleration").get<double>(),
            j.at("tolerance").get<double>(),
            j.at("minimum").get<double>(),
            j.at("maximum").get<double>(),
            j.at("defaultPosition").get<double>()};
}
} // namespace
MechanismTuning::MechanismTuning(Mechanism &mechanism) : mechanism_(mechanism), path_("/Tuning/" + mechanism.GetName())
{
    SetName(mechanism.GetName() + "Tuning");
    if (!replay::Active())
    {
        auto nt = nt::NetworkTableInstance::GetDefault();
        subscriber_ = nt.GetStringTopic(path_ + "/Request")
                          .Subscribe("", {.pollStorage = 16, .periodic = .02, .sendAll = true, .keepDuplicates = true});
        publisher_ = std::make_shared<nt::StringPublisher>(nt.GetStringTopic(path_ + "/Status").Publish());
    }
}
void MechanismTuning::Status(std::string status, std::optional<uint64_t> revision)
{
    status_ = std::move(status);
    const auto output = wpi::json{{"revision", revision.value_or(revision_)}, {"status", status_}};
    replay::RecordOutput(path_ + "/Status", output);
    if (publisher_)
        telemetry::Post(path_, [publisher = publisher_, output] { publisher->Set(output.dump()); });
}
void MechanismTuning::Process(const std::string &request)
{
    try
    {
        if (request.size() > 8192)
            throw std::invalid_argument("Transaction exceeds 8 KiB");
        const auto j = wpi::json::parse(request);
        if (!j.is_object() || !j.contains("revision") || !j.at("revision").is_number_unsigned())
            throw std::invalid_argument("Expected unsigned revision");
        const auto next = j.at("revision").get<uint64_t>();
        if (next <= revision_)
        {
            Status("rejected: stale revision");
            return;
        }
        // Consume rejected revisions as well: an unsafe request can never apply later.
        revision_ = next;
        if (j.size() > 3 || (!j.contains("gains") && !j.contains("setpoint")))
            throw std::invalid_argument("Expected gains or setpoint");
        for (auto it = j.begin(); it != j.end(); ++it)
            if (it.key() != "revision" && it.key() != "gains" && it.key() != "setpoint")
                throw std::invalid_argument("Unknown transaction field");
        std::optional<Gains> gains;
        if (j.contains("gains"))
        {
            gains = Decode(j.at("gains"));
            gains->Validate(mechanism_.IsPositionControlled());
        }
        std::optional<double> setpoint;
        if (j.contains("setpoint"))
        {
            setpoint = j.at("setpoint").get<double>();
            if (!std::isfinite(*setpoint))
                throw std::invalid_argument("Nonfinite setpoint");
            const auto &limits = gains ? *gains : mechanism_.GetGains();
            if (mechanism_.IsPositionControlled() && (*setpoint < limits.minimum || *setpoint > limits.maximum))
                throw std::invalid_argument("Setpoint outside limits");
            if (!mechanism_.IsPositionControlled() && std::abs(*setpoint) > limits.maxVelocity)
                throw std::invalid_argument("Setpoint exceeds maximum velocity");
        }
        if (gains && (frc::DriverStation::IsEnabled() || mechanism_.GetCurrentCommand() != nullptr))
        {
            Status("rejected: gains require disabled and unowned mechanism");
            return;
        }
        if (setpoint && (!tuning_ || !frc::DriverStation::IsTestEnabled()))
        {
            Status("rejected: setpoint requires TestCommand in enabled Test mode");
            return;
        }
        if (gains)
        {
            mechanism_.SetGains(*gains);
            configuring_ = true;
            configurationRevision_ = revision_;
        }
        pending_ = setpoint;
        Status("accepted");
    }
    catch (const std::exception &)
    {
        Status("rejected: invalid transaction");
    }
}
void MechanismTuning::Periodic()
{
    if (configuring_)
    {
        const auto state = mechanism_.ConfigurationState();
        Status(state == MotorConfigurationState::Pending ? "pending"
               : state == MotorConfigurationState::Fault ? "fault"
                                                         : "applied",
               configurationRevision_);
        configuring_ = state == MotorConfigurationState::Pending;
    }
    const auto input = replay::ProcessInputs(path_ + "/Requests", [this] {
        wpi::json requests = wpi::json::array();
        for (auto &&sample : subscriber_.ReadQueue())
            requests.push_back(sample.value);
        return wpi::json{{"requests", requests}};
    });
    for (const auto &request : input.at("requests"))
        Process(request.get<std::string>());
}
frc2::CommandPtr MechanismTuning::TestCommand()
{
    return frc2::FunctionalCommand(
               [this] {
                   // Discard packets already queued before this command acquired ownership.
                   // Record this drain separately so replay consumes the same input boundary.
                   replay::ProcessInputs(path_ + "/DiscardedOnStart", [this] {
                       wpi::json discarded = wpi::json::array();
                       for (auto &&sample : subscriber_.ReadQueue())
                           discarded.push_back(sample.value);
                       return wpi::json{{"requests", discarded}};
                   });
                   pending_.reset();
                   tuning_ = frc::DriverStation::IsTestEnabled();
                   mechanism_.Stop();
               },
               [this] {
                   if (!frc::DriverStation::IsTestEnabled())
                   {
                       pending_.reset();
                       mechanism_.Stop();
                       return;
                   }
                   if (pending_)
                   {
                       if (mechanism_.IsPositionControlled())
                           mechanism_.SetPosition(*pending_);
                       else
                           mechanism_.SetVelocity(*pending_);
                       pending_.reset();
                   }
               },
               [this](bool) {
                   tuning_ = false;
                   pending_.reset();
                   mechanism_.Stop();
               },
               [] { return !frc::DriverStation::IsTestEnabled(); }, {&mechanism_})
        .ToPtr();
}
} // namespace base
