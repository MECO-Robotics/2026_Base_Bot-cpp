#include "base/PieceDetection.h"
#include <photon/PhotonCamera.h>
#include "base/Replay.h"
#include <frc/RobotController.h>
#include <cmath>
#include <numbers>
#include <stdexcept>
namespace base
{
namespace
{
void Validate(const PieceDetectionConfig &config)
{
    if (config.name.empty() || !std::isfinite(config.targetHeight.value()) ||
        !std::isfinite(config.maxDistance.value()) || config.maxDistance <= units::meter_t{0} ||
        !std::isfinite(config.freshness.value()) || config.freshness <= units::second_t{0})
        throw std::invalid_argument("Invalid piece detection configuration");
    const auto q = config.robotToCamera.Rotation().GetQuaternion();
    for (double v : {config.robotToCamera.X().value(), config.robotToCamera.Y().value(),
                     config.robotToCamera.Z().value(), q.W(), q.X(), q.Y(), q.Z()})
        if (!std::isfinite(v))
            throw std::invalid_argument("Invalid piece camera mount");
}
class PhotonPieceIO final : public PieceDetectionIO
{
  public:
    explicit PhotonPieceIO(std::string name)
    {
        if (!replay::Active())
            camera_ = std::make_unique<photon::PhotonCamera>(name);
    }
    std::optional<PieceRay> ReadRay() override
    {
        for (const auto &result : camera_->GetAllUnreadResults())
        {
            ray_.reset();
            if (result.HasTargets())
            {
                const auto &target = result.GetBestTarget();
                ray_ = PieceRay{result.GetTimestamp(), target.GetYaw(), target.GetPitch()};
            }
        }
        return ray_;
    }

  private:
    std::unique_ptr<photon::PhotonCamera> camera_;
    std::optional<PieceRay> ray_;
};
} // namespace
std::unique_ptr<PieceDetectionIO> MakePhotonPieceDetectionIO(std::string name)
{
    return std::make_unique<PhotonPieceIO>(std::move(name));
}
namespace
{
std::unique_ptr<PieceDetectionIO> SelectIO(const PieceDetectionConfig &config, PhotonPieceDetection::Source source,
                                           const std::function<frc::Pose3d()> &robotPose)
{
    Validate(config);
    if (!robotPose)
        throw std::invalid_argument("Missing piece robot pose supplier");
    if (source)
        return std::make_unique<InjectedPieceDetectionIO>(std::move(source));
    return MakePhotonPieceDetectionIO(config.name);
}
} // namespace
PhotonPieceDetection::PhotonPieceDetection(PieceDetectionConfig config, std::function<frc::Pose3d()> robotPose,
                                           Source source)
    : PhotonPieceDetection(SelectIO(config, std::move(source), robotPose), config, robotPose)
{
}
PhotonPieceDetection::PhotonPieceDetection(std::unique_ptr<PieceDetectionIO> io, PieceDetectionConfig config,
                                           std::function<frc::Pose3d()> robotPose)
    : config_(std::move(config)), robotPose_(std::move(robotPose)), io_(std::move(io))
{
    Validate(config_);
    if (!robotPose_)
        throw std::invalid_argument("Missing piece robot pose supplier");
    if (!io_)
        throw std::invalid_argument("Null piece detection IO");
}
void PhotonPieceDetection::Periodic()
{
    const auto input = replay::ProcessInputs("PieceDetection/" + config_.name, [this] {
        lastRay_ = io_->ReadRay();
        wpi::json value = {{"now", frc::RobotController::GetFPGATime() * 1e-6}, {"ray", nullptr}};
        if (lastRay_ && std::isfinite(lastRay_->timestamp.value()) && std::isfinite(lastRay_->yawDegrees) &&
            std::isfinite(lastRay_->pitchDegrees))
            value["ray"] = {lastRay_->timestamp.value(), lastRay_->yawDegrees, lastRay_->pitchDegrees};
        return value;
    });
    if (input.at("ray").is_null())
    {
        target_.reset();
        projectedRay_.reset();
        return;
    }
    const auto &r = input.at("ray");
    const double age = input.at("now").get<double>() - r.at(0).get<double>();
    if (age < 0 || age > config_.freshness.value())
    {
        target_.reset();
        projectedRay_.reset();
        return;
    }
    const std::array<double, 3> observation{r.at(0).get<double>(), r.at(1).get<double>(), r.at(2).get<double>()};
    // A retained camera ray is one observation, not a new measurement from the
    // robot's latest pose. Keep its field intersection fixed until replacement.
    if (projectedRay_ && *projectedRay_ == observation)
        return;
    projectedRay_ = observation;
    target_.reset();
    const double yaw = -r.at(1).get<double>() * std::numbers::pi / 180;
    const double pitch = r.at(2).get<double>() * std::numbers::pi / 180;
    const auto camera = robotPose_().TransformBy(config_.robotToCamera);
    const auto direction = frc::Translation3d{
        units::meter_t{std::cos(pitch) * std::cos(yaw)}, units::meter_t{std::cos(pitch) * std::sin(yaw)},
        units::meter_t{
            std::sin(pitch)}}.RotateBy(camera.Rotation());
    if (!std::isfinite(direction.Z().value()) || std::abs(direction.Z().value()) < 1e-9)
        return;
    const double distance = (config_.targetHeight - camera.Z()).value() / direction.Z().value();
    if (!std::isfinite(distance) || distance <= 0 || distance > config_.maxDistance.value())
        return;
    const auto hit = camera.Translation() + direction * distance;
    if (std::isfinite(hit.X().value()) && std::isfinite(hit.Y().value()) && std::isfinite(hit.Z().value()))
        target_ = hit;
}
} // namespace base
