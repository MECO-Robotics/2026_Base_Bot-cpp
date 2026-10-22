#pragma once
#include <functional>
#include <array>
#include <optional>
#include <memory>
#include <string>
#include <utility>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose3d.h>
namespace base
{
struct PieceRay
{
    units::second_t timestamp;
    double yawDegrees{}, pitchDegrees{}; // Photon: right-positive yaw, up-positive pitch.
};
struct PieceDetectionConfig
{
    std::string name = "piece-camera";
    frc::Transform3d robotToCamera;
    units::meter_t targetHeight{0};
    units::meter_t maxDistance{10};
    units::second_t freshness{.15};
};
// Typed ray acquisition; projection and freshness remain subsystem policy.
class PieceDetectionIO
{
  public:
    virtual ~PieceDetectionIO() = default;
    virtual std::optional<PieceRay> ReadRay() = 0;
};
class InjectedPieceDetectionIO final : public PieceDetectionIO
{
  public:
    using Source = std::function<std::optional<PieceRay>()>;
    explicit InjectedPieceDetectionIO(Source source) : source_(std::move(source))
    {
    }
    std::optional<PieceRay> ReadRay() override
    {
        return source_ ? source_() : std::nullopt;
    }

  private:
    Source source_;
};
std::unique_ptr<PieceDetectionIO> MakePhotonPieceDetectionIO(std::string name);
// Horizontal target-plane intersection; no area-derived range assumption.
class PhotonPieceDetection : public frc2::SubsystemBase
{
  public:
    using Source = std::function<std::optional<PieceRay>()>;
    PhotonPieceDetection(PieceDetectionConfig config, std::function<frc::Pose3d()> robotPose, Source source = {});
    PhotonPieceDetection(std::unique_ptr<PieceDetectionIO> io, PieceDetectionConfig config,
                         std::function<frc::Pose3d()> robotPose);
    void Periodic() override;
    std::optional<frc::Translation3d> Target() const
    {
        return target_;
    }

  private:
    PieceDetectionConfig config_;
    std::function<frc::Pose3d()> robotPose_;
    std::unique_ptr<PieceDetectionIO> io_;
    std::optional<frc::Translation3d> target_;
    std::optional<PieceRay> lastRay_;
    std::optional<std::array<double, 3>> projectedRay_;
};
} // namespace base
