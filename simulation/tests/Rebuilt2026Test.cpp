#include "base/sim/Arena.h"
#include "base/sim/DrivePhysics.h"
#include "base/sim/Rebuilt2026.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>
using namespace base::sim;
void Check(bool ok, std::source_location at = std::source_location::current())
{
    if (!ok)
        throw std::runtime_error("REBUILT field contract line " + std::to_string(at.line()));
}
void SetField(const char *value)
{
#ifdef _WIN32
    _putenv_s("FRC_SIM_FIELD", value);
#else
    setenv("FRC_SIM_FIELD", value, 1);
#endif
}
int main()
{
    SetField("generic");
    Check(ArenaSceneFromEnvironment() == ArenaScene::Generic);
    SetField("rebuilt-2026");
    Check(ArenaSceneFromEnvironment() == ArenaScene::Rebuilt2026);
    SetField("typo");
    bool rejected = false;
    try
    {
        (void)ArenaSceneFromEnvironment();
    }
    catch (const std::invalid_argument &)
    {
        rejected = true;
    }
    Check(rejected);
    constexpr double in = .0254;
    for (auto mode : {Backend::Planar, Backend::Full3D})
    {
        SimulationClock clock(mode);
        auto &w = *clock.World();
        Arena arena(clock, [] { return BodyState{}; }, ArenaScene::Rebuilt2026);
        auto height = [&](double x, double y) {
            auto h = w.Raycast({x, y, 3}, {0, 0, -4});
            Check(h.has_value());
            return h->position.z;
        };
        // Perimeter uses the drawing frame; no leftover generic obstacle/ramp.
        for (double sign : {-1., 1.})
        {
            auto x = w.Raycast({8, 1, .9}, {sign * 20, 0, 0});
            Check(x.has_value());
            Check(std::abs(x->position.x - (sign < 0 ? 0 : kRebuiltLength)) < 1e-5);
            auto y = w.Raycast({8, 4, .9}, {0, sign * 20, 0});
            Check(y.has_value());
            Check(std::abs(y->position.y - (sign < 0 ? 0 : kRebuiltWidth)) < 1e-5);
        }
        Check(std::abs(height(11, 6.5)) < 1e-5);
        Check(std::abs(height(5.7, 6)) < 1e-5);
        // Actual collision query: field corners, hub, both isolated poles and open gap.
        Check(std::abs(height(1, 1)) < 1e-5);
        Check(height(182.11 * in, 158.845 * in) > 1.9);
        for (bool red : {false, true})
        {
            const double x = red ? kRebuiltLength - 41.755 * in : 41.755 * in;
            const double y = (red ? 170.22 : 147.47) * in;
            for (double sign : {-1., 1.})
            {
                Check(height(x, y + sign * 16.875 * in) > 1.8);
                const double oldX = red ? kRebuiltLength - 22.245 * in : 22.245 * in;
                Check(std::abs(height(oldX, y + sign * 16.875 * in)) < 1e-5);
            }
            Check(std::abs(height(x, y)) < 1e-5);
        }
        // Trench ground supports block both modes, but nearby lanes have no overhead collider.
        for (double x : {182.11 * in, 469.11 * in})
        {
            for (double y : {30 * in, kRebuiltWidth - 30 * in})
            {
                Check(std::abs(height(x, y)) < 1e-5);
                auto clear = w.Raycast({x - 1, y, .3}, {2, 0, 0});
                Check(!clear.has_value());
            }
            for (double y : {56.345 * in, 261.345 * in})
            {
                Check(std::abs(height(x, y) - 22.25 * in) < 1e-5);
                auto hit = w.Raycast({x - 1, y, .3}, {2, 0, 0});
                Check(hit.has_value());
                Check(std::abs(hit->position.x - (x - 23.5 * in)) < 1e-5);
            }
            for (double y : {98.845 * in, 218.845 * in})
                Check(std::abs(height(x, y) - (mode == Backend::Full3D ? 6.513 * in : 0)) < 1e-5);
        }
        for (bool red : {false, true})
        {
            const double x = red ? kRebuiltLength - 13.5 * in : 13.5 * in;
            const double y = (red ? 82.84 : 234.85) * in;
            const double railHeight = mode == Backend::Full3D ? 1.125 * in : 0;
            Check(std::abs(height(x, y)) < 1e-5);
            for (double sign : {-1., 1.})
                Check(std::abs(height(x, y + sign * 19.5 * in) - railHeight) < 1e-5);
            Check(std::abs(height(red ? kRebuiltLength - 25.5 * in : 25.5 * in, y) - railHeight) < 1e-5);
        }
        if (mode == Backend::Full3D)
            for (double sign : {-1., 1.})
            {
                auto hit = w.Raycast({182.11 * in + sign * 11.1 * in, 98.845 * in, 1}, {0, 0, -2});
                Check(hit.has_value());
                Check(std::abs(hit->position.z -
                               (6.513 * in - 11.1 * in * std::tan(15 * 3.14159265358979323846 / 180))) < 1e-5);
                Check(sign * hit->normal.x > .25 && hit->normal.z > .96);
            }
        // Drive the real spring/tire model across a bump in each direction.
        for (double direction : {-1., 1.})
        {
            DrivePhysics d(w);
            const double bumpX = 182.11 * in;
            d.Reset({{bumpX - direction * 1.5, 98.845 * in, .3}, {}});
            for (int n = 0; n < 250; ++n)
            {
                d.BeforePhysics(.004);
                w.Step(.004);
                d.AfterPhysics(.004);
            }
            const double floorZ = d.Truth().body.pose.position.z;
            double maxZ = floorZ, maxPitch = 0;
            for (int n = 0; n < 1400; ++n)
            {
                for (size_t i = 0; i < 4; ++i)
                    d.SetModuleVoltages(i, n < 850 ? direction * 3 : 0, 0);
                d.BeforePhysics(.004);
                w.Step(.004);
                d.AfterPhysics(.004);
                auto t = d.Truth();
                maxZ = std::max(maxZ, t.body.pose.position.z);
                maxPitch = std::max(maxPitch, std::abs(t.body.pose.rotation.y));
            }
            auto t = d.Truth();
            std::cout << "mode " << int(mode) << " direction " << direction << " x " << t.body.pose.position.x
                      << " maxZ " << maxZ << " pitch " << maxPitch << " finalZ " << t.body.pose.position.z << '\n';
            Check(direction * (t.body.pose.position.x - bumpX) > .8);
            if (mode == Backend::Full3D)
            {
                Check(maxZ > floorZ + .04);
                Check(maxPitch > .025);
                Check(std::abs(t.body.pose.position.z - .27) < .03);
            }
            else
            {
                Check(maxZ < .301);
                Check(maxPitch < 1e-5);
            }
        }
        if (mode == Backend::Full3D)
        {
            DrivePhysics depotDrive(w);
            depotDrive.Reset({{1.6, 234.85 * in, .3}, {}});
            for (int n = 0; n < 250; ++n)
            {
                depotDrive.BeforePhysics(.004);
                w.Step(.004);
                depotDrive.AfterPhysics(.004);
            }
            const double floorZ = depotDrive.Truth().body.pose.position.z;
            double maxVerticalSpeed = 0, maxRise = 0;
            for (int n = 0; n < 600; ++n)
            {
                for (size_t i = 0; i < 4; ++i)
                    depotDrive.SetModuleVoltages(i, n < 350 ? -2 : 0, 0);
                depotDrive.BeforePhysics(.004);
                w.Step(.004);
                depotDrive.AfterPhysics(.004);
                maxRise = std::max(maxRise, depotDrive.Truth().body.pose.position.z - floorZ);
                maxVerticalSpeed = std::max(maxVerticalSpeed, std::abs(depotDrive.Truth().body.linearVelocity.z));
            }
            std::cout << "depot x " << depotDrive.Truth().body.pose.position.x << " rise " << maxRise << " maxVz "
                      << maxVerticalSpeed << '\n';
            Check(depotDrive.Truth().body.pose.position.x < .65);
            Check(maxVerticalSpeed > .05 && maxRise > .004);
        }
    }
}
