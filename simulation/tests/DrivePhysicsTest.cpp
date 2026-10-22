#include "base/sim/DrivePhysics.h"
#include <cmath>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>
using namespace base::sim;
namespace
{
void Check(bool ok, std::source_location at = std::source_location::current())
{
    if (!ok)
        throw std::runtime_error("Drive physics contract at line " + std::to_string(at.line()));
}
void Step(SimulationWorld &world, DrivePhysics &drive, int count)
{
    for (int n = 0; n < count; ++n)
    {
        drive.BeforePhysics(.004);
        world.Step(.004);
        drive.AfterPhysics(.004);
    }
}
void Floor(SimulationWorld &world)
{
    BodySpec f;
    f.halfExtents = {20, 20, .1};
    f.pose.position.z = -.1;
    world.CreateBody(f);
}
double Energy(const DriveTruth &t, const DrivePhysicsConfig &c)
{
    double e = .5 * c.mass *
               (t.body.linearVelocity.x * t.body.linearVelocity.x + t.body.linearVelocity.y * t.body.linearVelocity.y);
    e += .5 * c.inertia.z * t.body.angularVelocity.z * t.body.angularVelocity.z;
    for (auto m : t.modules)
        e += .5 * c.wheelInertia * m.wheelVelocity * m.wheelVelocity +
             .5 * c.steerInertia * m.steerVelocity * m.steerVelocity;
    return e;
}
} // namespace
int main()
{
    // Force and torque act at module locations with configured mass and inertia.
    {
        SimulationWorld w(Backend::Planar);
        DrivePhysicsConfig c;
        c.friction = .15;
        DrivePhysics d(w, c);
        for (size_t i = 0; i < 4; ++i)
            d.SetModuleVoltages(i, 12, 0);
        d.BeforePhysics(.004);
        auto t = d.Truth();
        double fx = 0, tz = 0;
        for (size_t i = 0; i < 4; ++i)
        {
            auto m = t.modules[i];
            fx += m.longitudinalForce;
            tz -= c.moduleLocations[i].y * m.longitudinalForce;
            Check(std::hypot(m.longitudinalForce, m.lateralForce) <= c.friction * m.normalForce + 1e-8);
        }
        w.Step(.004);
        d.AfterPhysics(.004);
        t = d.Truth();
        Check(std::abs(t.body.linearVelocity.x - fx / c.mass * .004) < 1e-5);
        Check(std::abs(t.body.angularVelocity.z - tz / c.inertia.z * .004) < 1e-5);
        d.Reset({{0, 0, c.rideHeight}, {}});
        d.SetModuleVoltages(0, 12, 0);
        d.SetModuleVoltages(1, -12, 0);
        d.SetModuleVoltages(2, 12, 0);
        d.SetModuleVoltages(3, -12, 0);
        d.BeforePhysics(.004);
        t = d.Truth();
        tz = 0;
        for (size_t i = 0; i < 4; ++i)
            tz -= c.moduleLocations[i].y * t.modules[i].longitudinalForce;
        w.Step(.004);
        d.AfterPhysics(.004);
        Check(std::abs(d.Truth().body.angularVelocity.z - tz / c.inertia.z * .004) < 1e-5);
    }
    // Tire load can move a low-voltage motor from the linear electrical branch
    // into current limiting. Check torque balance at the final loaded speed.
    {
        SimulationWorld w(Backend::Planar);
        DrivePhysicsConfig c;
        DrivePhysics d(w, c);
        d.SetModuleVoltages(0, 2.85, 0);
        d.BeforePhysics(.004);
        const auto m = d.Truth().modules[0];
        const double inferredCurrent = (c.wheelInertia * m.wheelVelocity / .004 + c.wheelRadius * m.longitudinalForce) /
                                       (c.driveRatio * c.motorKt);
        Check(inferredCurrent <= c.driveCurrentLimit + 1e-8);
        Check(std::abs(inferredCurrent - c.driveCurrentLimit) < 1e-6);
        Check(std::abs(m.driveCurrent - inferredCurrent) < 1e-6);
    }
    // Powered wall contact stops truth while spinning wheels slip; zero voltage dissipates energy.
    {
        SimulationWorld w(Backend::Planar);
        DrivePhysicsConfig c;
        c.friction = .4;
        DrivePhysics d(w, c);
        BodySpec wall;
        wall.halfExtents = {.1, 3, 1};
        wall.pose.position = {2, 0, 1};
        w.CreateBody(wall);
        for (size_t i = 0; i < 4; ++i)
            d.SetModuleVoltages(i, 12, 0);
        Step(w, d, 1000);
        auto t = d.Truth();
        Check(t.body.pose.position.x < 1.5 && t.body.pose.position.x > 1.3);
        Check(std::abs(t.body.linearVelocity.x) < .1 && t.modules[0].longitudinalSlip > 1);
        for (size_t i = 0; i < 4; ++i)
            d.SetModuleVoltages(i, 0, 0);
        double initial = Energy(d.Truth(), d.Config());
        Step(w, d, 500);
        Check(Energy(d.Truth(), d.Config()) < initial * .01);
        d.Reset({{-.5, 1, .3}, {}});
        t = d.Truth();
        Check(std::abs(t.body.pose.position.x + .5) < 1e-6 && t.body.linearVelocity.x == 0 &&
              t.modules[0].wheelVelocity == 0 && t.timestamp == 0);
    }
    // Coupling changes rotor coordinates and produces steering reaction.
    {
        SimulationWorld w(Backend::Planar);
        DrivePhysicsConfig c;
        c.couplingRatio = 3;
        DrivePhysics d(w, c);
        d.SetModuleVoltages(0, 6, 0);
        Step(w, d, 20);
        auto m = d.Truth().modules[0];
        Check(m.steerVelocity > 0);
        Check(std::abs(m.driveRotorAngle - (c.driveRatio * m.wheelAngle + c.couplingRatio * m.steerAngle)) < 1e-12);
        const double energy = Energy(d.Truth(), c);
        d.SetModuleVoltages(0, 0, 0);
        Step(w, d, 500);
        Check(Energy(d.Truth(), c) < energy * .05);
    }
    // Full3D springs support the chassis without chassis-floor contact.
    {
        SimulationWorld w(Backend::Full3D);
        Floor(w);
        DrivePhysics d(w);
        Step(w, d, 1000);
        auto t = d.Truth();
        double expected =
            d.Config().suspensionRestLength - d.Config().mass * 9.80665 / (4 * d.Config().suspensionStiffness);
        Check(std::abs(t.body.pose.position.z - expected) < .01);
        Check(std::abs(t.body.linearVelocity.z) < .01 && std::abs(t.body.pose.rotation.x) < .01);
    }
    {
        SimulationWorld w(Backend::Full3D);
        Floor(w);
        DrivePhysicsConfig c;
        c.centerOfMass.z = .5;
        DrivePhysics d(w, c);
        // Elevated COM starts outside the support footprint at this roll.
        d.Reset({{0, 0, .6}, {std::sin(.5), 0, 0, std::cos(.5)}});
        Step(w, d, 700);
        Check(std::abs(d.Truth().body.pose.rotation.x) > .5);
    }
    // A ramp supports a tilted chassis and transfers traction uphill.
    {
        SimulationWorld w(Backend::Full3D);
        Floor(w);
        BodySpec ramp;
        ramp.halfExtents = {2, 2, .1};
        ramp.pose.position = {0, 0, .5};
        ramp.pose.rotation = {0, std::sin(-.08), 0, std::cos(.08)};
        w.CreateBody(ramp);
        DrivePhysics d(w);
        d.Reset({{0, 0, 1}, {}});
        Step(w, d, 700);
        auto before = d.Truth();
        Check(before.body.pose.position.z > .65);
        for (size_t i = 0; i < 4; ++i)
            d.SetModuleVoltages(i, 3, 0);
        Step(w, d, 150);
        auto after = d.Truth();
        Check(after.body.pose.position.x > before.body.pose.position.x + .1);
    }
    {
        SimulationWorld w(Backend::Planar);
        DrivePhysicsConfig c;
        c.driveCurrentLimit = 1000;
        c.steerCurrentLimit = 1000;
        c.frictionVoltage = 1;
        DrivePhysics d(w, c);
        for (size_t i = 0; i < 4; ++i)
            d.SetModuleVoltages(i, 6, 0);
        d.BeforePhysics(.004);
        double winding = 0;
        for (const auto &m : d.Truth().modules)
            winding += m.driveCurrent;
        Check(winding > 0 && std::abs(d.SupplyCurrent(12) - 6 * winding / 12) < 1e-9);
        for (size_t i = 0; i < 4; ++i)
            d.SetModuleVoltages(i, -6, 0);
        d.BeforePhysics(.004);
        winding = 0;
        for (const auto &m : d.Truth().modules)
            winding += m.driveCurrent;
        Check(winding < 0 && std::abs(d.SupplyCurrent(12) + 6 * winding / 12) < 1e-9);
        for (size_t i = 0; i < 4; ++i)
            d.SetModuleVoltages(i, 0, 0);
        d.BeforePhysics(.004);
        Check(d.SupplyCurrent(12) == 0 && d.SupplyCurrent(0) == 0);
    }

    {
        SimulationWorld w(Backend::Planar);
        DrivePhysicsConfig c;
        c.driveCurrentLimit = 10;
        c.frictionVoltage = 0;
        c.wheelInertia = 1e9;
        DrivePhysics d(w, c);
        for (double command : {6., -6.})
        {
            for (size_t i = 0; i < 4; ++i)
                d.SetModuleVoltages(i, command, 0);
            d.BeforePhysics(.004);
            const double expected = 4 * 10 * 10 * c.motorResistance / 12;
            Check(std::abs(d.SupplyCurrent(12) - expected) < 1e-8);
        }
    }
    std::cout << "Drivetrain physics contracts passed\n";
}
