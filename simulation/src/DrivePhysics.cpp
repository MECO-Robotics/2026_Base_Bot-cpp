#include "base/sim/DrivePhysics.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace base::sim {
namespace {
Vec3 Add(Vec3 a, Vec3 b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
Vec3 Scale(Vec3 a, double s) { return {a.x*s,a.y*s,a.z*s}; }
double Dot(Vec3 a, Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
Vec3 Cross(Vec3 a, Vec3 b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
Vec3 Rotate(Quaternion q, Vec3 v) {
  const Vec3 u{q.x,q.y,q.z};
  return Add(v, Add(Scale(Cross(u,v),2*q.w), Scale(Cross(u,Cross(u,v)),2)));
}
Vec3 Unit(Vec3 a) { double n=std::sqrt(Dot(a,a)); return n>1e-12?Scale(a,1/n):Vec3{}; }
bool Finite(Vec3 v) { return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z); }
double FrictionVoltage(double voltage, double loss) { return std::copysign(std::max(0.0,std::abs(voltage)-loss),voltage); }
void Validate(const DrivePhysicsConfig &c) {
  const double positive[]{c.mass,c.inertia.x,c.inertia.y,c.inertia.z,c.bumperHalfExtents.x,c.bumperHalfExtents.y,c.bumperHalfExtents.z,c.rideHeight,c.wheelRadius,c.driveRatio,c.steerRatio,c.wheelInertia,c.steerInertia,c.longitudinalStiffness,c.lateralStiffness,c.suspensionRestLength,c.suspensionTravel,c.suspensionStiffness,c.suspensionDamping,c.motorResistance,c.motorKt,c.motorKv,c.driveCurrentLimit,c.steerCurrentLimit};
  for(double x:positive) if(!std::isfinite(x)||x<=0) throw std::invalid_argument("Drive physics requires positive finite physical parameters");
  if(!Finite(c.centerOfMass)||!std::isfinite(c.couplingRatio)||!std::isfinite(c.friction)||c.friction<0||!std::isfinite(c.frictionVoltage)||c.frictionVoltage<0) throw std::invalid_argument("Invalid drivetrain friction, coupling, or center of mass");
  for(auto p:c.moduleLocations) if(!Finite(p)) throw std::invalid_argument("Invalid module location");
}
// Backward Euler on rotor electrical damping, with current-limited branch solved
// consistently. This is passive at zero volts even for high steering reductions.
double MotorStep(double omega, double dt, double inertia, double ratio,
                 double external, double voltage, double offsetRotorSpeed,
                 double limit, const DrivePhysicsConfig &c) {
  const double a=ratio*c.motorKt/c.motorResistance;
  const double damping=a*ratio/c.motorKv;
  double next=(inertia*omega+dt*(a*(voltage-offsetRotorSpeed/c.motorKv)+external))/(inertia+dt*damping);
  double current=(voltage-(ratio*next+offsetRotorSpeed)/c.motorKv)/c.motorResistance;
  if(std::abs(current)>limit) next=omega+dt*(ratio*c.motorKt*std::copysign(limit,current)+external)/inertia;
  return next;
}
} // namespace
DrivePhysics::DrivePhysics(SimulationWorld &world, DrivePhysicsConfig config):world_(world),config_(config) {
  Validate(config_);
  BodySpec spec;
  spec.mass=config_.mass; spec.inertia=config_.inertia; spec.centerOfMass=config_.centerOfMass;
  spec.halfExtents=config_.bumperHalfExtents; spec.friction=.2;
  spec.planar=world.Mode()==Backend::Planar;
  spec.pose.position.z=config_.rideHeight;
  body_=world_.CreateBody(spec);
}
DrivePhysics::~DrivePhysics() { world_.DestroyBody(body_); }
void DrivePhysics::SetModuleVoltages(size_t i,double drive,double steer) {
  if(i>=4) throw std::out_of_range("Drive module");
  if(!std::isfinite(drive)||!std::isfinite(steer)) throw std::invalid_argument("Nonfinite motor voltage");
  voltages_[i]={std::clamp(drive,-16.0,16.0),std::clamp(steer,-16.0,16.0)};
}
void DrivePhysics::BeforePhysics(double dt) {
  if(!std::isfinite(dt)||dt<=0||dt>.02) throw std::invalid_argument("Drive substep must be in (0,20ms]");
  auto body=world_.ReadBody(body_);
  auto q=body.pose.rotation;
  Vec3 down=Rotate(q,{0,0,-1});
  for(size_t i=0;i<4;++i) {
    auto &m=modules_[i]; const auto &c=config_;
    Vec3 lever=Rotate(q,c.moduleLocations[i]);
    Vec3 mount=Add(body.pose.position,lever);
    Vec3 velocity=Add(body.linearVelocity,Cross(body.angularVelocity,Add(lever,Scale(Rotate(q,c.centerOfMass),-1))));
    Vec3 contact=Add(mount,Scale(down,c.suspensionRestLength));
    Vec3 normal{0,0,1}; double normalForce=c.mass*9.80665/4;
    if(world_.Mode()==Backend::Full3D) {
      normalForce=0;
      const double rayLength=c.suspensionRestLength+c.suspensionTravel;
      auto hit=world_.Raycast(mount,Scale(down,rayLength),body_);
      if(hit) {
        normal=hit->normal;
        contact=hit->position;
        // Only supporting faces act as tire contacts. Walls remain bumper contacts.
        if(Dot(normal,Scale(down,-1))>.25) {
          double compression=c.suspensionRestLength-hit->fraction*rayLength;
          normalForce=std::max(0.0,c.suspensionStiffness*compression-c.suspensionDamping*Dot(velocity,normal));
          world_.AddForce(body_,Scale(normal,normalForce),mount);
        }
      }
    }
    // Steering motor sees coupling torque from the drive motor as dictated by
    // rotor angle = driveRatio*wheelAngle + couplingRatio*steerAngle.
    double driveV=FrictionVoltage(voltages_[i][0],c.frictionVoltage);
    double steerV=FrictionVoltage(voltages_[i][1],c.frictionVoltage);
    double driveCurrent=std::clamp((driveV-(c.driveRatio*m.wheelVelocity+c.couplingRatio*m.steerVelocity)/c.motorKv)/c.motorResistance,-c.driveCurrentLimit,c.driveCurrentLimit);
    m.steerVelocity=MotorStep(m.steerVelocity,dt,c.steerInertia,c.steerRatio,c.couplingRatio*c.motorKt*driveCurrent,steerV,0,c.steerCurrentLimit,c);
    m.steerAngle+=m.steerVelocity*dt;
    Vec3 longitudinal=Rotate(q,{std::cos(m.steerAngle),std::sin(m.steerAngle),0});
    longitudinal=Unit(Add(longitudinal,Scale(normal,-Dot(longitudinal,normal))));
    Vec3 lateral=Unit(Cross(normal,longitudinal));
    Vec3 contactVelocity=Add(body.linearVelocity,Cross(body.angularVelocity,Add(Add(contact,Scale(body.pose.position,-1)),Scale(Rotate(q,c.centerOfMass),-1))));
    double forward=Dot(contactVelocity,longitudinal),side=Dot(contactVelocity,lateral);
    // Solve motor current limiting and combined tire saturation at the final
    // loaded wheel speed. Selecting the electrical branch at an unloaded speed
    // can exceed the current limit once tire reaction slows the wheel.
    const double maximum=c.friction*normalForce;
    const double freeLateral=-c.lateralStiffness*side/(1+dt*c.lateralStiffness*4/c.mass);
    auto forces=[&](double omega) {
      double fx=c.longitudinalStiffness*(omega*c.wheelRadius-forward);
      double fy=freeLateral;
      const double magnitude=std::hypot(fx,fy);
      if(magnitude>maximum&&magnitude>0) {fx*=maximum/magnitude;fy*=maximum/magnitude;}
      return std::array<double,2>{fx,fy};
    };
    auto current=[&](double omega) {
      return std::clamp((driveV-(c.driveRatio*omega+c.couplingRatio*m.steerVelocity)/c.motorKv)/c.motorResistance,-c.driveCurrentLimit,c.driveCurrentLimit);
    };
    // The backward-Euler residual is monotone: inertia, electrical damping,
    // and longitudinal tire damping all oppose motion. Torque bounds bracket
    // its unique zero even across both piecewise saturation transitions.
    const double maxDelta=dt*(c.driveRatio*c.motorKt*c.driveCurrentLimit+c.wheelRadius*maximum)/c.wheelInertia;
    double lower=m.wheelVelocity-maxDelta,upper=m.wheelVelocity+maxDelta;
    const double previous=m.wheelVelocity;
    for(int iteration=0;iteration<44;++iteration) {
      const double omega=(lower+upper)*.5;
      const double residual=c.wheelInertia*(omega-previous)/dt-c.driveRatio*c.motorKt*current(omega)+c.wheelRadius*forces(omega)[0];
      if(residual>0) upper=omega;else lower=omega;
    }
    m.wheelVelocity=(lower+upper)*.5;
    const auto tire=forces(m.wheelVelocity);
    const double fx=tire[0],fy=tire[1];
    m.driveCurrent=current(m.wheelVelocity);
    m.steerCurrent=std::clamp((steerV-c.steerRatio*m.steerVelocity/c.motorKv)/c.motorResistance,-c.steerCurrentLimit,c.steerCurrentLimit);
    m.wheelAngle+=m.wheelVelocity*dt;
    m.driveRotorAngle=c.driveRatio*m.wheelAngle+c.couplingRatio*m.steerAngle;
    m.driveRotorVelocity=c.driveRatio*m.wheelVelocity+c.couplingRatio*m.steerVelocity;
    m.steerRotorAngle=c.steerRatio*m.steerAngle;
    m.steerRotorVelocity=c.steerRatio*m.steerVelocity;
    m.longitudinalSlip=c.wheelRadius*m.wheelVelocity-forward;
    m.normalForce=normalForce;m.longitudinalForce=fx;m.lateralForce=fy;
    world_.AddForce(body_,Add(Scale(longitudinal,fx),Scale(lateral,fy)),contact);
  }
}
void DrivePhysics::AfterPhysics(double dt) {time_+=dt;}
void DrivePhysics::Reset(Pose pose) {
  BodyState state;state.pose=pose;world_.SetBodyState(body_,state);
  modules_={};voltages_={};time_=0;
}
double DrivePhysics::SupplyCurrent(double busVoltage) const {
  if (!std::isfinite(busVoltage) || busVoltage < 0) throw std::invalid_argument("Invalid bus voltage");
  if (busVoltage == 0) return 0;
  double power = 0;
  for (size_t i=0; i<modules_.size(); ++i) {
    const auto &m=modules_[i];
    // Restore the configured friction drop removed before the electrical solve.
    // Current limiting can still reduce terminal voltage below the request.
    const double driveV = m.driveCurrent * config_.motorResistance + m.driveRotorVelocity / config_.motorKv + std::copysign(config_.frictionVoltage, voltages_[i][0]);
    const double steerV = m.steerCurrent * config_.motorResistance + m.steerRotorVelocity / config_.motorKv + std::copysign(config_.frictionVoltage, voltages_[i][1]);
    power += std::max(0., std::clamp(driveV, -std::abs(voltages_[i][0]), std::abs(voltages_[i][0])) * m.driveCurrent)
           + std::max(0., std::clamp(steerV, -std::abs(voltages_[i][1]), std::abs(voltages_[i][1])) * m.steerCurrent);
  }
  return power / busVoltage;
}
DriveTruth DrivePhysics::Truth() const {return {world_.ReadBody(body_),modules_,time_};}
} // namespace base::sim
