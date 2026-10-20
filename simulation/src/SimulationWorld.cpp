#include "base/sim/SimulationWorld.h"
#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <map>
#include <stdexcept>
#include <string_view>
#include <utility>
namespace base::sim {
namespace {
JPH::Vec3 V(Vec3 v) { return {float(v.x),float(v.y),float(v.z)}; }
Vec3 V(JPH::Vec3 v) { return {v.GetX(),v.GetY(),v.GetZ()}; }
JPH::Quat Q(Quaternion q) { return {float(q.x),float(q.y),float(q.z),float(q.w)}; }
bool Finite(Vec3 v) { return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z); }
void CheckPose(Pose p) { auto q=p.rotation; double n=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w; if(!Finite(p.position)||!std::isfinite(n)||std::abs(n-1)>1e-5) throw std::invalid_argument("Invalid simulation pose"); }
struct Global { Global(){JPH::RegisterDefaultAllocator();JPH::Factory::sInstance=new JPH::Factory;JPH::RegisterTypes();} ~Global(){JPH::UnregisterTypes();delete JPH::Factory::sInstance;JPH::Factory::sInstance=nullptr;} };
struct Layers : JPH::BroadPhaseLayerInterface { uint GetNumBroadPhaseLayers() const override{return 2;} JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer l)const override{return JPH::BroadPhaseLayer(l);} 
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer)const override{return "simulation";}
#endif
};
struct Pair : JPH::ObjectLayerPairFilter {bool ShouldCollide(JPH::ObjectLayer a,JPH::ObjectLayer b) const override{return a||b;}};
struct BroadPair : JPH::ObjectVsBroadPhaseLayerFilter {bool ShouldCollide(JPH::ObjectLayer a,JPH::BroadPhaseLayer b)const override{return a||b.GetValue();}};
}
Backend BackendFromEnvironment(){auto* p=std::getenv("FRC_SIM_PHYSICS");if(!p||std::string_view(p)=="phoenix")return Backend::Phoenix;if(std::string_view(p)=="jolt-planar")return Backend::Planar;if(std::string_view(p)=="jolt-3d")return Backend::Full3D;throw std::invalid_argument("FRC_SIM_PHYSICS must be phoenix, jolt-planar, or jolt-3d");}
struct SimulationWorld::Impl {
 Backend mode; Layers layers; Pair pair; BroadPair broad; JPH::PhysicsSystem physics; JPH::TempAllocatorImpl allocator{16*1024*1024}; JPH::JobSystemSingleThreaded jobs{2048};
 struct Entry {JPH::BodyID id; uint32_t generation{1};bool alive{};BodySpec spec;bool enabled{true};};std::vector<Entry> bodies;
 explicit Impl(Backend m):mode(m){static Global global;physics.Init(4096,0,16384,4096,layers,broad,pair);physics.SetGravity({0,0,-9.80665f});}
 Entry& Get(BodyHandle h){if(h.index>=bodies.size()||!bodies[h.index].alive||bodies[h.index].generation!=h.generation)throw std::invalid_argument("Stale simulation body handle");return bodies[h.index];}
 const Entry& Get(BodyHandle h)const{return const_cast<Impl*>(this)->Get(h);}
 ~Impl(){auto& bi=physics.GetBodyInterface();for(auto&e:bodies)if(e.alive){if(e.enabled)bi.RemoveBody(e.id);bi.DestroyBody(e.id);}}
};
SimulationWorld::SimulationWorld(Backend m){if(m==Backend::Phoenix)throw std::invalid_argument("Phoenix has no Jolt world");impl_=std::make_unique<Impl>(m);}
SimulationWorld::~SimulationWorld()=default;
Backend SimulationWorld::Mode()const{return impl_->mode;}
BodyHandle SimulationWorld::CreateBody(const BodySpec&s){CheckPose(s.pose);if(!std::isfinite(s.mass)||s.mass<0||!Finite(s.halfExtents)||s.halfExtents.x<=0||s.halfExtents.y<=0||s.halfExtents.z<=0||!std::isfinite(s.radius)||s.radius<=0||!Finite(s.inertia)||!Finite(s.centerOfMass)||s.inertia.x<0||s.inertia.y<0||s.inertia.z<0||!std::isfinite(s.friction)||s.friction<0||!std::isfinite(s.restitution)||s.restitution<0||s.restitution>1)throw std::invalid_argument("Invalid simulation body configuration");
 JPH::RefConst<JPH::Shape> shape;if(s.shape==BodySpec::Shape::Box)shape=new JPH::BoxShape(V(s.halfExtents));else shape=new JPH::SphereShape(float(s.radius));if(s.centerOfMass.x||s.centerOfMass.y||s.centerOfMass.z)shape=new JPH::OffsetCenterOfMassShape(shape,V(s.centerOfMass));
 JPH::BodyCreationSettings c(shape,JPH::RVec3(V(s.pose.position)),Q(s.pose.rotation),s.mass>0?JPH::EMotionType::Dynamic:JPH::EMotionType::Static,s.mass>0?1:0);c.mFriction=float(s.friction);c.mRestitution=float(s.restitution);c.mLinearDamping=0;c.mAngularDamping=0;
 if(s.mass>0){c.mOverrideMassProperties=JPH::EOverrideMassProperties::CalculateInertia;c.mMassPropertiesOverride.mMass=float(s.mass);if(s.inertia.x>0&&s.inertia.y>0&&s.inertia.z>0){c.mOverrideMassProperties=JPH::EOverrideMassProperties::MassAndInertiaProvided;c.mMassPropertiesOverride.mInertia=JPH::Mat44::sScale(V(s.inertia));}if(s.planar)c.mAllowedDOFs=JPH::EAllowedDOFs::Plane2D;}
 auto&bi=impl_->physics.GetBodyInterface();auto id=bi.CreateAndAddBody(c,JPH::EActivation::Activate);if(id.IsInvalid())throw std::runtime_error("Jolt body capacity exceeded");uint32_t i=0;while(i<impl_->bodies.size()&&impl_->bodies[i].alive)++i;if(i==impl_->bodies.size())impl_->bodies.emplace_back();auto&e=impl_->bodies[i];e.id=id;e.alive=true;e.spec=s;e.enabled=true;return {i,e.generation};}
void SimulationWorld::DestroyBody(BodyHandle h){auto&e=impl_->Get(h);auto&bi=impl_->physics.GetBodyInterface();if(e.enabled)bi.RemoveBody(e.id);bi.DestroyBody(e.id);e.alive=false;++e.generation;}
BodyState SimulationWorld::ReadBody(BodyHandle h)const{auto&e=impl_->Get(h);auto&bi=impl_->physics.GetBodyInterface();auto p=bi.GetPosition(e.id);auto q=bi.GetRotation(e.id);return {{{p.GetX(),p.GetY(),p.GetZ()},{q.GetX(),q.GetY(),q.GetZ(),q.GetW()}},V(bi.GetLinearVelocity(e.id)),V(bi.GetAngularVelocity(e.id)),e.enabled};}
void SimulationWorld::SetBodyState(BodyHandle h,const BodyState&s){CheckPose(s.pose);if(!Finite(s.linearVelocity)||!Finite(s.angularVelocity))throw std::invalid_argument("Invalid body velocity");auto&e=impl_->Get(h);auto&bi=impl_->physics.GetBodyInterface();bi.SetPositionAndRotation(e.id,JPH::RVec3(V(s.pose.position)),Q(s.pose.rotation),JPH::EActivation::Activate);bi.SetLinearAndAngularVelocity(e.id,V(s.linearVelocity),V(s.angularVelocity));SetEnabled(h,s.enabled);}
void SimulationWorld::SetEnabled(BodyHandle h,bool enable){auto&e=impl_->Get(h);if(enable==e.enabled)return;auto&bi=impl_->physics.GetBodyInterface();if(enable)bi.AddBody(e.id,JPH::EActivation::Activate);else bi.RemoveBody(e.id);e.enabled=enable;}
void SimulationWorld::AddForce(BodyHandle h,Vec3 f,Vec3 p){if(!Finite(f)||!Finite(p))throw std::invalid_argument("Invalid force");auto&e=impl_->Get(h);if(e.enabled&&e.spec.mass>0)impl_->physics.GetBodyInterface().AddForce(e.id,V(f),JPH::RVec3(V(p)));}
void SimulationWorld::AddTorque(BodyHandle h,Vec3 t){if(!Finite(t))throw std::invalid_argument("Invalid torque");auto&e=impl_->Get(h);if(e.enabled&&e.spec.mass>0)impl_->physics.GetBodyInterface().AddTorque(e.id,V(t));}
std::optional<RayHit> SimulationWorld::Raycast(Vec3 o,Vec3 d,std::optional<BodyHandle>x)const{if(!Finite(o)||!Finite(d))throw std::invalid_argument("Invalid ray");JPH::BodyFilter filter;JPH::IgnoreSingleBodyFilter exclude(x?impl_->Get(*x).id:JPH::BodyID());JPH::RayCastResult hit;JPH::RRayCast ray(JPH::RVec3(V(o)),V(d));if(!impl_->physics.GetNarrowPhaseQuery().CastRay(ray,hit,{}, {},x?static_cast<const JPH::BodyFilter&>(exclude):filter))return {};for(uint32_t i=0;i<impl_->bodies.size();++i){auto&e=impl_->bodies[i];if(e.alive&&e.id==hit.mBodyID){auto point=ray.GetPointOnRay(hit.mFraction);return RayHit{{i,e.generation},{point.GetX(),point.GetY(),point.GetZ()},V(impl_->physics.GetBodyInterface().GetTransformedShape(e.id).GetWorldSpaceSurfaceNormal(hit.mSubShapeID2,point)),hit.mFraction};}}return {};}
void SimulationWorld::Step(double dt){if(!std::isfinite(dt)||dt<=0||dt>0.02)throw std::invalid_argument("Invalid physics step");if(impl_->physics.Update(float(dt),1,&impl_->allocator,&impl_->jobs)!=JPH::EPhysicsUpdateError::None)throw std::runtime_error("Jolt physics capacity exceeded");}
void SimulationWorld::Reset(){for(uint32_t i=0;i<impl_->bodies.size();++i){auto&e=impl_->bodies[i];if(e.alive)SetBodyState({i,e.generation},{e.spec.pose,{},{},true});}}
struct SimulationClock::Registry{uint64_t next{};std::map<uint64_t,PlantCallbacks> callbacks;};
Registration::Registration(std::function<void()>r):release_(std::move(r)){}Registration::~Registration(){if(release_)release_();}Registration::Registration(Registration&&r)noexcept:release_(std::exchange(r.release_,{})){}Registration&Registration::operator=(Registration&&r)noexcept{if(this!=&r){if(release_)release_();release_=std::exchange(r.release_,{});}return *this;}
SimulationClock::SimulationClock(Backend b):registry_(std::make_shared<Registry>()){if(b!=Backend::Phoenix)world_=std::make_unique<SimulationWorld>(b);}SimulationClock::~SimulationClock()=default;
Registration SimulationClock::Register(PlantCallbacks c){auto id=registry_->next++;registry_->callbacks.emplace(id,std::move(c));return Registration([weak=std::weak_ptr<Registry>(registry_),id]{if(auto r=weak.lock())r->callbacks.erase(id);});}
void SimulationClock::StepRobotPeriod(){using Clock=std::chrono::steady_clock;for(int i=0;i<5;++i){auto start=Clock::now();std::vector<uint64_t>ids;for(auto&[id,c]:registry_->callbacks)ids.push_back(id);for(auto id:ids)if(auto it=registry_->callbacks.find(id);it!=registry_->callbacks.end()){auto f=it->second.beforePhysics;if(f)f(.004);}if(world_)world_->Step(.004);time_+=.004;for(auto id:ids)if(auto it=registry_->callbacks.find(id);it!=registry_->callbacks.end()){auto f=it->second.afterPhysics;if(f)f(.004);}double t=std::chrono::duration<double>(Clock::now()-start).count();timing_.seconds=t;timing_.maximumSeconds=std::max(timing_.maximumSeconds,t);++timing_.substeps;if(t>.004)++timing_.deadlineMisses;}}
void SimulationClock::Reset(){if(world_)world_->Reset();time_=0;timing_={};std::vector<std::function<void()>> callbacks;for(auto&[id,c]:registry_->callbacks)if(c.reset)callbacks.push_back(c.reset);for(auto&f:callbacks)f();}
SimulationClock& GetSimulationRuntime(){static SimulationClock clock(BackendFromEnvironment());return clock;}
}
