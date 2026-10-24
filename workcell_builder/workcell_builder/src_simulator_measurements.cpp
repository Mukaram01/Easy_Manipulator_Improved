// Read-only physics contact identities/points and poses. Fortress does not
// populate contact normals/depths; the execution owner queries those separately
// from MoveIt using measured state. No physics parameters or commands change.
#include "src_simulator_physics_poses.hpp"
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/ContactSensorData.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointTransmittedWrench.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/stringmsg.pb.h>
#include <iomanip>
#include <sstream>
#include <set>
#include <time.h>
#include <unistd.h>
namespace workcell {
namespace sim=ignition::gazebo;
namespace c=sim::components;
// CLOCK_MONOTONIC_RAW is shared by every process of this boot and is never
// stepped or rate-adjusted; consumers (scripts/measurement_clock.py) judge
// freshness, ordering and latency against it. wall_ns stays evidence: on WSL2
// (LENLEG 2026-10-23) CLOCK_REALTIME stepped ~1.6-2 s every 32 s and the
// kernel tick slowed CLOCK_MONOTONIC by up to 8.3 %.
inline long long MonotonicRawNs() {
  timespec now{};clock_gettime(CLOCK_MONOTONIC_RAW,&now);
  return static_cast<long long>(now.tv_sec)*1000000000LL+now.tv_nsec;
}
class SimulatorMeasurements: public sim::System, public sim::ISystemConfigure,
 public sim::ISystemPreUpdate, public sim::ISystemPostUpdate {
 ignition::transport::Node node;
 ignition::transport::Node::Publisher publisher;
 std::string run;
 PhysicsPoseQueries physics_poses;
 std::string pose_error;
 public:
 void Configure(const sim::Entity&, const std::shared_ptr<const sdf::Element>& cfg,
   sim::EntityComponentManager&, sim::EventManager&) override {
   run=cfg->Get<std::string>("run_id");
   publisher=node.Advertise<ignition::msgs::StringMsg>(cfg->Get<std::string>("topic"));

 }
 void PreUpdate(const sim::UpdateInfo& info, sim::EntityComponentManager& ecm) override {
   pose_error.clear();
   try {physics_poses.Prepare(info,ecm);}
   catch(const std::exception& error) {pose_error=error.what();}
   ecm.Each<c::Collision>([&](const sim::Entity& e,const c::Collision*) {
     if(!ecm.Component<c::ContactSensorData>(e))ecm.CreateComponent(e,c::ContactSensorData());
     return true;
   });
   ecm.Each<c::Joint>([&](const sim::Entity& e,const c::Joint*) {
     if(!ecm.Component<c::JointPosition>(e))ecm.CreateComponent(e,c::JointPosition());
     if(!ecm.Component<c::JointVelocity>(e))ecm.CreateComponent(e,c::JointVelocity());
     // Read-only: the Physics system only fills this component (actuator/contact
     // load evidence for grasp retention); nothing reads it back into the step.
     if(!ecm.Component<c::JointTransmittedWrench>(e))ecm.CreateComponent(e,c::JointTransmittedWrench());
     return true;
   });
 }
 void PostUpdate(const sim::UpdateInfo& info,const sim::EntityComponentManager& ecm) override {
   if(info.paused || !publisher.HasConnections())return;
   const auto acquisition=std::chrono::steady_clock::now();
   std::ostringstream out;out<<std::setprecision(17);
   out<<"{\"run_id\":"<<std::quoted(run)<<",\"pid\":"<<getpid()<<",\"iteration\":"<<info.iterations
      <<",\"sim_ns\":"<<std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count()
      <<",\"wall_ns\":"<<std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count()
      <<",\"monotonic_raw_ns\":"<<MonotonicRawNs();
   auto name=[&](sim::Entity e){return sim::scopedName(e,ecm,"::",false);};
   struct PoseMeasurement {sim::Entity entity,link,query; bool model; ignition::math::Pose3d pose;};
   std::vector<PoseMeasurement> poses;
   try {
     if(!pose_error.empty()) throw std::runtime_error(pose_error);
     ecm.Each<c::Model>([&](const sim::Entity& e,const c::Model*) {
       const auto link=physics_poses.CanonicalEntity(e);
       poses.push_back({e,link,physics_poses.QueryEntity(link),true,physics_poses.ModelPose(e,info,ecm)});
       return true;
     });
     ecm.Each<c::Link>([&](const sim::Entity& e,const c::Link*) {
       poses.push_back({e,e,physics_poses.QueryEntity(e),false,physics_poses.LinkPose(e,info,ecm)});
       return true;
     });
   } catch(const std::exception& error) {
     out<<",\"error\":"<<std::quoted(error.what())<<"}";
     ignition::msgs::StringMsg msg;msg.set_data(out.str());publisher.Publish(msg);return;
   }
   out<<",\"pose_source\":{\"method\":"<<std::quoted(PhysicsPoseQueries::Method)
      <<",\"frame\":\"world\",\"read_only\":true,\"gazebo_version\":"<<std::quoted(IGNITION_GAZEBO_VERSION_FULL)
      <<",\"query_iteration\":"<<info.iterations
      <<",\"query_sim_ns\":"<<std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count()<<"}";
   bool first=true;out<<",\"pose_entities\":{";
   for(const auto& entry:poses) {
     if(!first)out<<",";first=false;
     out<<std::quoted(name(entry.entity))<<":{\"entity\":"<<entry.entity
        <<",\"link_entity\":"<<entry.link<<",\"query_entity\":"<<entry.query
        <<",\"kind\":"<<std::quoted(entry.model?"model":"link")<<"}";
   }
   first=true;out<<"},\"poses\":{";
   for(const auto& entry:poses) {
     if(!first)out<<",";first=false;const auto& p=entry.pose;
     out<<std::quoted(name(entry.entity))<<":["<<p.Pos().X()<<","<<p.Pos().Y()<<","<<p.Pos().Z()<<","<<p.Rot().X()<<","<<p.Rot().Y()<<","<<p.Rot().Z()<<","<<p.Rot().W()<<"]";
   }
   first=true;out<<"},\"joints\":{";
   ecm.Each<c::Joint,c::JointPosition,c::JointVelocity>([&](const sim::Entity& e,const c::Joint*,const c::JointPosition* p,const c::JointVelocity* v){
     if(p->Data().size()==1 && v->Data().size()==1){if(!first)out<<",";first=false;out<<std::quoted(name(e))<<":["<<p->Data()[0]<<","<<v->Data()[0]<<"]";}return true;
   });
   // Transmitted wrench in the joint frame and its torque about the joint axis.
   first=true;out<<"},\"joint_wrenches\":{";
   ecm.Each<c::Joint,c::JointTransmittedWrench>([&](const sim::Entity& e,const c::Joint*,const c::JointTransmittedWrench* w){
     const auto& f=w->Data().force();const auto& t=w->Data().torque();
     double axis_torque=0.;
     if(const auto* axis=ecm.Component<c::JointAxis>(e)) {
       const auto u=axis->Data().Xyz().Normalized();axis_torque=t.x()*u.X()+t.y()*u.Y()+t.z()*u.Z();
     }
     if(!first)out<<",";first=false;
     out<<std::quoted(name(e))<<":["<<f.x()<<","<<f.y()<<","<<f.z()<<","<<t.x()<<","<<t.y()<<","<<t.z()<<","<<axis_torque<<"]";
     return true;
   });
   first=true;out<<"},\"contacts\":[";
   std::set<std::pair<uint64_t,uint64_t>> seen;
   ecm.Each<c::Collision,c::ContactSensorData>([&](const sim::Entity&,const c::Collision*,const c::ContactSensorData* data){
     for(const auto& x:data->Data().contact()){
       const auto a=x.collision1().id(),b=x.collision2().id();
       if(!seen.insert(std::minmax(a,b)).second)continue;
       if(!first)out<<",";first=false;out<<"{\"a\":"<<std::quoted(name(a))<<",\"b\":"<<std::quoted(name(b))<<",\"points\":[";
       for(int i=0;i<x.position_size();++i){if(i)out<<",";const auto& p=x.position(i);out<<"["<<p.x()<<","<<p.y()<<","<<p.z()<<"]";}
       out<<"]}";
     }return true;
   });
   first=true;out<<"],\"collisions\":[";
   ecm.Each<c::Collision>([&](const sim::Entity& e,const c::Collision*){if(!first)out<<",";first=false;out<<std::quoted(name(e));return true;});
   out<<"],\"publish_wall_ns\":"<<std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count()
      <<",\"publish_monotonic_raw_ns\":"<<MonotonicRawNs()
      <<",\"serialization_ns\":"<<std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-acquisition).count()<<"}";
   ignition::msgs::StringMsg msg;msg.set_data(out.str());publisher.Publish(msg);
 }
};
}
IGNITION_ADD_PLUGIN(workcell::SimulatorMeasurements,ignition::gazebo::System,
 workcell::SimulatorMeasurements::ISystemConfigure,workcell::SimulatorMeasurements::ISystemPreUpdate,
 workcell::SimulatorMeasurements::ISystemPostUpdate)
