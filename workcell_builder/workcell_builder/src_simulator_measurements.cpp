// Read-only physics contact identities/points and poses. Fortress does not
// populate contact normals/depths; the execution owner queries those separately
// from MoveIt using measured state. No physics parameters or commands change.
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/ContactSensorData.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/stringmsg.pb.h>
#include <iomanip>
#include <sstream>
#include <set>
#include <unistd.h>
namespace workcell {
namespace sim=ignition::gazebo;
namespace c=sim::components;
class SimulatorMeasurements: public sim::System, public sim::ISystemConfigure,
 public sim::ISystemPreUpdate, public sim::ISystemPostUpdate {
 ignition::transport::Node node;
 ignition::transport::Node::Publisher publisher;
 std::string run;
 public:
 void Configure(const sim::Entity&, const std::shared_ptr<const sdf::Element>& cfg,
   sim::EntityComponentManager&, sim::EventManager&) override {
   run=cfg->Get<std::string>("run_id");
   publisher=node.Advertise<ignition::msgs::StringMsg>(cfg->Get<std::string>("topic"));

 }
 void PreUpdate(const sim::UpdateInfo&, sim::EntityComponentManager& ecm) override {
   ecm.Each<c::Collision>([&](const sim::Entity& e,const c::Collision*) {
     if(!ecm.Component<c::ContactSensorData>(e))ecm.CreateComponent(e,c::ContactSensorData());
     return true;
   });
   ecm.Each<c::Joint>([&](const sim::Entity& e,const c::Joint*) {
     if(!ecm.Component<c::JointPosition>(e))ecm.CreateComponent(e,c::JointPosition());
     if(!ecm.Component<c::JointVelocity>(e))ecm.CreateComponent(e,c::JointVelocity());
     return true;
   });
 }
 void PostUpdate(const sim::UpdateInfo& info,const sim::EntityComponentManager& ecm) override {
   if(info.paused || !publisher.HasConnections())return;
   const auto acquisition=std::chrono::steady_clock::now();
   std::ostringstream out;out<<std::setprecision(17);
   out<<"{\"run_id\":"<<std::quoted(run)<<",\"pid\":"<<getpid()<<",\"iteration\":"<<info.iterations
      <<",\"sim_ns\":"<<std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count()
      <<",\"wall_ns\":"<<std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
   auto name=[&](sim::Entity e){return sim::scopedName(e,ecm,"::",false);};
   bool first=true;out<<",\"poses\":{";
   auto pose=[&](sim::Entity e){if(!first)out<<",";first=false;auto p=sim::worldPose(e,ecm);
     out<<std::quoted(name(e))<<":["<<p.Pos().X()<<","<<p.Pos().Y()<<","<<p.Pos().Z()<<","<<p.Rot().X()<<","<<p.Rot().Y()<<","<<p.Rot().Z()<<","<<p.Rot().W()<<"]";};
   ecm.Each<c::Model>([&](const sim::Entity& e,const c::Model*){pose(e);return true;});
   ecm.Each<c::Link>([&](const sim::Entity& e,const c::Link*){pose(e);return true;});
   first=true;out<<"},\"joints\":{";
   ecm.Each<c::Joint,c::JointPosition,c::JointVelocity>([&](const sim::Entity& e,const c::Joint*,const c::JointPosition* p,const c::JointVelocity* v){
     if(p->Data().size()==1 && v->Data().size()==1){if(!first)out<<",";first=false;out<<std::quoted(name(e))<<":["<<p->Data()[0]<<","<<v->Data()[0]<<"]";}return true;
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
      <<",\"serialization_ns\":"<<std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-acquisition).count()<<"}";
   ignition::msgs::StringMsg msg;msg.set_data(out.str());publisher.Publish(msg);
 }
};
}
IGNITION_ADD_PLUGIN(workcell::SimulatorMeasurements,ignition::gazebo::System,
 workcell::SimulatorMeasurements::ISystemConfigure,workcell::SimulatorMeasurements::ISystemPreUpdate,
 workcell::SimulatorMeasurements::ISystemPostUpdate)
