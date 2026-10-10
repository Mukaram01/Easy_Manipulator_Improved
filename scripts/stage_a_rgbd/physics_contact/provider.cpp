// Read-only simulation measurement boundary. Never publishes EPD poses or goals.
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/Geometry.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/stringmsg.pb.h>
#include <sdf/Box.hh>
#include <sdf/Plane.hh>
#include <jsoncpp/json/json.h>
#include <set>
#include <sstream>
#include <fstream>
#include <ignition/gazebo/components/ContactSensorData.hh>

namespace sim=ignition::gazebo;
namespace components=sim::components;
Json::Value xyz(const ignition::math::Vector3d &v) {
  Json::Value a(Json::arrayValue);a.append(v.X());a.append(v.Y());a.append(v.Z());return a;
}
class WorkcellPhysicsContactMeasurement final:public sim::System,
    public sim::ISystemConfigure,public sim::ISystemPostUpdate {
  sim::Entity world=sim::kNullEntity;
  std::string supportName,runId,worldName;
  std::set<std::string> workpieces;
  std::string diagnosticOutput;
  std::uint64_t diagnosticStep=0;
  ignition::transport::Node node;
  ignition::transport::Node::Publisher publisher;
 public:
  void Configure(const sim::Entity &entity,const std::shared_ptr<const sdf::Element> &sdf,
                 sim::EntityComponentManager &ecm,sim::EventManager &) override {
    diagnosticOutput=sdf->Get<std::string>("diagnostic_output", "").first;
    diagnosticStep=sdf->Get<std::uint64_t>("diagnostic_step", 0).first;
    if(!diagnosticOutput.empty())ecm.Each<components::Collision>([&](const auto &id,const auto*) {
      if(!ecm.Component<components::ContactSensorData>(id))ecm.CreateComponent(id,components::ContactSensorData());
      return true;
    });
    world=entity;worldName=ecm.Component<components::Name>(world)->Data();
    supportName=sdf->Get<std::string>("support_collision");
    runId=sdf->Get<std::string>("run_id");
    std::istringstream names(sdf->Get<std::string>("workpieces"));std::string name;
    while(names>>name)workpieces.insert(name);
    publisher=node.Advertise<ignition::msgs::StringMsg>("/stage_a/physics_contact_state");
  }
  void PostUpdate(const sim::UpdateInfo &info,const sim::EntityComponentManager &ecm) override {
    if(info.paused)return;
    Json::Value record;
    record["schema"]="workcell_physics_contact_state/v1";
    record["run_id"]=runId;record["world"]=worldName;record["frame_id"]="world";
    record["step"]=Json::UInt64(info.iterations);
    record["stamp_ns"]=Json::Int64(std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count());
    record["dt_ns"]=Json::Int64(std::chrono::duration_cast<std::chrono::nanoseconds>(info.dt).count());
    record["state_source"]="Fortress_PostUpdate_ECM";
    record["authority"]="simulation_only_internal_measurement_not_planning_geometry";
    record["pairs"]=Json::Value(Json::arrayValue);
    sim::Entity support=sim::kNullEntity;
    std::vector<sim::Entity> boxes;
    std::set<std::string> seen;bool complete=!runId.empty()&&!workpieces.empty();
    ecm.Each<components::Collision,components::Geometry>([&](const sim::Entity &id,const auto *,const auto *) {
      auto link=ecm.Component<components::ParentEntity>(id);
      auto model=link?ecm.Component<components::ParentEntity>(link->Data()):nullptr;
      auto stat=model?ecm.Component<components::Static>(model->Data()):nullptr;
      auto name=model?ecm.Component<components::Name>(model->Data()):nullptr;
      const auto scoped=sim::scopedName(id,ecm,"::",false);
      if(scoped==supportName) {
        if(support!=sim::kNullEntity||!stat||!stat->Data())complete=false;
        support=id;
      }
      if(stat&&!stat->Data()) {
        if(!name||!workpieces.count(name->Data())||!seen.insert(name->Data()).second)complete=false;
        boxes.push_back(id);
      }
      return true;
    });
    complete=complete&&support!=sim::kNullEntity&&seen==workpieces;
    record["support_collision_id"]=Json::UInt64(support);
    record["support_collision_name"]=supportName;
    if(support!=sim::kNullEntity) {
      const auto &supportGeometry=ecm.Component<components::Geometry>(support)->Data();
      if(supportGeometry.Type()!=sdf::GeometryType::PLANE)complete=false;
      else {
        auto planePose=sim::worldPose(support,ecm);
        for(const auto id:boxes) {
          const auto &geometry=ecm.Component<components::Geometry>(id)->Data();
          if(geometry.Type()!=sdf::GeometryType::BOX){complete=false;continue;}
          const auto pose=sim::worldPose(id,ecm);
          auto link=ecm.Component<components::ParentEntity>(id)->Data();
          auto model=ecm.Component<components::ParentEntity>(link)->Data();
          Json::Value pair;
          pair["collision_id"]=Json::UInt64(id);pair["collision_name"]=sim::scopedName(id,ecm,"::",false);
          pair["model_name"]=ecm.Component<components::Name>(model)->Data();
          pair["shape"]="BOX";pair["dimensions_m"]=xyz(geometry.BoxShape()->Size());
          // Raw ECM state is confined to this separate measurement stream/file.
          // Consumers never copy these fields into normalized EPD objects.
          pair["centre_world"]=xyz(pose.Pos());
          Json::Value matrix(Json::arrayValue);
          const auto x=pose.Rot().RotateVector(ignition::math::Vector3d::UnitX);
          const auto y=pose.Rot().RotateVector(ignition::math::Vector3d::UnitY);
          const auto z=pose.Rot().RotateVector(ignition::math::Vector3d::UnitZ);
          for(int row=0;row<3;++row){Json::Value r(Json::arrayValue);r.append(x[row]);r.append(y[row]);r.append(z[row]);matrix.append(r);}
          pair["axes_world"]=matrix;
          pair["support_collision_id"]=Json::UInt64(support);pair["support_collision_name"]=supportName;
          pair["support_shape"]="PLANE";pair["support_point_world"]=xyz(planePose.Pos());
          pair["support_normal_world"]=xyz(planePose.Rot().RotateVector(supportGeometry.PlaneShape()->Normal()));
          pair["backend_state_error_bound_m"]=Json::Value();
          pair["state_error_reason"]="ECM notification cache and DART transform conversion unqualified";
          record["pairs"].append(pair);
        }
      }
    }
    record["complete"]=complete;
    record["decision"]="BLOCKED_BACKEND_STATE_ERROR";
    if(!complete)record["failure_reason"]="missing, duplicate or unsupported loaded collision inventory";
    Json::StreamWriterBuilder writer;writer["indentation"]="";writer["precision"]=17;
    if(!diagnosticOutput.empty()&&info.iterations==diagnosticStep) {
      record["diagnostic_contacts"]=Json::Value(Json::arrayValue);
      ecm.Each<components::Collision,components::ContactSensorData>([&](const auto &id,const auto*,const auto *data) {
        for(const auto &contact:data->Data().contact()) {
          Json::Value c;c["owner_collision_id"]=Json::UInt64(id);
          c["collision1"]=contact.collision1().id();c["collision2"]=contact.collision2().id();
          c["depth_m"]=Json::Value(Json::arrayValue);for(auto d:contact.depth())c["depth_m"].append(d);
          record["diagnostic_contacts"].append(c);
        }return true;
      });
      std::ofstream output(diagnosticOutput);output<<Json::writeString(writer,record)<<"\n";
    }
    ignition::msgs::StringMsg message;message.set_data(Json::writeString(writer,record));publisher.Publish(message);
  }
};
IGNITION_ADD_PLUGIN(WorkcellPhysicsContactMeasurement,sim::System,
  WorkcellPhysicsContactMeasurement::ISystemConfigure,WorkcellPhysicsContactMeasurement::ISystemPostUpdate)
