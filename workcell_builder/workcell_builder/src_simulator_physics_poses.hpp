#pragma once

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/CanonicalLink.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>

namespace workcell {
namespace sim=ignition::gazebo;
namespace c=sim::components;

// Fortress Physics::UpdateSim updates a requested WorldPose on a non-Link child
// using LinkFrameDataAtOffset every physics step. Link WorldPose and worldPose()
// instead follow ChangedWorldPoses, whose DART output suppresses sub-micrometre
// motion. These identity observation entities have no physics body or commands.
class PhysicsPoseQueries {
 public:
  static constexpr const char* Method="physics_link_frame_data_at_offset";

  void Prepare(const sim::UpdateInfo& info, sim::EntityComponentManager& ecm) {
    prepared_=false;
    std::vector<sim::Entity> links;
    ecm.Each<c::Link>([&](const sim::Entity& e,const c::Link*) {links.push_back(e);return true;});
    for(const auto link:links) {
      if(!queries_.count(link)) {
        const auto query=ecm.CreateEntity();
        ecm.CreateComponent(query,c::Pose());
        ecm.CreateComponent(query,c::ParentEntity(link));
        ecm.CreateComponent(query,c::Name("workcell_physics_pose_query"));
        ecm.CreateComponent(query,c::WorldPose());
        if(!ecm.SetParentEntity(query,link)) Fail("cannot parent query",link);
        queries_[link]=query;
      }
      auto* pose=ecm.Component<c::WorldPose>(queries_.at(link));
      if(!pose) Fail("missing query component",link);
      // A missed physics write must never reuse a finite pose from another tick.
      ignition::math::Pose3d unavailable;
      unavailable.Pos().X()=std::numeric_limits<double>::quiet_NaN();
      *pose=c::WorldPose(unavailable);
    }
    ecm.Each<c::Model>([&](const sim::Entity& model,const c::Model*) {
      const auto* canonical=ecm.Component<c::ModelCanonicalLink>(model);
      if(!canonical) Fail("missing canonical link",model);
      const auto link=canonical->Data();
      const auto* parent=ecm.Component<c::ParentEntity>(link);
      const auto* offset=ecm.Component<c::Pose>(link);
      if(!parent || parent->Data()!=model || ecm.ParentEntity(link)!=model ||
         !ecm.Component<c::CanonicalLink>(link) || !offset || !Finite(offset->Data()))
        Fail("unsupported canonical link frame",model);
      // Canonical link local Pose is fixed in Fortress. Nested canonical links
      // are deliberately unsupported rather than composing cached model poses.
      if(!models_.count(model)) models_[model]={link,offset->Data()};
      const auto& saved=models_.at(model);
      if(saved.link!=link || !Exact(saved.offset,offset->Data()))
        Fail("canonical link binding changed",model);
      return true;
    });
    iteration_=info.iterations; sim_time_=info.simTime; prepared_=true;
  }

  sim::Entity QueryEntity(sim::Entity link) const {
    const auto it=queries_.find(link);
    if(it==queries_.end()) Fail("unbound link",link);
    return it->second;
  }
  sim::Entity CanonicalEntity(sim::Entity model) const {
    const auto it=models_.find(model);
    if(it==models_.end()) Fail("unbound model",model);
    return it->second.link;
  }
  ignition::math::Pose3d LinkPose(sim::Entity link,const sim::UpdateInfo& info,
                                 const sim::EntityComponentManager& ecm) const {
    if(!prepared_ || info.iterations!=iteration_ || info.simTime!=sim_time_)
      Fail("query tick mismatch",link);
    const auto query=QueryEntity(link);
    const auto* parent=ecm.Component<c::ParentEntity>(query);
    const auto* local=ecm.Component<c::Pose>(query);
    const auto* pose=ecm.Component<c::WorldPose>(query);
    if(!ecm.Component<c::Link>(link) || !parent || parent->Data()!=link ||
       ecm.ParentEntity(query)!=link || !local || !Exact(local->Data(),ignition::math::Pose3d()) ||
       !pose || !Finite(pose->Data())) Fail("missing or stale physics query",link);
    return pose->Data();
  }
  ignition::math::Pose3d ModelPose(sim::Entity model,const sim::UpdateInfo& info,
                                  const sim::EntityComponentManager& ecm) const {
    const auto link=CanonicalEntity(model);
    const auto* canonical=ecm.Component<c::ModelCanonicalLink>(model);
    const auto* parent=ecm.Component<c::ParentEntity>(link);
    const auto* offset=ecm.Component<c::Pose>(link);
    if(!ecm.Component<c::Model>(model) || !canonical || canonical->Data()!=link ||
       !ecm.Component<c::CanonicalLink>(link) || !parent || parent->Data()!=model ||
       ecm.ParentEntity(link)!=model || !offset || !Exact(offset->Data(),models_.at(model).offset))
      Fail("model frame binding changed",model);
    const auto result=LinkPose(link,info,ecm)*models_.at(model).offset.Inverse();
    if(!Finite(result)) Fail("nonfinite model pose",model);
    return result;
  }
 private:
  struct ModelBinding {sim::Entity link; ignition::math::Pose3d offset;};
  std::map<sim::Entity,sim::Entity> queries_;
  std::map<sim::Entity,ModelBinding> models_;
  bool prepared_=false;
  uint64_t iteration_=0;
  std::chrono::steady_clock::duration sim_time_{};
  static bool Exact(const ignition::math::Pose3d& a,const ignition::math::Pose3d& b) {
    return a.Pos().X()==b.Pos().X() && a.Pos().Y()==b.Pos().Y() && a.Pos().Z()==b.Pos().Z() &&
      a.Rot().W()==b.Rot().W() && a.Rot().X()==b.Rot().X() &&
      a.Rot().Y()==b.Rot().Y() && a.Rot().Z()==b.Rot().Z();
  }
  static bool Finite(const ignition::math::Pose3d& p) {
    const auto& q=p.Rot();
    return std::isfinite(p.Pos().X()) && std::isfinite(p.Pos().Y()) && std::isfinite(p.Pos().Z()) &&
      std::isfinite(q.W()) && std::isfinite(q.X()) && std::isfinite(q.Y()) && std::isfinite(q.Z()) &&
      std::abs(q.W()*q.W()+q.X()*q.X()+q.Y()*q.Y()+q.Z()*q.Z()-1.)<=1e-5;
  }
  [[noreturn]] static void Fail(const char* what,sim::Entity entity) {
    throw std::runtime_error(std::string("SIMULATOR_PHYSICS_POSE_UNAVAILABLE: ")+what+" entity="+std::to_string(entity));
  }
};
}
