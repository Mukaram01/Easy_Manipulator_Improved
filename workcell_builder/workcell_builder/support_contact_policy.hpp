#pragma once
#include <moveit/collision_detection/collision_common.h>
#include <cmath>
#include <string>

namespace workcell {
// Engineering tolerance for initial object/support contact ONLY. Never padding
// or a global FCL tolerance. The bound is inclusive and is not runtime-inflatable.
inline constexpr double support_contact_tolerance_m = 1e-4;
struct SupportContact {
  std::string object, support;
  double floor_z;
  bool operator()(const collision_detection::Contact& c) const {
    using namespace collision_detection;
    bool forward = c.body_name_1 == support && c.body_type_1 == BodyTypes::WORLD_OBJECT &&
                   c.body_name_2 == object && c.body_type_2 == BodyTypes::ROBOT_ATTACHED;
    bool reverse = c.body_name_2 == support && c.body_type_2 == BodyTypes::WORLD_OBJECT &&
                   c.body_name_1 == object && c.body_type_1 == BodyTypes::ROBOT_ATTACHED;
    return (forward || reverse) && std::isfinite(c.depth) && c.depth >= 0 &&
      c.depth <= support_contact_tolerance_m && c.pos.allFinite() && c.normal.allFinite() &&
      std::isfinite(floor_z) && std::abs(c.pos.z()-floor_z) <= support_contact_tolerance_m &&
      (forward ? c.normal.z() : -c.normal.z()) >= 1.0-1e-6;
  }
};
}  // namespace workcell

#include <moveit/collision_detection/world.h>
#include <moveit/robot_state/robot_state.h>
#include <geometric_shapes/shapes.h>
namespace workcell {
// Certify floor from the actual collision geometry, never from the contact we
// want to forgive. Lowest upward face under a footprint point excludes a raised
// lip/ledge on the same owner. Unsupported/ambiguous geometry fails closed.
inline double floorAt(const collision_detection::World::Object& owner, const Eigen::Vector2d& xy) {
  double floor=std::numeric_limits<double>::infinity();
  for(std::size_t i=0;i<owner.shapes_.size();++i) {
    const auto& t=owner.global_shape_poses_[i]; const auto& shape=owner.shapes_[i];
    if(shape->type==shapes::BOX) {
      const auto* box=static_cast<const shapes::Box*>(shape.get());
      if(t.linear().col(2).z()<1.-1e-9) continue;
      const Eigen::Vector3d top=t*Eigen::Vector3d(0,0,box->size[2]/2);
      const Eigen::Vector3d local=t.inverse()*Eigen::Vector3d(xy.x(),xy.y(),top.z());
      if(std::abs(local.x())<=box->size[0]/2 && std::abs(local.y())<=box->size[1]/2) floor=std::min(floor,top.z());
    } else if(shape->type==shapes::MESH) {
      const auto* mesh=static_cast<const shapes::Mesh*>(shape.get());
      for(unsigned int j=0;j<mesh->triangle_count;++j) {
        Eigen::Vector3d v[3];
        for(int k=0;k<3;++k) v[k]=t*Eigen::Map<const Eigen::Vector3d>(mesh->vertices+3*mesh->triangles[3*j+k]);
        Eigen::Vector3d normal=(v[1]-v[0]).cross(v[2]-v[0]);
        if(normal.norm()<1e-15 || normal.normalized().z()<1.-1e-9) continue;
        Eigen::Matrix2d basis; basis.col(0)=(v[1]-v[0]).head<2>(); basis.col(1)=(v[2]-v[0]).head<2>();
        const Eigen::Vector2d b=basis.inverse()*(xy-v[0].head<2>());
        if(b.x()>=-1e-10 && b.y()>=-1e-10 && b.sum()<=1.+1e-10) floor=std::min(floor,v[0].z());
      }
    }
  }
  return floor;
}

// Conservative bound on carried-point travel under joint interpolation. Link
// offsets, prismatic extension and attachment radius bound every revolute lever
// arm. Thus the sampling scale is metres tied to the policy, not robot-specific
// radians. Unknown multi-variable joint interpolation fails closed.
inline double carriedTravelBound(const moveit::core::RobotState& a, const moveit::core::RobotState& b,
                                 double attachment_radius) {
  double reach=attachment_radius, linear=0., angular=0.;
  for(const auto* link:a.getRobotModel()->getLinkModels()) reach+=link->getJointOriginTransform().translation().norm();
  for(const auto* joint:a.getRobotModel()->getJointModels()) {
    if(joint->getVariableCount()==0) continue;
    if(joint->getVariableCount()!=1) return std::numeric_limits<double>::infinity();
    double x=a.getJointPositions(joint)[0],y=b.getJointPositions(joint)[0];
    if(joint->getType()==moveit::core::JointModel::PRISMATIC) { linear+=std::abs(y-x);reach+=std::max(std::abs(x),std::abs(y)); }
    else if(joint->getType()==moveit::core::JointModel::REVOLUTE) angular+=std::abs(y-x);
    else return std::numeric_limits<double>::infinity();
  }
  return linear+angular*reach;
}
}
