#pragma once
#include "support_contact_policy.hpp"
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model/prismatic_joint_model.h>
#include <moveit/collision_detection_fcl/collision_env_fcl.h>
#include <joint_trajectory_controller/trajectory.hpp>
#include <boost/numeric/interval.hpp>
#include <chrono>
#include <array>
#include <functional>
#include <map>
#include <set>

namespace workcell {
// This certifies collision exclusion in a frozen PlanningScene, not physical
// tracking, moving-world prediction, or arbitrary user feasibility callbacks.
enum class ControllerCertificate { CERTIFIED_CLEAR, COLLISION, UNCERTIFIED };
struct ControllerAuditOptions {
  unsigned max_depth=32;
  std::size_t max_intervals=120001;
  int64_t min_interval_ns=1;
  bool stationary_world=true;
};
struct ControllerAuditReport {
  ControllerCertificate result=ControllerCertificate::UNCERTIFIED;
  std::size_t inspected=0, certified=0, subdivided=0;
  unsigned deepest=0;
  int64_t failure_begin_ns=-1, failure_end_ns=-1;
  double wall_seconds=0.;
  std::string reason;
};
namespace controller_certificate {
using Interval=boost::numeric::interval<double>;
using Controls=std::vector<Interval>;
using Polynomials=std::map<std::string,Controls>;
inline double magnitude(const Interval& x) { return std::max(std::abs(x.lower()),std::abs(x.upper())); }
inline void finite(const Interval& x) {
  if (!std::isfinite(x.lower()) || !std::isfinite(x.upper())) throw std::runtime_error("NONFINITE_INTERVAL");
}
inline Interval hull(const Controls& c) {
  if(c.empty()) throw std::runtime_error("EMPTY_POLYNOMIAL");
  double lo=c.front().lower(),hi=c.front().upper();
  for(const auto& v:c) {finite(v);lo=std::min(lo,v.lower());hi=std::max(hi,v.upper());}
  return Interval(lo,hi);
}
inline unsigned choose(unsigned n,unsigned k) {
  unsigned value=1;
  for(unsigned i=1;i<=k;++i) value=value*(n+1-i)/i;
  return value;
}
// Outward-rounded enclosure of the installed Humble JTC 2.54.0 power
// coefficients, transformed to Bernstein form. The operation order below is
// the controller's trajectory.cpp. Bounds include coefficient cancellation,
// unlike endpoint displacement or ideal Hermite coefficients alone.
inline Controls polynomial(const trajectory_msgs::msg::JointTrajectoryPoint& a,
    const trajectory_msgs::msg::JointTrajectoryPoint& b,std::size_t j,int64_t ns,double* evaluation_error=nullptr) {
  const double seconds=rclcpp::Duration::from_nanoseconds(ns).seconds();
  const Interval t(seconds);
  Interval T[6]={Interval(1.),t};
  for(unsigned k=2;k<6;++k) T[k]=T[k-1]*t;
  const Interval p(a.positions.at(j)),q(b.positions.at(j));
  const bool v=!a.velocities.empty()&&!b.velocities.empty();
  const bool acc=!a.accelerations.empty()&&!b.accelerations.empty();
  if(acc&&!v) throw std::runtime_error("ACCELERATION_WITHOUT_VELOCITY_UNSUPPORTED");
  Controls c;
  if(!v) c={p,(q-p)/t};
  else {
    const Interval u(a.velocities.at(j)),w(b.velocities.at(j));
    if(!acc) c={p,u,(-3.*p+3.*q-2.*u*T[1]-w*T[1])/T[2],
                         (2.*p-2.*q+u*T[1]+w*T[1])/T[3]};
    else {
      const Interval x(a.accelerations.at(j)),y(b.accelerations.at(j));
      c={p,u,.5*x,
        (-20.*p+20.*q-3.*x*T[2]+y*T[2]-12.*u*T[1]-8.*w*T[1])/(2.*T[3]),
        (30.*p-30.*q+3.*x*T[2]-2.*y*T[2]+16.*u*T[1]+14.*w*T[1])/(2.*T[4]),
        (-12.*p+12.*q-x*T[2]+y*T[2]-6.*u*T[1]-6.*w*T[1])/(2.*T[5])};
    }
  }
  // The installed implementation computes powers and a sum of <=6 terms in
  // double, after converting nanoseconds to seconds. gamma_128 is deliberately
  // above the number of rounded operations, including time conversion. Charging
  // the entire segment's absolute power-term sum remains valid after subdivision
  // and at cancellation points; a position-relative epsilon would not.
  Interval terms(0.);
  for(unsigned k=0;k<c.size();++k) terms+=boost::numeric::abs(c[k]*T[k]);
  finite(terms);
  const double eps=std::numeric_limits<double>::epsilon();
  const Interval error=terms*(128.*eps/(1.-128.*eps))+std::numeric_limits<double>::min();
  finite(error);if(evaluation_error) *evaluation_error=error.upper();
  Controls bernstein(c.size(),Interval(0.));
  const unsigned degree=c.size()-1;
  for(unsigned i=0;i<=degree;++i)
    for(unsigned k=0;k<=i;++k)
      bernstein[i]+=c[k]*T[k]*double(choose(i,k))/double(choose(degree,k));
  hull(bernstein);
  return bernstein;
}
inline std::pair<Controls,Controls> split(const Controls& c,const Interval& fraction) {
  Controls row=c,left{row.front()},right{row.back()};
  const Interval u(fraction),v=Interval(1.)-u;
  while(row.size()>1) {
    for(std::size_t i=0;i+1<row.size();++i) row[i]=v*row[i]+u*row[i+1];
    row.pop_back();left.push_back(row.front());right.push_back(row.back());
  }
  std::reverse(right.begin(),right.end());return {left,right};
}
inline double normUpper(const Eigen::Vector3d& vector) {
  if(!vector.allFinite()) throw std::runtime_error("NONFINITE_VECTOR");
  Interval sum(0.);for(unsigned i=0;i<3;++i) sum+=boost::numeric::square(Interval(vector[i]));
  const auto norm=boost::numeric::sqrt(sum);finite(norm);return norm.upper();
}
inline double radius(const shapes::Shape& shape) {
  double r=0.;
  switch(shape.type) {
    case shapes::BOX: {
      const auto& b=static_cast<const shapes::Box&>(shape);
      for(double side:b.size) if(!std::isfinite(side)||side<=0.) throw std::runtime_error("BOX_SIZE_INVALID");
      r=normUpper(Eigen::Vector3d(b.size[0],b.size[1],b.size[2]))/2.;break;
    }
    case shapes::SPHERE:r=static_cast<const shapes::Sphere&>(shape).radius;break;
    case shapes::CYLINDER: {
      const auto& c=static_cast<const shapes::Cylinder&>(shape);
      if(!std::isfinite(c.radius)||!std::isfinite(c.length)||c.radius<=0.||c.length<=0.) throw std::runtime_error("SHAPE_SIZE_INVALID");
      r=normUpper(Eigen::Vector3d(c.radius,c.length/2.,0.));break;
    }
    case shapes::CONE: {
      const auto& c=static_cast<const shapes::Cone&>(shape);
      if(!std::isfinite(c.radius)||!std::isfinite(c.length)||c.radius<=0.||c.length<=0.) throw std::runtime_error("SHAPE_SIZE_INVALID");
      r=normUpper(Eigen::Vector3d(c.radius,c.length/2.,0.));break;
    }
    case shapes::MESH: {
      const auto& m=static_cast<const shapes::Mesh&>(shape);
      if(!m.vertex_count||!m.triangle_count) throw std::runtime_error("EMPTY_COLLISION_MESH");
      for(unsigned i=0;i<m.vertex_count;++i) {
        Eigen::Map<const Eigen::Vector3d> vertex(m.vertices+3*i);
        if(!vertex.allFinite()) throw std::runtime_error("NONFINITE_COLLISION_MESH");
        r=std::max(r,normUpper(vertex));
      }
      break;
    }
    default:throw std::runtime_error("UNBOUNDED_OR_UNSUPPORTED_MOVING_SHAPE");
  }
  if(!std::isfinite(r)||r<=0.) throw std::runtime_error("INVALID_GEOMETRY_RADIUS");
  return std::nextafter(r,std::numeric_limits<double>::infinity());
}
// Every ancestor contributes. The downstream radius includes fixed offsets,
// collision origin, actual shape radius, and the full prismatic position hull.
// Mimic polynomials have already been expanded with their multiplier/offset.
inline double displacement(const moveit::core::LinkModel* link,double shape_radius,
    const Polynomials& polynomials,const moveit::core::RobotState& anchor,
    const std::map<std::string,double>& errors={}) {
  Interval reach(shape_radius),travel(0.);unsigned chain_length=0;
  for(auto* cursor=link;cursor;cursor=cursor->getParentLinkModel()) {
    ++chain_length;
    const auto* joint=cursor->getParentJointModel();
    if(joint->getVariableCount()) {
      if(joint->getVariableCount()!=1) throw std::runtime_error("MULTIVARIABLE_JOINT_UNSUPPORTED");
      const auto& name=joint->getVariableNames()[0];
      const auto range=hull(polynomials.at(name));
      // Include floating evaluation/FK roundoff in the direction of rejection.
      const double error=256.*std::numeric_limits<double>::epsilon()*(1.+magnitude(range))+(errors.count(name)?errors.at(name):0.);
      const Interval delta=magnitude(range-Interval(anchor.getVariablePosition(name)))+Interval(error);
      if(joint->getType()==moveit::core::JointModel::PRISMATIC) {
        travel+=delta;reach+=Interval(std::max(magnitude(range+Interval(-error,error)),std::abs(anchor.getVariablePosition(name))));
      } else if(joint->getType()==moveit::core::JointModel::REVOLUTE) travel+=delta*reach;
      else throw std::runtime_error("JOINT_TYPE_UNSUPPORTED");
    }
    const auto& offset=cursor->getJointOriginTransform();
    if(!offset.matrix().allFinite()) throw std::runtime_error("NONFINITE_JOINT_TRANSFORM");
    reach+=Interval(normUpper(offset.translation()));
  }
  const double neps=128.*double(chain_length+4)*std::numeric_limits<double>::epsilon();
  if(neps>=.5) throw std::runtime_error("TRANSFORM_ERROR_UNBOUNDED");
  travel+=reach*(neps/(1.-neps));
  finite(travel);return travel.upper();
}
inline std::map<std::string,double> bodyDisplacements(const planning_scene::PlanningScene& scene,
    const moveit::core::RobotState& anchor,const Polynomials& p,const std::map<std::string,double>& errors={}) {
  std::map<std::string,double> result;
  const auto& env=scene.getCollisionEnv();
  for(const auto* link:scene.getRobotModel()->getLinkModelsWithCollisionGeometry()) {
    double reach=0.;
    const double padding=env->getLinkPadding(link->getName()),scale=env->getLinkScale(link->getName());
    // Scaling/padding can change mesh vertices by implementation-specific rules.
    // The current Stage A model uses neither; unknown inflation fails closed.
    if(padding!=0.||scale!=1.) throw std::runtime_error("PADDED_GEOMETRY_BOUND_UNSUPPORTED");
    for(std::size_t k=0;k<link->getShapes().size();++k)
      reach=std::max(reach,(Interval(normUpper(link->getCollisionOriginTransforms()[k].translation()))+radius(*link->getShapes()[k])).upper());
    result[link->getName()]=displacement(link,reach,p,anchor,errors);
  }
  std::vector<const moveit::core::AttachedBody*> bodies;anchor.getAttachedBodies(bodies);
  for(const auto* body:bodies) {
    double reach=0.;
    for(std::size_t k=0;k<body->getShapes().size();++k)
      reach=std::max(reach,(Interval(normUpper(body->getPose().translation()))+normUpper(body->getShapePoses()[k].translation())+radius(*body->getShapes()[k])).upper());
    result[body->getName()]=displacement(body->getAttachedLink(),reach,p,anchor,errors);
  }
  return result;
}
struct GeometryBody {
  std::string name;
  collision_detection::BodyType type;
  std::vector<shapes::ShapeConstPtr> shapes;
  EigenSTL::vector_Isometry3d poses;
  std::set<std::string> touch_links;
};
using GeometryBodies=std::map<std::string,GeometryBody>;
inline GeometryBodies geometryBodies(const planning_scene::PlanningScene& scene,const moveit::core::RobotState& state) {
  GeometryBodies bodies;
  auto add=[&](GeometryBody body) {
    if(body.shapes.empty()||body.shapes.size()!=body.poses.size()) throw std::runtime_error("COLLISION_GEOMETRY_INCOMPLETE");
    for(std::size_t i=0;i<body.shapes.size();++i) {
      if(!body.shapes[i]||!body.poses[i].matrix().allFinite()) throw std::runtime_error("COLLISION_GEOMETRY_NONFINITE");
      radius(*body.shapes[i]); // validates finite, bounded supported geometry
      if(body.shapes[i]->type==shapes::MESH) {
        const auto& mesh=static_cast<const shapes::Mesh&>(*body.shapes[i]);
        for(unsigned k=0;k<mesh.triangle_count*3;++k)
          if(mesh.triangles[k]>=mesh.vertex_count) throw std::runtime_error("MESH_INDEX_INVALID");
      }
    }
    if(!bodies.emplace(body.name,std::move(body)).second) throw std::runtime_error("COLLISION_BODY_ID_AMBIGUOUS");
  };
  for(const auto* link:scene.getRobotModel()->getLinkModelsWithCollisionGeometry()) {
    GeometryBody body{link->getName(),collision_detection::BodyTypes::ROBOT_LINK,link->getShapes(),{}, {}};
    for(std::size_t i=0;i<body.shapes.size();++i) body.poses.push_back(state.getCollisionBodyTransform(link,i));
    for(const auto* env:{scene.getCollisionEnv().get(),scene.getCollisionEnvUnpadded().get()})
      if(env->getLinkPadding(body.name)!=0.||env->getLinkScale(body.name)!=1.)
        throw std::runtime_error("PADDED_GEOMETRY_BOUND_UNSUPPORTED");
    add(std::move(body));
  }
  std::vector<const moveit::core::AttachedBody*> attached;state.getAttachedBodies(attached);
  for(const auto* body:attached) add({body->getName(),collision_detection::BodyTypes::ROBOT_ATTACHED,
    body->getShapes(),body->getGlobalCollisionBodyTransforms(),body->getTouchLinks()});
  for(const auto& id:scene.getWorld()->getObjectIds()) {
    const auto obj=scene.getWorld()->getObject(id);
    add({id,collision_detection::BodyTypes::WORLD_OBJECT,obj->shapes_,obj->global_shape_poses_,{}});
  }
  return bodies;
}
inline bool requiredPair(const GeometryBody& a,const GeometryBody& b,
    const collision_detection::AllowedCollisionMatrix& acm) {
  using namespace collision_detection;
  if(a.type==BodyTypes::WORLD_OBJECT&&b.type==BodyTypes::WORLD_OBJECT) return false;
  AllowedCollision::Type type;
  if(acm.getAllowedCollision(a.name,b.name,type)&&type==AllowedCollision::ALWAYS) return false;
  if(a.type==BodyTypes::ROBOT_ATTACHED&&b.type==BodyTypes::ROBOT_LINK&&a.touch_links.count(b.name)) return false;
  if(b.type==BodyTypes::ROBOT_ATTACHED&&a.type==BodyTypes::ROBOT_LINK&&b.touch_links.count(a.name)) return false;
  return true;
}
inline Interval projection(const GeometryBody& body,const Eigen::Vector3d& axis) {
  double low=INFINITY,high=-INFINITY;
  for(std::size_t k=0;k<body.shapes.size();++k) {
    const auto& transform=body.poses[k];const auto& shape=*body.shapes[k];
    Interval center(0.);Interval direction[3]={Interval(0.),Interval(0.),Interval(0.)};
    for(unsigned i=0;i<3;++i) {
      center+=Interval(axis[i])*transform.translation()[i];
      for(unsigned j=0;j<3;++j) direction[j]+=Interval(axis[i])*transform.linear()(i,j);
    }
    Interval extent(0.);Interval projected(0.);
    if(shape.type==shapes::MESH) {
      const auto& mesh=static_cast<const shapes::Mesh&>(shape);double lo=INFINITY,hi=-INFINITY;
      for(unsigned i=0;i<mesh.vertex_count;++i) {
        Interval value(0.);for(unsigned j=0;j<3;++j) value+=direction[j]*mesh.vertices[3*i+j];
        finite(value);lo=std::min(lo,value.lower());hi=std::max(hi,value.upper());
      }
      projected=center+Interval(lo,hi);
    } else {
      if(shape.type==shapes::BOX) {
        const auto& box=static_cast<const shapes::Box&>(shape);
        for(unsigned j=0;j<3;++j) {
          if(!std::isfinite(box.size[j])||box.size[j]<=0.) throw std::runtime_error("BOX_SIZE_INVALID");
          extent+=boost::numeric::abs(direction[j])*box.size[j]/2.;
        }
      } else if(shape.type==shapes::SPHERE) {
        const double r=static_cast<const shapes::Sphere&>(shape).radius;
        if(!std::isfinite(r)||r<=0.) throw std::runtime_error("SPHERE_RADIUS_INVALID");
        extent=boost::numeric::sqrt(boost::numeric::square(direction[0])+boost::numeric::square(direction[1])+boost::numeric::square(direction[2]))*r;
      } else if(shape.type==shapes::CYLINDER||shape.type==shapes::CONE) {
        double r,length;
        if(shape.type==shapes::CYLINDER) {const auto& s=static_cast<const shapes::Cylinder&>(shape);r=s.radius;length=s.length;}
        else {const auto& s=static_cast<const shapes::Cone&>(shape);r=s.radius;length=s.length;}
        if(!std::isfinite(r)||!std::isfinite(length)||r<=0.||length<=0.) throw std::runtime_error("CYLINDER_SIZE_INVALID");
        extent=boost::numeric::sqrt(boost::numeric::square(direction[0])+boost::numeric::square(direction[1]))*r+boost::numeric::abs(direction[2])*length/2.;
      } else throw std::runtime_error("SHAPE_PROJECTION_UNSUPPORTED");
      finite(extent);projected=center+Interval(-extent.upper(),extent.upper());
    }
    finite(projected);low=std::min(low,projected.lower());high=std::max(high,projected.upper());
  }
  return Interval(low,high);
}
// FCL/GJK distance can overestimate true clearance by more than its termination
// tolerance. Its nearest points supply only a candidate axis. Projection of ALL
// geometry vertices/supports gives an independent outward-rounded lower bound.
inline double clearanceLowerBound(const GeometryBody& a,const GeometryBody& b,
    const collision_detection::DistanceResultsData& distance,double needed=INFINITY,
    const std::map<std::string,std::array<Interval,3>>* coordinate_projections=nullptr) {
  std::vector<Eigen::Vector3d> axes{Eigen::Vector3d::UnitX(),Eigen::Vector3d::UnitY(),Eigen::Vector3d::UnitZ()};
  const Eigen::Vector3d difference=distance.nearest_points[1]-distance.nearest_points[0];
  if(difference.allFinite()&&difference.norm()>0.&&std::isfinite(difference.norm())) axes.push_back(difference.normalized());
  double lower=0.;
  for(std::size_t i=0;i<axes.size();++i) {
    const auto& axis=axes[i];
    const auto x=coordinate_projections&&i<3?coordinate_projections->at(a.name)[i]:projection(a,axis);
    const auto y=coordinate_projections&&i<3?coordinate_projections->at(b.name)[i]:projection(b,axis);
    const Interval gap1=Interval(y.lower())-x.upper(),gap2=Interval(x.lower())-y.upper();
    Interval norm(0.);for(unsigned j=0;j<3;++j) norm+=boost::numeric::square(Interval(axis[j]));
    norm=boost::numeric::sqrt(norm);
    const double gap=std::max(gap1.lower(),gap2.lower());
    if(gap>0.) lower=std::max(lower,(Interval(gap)/norm).lower());
    if(lower>needed) return lower;
  }
  return lower;
}

// Narrow analytic extension of the existing typed conditional policies. A box
// translated monotonically upward from a horizontal box face cannot deepen its
// initial contact or recontact that face. Arbitrary callbacks/mesh contacts,
// rotations and lateral motion have no such certificate and remain unmasked.
inline bool upwardBoxContact(const planning_scene::PlanningScene& scene,
    const moveit::core::RobotState& state,const Polynomials& p,const std::string& object,
    const std::string& world,const SupportContact* support,const std::map<std::string,double>& errors,
    int64_t span_ns,bool boundary_monotone) {
  const auto* body=state.getAttachedBody(object);const auto obstacle=scene.getWorld()->getObject(world);
  if(!body||!obstacle||body->getShapes().size()!=1||obstacle->shapes_.size()!=1||
     body->getShapes()[0]->type!=shapes::BOX||obstacle->shapes_[0]->type!=shapes::BOX) return false;
  const auto& a=body->getGlobalCollisionBodyTransforms()[0];const auto& b=obstacle->global_shape_poses_[0];
  if((a.linear()-Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff()!=0.||
     (b.linear()-Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff()!=0.) return false;
  Interval evaluation_motion(0.),vertical_range(0.),minimum_advance(0.),fk_scale(0.);
  unsigned chain_length=0;
  for(auto* link=body->getAttachedLink();link;link=link->getParentLinkModel()) {
    ++chain_length;fk_scale+=Interval(normUpper(link->getJointOriginTransform().translation()));
    const auto* joint=link->getParentJointModel();if(!joint->getVariableCount()) continue;
    const auto& c=p.at(joint->getVariableNames()[0]);
    evaluation_motion+=Interval(errors.at(joint->getVariableNames()[0]));
    if(joint->getType()==moveit::core::JointModel::PRISMATIC) fk_scale+=Interval(magnitude(hull(c)));
    bool constant=hull(c).lower()==hull(c).upper();
    if(constant) continue;
    if(joint->getType()!=moveit::core::JointModel::PRISMATIC) return false;
    const auto* prism=static_cast<const moveit::core::PrismaticJointModel*>(joint);
    const Eigen::Vector3d axis=state.getGlobalLinkTransform(link).linear()*prism->getAxis();
    if(axis.x()!=0.||axis.y()!=0.||std::abs(axis.z())!=1.) return false;
    double minimum=INFINITY;
    for(std::size_t k=1;k<c.size();++k) {
      const Interval advance=axis.z()*(c[k]-c[k-1])*double(c.size()-1)/double(span_ns);
      if(advance.lower()<0.) return false;
      minimum=std::min(minimum,advance.lower());
    }
    if(!std::isfinite(minimum)) return false;
    minimum_advance+=Interval(minimum);
    vertical_range+=axis.z()*(hull(c)-state.getVariablePosition(joint->getVariableNames()[0]));
  }
  const auto& x=static_cast<const shapes::Box&>(*body->getShapes()[0]);
  const auto& y=static_cast<const shapes::Box&>(*obstacle->shapes_[0]);
  const double bottom=a.translation().z()-x.size[2]/2.;
  const double top=b.translation().z()+y.size[2]/2.;
  const double penetration=top-bottom;
  // Require a strict numerical margin; never enlarge the contact allowance.
  // Error bound for rigid 3x3/translation composition and face arithmetic.
  // 128 operations per ancestor plus four face/shape transforms exceeds the
  // multiply/add count; charge gamma_n times the absolute coordinate scale.
  const double neps=128.*double(chain_length+4)*std::numeric_limits<double>::epsilon();
  if(neps>=.5) return false;
  fk_scale+=Interval(normUpper(a.translation()))+normUpper(b.translation())+
    normUpper(body->getPose().translation())+normUpper(body->getShapePoses()[0].translation())+
    x.size[2]/2.+y.size[2]/2.;
  const double guard=(evaluation_motion*2.+fk_scale*(neps/(1.-neps))+std::numeric_limits<double>::min()).upper();
  if(!std::isfinite(guard)||penetration+guard>support_contact_tolerance_m) return false;
  // Depth headroom alone cannot prove irreversible expiry. Either the entire
  // error tube stays in contact, or evaluated positions must strictly increase
  // on JTC's int64 nanosecond time lattice through separation. The nominal
  // continuous polynomial is also monotone (derivative hull above). Check the
  // switch to the exact stored waypoint separately to cover controller boundaries.
  const bool remains_in_contact=(vertical_range+guard).upper()<=penetration;
  if(!remains_in_contact&&(minimum_advance.lower()<=2.*guard||!boundary_monotone)) return false;
  // A previously separated upright pair is also certified by strict monotone
  // separation; this does not re-enable its contact allowance.
  if(penetration<0.) return minimum_advance.lower()>2.*guard&&boundary_monotone;
  for(unsigned axis=0;axis<2;++axis)
    if((x.size[axis]+y.size[axis])/2.-std::abs(a.translation()[axis]-b.translation()[axis])<=penetration+guard) return false;
  return !support||std::abs(top-support->floor_z)+penetration+guard<=support_contact_tolerance_m;
}

inline ControllerAuditReport certify(const robot_trajectory::RobotTrajectory& trajectory,
    const planning_scene::PlanningScene& scene,const ControllerAuditOptions& options={}) {
  ControllerAuditReport report;const auto started=std::chrono::steady_clock::now();
  auto finish=[&]() {report.wall_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();return report;};
  try {
    if(!options.stationary_world) throw std::runtime_error("MOVING_WORLD_WITHOUT_BOUND");
    if(!dynamic_cast<const collision_detection::CollisionEnvFCL*>(scene.getCollisionEnv().get())||
       !dynamic_cast<const collision_detection::CollisionEnvFCL*>(scene.getCollisionEnvUnpadded().get()))
      throw std::runtime_error("FCL_DISTANCE_REQUIRED");
    if(trajectory.getRobotModel()!=scene.getRobotModel()||trajectory.getWayPointCount()<2||
       trajectory.getWayPointCount()>120001) throw std::runtime_error("TRAJECTORY_MODEL_OR_COUNT_INVALID");
    double total=0.;
    for(std::size_t i=0;i<trajectory.getWayPointCount();++i) {
      const double dt=trajectory.getWayPointDurationFromPrevious(i);total+=dt;
      if(!std::isfinite(dt)||(i?dt<=0.:dt!=0.)||!std::isfinite(total)||total>120.)
        throw std::runtime_error("TRAJECTORY_TIMING_INVALID");
    }
    moveit_msgs::msg::RobotTrajectory message;trajectory.getRobotTrajectoryMsg(message);
    const auto& msg=message.joint_trajectory;
    if(msg.joint_names.empty()||!message.multi_dof_joint_trajectory.points.empty()||
       std::set<std::string>(msg.joint_names.begin(),msg.joint_names.end()).size()!=msg.joint_names.size())
      throw std::runtime_error("TRAJECTORY_JOINTS_INVALID");
    auto stamp=[](const auto& point){return int64_t(point.time_from_start.sec)*1000000000LL+point.time_from_start.nanosec;};
    for(std::size_t i=0;i<msg.points.size();++i) {
      const auto& point=msg.points[i];
      for(const auto* values:{&point.positions,&point.velocities,&point.accelerations})
        if((values==&point.positions||!values->empty())&&(values->size()!=msg.joint_names.size()||
          !std::all_of(values->begin(),values->end(),[](double v){return std::isfinite(v);})))
          throw std::runtime_error("TRAJECTORY_FIELDS_INVALID");
      if(point.time_from_start.nanosec>=1000000000U||(i?stamp(point)<=stamp(msg.points[i-1]):stamp(point)!=0))
        throw std::runtime_error("TRAJECTORY_TIMING_INVALID");
    }
    auto first=trajectory.getFirstWayPoint();first.update();
    geometryBodies(scene,first); // fail before FCL on malformed or unsupported geometry
    // Conditional permissions are identifiable policies, not arbitrary sampled
    // predicates. Keep exact pairs and never change the PlanningScene ACM.
    struct Conditional {std::string object,world;SupportContact support;bool floor=false;};
    std::vector<Conditional> initial;
    std::vector<const moveit::core::AttachedBody*> attached;first.getAttachedBodies(attached);
    for(const auto* body:attached) for(const auto& id:scene.getWorld()->getObjectIds()) {
      collision_detection::DecideContactFn fn;
      if(!scene.getAllowedCollisionMatrix().getAllowedCollision(body->getName(),id,fn)) continue;
      if(const auto* policy=fn.target<SupportContact>()) initial.push_back({body->getName(),id,*policy,true});
      else if(fn.target<PileContact>()) initial.push_back({body->getName(),id,{},false});
    }
    joint_trajectory_controller::Trajectory jtc;
    for(std::size_t segment=1;segment<msg.points.size();++segment) {
      const auto& a=msg.points[segment-1];const auto& b=msg.points[segment];
      const int64_t begin=stamp(a),end=stamp(b);
      Polynomials p;std::map<std::string,double> errors;
      for(std::size_t j=0;j<msg.joint_names.size();++j) {
        if(std::find(scene.getRobotModel()->getVariableNames().begin(),scene.getRobotModel()->getVariableNames().end(),msg.joint_names[j])==scene.getRobotModel()->getVariableNames().end()) throw std::runtime_error("JOINT_UNKNOWN");
        p.emplace(msg.joint_names[j],polynomial(a,b,j,end-begin,&errors[msg.joint_names[j]]));
      }
      std::function<Controls(const moveit::core::JointModel*,unsigned)> expand;
      expand=[&](const auto* joint,unsigned depth)->Controls {
        if(depth>scene.getRobotModel()->getJointModels().size()||joint->getVariableCount()!=1)
          throw std::runtime_error("JOINT_EXPANSION_UNSUPPORTED");
        const auto& name=joint->getVariableNames()[0];
        if(joint->getMimic()) {
          auto c=expand(joint->getMimic(),depth+1);
          for(auto& v:c) v=v*joint->getMimicFactor()+joint->getMimicOffset();
          errors[name]=(Interval(errors.at(joint->getMimic()->getVariableNames()[0]))*std::abs(joint->getMimicFactor())+
            16.*std::numeric_limits<double>::epsilon()*(1.+magnitude(hull(c)))).upper();
          p[name]=c;return c;
        }
        if(p.count(name)) return p.at(name);
        const double q=first.getVariablePosition(name);
        for(std::size_t k=0;k<trajectory.getWayPointCount();++k)
          if(trajectory.getWayPoint(k).getVariablePosition(name)!=q) throw std::runtime_error("UNCOMMANDED_JOINT_MOVED");
        p[name]={Interval(q)};errors[name]=0.;return p.at(name);
      };
      for(const auto* joint:scene.getRobotModel()->getJointModels()) if(joint->getVariableCount()) expand(joint,0);
      auto evaluate=[&](int64_t time) {
        trajectory_msgs::msg::JointTrajectoryPoint sample;
        jtc.interpolate_between_points(rclcpp::Time(begin),a,rclcpp::Time(end),b,rclcpp::Time(time),sample);
        // sample() selects the next segment's start or the stored final point
        // at a boundary; it does not evaluate the previous spline at T there.
        if(time==end) {
          sample=b;
          if(sample.velocities.empty()) sample.velocities.resize(msg.joint_names.size(),0.);
          if(sample.accelerations.empty()) sample.accelerations.resize(msg.joint_names.size(),0.);
        }
        auto state=first;
        for(const auto* values:{&sample.positions,&sample.velocities,&sample.accelerations})
          if(values->size()!=msg.joint_names.size()||!std::all_of(values->begin(),values->end(),[](double v){return std::isfinite(v);}))
            throw std::runtime_error("INTERPOLATION_NONFINITE");
        for(std::size_t j=0;j<msg.joint_names.size();++j) state.setVariablePosition(msg.joint_names[j],sample.positions[j]);
        state.update();return state;
      };
      std::function<bool(int64_t,int64_t,const Polynomials&,unsigned)> visit;
      visit=[&](int64_t lo,int64_t hi,const Polynomials& controls,unsigned depth) {
        report.failure_begin_ns=lo;report.failure_end_ns=hi;
        if(++report.inspected>options.max_intervals) {report.reason="INTERVAL_LIMIT";return false;}
        report.deepest=std::max(report.deepest,depth);
        const int64_t mid=lo+(hi-lo)/2;
        const auto anchor=evaluate(mid);
        // Midpoint is a collision witness, never a clearance certificate.
        for(int64_t t:{lo,mid,hi}) {
          auto state=t==mid?anchor:evaluate(t);
          if(!state.satisfiesBounds()) {report.reason="JOINT_LIMIT";return false;}
          if(scene.isStateColliding(state,"")) {report.result=ControllerCertificate::COLLISION;report.reason="FCL_COLLISION";return false;}
        }
        const auto movement=bodyDisplacements(scene,anchor,controls,errors);
        auto acm=scene.getAllowedCollisionMatrix();
        const auto start=evaluate(lo);
        for(const auto& pair:initial) {
          bool boundary_monotone=true;
          if(hi==end) {
            const auto prior=evaluate(end-1),terminal=evaluate(end);
            boundary_monotone=terminal.getAttachedBody(pair.object)->getGlobalPose().translation().z()>=
                              prior.getAttachedBody(pair.object)->getGlobalPose().translation().z();
          }
          if(upwardBoxContact(scene,start,controls,pair.object,pair.world,pair.floor?&pair.support:nullptr,
              errors,hi-lo,boundary_monotone))
            acm.setEntry(pair.object,pair.world,true); // distance query only; separate analytic proof above
        }
        const auto geometry=geometryBodies(scene,anchor);
        std::map<std::string,std::array<Interval,3>> coordinate_projections;
        for(const auto& body:geometry)
          coordinate_projections[body.first]={projection(body.second,Eigen::Vector3d::UnitX()),
            projection(body.second,Eigen::Vector3d::UnitY()),projection(body.second,Eigen::Vector3d::UnitZ())};
        // Cheap support-plane certificates first. Excluding an already-proven
        // pair from the private distance query avoids expensive far-mesh GJK;
        // it does not alter scene policy or omit an unresolved pair.
        collision_detection::DistanceResultsData no_axis;
        for(auto a=geometry.begin();a!=geometry.end();++a) for(auto b=std::next(a);b!=geometry.end();++b) {
          if(!requiredPair(a->second,b->second,acm)) continue;
          Interval relative(1e-12);
          for(const auto* body:{&a->second,&b->second})
            if(body->type!=collision_detection::BodyTypes::WORLD_OBJECT) relative+=Interval(movement.at(body->name));
          if(clearanceLowerBound(a->second,b->second,no_axis,relative.upper(),&coordinate_projections)>relative.upper())
            acm.setEntry(a->first,b->first,true);
        }
        collision_detection::DistanceRequest req;req.type=collision_detection::DistanceRequestType::SINGLE;req.acm=&acm;req.enable_nearest_points=true;
        collision_detection::DistanceResult world,self;
        scene.getCollisionEnv()->distanceRobot(req,world,anchor);
        scene.getCollisionEnvUnpadded()->distanceSelf(req,self,anchor);
        bool clear=!world.collision&&!self.collision;
        for(auto a=geometry.begin();a!=geometry.end();++a) for(auto b=std::next(a);b!=geometry.end();++b) {
          if(!requiredPair(a->second,b->second,acm)) continue;
          const bool robot_world=a->second.type==collision_detection::BodyTypes::WORLD_OBJECT||b->second.type==collision_detection::BodyTypes::WORLD_OBJECT;
          // FCL legitimately stops a distance traversal at its first overlap.
          // Such an interval is unresolved and subdivided, never certified from
          // the partial map. A collision-free traversal must cover every pair.
          if(robot_world?world.collision:self.collision) {clear=false;continue;}
          const auto& distances=robot_world?world.distances:self.distances;
          const auto found=distances.find({a->first,b->first});
          if(found==distances.end()||found->second.size()!=1) throw std::runtime_error("FCL_PAIR_COVERAGE_INCOMPLETE");
          const auto& d=found->second.front();
          if(!std::isfinite(d.distance)) throw std::runtime_error("FCL_DISTANCE_NONFINITE");
          Interval relative(0.);
          for(const auto* body:{&a->second,&b->second})
            if(body->type!=collision_detection::BodyTypes::WORLD_OBJECT) relative+=Interval(movement.at(body->name));
          // Spatial roundoff is charged against clearance, never forgiven as contact.
          constexpr double distance_guard=1e-12;
          const double lower=clearanceLowerBound(a->second,b->second,d,(relative+distance_guard).upper(),&coordinate_projections);
          if(d.distance<=0.||lower<=(relative+distance_guard).upper()) clear=false;
        }
        // Bernstein hull also excludes unsampled position-limit violations.
        for(const auto& item:controls) {
          const auto range=hull(item.second)+Interval(-errors.at(item.first),errors.at(item.first));const auto& bounds=scene.getRobotModel()->getVariableBounds(item.first);
          if(bounds.position_bounded_&&(range.lower()<bounds.min_position_||range.upper()>bounds.max_position_)) clear=false;
        }
        if(clear) {++report.certified;return true;}
        if(depth>=options.max_depth||hi-lo<=options.min_interval_ns||mid==lo||mid==hi) {
          report.reason="PRECISION_OR_DEPTH_LIMIT";return false;
        }
        ++report.subdivided;Polynomials left,right;
        for(const auto& item:controls) {
          auto halves=split(item.second,Interval(double(mid-lo))/Interval(double(hi-lo)));
          left[item.first]=std::move(halves.first);right[item.first]=std::move(halves.second);
        }
        return visit(lo,mid,left,depth+1)&&visit(mid,hi,right,depth+1);
      };
      if(!visit(begin,end,p,0)) return finish();
    }
    report.result=ControllerCertificate::CERTIFIED_CLEAR;report.reason="ALL_INTERVALS_CERTIFIED";
    report.failure_begin_ns=report.failure_end_ns=-1;
  } catch(const std::exception& e) {report.result=ControllerCertificate::UNCERTIFIED;report.reason=e.what();}
  catch(...) {report.result=ControllerCertificate::UNCERTIFIED;report.reason="UNKNOWN_CERTIFICATION_FAILURE";}
  return finish();
}
} // namespace controller_certificate
} // namespace workcell
