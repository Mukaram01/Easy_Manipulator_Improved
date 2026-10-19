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
  std::size_t max_contact_timestamps=120001;
  int64_t min_interval_ns=1;
  bool stationary_world=true;
  // Optional owner interruption check; exceptions fail certification closed.
  std::function<void()> checkpoint;
};
struct ControllerAuditReport {
  ControllerCertificate result=ControllerCertificate::UNCERTIFIED;
  std::size_t inspected=0, certified=0, subdivided=0;
  std::size_t contact_timestamps=0, initial_contacts_expired=0;
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
inline double validatedRadius(const shapes::Shape& shape) {
  const double result=radius(shape);
  if(shape.type==shapes::MESH) {
    const auto& mesh=static_cast<const shapes::Mesh&>(shape);
    for(unsigned k=0;k<mesh.triangle_count*3;++k)
      if(mesh.triangles[k]>=mesh.vertex_count) throw std::runtime_error("MESH_INDEX_INVALID");
  }
  return result;
}
// A frozen certificate shares immutable shape geometry across all intervals.
// Keep ownership with each pointer key, and discard these bounds after that
// certificate. Poses, body identities, padding and scale are never cached.
class ShapeValidationCache {
 public:
  double radius(const shapes::ShapeConstPtr& shape) {
    if(!shape) throw std::runtime_error("COLLISION_GEOMETRY_NONFINITE");
    const auto found=entries_.find(shape.get());
    if(found!=entries_.end()) return found->second.second;
    const double result=validatedRadius(*shape);
    entries_.emplace(shape.get(),std::make_pair(shape,result));
    return result;
  }
 private:
  std::map<const shapes::Shape*,std::pair<shapes::ShapeConstPtr,double>> entries_;
};
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
    const moveit::core::RobotState& anchor,const Polynomials& p,const std::map<std::string,double>& errors={},
    ShapeValidationCache* shape_cache=nullptr) {
  std::map<std::string,double> result;
  const auto shape_radius=[&](const shapes::ShapeConstPtr& shape) {
    return shape_cache?shape_cache->radius(shape):radius(*shape);
  };
  const auto& env=scene.getCollisionEnv();
  for(const auto* link:scene.getRobotModel()->getLinkModelsWithCollisionGeometry()) {
    double reach=0.;
    const double padding=env->getLinkPadding(link->getName()),scale=env->getLinkScale(link->getName());
    // Scaling/padding can change mesh vertices by implementation-specific rules.
    // The current Stage A model uses neither; unknown inflation fails closed.
    if(padding!=0.||scale!=1.) throw std::runtime_error("PADDED_GEOMETRY_BOUND_UNSUPPORTED");
    for(std::size_t k=0;k<link->getShapes().size();++k)
      reach=std::max(reach,(Interval(normUpper(link->getCollisionOriginTransforms()[k].translation()))+shape_radius(link->getShapes()[k])).upper());
    result[link->getName()]=displacement(link,reach,p,anchor,errors);
  }
  std::vector<const moveit::core::AttachedBody*> bodies;anchor.getAttachedBodies(bodies);
  for(const auto* body:bodies) {
    double reach=0.;
    for(std::size_t k=0;k<body->getShapes().size();++k)
      reach=std::max(reach,(Interval(normUpper(body->getPose().translation()))+normUpper(body->getShapePoses()[k].translation())+shape_radius(body->getShapes()[k])).upper());
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
inline GeometryBodies geometryBodies(const planning_scene::PlanningScene& scene,const moveit::core::RobotState& state,
    ShapeValidationCache* shape_cache=nullptr) {
  GeometryBodies bodies;
  auto add=[&](GeometryBody body) {
    if(body.shapes.empty()||body.shapes.size()!=body.poses.size()) throw std::runtime_error("COLLISION_GEOMETRY_INCOMPLETE");
    for(std::size_t i=0;i<body.shapes.size();++i) {
      if(!body.shapes[i]||!body.poses[i].matrix().allFinite()) throw std::runtime_error("COLLISION_GEOMETRY_NONFINITE");
      if(shape_cache) shape_cache->radius(body.shapes[i]);
      else validatedRadius(*body.shapes[i]);
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
  // One rounding guard covers interval-only arithmetic and restores the caller
  // on return or exception. FK/FCL computations remain outside this scope.
  using LocalInterval=boost::numeric::interval_lib::unprotect<Interval>::type;
  Interval::traits_type::rounding rounding_guard;
  double low=INFINITY,high=-INFINITY;
  for(std::size_t k=0;k<body.shapes.size();++k) {
    const auto& transform=body.poses[k];const auto& shape=*body.shapes[k];
    LocalInterval center(0.);LocalInterval direction[3]={LocalInterval(0.),LocalInterval(0.),LocalInterval(0.)};
    for(unsigned i=0;i<3;++i) {
      center+=LocalInterval(axis[i])*transform.translation()[i];
      for(unsigned j=0;j<3;++j) direction[j]+=LocalInterval(axis[i])*transform.linear()(i,j);
    }
    LocalInterval extent(0.);LocalInterval projected(0.);
    if(shape.type==shapes::MESH) {
      const auto& mesh=static_cast<const shapes::Mesh&>(shape);double lo=INFINITY,hi=-INFINITY;
      for(unsigned i=0;i<mesh.vertex_count;++i) {
        LocalInterval value(0.);for(unsigned j=0;j<3;++j) value+=direction[j]*mesh.vertices[3*i+j];
        finite(value);lo=std::min(lo,value.lower());hi=std::max(hi,value.upper());
      }
      projected=center+LocalInterval(lo,hi);
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
      finite(extent);projected=center+LocalInterval(-extent.upper(),extent.upper());
    }
    finite(projected);low=std::min(low,projected.lower());high=std::max(high,projected.upper());
  }
  return LocalInterval(low,high);
}
inline double projectedClearanceLowerBound(const Interval& x,const Interval& y,const Eigen::Vector3d& axis) {
  const Interval gap1=Interval(y.lower())-x.upper(),gap2=Interval(x.lower())-y.upper();
  Interval norm(0.);for(unsigned j=0;j<3;++j) norm+=boost::numeric::square(Interval(axis[j]));
  norm=boost::numeric::sqrt(norm);
  const double gap=std::max(gap1.lower(),gap2.lower());
  return gap>0.?(Interval(gap)/norm).lower():0.;
}
inline double triangleCoverClearanceLowerBound(const GeometryBody& body,const GeometryBody& surface,double needed) {
  // FCL tests a mesh through its original indexed triangles. Prove that same
  // surface cover against the full other body, retaining the MINIMUM gap. A
  // concave mesh's cavity must not become part of its convex support hull.
  if(body.shapes.size()!=1||surface.shapes.size()!=1||
      (body.shapes[0]->type!=shapes::BOX&&body.shapes[0]->type!=shapes::MESH)||
      surface.shapes[0]->type!=shapes::MESH) return 0.;
  const auto& mesh=static_cast<const shapes::Mesh&>(*surface.shapes[0]);
  if(!mesh.triangle_count) return 0.;
  auto triangle=std::make_shared<shapes::Mesh>(3,1);
  for(unsigned i=0;i<3;++i) triangle->triangles[i]=i;
  GeometryBody piece{surface.name,surface.type,{triangle},surface.poses,{}};
  // The full body's three coordinate projections are identical for every
  // triangle in this call. Reuse exactly those outward-rounded intervals.
  const std::array<Interval,3> coordinates{projection(body,Eigen::Vector3d::UnitX()),
    projection(body,Eigen::Vector3d::UnitY()),projection(body,Eigen::Vector3d::UnitZ())};
  double minimum=INFINITY;
  for(unsigned t=0;t<mesh.triangle_count;++t) {
    Eigen::Vector3d vertices[3];
    for(unsigned i=0;i<3;++i) {
      const unsigned id=mesh.triangles[3*t+i];
      if(id>=mesh.vertex_count) throw std::runtime_error("MESH_INDEX_INVALID");
      for(unsigned j=0;j<3;++j) vertices[i][j]=triangle->vertices[3*i+j]=mesh.vertices[3*id+j];
    }
    // These floating-point SAT directions are candidates only. All clearance
    // evidence comes from unchanged outward projection and interval norm.
    Eigen::Vector3d edges[3];
    for(unsigned i=0;i<3;++i) edges[i]=surface.poses[0].linear()*(vertices[(i+1)%3]-vertices[i]);
    std::vector<Eigen::Vector3d> axes{Eigen::Vector3d::UnitX(),Eigen::Vector3d::UnitY(),Eigen::Vector3d::UnitZ(),edges[0].cross(edges[1])};
    for(unsigned j=0;j<3;++j) {
      axes.push_back(body.poses[0].linear().col(j));
      for(const auto& edge:edges) axes.push_back(edge.cross(body.poses[0].linear().col(j)));
    }
    double lower=0.;
    for(std::size_t i=0;i<axes.size();++i) {
      const auto& axis=axes[i];
      if(!axis.allFinite()||!std::isfinite(axis.norm())||axis.norm()<=0.) continue;
      lower=std::max(lower,projectedClearanceLowerBound(i<3?coordinates[i]:projection(body,axis),projection(piece,axis),axis));
      if(lower>needed) break;
    }
    // An unresolved triangle cannot be omitted from the mesh-wide minimum.
    if(lower<=0.||(std::isfinite(needed)&&lower<=needed)) return 0.;
    minimum=std::min(minimum,lower);
  }
  return minimum;
}
// FCL/GJK distance can overestimate true clearance by more than its termination
// tolerance. Its nearest points supply only a candidate axis. Projection of ALL
// geometry vertices/supports gives an independent outward-rounded lower bound.
inline Eigen::Vector3d meshAxisCandidate(const GeometryBody& a,const GeometryBody& b,const Eigen::Vector3d& witness) {
  // Bounded Gilbert/Frank-Wolfe search on the vertex-cloud difference. This is
  // only a heuristic for choosing an axis: no floating-point search result is
  // a clearance bound. projection() must independently check every vertex.
  std::array<std::vector<Eigen::Vector3d>,2> points;
  const GeometryBody* bodies[2]={&a,&b};
  for(unsigned side=0;side<2;++side) {
    const auto& body=*bodies[side];
    for(std::size_t k=0;k<body.shapes.size();++k) {
      if(body.shapes[k]->type!=shapes::MESH) return Eigen::Vector3d::Zero();
      const auto& mesh=static_cast<const shapes::Mesh&>(*body.shapes[k]);
      for(unsigned i=0;i<mesh.vertex_count;++i)
        points[side].push_back(body.poses[k]*Eigen::Vector3d(mesh.vertices[3*i],mesh.vertices[3*i+1],mesh.vertices[3*i+2]));
    }
    if(points[side].empty()) return Eigen::Vector3d::Zero();
  }
  // Preserve the distance witness magnitude: this search minimizes a point's
  // squared norm, so normalizing the initial point changes its convergence.
  Eigen::Vector3d v=witness,best=Eigen::Vector3d::Zero();double best_gap=0.;
  for(unsigned iteration=0;iteration<32;++iteration) {
    const double length=v.norm();if(!v.allFinite()||!std::isfinite(length)||length<=0.) break;
    const auto x=std::max_element(points[0].begin(),points[0].end(),[&](const auto& p,const auto& q){return p.dot(v)<q.dot(v);});
    const auto y=std::min_element(points[1].begin(),points[1].end(),[&](const auto& p,const auto& q){return p.dot(v)<q.dot(v);});
    const Eigen::Vector3d w=*y-*x;const double gap=v.dot(w)/length;
    if(std::isfinite(gap)&&gap>best_gap) {best_gap=gap;best=v;}
    const Eigen::Vector3d delta=w-v;const double squared=delta.squaredNorm();
    if(!std::isfinite(squared)||squared<=0.) break;
    const double step=std::clamp(-v.dot(delta)/squared,0.,1.);
    if(!std::isfinite(step)||step<=0.) break;
    v+=step*delta;
  }
  return best;
}
inline double clearanceLowerBound(const GeometryBody& a,const GeometryBody& b,
    const collision_detection::DistanceResultsData& distance,double needed=INFINITY,
    const std::map<std::string,std::array<Interval,3>>* coordinate_projections=nullptr) {
  std::vector<Eigen::Vector3d> axes{Eigen::Vector3d::UnitX(),Eigen::Vector3d::UnitY(),Eigen::Vector3d::UnitZ()};
  const Eigen::Vector3d difference=distance.nearest_points[1]-distance.nearest_points[0];
  if(difference.allFinite()&&difference.norm()>0.&&std::isfinite(difference.norm())) axes.push_back(difference.normalized());
  auto checked_axis=[&](const Eigen::Vector3d& axis,std::size_t i) {
    const auto x=coordinate_projections&&i<3?coordinate_projections->at(a.name)[i]:projection(a,axis);
    const auto y=coordinate_projections&&i<3?coordinate_projections->at(b.name)[i]:projection(b,axis);
    return projectedClearanceLowerBound(x,y,axis);
  };
  double lower=0.;
  for(std::size_t i=0;i<axes.size();++i) {
    lower=std::max(lower,checked_axis(axes[i],i));
    if(lower>needed) return lower;
  }
  // Keep the coordinate-only prepass cheap. After a real distance query, an
  // inadequate nearest-point axis may be replaced by a mesh-hull candidate.
  if(axes.size()>3) {
    const Eigen::Vector3d candidate=meshAxisCandidate(a,b,difference);
    if(candidate.allFinite()&&candidate.norm()>0.&&std::isfinite(candidate.norm()))
      lower=std::max(lower,checked_axis(candidate,3));
    if(lower<=needed) lower=std::max(lower,triangleCoverClearanceLowerBound(a,b,needed));
    if(lower<=needed) lower=std::max(lower,triangleCoverClearanceLowerBound(b,a,needed));
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

} } // shared helpers
#include "detached_contact_certificate.hpp"
namespace workcell { namespace controller_certificate {
// Convex BOX proofs for initially permitted attached/world contact. The nominal
// polynomial and the controller's integer-nanosecond values are separate proof
// domains; an absolute FK error tube alone cannot prove irreversible separation.
using IVector=std::array<Interval,3>;
inline IVector cross(const IVector& a,const IVector& b) {
  return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};
}
inline IVector sum(const IVector& a,const IVector& b) {return {a[0]+b[0],a[1]+b[1],a[2]+b[2]};}
inline IVector column(const IMatrix& a,unsigned j) {return {a[0][j],a[1][j],a[2][j]};}
inline std::array<IVector,8> boxCorners(const IMatrix& pose,const shapes::Box& box) {
  std::array<IVector,8> result;
  for(unsigned k=0;k<8;++k) for(unsigned i=0;i<3;++i) {
    result[k][i]=pose[i][3];
    for(unsigned j=0;j<3;++j) result[k][i]+=pose[i][j]*((k&(1<<j)?1.:-1.)*box.size[j]/2.);
  }
  return result;
}
inline bool singleBox(const GeometryBody& body) {return body.shapes.size()==1&&body.shapes[0]->type==shapes::BOX;}
// Negative values bound the translation needed to separate the full shapes,
// rather than the intersection width (which is unsound for containment).
inline double boxGap(const GeometryBody& moving,const GeometryBody& fixed) {
  if(!singleBox(moving)||!singleBox(fixed)) return -INFINITY;
  std::vector<Eigen::Vector3d> axes;
  const auto& a=moving.poses[0].linear();const auto& b=fixed.poses[0].linear();
  for(unsigned i=0;i<3;++i) {
    axes.push_back(a.col((i+1)%3).cross(a.col((i+2)%3)));
    axes.push_back(b.col((i+1)%3).cross(b.col((i+2)%3)));
  }
  for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) axes.push_back(a.col(i).cross(b.col(j)));
  double best=-INFINITY;
  for(const auto& n:axes) {
    const double length=n.norm();if(!std::isfinite(length)||length==0.) continue;
    const Eigen::Vector3d axis=n/length;
    const auto x=projection(moving,axis),y=projection(fixed,axis);
    Interval norm2(0.);for(unsigned k=0;k<3;++k) norm2+=boost::numeric::square(Interval(axis[k]));
    const auto norm=boost::numeric::sqrt(norm2);if(norm.lower()<=0.) continue;
    best=std::max(best,std::max(((Interval(x.lower())-y.upper())/norm).lower(),
                             ((Interval(y.lower())-x.upper())/norm).lower()));
  }
  return best;
}
// A common interior point with more clearance to every moving face than the
// full body displacement cannot leave either convex solid during this interval.
// Floating polytope intersections only propose a witness. Every face inequality
// is subsequently checked with outward arithmetic; proposal tolerances confer
// no collision permission.
inline bool boxContactInterior(const GeometryBody& moving,const GeometryBody& fixed,double motion) {
  if(!singleBox(moving)||!singleBox(fixed)) return false;
  struct Face {IVector normal;Interval offset;double erosion;Eigen::Vector3d n;double d;};
  std::vector<Face> faces;
  for(const auto* body:{&moving,&fixed}) {
    const auto pose=intervalMatrix(body->poses[0]);const auto& box=static_cast<const shapes::Box&>(*body->shapes[0]);
    for(unsigned i=0;i<3;++i) for(double sign:{-1.,1.}) {
      auto n=cross(column(pose,(i+1)%3),column(pose,(i+2)%3));
      Interval determinant(0.);for(unsigned j=0;j<3;++j) determinant+=n[j]*pose[j][i];
      if(determinant.lower()<=0.) return false;
      Interval offset=determinant*box.size[i]/2.,norm2(0.);
      Eigen::Vector3d proposal;
      for(unsigned j=0;j<3;++j) {
        n[j]*=sign;offset+=n[j]*pose[j][3];norm2+=boost::numeric::square(n[j]);
        proposal[j]=boost::numeric::median(n[j]);
      }
      const double erosion=body==&moving?(boost::numeric::sqrt(norm2)*motion).upper():0.;
      faces.push_back({n,offset,erosion,proposal,boost::numeric::median(offset)});
    }
  }
  Eigen::Vector3d centroid=Eigen::Vector3d::Zero();unsigned count=0;
  for(unsigned i=0;i<faces.size();++i) for(unsigned j=i+1;j<faces.size();++j) for(unsigned k=j+1;k<faces.size();++k) {
    Eigen::Matrix3d matrix;matrix.row(0)=faces[i].n;matrix.row(1)=faces[j].n;matrix.row(2)=faces[k].n;
    if(std::abs(matrix.determinant())<1e-12) continue;
    const Eigen::Vector3d point=matrix.inverse()*Eigen::Vector3d(faces[i].d,faces[j].d,faces[k].d);
    if(!point.allFinite()) continue;
    bool inside=true;for(const auto& f:faces) if(f.n.dot(point)>f.d+1e-12) {inside=false;break;}
    if(inside) {centroid+=point;++count;}
  }
  if(!count) return false;
  centroid/=count;
  for(const auto& f:faces) {
    Interval margin=f.offset;
    for(unsigned i=0;i<3;++i) margin-=f.normal[i]*centroid[i];
    if(margin.lower()<=f.erosion) return false;
  }
  return true;
}
// SAT for the continuous nominal BOXes, with consistently labelled axes across
// subdivisions. Every potentially separating signed gap must be nondecreasing;
// a gap proved strictly negative throughout cannot cause expiry in that leaf.
// Include axis derivatives: the attached box may rotate about revolute joints.
inline bool nominalBoxNoReentry(const planning_scene::PlanningScene& scene,
    const moveit::core::AttachedBody& body,const GeometryBody& fixed,
    const Polynomials& p,const Polynomials& speeds) {
  if(body.getShapes().size()!=1||body.getShapes()[0]->type!=shapes::BOX||!singleBox(fixed)) return false;
  const auto origin=multiply(intervalMatrix(body.getPose()),intervalMatrix(body.getShapePoses()[0]));
  IMatrix link;linkDerivative(body.getAttachedLink(),p,"",&link);
  const auto a=multiply(link,origin),b=intervalMatrix(fixed.poses[0]);
  auto ad=zeroMatrix();
  for(const auto* joint:scene.getRobotModel()->getJointModels()) {
    if(!joint->getVariableCount()||joint->getMimic()) continue;
    const auto& name=joint->getVariableNames().at(0);const auto speed=hull(speeds.at(name));
    if(speed.lower()==0.&&speed.upper()==0.) continue;
    auto d=multiply(linkDerivative(body.getAttachedLink(),p,name),origin);
    for(auto& row:d) for(auto& value:row) value*=speed;
    ad=addMatrix(ad,d);
  }
  const auto& box=static_cast<const shapes::Box&>(*body.getShapes()[0]);
  const auto av=boxCorners(a,box),adv=boxCorners(ad,box),bv=boxCorners(b,static_cast<const shapes::Box&>(*fixed.shapes[0]));
  const IVector zero{Interval(0.),Interval(0.),Interval(0.)};
  std::vector<std::pair<IVector,IVector>> axes;
  for(unsigned i=0;i<3;++i) {
    const unsigned j=(i+1)%3,k=(i+2)%3;
    axes.push_back({cross(column(b,j),column(b,k)),zero});
    axes.push_back({cross(column(a,j),column(a,k)),sum(cross(column(ad,j),column(a,k)),cross(column(a,j),column(ad,k)))});
  }
  for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j)
    axes.push_back({cross(column(b,i),column(a,j)),cross(column(b,i),column(ad,j))});
  for(const auto& axis:axes) {
    const auto& n=axis.first;const auto& nd=axis.second;Interval norm2(0.);bool exact_zero=true;
    for(const auto& x:n) {norm2+=boost::numeric::square(x);exact_zero&=x.lower()==0.&&x.upper()==0.;}
    if(exact_zero) continue;
    if(norm2.lower()<=0.) return false;
    for(double sign:{-1.,1.}) {
      double gap_upper=INFINITY,minimum_speed=INFINITY;
      for(unsigned i=0;i<8;++i) for(unsigned j=0;j<8;++j) {
        Interval gap(0.),speed(0.);
        for(unsigned k=0;k<3;++k) {
          const auto delta=av[i][k]-bv[j][k];
          gap+=sign*n[k]*delta;speed+=sign*(nd[k]*delta+n[k]*adv[i][k]);
        }
        gap_upper=std::min(gap_upper,gap.upper());minimum_speed=std::min(minimum_speed,speed.lower());
      }
      if(!(gap_upper<0.||minimum_speed>=0.)) return false;
    }
  }
  return true;
}

inline ControllerAuditReport certify(const robot_trajectory::RobotTrajectory& trajectory,
    const planning_scene::PlanningScene& scene,const ControllerAuditOptions& options={},
    DetachedAudit* detached=nullptr,std::size_t contact_cap=1000000) {
  ControllerAuditReport report;const auto started=std::chrono::steady_clock::now();
  auto finish=[&]() {report.wall_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();return report;};
  const auto checkpoint=[&] { if(options.checkpoint) options.checkpoint(); };
  try {
    checkpoint();
    if(!options.stationary_world) throw std::runtime_error("MOVING_WORLD_WITHOUT_BOUND");
    if(!dynamic_cast<const collision_detection::CollisionEnvFCL*>(scene.getCollisionEnv().get())||
       !dynamic_cast<const collision_detection::CollisionEnvFCL*>(scene.getCollisionEnvUnpadded().get()))
      throw std::runtime_error("FCL_DISTANCE_REQUIRED");
    if(trajectory.getRobotModel()!=scene.getRobotModel()||trajectory.getWayPointCount()<2||
       trajectory.getWayPointCount()>120001) throw std::runtime_error("TRAJECTORY_MODEL_OR_COUNT_INVALID");
    double total=0.;
    for(std::size_t i=0;i<trajectory.getWayPointCount();++i) {
      checkpoint();
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
      checkpoint();
      const auto& point=msg.points[i];
      for(const auto* values:{&point.positions,&point.velocities,&point.accelerations})
        if((values==&point.positions||!values->empty())&&(values->size()!=msg.joint_names.size()||
          !std::all_of(values->begin(),values->end(),[](double v){return std::isfinite(v);})))
          throw std::runtime_error("TRAJECTORY_FIELDS_INVALID");
      if(point.time_from_start.nanosec>=1000000000U||(i?stamp(point)<=stamp(msg.points[i-1]):stamp(point)!=0))
        throw std::runtime_error("TRAJECTORY_TIMING_INVALID");
    }
    auto first=trajectory.getFirstWayPoint();first.update();
    ShapeValidationCache shape_cache;
    geometryBodies(scene,first,&shape_cache); // fail before FCL on malformed or unsupported geometry
    if(detached) {
      for(const auto& name:scene.getRobotModel()->getVariableNames())
        if(first.getVariablePosition(name)!=scene.getCurrentState().getVariablePosition(name))
          throw std::runtime_error("DETACHED_START_STATE_MISMATCH");
      detached->pairs=enumerateDetached(scene,first,detached->epoch,contact_cap);
      detached->enumeration_complete=true;
    }
    // Conditional permissions are identifiable policies, not arbitrary sampled
    // predicates. Keep exact pairs and never change the PlanningScene ACM.
    struct Conditional {
      std::string object,world;SupportContact support;bool floor=false;
      bool actual_expired=false, nominal_crossing=false, retired=false;
      // Floor pair whose attached body and support are single BOXes: if the
      // upward analytic proof fails, it uses the convex-BOX initial-contact
      // proof below, with SupportContact (not PileContact) as its predicate.
      bool convex=false;
      // Summed motion bound of the carried BOX over certified leaves while this
      // floor pair is still in contact; dragging is limited to the tolerance.
      double contact_travel=0.;
      // Full-shape SAT separation at t=0. While in contact the separation may
      // never fall below it (no press into the support), apart from the
      // numerical guard below.
      double initial_gap=-INFINITY;
      bool convexProof() const {return !floor||convex;}
      bool admits(const collision_detection::Contact& contact) const {
        return floor?support(contact):PileContact{object,world}(contact);
      }
    };
    std::vector<Conditional> initial;
    std::vector<const moveit::core::AttachedBody*> attached;first.getAttachedBodies(attached);
    for(const auto* body:attached) for(const auto& id:scene.getWorld()->getObjectIds()) {
      collision_detection::DecideContactFn fn;
      if(!scene.getAllowedCollisionMatrix().getAllowedCollision(body->getName(),id,fn)) continue;
      if(const auto* policy=fn.target<SupportContact>()) {
        initial.push_back({body->getName(),id,*policy,true});
        const auto owner=scene.getWorld()->getObject(id);
        initial.back().convex=body->getShapes().size()==1&&body->getShapes()[0]->type==shapes::BOX&&
          owner&&owner->shapes_.size()==1&&owner->shapes_[0]->type==shapes::BOX&&
          policy->object==body->getName()&&policy->support==id;
      }
      else if(fn.target<PileContact>()) initial.push_back({body->getName(),id,{},false});
    }
    auto contact_acm=scene.getAllowedCollisionMatrix();
    bool has_pile=false;
    for(const auto& pair:initial) if(pair.convexProof()) {
      contact_acm.setEntry(pair.object,pair.world,false);has_pile=true;
    }
    if(has_pile) {
      const auto contacts=completeContacts(scene,first,contact_cap,&contact_acm);
      for(auto& pair:initial) if(pair.convexProof()) {
        const auto found=contacts.contacts.find(pairKey(pair.object,pair.world));
        if(found==contacts.contacts.end()) {
          pair.actual_expired=pair.retired=true;
        } else for(const auto& contact:found->second)
          if(!pair.admits(contact))
            throw std::runtime_error(pair.floor?"INITIAL_SUPPORT_CONTACT_INVALID":"INITIAL_PILE_CONTACT_INVALID");
      }
      const auto start_geometry=geometryBodies(scene,first,&shape_cache);
      for(auto& pair:initial) if(pair.floor&&pair.convex&&!pair.retired) {
        pair.initial_gap=boxGap(start_geometry.at(pair.object),start_geometry.at(pair.world));
        if(!std::isfinite(pair.initial_gap)) throw std::runtime_error("INITIAL_SUPPORT_GEOMETRY_INVALID");
      }
    }
    joint_trajectory_controller::Trajectory jtc;
    for(std::size_t segment=1;segment<msg.points.size();++segment) {
      checkpoint();
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
        checkpoint();
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
      if(detached) {
        const auto controller_start=evaluate(begin);
        for(const auto& name:scene.getRobotModel()->getVariableNames())
          if(controller_start.getVariablePosition(name)!=trajectory.getWayPoint(segment-1).getVariablePosition(name))
            throw std::runtime_error("DETACHED_CONTROLLER_START_DISCONTINUITY");
      }
      const auto derivatives=(detached||has_pile)?derivativeControls(p,end-begin):Polynomials{};
      std::function<bool(int64_t,int64_t,const Polynomials&,const Polynomials&,unsigned)> visit;
      visit=[&](int64_t lo,int64_t hi,const Polynomials& controls,const Polynomials& speeds,unsigned depth) {
        checkpoint();
        report.failure_begin_ns=lo;report.failure_end_ns=hi;
        if(++report.inspected>options.max_intervals) {report.reason="INTERVAL_LIMIT";return false;}
        report.deepest=std::max(report.deepest,depth);
        const int64_t mid=lo+(hi-lo)/2;
        const auto anchor=evaluate(mid);
        // Midpoint is a collision witness, never a clearance certificate.
        for(int64_t t:{lo,mid,hi}) {
          checkpoint();
          auto state=t==mid?anchor:evaluate(t);
          if(!state.satisfiesBounds()) {report.reason="JOINT_LIMIT";return false;}
          if(detached) {
            const auto contacts=completeContacts(scene,state,contact_cap);
            for(const auto& contact:contacts.contacts) if(!activePair(detached->pairs,contact.first)) {
              report.result=ControllerCertificate::COLLISION;report.reason="DETACHED_NEW_OR_EXPIRED_CONTACT";return false;
            }
          } else if(scene.isStateColliding(state,"")) {report.result=ControllerCertificate::COLLISION;report.reason="FCL_COLLISION";return false;}
        }
        const auto movement=bodyDisplacements(scene,anchor,controls,errors,&shape_cache);
        auto acm=scene.getAllowedCollisionMatrix();
        const auto start=evaluate(lo);
        bool detached_proven=true;
        auto proposed=detached?detached->pairs:std::vector<DetachedPair>{};
        for(auto& pair:proposed) if(pair.state==DetachedPairState::ACTIVE_INITIAL_CONTACT) {
          const auto* link=scene.getRobotModel()->getLinkModel(pair.robot_link);
          const bool stationary=stationaryLink(link,controls);
          bool proven=separatingInterval(pair,scene,anchor,controls,speeds,errors);
          if(proven&&hi==end&&!stationary)
            proven=detachedBoundaryMonotone(pair,scene,evaluate(end-1),evaluate(end),errors);
          if(!proven) {detached_proven=false;continue;}
          acm.setEntry(pair.robot_link,pair.world_object,true); // interval proof ONLY, never scene policy
          const double gap=detachedGap(pair,scene,evaluate(hi),errors);
          if(gap>0.) {
            pair.state=DetachedPairState::SEPARATED;
            pair.transitions.push_back({pair.state,lo,hi,gap});
            pair.state=DetachedPairState::EXPIRED;
            pair.transitions.push_back({pair.state,lo,hi,gap});
          }
        }
        const auto geometry=geometryBodies(scene,anchor,&shape_cache);
        auto proposed_initial=initial;
        bool initial_proven=true, enumerate_crossing=false;
        for(auto& pair:proposed_initial) {
          if(pair.floor) {
            bool boundary_monotone=true;
            if(hi==end) {
              const auto prior=evaluate(end-1),terminal=evaluate(end);
              boundary_monotone=terminal.getAttachedBody(pair.object)->getGlobalPose().translation().z()>=
                                prior.getAttachedBody(pair.object)->getGlobalPose().translation().z();
            }
            if(upwardBoxContact(scene,start,controls,pair.object,pair.world,&pair.support,
                errors,hi-lo,boundary_monotone)) {
              acm.setEntry(pair.object,pair.world,true);continue; // distance query only; separate analytic proof above
            }
            // Unsupported floor geometry keeps its conditional entry and therefore
            // still requires the ordinary strict distance proof (fail closed).
            if(!pair.convex) continue;
          }
          acm.setEntry(pair.object,pair.world,false);
          const auto& moving=geometry.at(pair.object);const auto& fixed=geometry.at(pair.world);
          const double travel=movement.at(pair.object);
          const double gap=boxGap(moving,fixed);
          // A strict interval encloses both nominal and actual geometry. Once
          // retired, the ordinary collision certificate must prove every
          // subsequent interval, including both sides of stored waypoints.
          if(gap>(Interval(travel)+1e-12).upper()) {
            pair.retired=pair.actual_expired=true;
            acm.setEntry(pair.object,pair.world,true);continue;
          }
          if(pair.retired) continue;
          if(pair.floor) {
            // Still in (or possibly in) support contact: no approach along the
            // support normal and no more than the contact tolerance of total
            // carried motion before expiry. Otherwise refine or fail closed.
            constexpr double press_guard_m=1e-9;  // numerical only, far below the 1e-4 m tolerance
            if(pair.contact_travel+travel>support_contact_tolerance_m||
               (Interval(gap)-travel).lower()<pair.initial_gap-press_guard_m) {
              initial_proven=false;continue;
            }
            pair.contact_travel+=travel;
          }
          const bool depth_bounded=(Interval(gap)-travel).lower()>=-support_contact_tolerance_m;
          const auto nominal_clear=[&]() {return nominalBoxNoReentry(scene,
            *anchor.getAttachedBody(pair.object),fixed,controls,speeds);};
          if(depth_bounded&&!pair.actual_expired&&boxContactInterior(moving,fixed,travel)&&
              (!pair.nominal_crossing||(hi!=end&&nominal_clear()))) {
            acm.setEntry(pair.object,pair.world,true);continue;
          }
          // Prefer subdivision while an interior witness survives the numeric
          // error floor. Exhaustive work is reserved for the crossing region,
          // rather than consuming its bounded budget on ordinary contact.
          Polynomials constant;
          for(const auto& item:controls) constant[item.first]={Interval(anchor.getVariablePosition(item.first))};
          const double error_floor=bodyDisplacements(scene,anchor,constant,errors,&shape_cache).at(pair.object);
          const bool can_refine=hi-lo>options.min_interval_ns&&
            ((!pair.actual_expired&&boxContactInterior(moving,fixed,error_floor))||
             gap>(Interval(error_floor)+1e-12).upper());
          // No crossing-mode handoff at a stored waypoint: the final leaf
          // must instead have strict clearance covering its nominal limit,
          // stored endpoint and FK/evaluation bounds. The next segment is
          // checked with its own bounds by the retired-pair ordinary proof.
          if(depth_bounded&&!can_refine&&hi!=end&&hi-lo<=4096&&nominal_clear()) {
            pair.nominal_crossing=true;enumerate_crossing=true;
            acm.setEntry(pair.object,pair.world,true);continue;
          }
          initial_proven=false;continue;
        }
        if(initial_proven&&enumerate_crossing) {
          // Every integer timestamp is checked, without a stride. The budget
          // counts attempted work even if another pair later rejects this leaf.
          // Expiry flags themselves commit only with the complete interval.
          for(int64_t t=lo;t<=hi;++t) {
            checkpoint();
            if(++report.contact_timestamps>options.max_contact_timestamps) {
              report.reason="CONTACT_TIMESTAMP_LIMIT";return false;
            }
            const auto state=evaluate(t);
            if(!state.satisfiesBounds()) {report.reason="JOINT_LIMIT";return false;}
            const auto contacts=completeContacts(scene,state,contact_cap,&contact_acm);
            for(const auto& item:contacts.contacts) {
              auto pair=std::find_if(proposed_initial.begin(),proposed_initial.end(),[&](const auto& candidate) {
                return candidate.convexProof()&&pairKey(candidate.object,candidate.world)==item.first;
              });
              if(pair==proposed_initial.end()||pair->actual_expired) {
                report.result=ControllerCertificate::COLLISION;report.reason="INITIAL_CONTACT_NEW_OR_EXPIRED";return false;
              }
              for(const auto& contact:item.second) if(!pair->admits(contact)) {
                report.result=ControllerCertificate::COLLISION;report.reason="INITIAL_CONTACT_DEPTH_OR_IDENTITY";return false;
              }
            }
            const auto bodies=geometryBodies(scene,state,&shape_cache);
            for(auto& pair:proposed_initial) if(pair.convexProof()&&!pair.retired&&pair.nominal_crossing) {
              if(!contacts.contacts.count(pairKey(pair.object,pair.world))) {
                // FCL absence alone cannot establish separation at its numeric
                // termination boundary. Require full-shape positive clearance.
                if(boxGap(bodies.at(pair.object),bodies.at(pair.world))<=0.) {initial_proven=false;break;}
                pair.actual_expired=true;
              }
            }
            if(!initial_proven) break;
          }
        }
        std::map<std::string,std::array<Interval,3>> coordinate_projections;
        for(const auto& body:geometry)
          coordinate_projections[body.first]={projection(body.second,Eigen::Vector3d::UnitX()),
            projection(body.second,Eigen::Vector3d::UnitY()),projection(body.second,Eigen::Vector3d::UnitZ())};
        // Cheap support-plane certificates first. Excluding an already-proven
        // pair from the private distance query avoids expensive far-mesh GJK;
        // it does not alter scene policy or omit an unresolved pair.
        collision_detection::DistanceResultsData no_axis;
        for(auto a=geometry.begin();a!=geometry.end();++a) for(auto b=std::next(a);b!=geometry.end();++b) {
          checkpoint();
          if(!requiredPair(a->second,b->second,acm)) continue;
          Interval relative(1e-12);
          for(const auto* body:{&a->second,&b->second})
            if(body->type!=collision_detection::BodyTypes::WORLD_OBJECT) relative+=Interval(movement.at(body->name));
          if(clearanceLowerBound(a->second,b->second,no_axis,relative.upper(),&coordinate_projections)>relative.upper())
            acm.setEntry(a->first,b->first,true);
        }
        collision_detection::DistanceRequest req;req.type=collision_detection::DistanceRequestType::SINGLE;req.acm=&acm;req.enable_nearest_points=true;
        collision_detection::DistanceResult world,self;
        checkpoint();
        scene.getCollisionEnv()->distanceRobot(req,world,anchor);
        checkpoint();
        scene.getCollisionEnvUnpadded()->distanceSelf(req,self,anchor);
        checkpoint();
        bool clear=detached_proven&&initial_proven&&!world.collision&&!self.collision;
        for(auto a=geometry.begin();a!=geometry.end();++a) for(auto b=std::next(a);b!=geometry.end();++b) {
          checkpoint();
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
        if(clear) {
          if(detached) detached->pairs=std::move(proposed);
          for(std::size_t i=0;i<initial.size();++i)
            if(!initial[i].actual_expired&&proposed_initial[i].actual_expired) ++report.initial_contacts_expired;
          initial=std::move(proposed_initial);++report.certified;return true;
        }
        if(depth>=options.max_depth||hi-lo<=options.min_interval_ns||mid==lo||mid==hi) {
          report.reason=detached_proven?"PRECISION_OR_DEPTH_LIMIT":"DETACHED_MONOTONICITY_UNCERTIFIED";return false;
        }
        ++report.subdivided;Polynomials left,right;
        for(const auto& item:controls) {
          auto halves=split(item.second,Interval(double(mid-lo))/Interval(double(hi-lo)));
          left[item.first]=std::move(halves.first);right[item.first]=std::move(halves.second);
        }
        Polynomials left_speeds,right_speeds;
        for(const auto& item:speeds) {
          auto halves=split(item.second,Interval(double(mid-lo))/Interval(double(hi-lo)));
          left_speeds[item.first]=std::move(halves.first);right_speeds[item.first]=std::move(halves.second);
        }
        return visit(lo,mid,left,left_speeds,depth+1)&&visit(mid,hi,right,right_speeds,depth+1);
      };
      if(!visit(begin,end,p,derivatives,0)) return finish();
    }
    if(detached) {
      for(const auto& pair:detached->pairs) if(pair.state!=DetachedPairState::EXPIRED)
        throw std::runtime_error("DETACHED_CONTACTS_UNEXPIRED");
      if(scene.isStateColliding(trajectory.getLastWayPoint(),"")) {
        report.result=ControllerCertificate::COLLISION;report.reason="DETACHED_FINAL_STATE_COLLISION";return finish();
      }
    }
    checkpoint();
    report.result=ControllerCertificate::CERTIFIED_CLEAR;report.reason="ALL_INTERVALS_CERTIFIED";
    report.failure_begin_ns=report.failure_end_ns=-1;
  } catch(const std::exception& e) {report.result=ControllerCertificate::UNCERTIFIED;report.reason=e.what();}
  catch(...) {report.result=ControllerCertificate::UNCERTIFIED;report.reason="UNKNOWN_CERTIFICATION_FAILURE";}
  return finish();
}
inline DetachedAudit certifyDetached(const robot_trajectory::RobotTrajectory& trajectory,
    const planning_scene::PlanningScene& scene,const std::string& epoch,
    const ControllerAuditOptions& options={},std::size_t contact_cap=1000000) {
  DetachedAudit result;result.epoch=epoch;
  result.audit=certify(trajectory,scene,options,&result,contact_cap);return result;
}
} // namespace controller_certificate
} // namespace workcell
