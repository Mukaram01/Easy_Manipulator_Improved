#pragma once
// Included after the shared interval/geometry helpers in controller_interval_certificate.hpp.
// This extension is opt-in and never installs a PlanningScene contact permission.
#include <moveit/robot_model/revolute_joint_model.h>
#include <iomanip>
#include <sstream>

namespace workcell { namespace controller_certificate {
enum class DetachedPairState { ACTIVE_INITIAL_CONTACT, SEPARATED, EXPIRED };
struct DetachedTransition {
  DetachedPairState state;
  int64_t begin_ns, end_ns;
  double clearance_lower;
};
struct DetachedPair {
  std::string robot_link, world_object, epoch, geometry_identity;
  std::string certificate_type="FIXED_BOX_FACE_FULL_MESH_SUPPORT";
  EigenSTL::vector_Isometry3d robot_poses, world_poses;
  std::vector<collision_detection::Contact> initial_evidence;
  Eigen::Vector3d axis=Eigen::Vector3d::Zero();
  double world_support=0., initial_gap=0.;
  DetachedPairState state=DetachedPairState::ACTIVE_INITIAL_CONTACT;
  std::vector<DetachedTransition> transitions;
};
struct DetachedAudit {
  ControllerAuditReport audit;
  std::string epoch;
  bool enumeration_complete=false;
  std::vector<DetachedPair> pairs;
};
using PairKey=std::pair<std::string,std::string>;
inline PairKey pairKey(std::string a,std::string b) {return a<b?PairKey{a,b}:PairKey{b,a};}
inline const DetachedPair* activePair(const std::vector<DetachedPair>& pairs,const PairKey& key) {
  for(const auto& p:pairs)
    if(pairKey(p.robot_link,p.world_object)==key&&p.state==DetachedPairState::ACTIVE_INITIAL_CONTACT) return &p;
  return nullptr;
}
// Both caps are checked. Reaching a cap is ambiguous, hence never complete.
inline collision_detection::CollisionResult completeContacts(const planning_scene::PlanningScene& scene,
    const moveit::core::RobotState& state,std::size_t cap=1000000) {
  if(!cap) throw std::runtime_error("CONTACT_ENUMERATION_INCOMPLETE");
  collision_detection::CollisionRequest req;req.contacts=true;req.max_contacts=cap;req.max_contacts_per_pair=cap;
  collision_detection::CollisionResult result;scene.checkCollision(req,result,state);
  std::size_t count=0;
  for(const auto& item:result.contacts) {
    if(item.second.empty()||item.second.size()>=cap) throw std::runtime_error("CONTACT_ENUMERATION_INCOMPLETE");
    count+=item.second.size();
  }
  if(result.contact_count>=cap||count!=result.contact_count||result.collision!=(count!=0))
    throw std::runtime_error("CONTACT_ENUMERATION_INCOMPLETE");
  return result;
}
inline std::string geometryIdentity(const GeometryBody& body) {
  // Exact, inspectable value identity rather than pointer identity or std::hash.
  std::ostringstream out;out<<std::hexfloat<<body.name.size()<<':'<<body.name<<':'<<int(body.type)<<':'<<body.shapes.size();
  for(std::size_t i=0;i<body.shapes.size();++i) {
    const auto& shape=*body.shapes[i];out<<':'<<int(shape.type);
    if(shape.type==shapes::MESH) {
      const auto& m=static_cast<const shapes::Mesh&>(shape);out<<':'<<m.vertex_count<<':'<<m.triangle_count;
      for(unsigned j=0;j<3*m.vertex_count;++j) out<<':'<<m.vertices[j];
      for(unsigned j=0;j<3*m.triangle_count;++j) out<<':'<<m.triangles[j];
    } else if(shape.type==shapes::BOX) for(double x:static_cast<const shapes::Box&>(shape).size) out<<':'<<x;
    else throw std::runtime_error("DETACHED_GEOMETRY_UNSUPPORTED");
    for(unsigned j=0;j<16;++j) out<<':'<<body.poses[i].matrix().data()[j];
  }
  return out.str();
}
inline Interval axisMagnitude(const Eigen::Vector3d& axis) {
  Interval squared(0.);for(unsigned i=0;i<3;++i) squared+=boost::numeric::square(Interval(axis[i]));
  const auto norm=boost::numeric::sqrt(squared);finite(norm);
  if(norm.lower()<=0.) throw std::runtime_error("DETACHED_PLANE_AXIS_INVALID");
  return norm;
}
inline double detachedError(const DetachedPair&,const planning_scene::PlanningScene&,
    const moveit::core::RobotState&,const std::map<std::string,double>&,const Polynomials* full_controls=nullptr);
inline std::vector<DetachedPair> enumerateDetached(const planning_scene::PlanningScene& scene,
    const moveit::core::RobotState& state,const std::string& epoch,std::size_t cap=1000000) {
  if(epoch.empty()) throw std::runtime_error("RECOVERY_EPOCH_REQUIRED");
  std::vector<const moveit::core::AttachedBody*> attached;state.getAttachedBodies(attached);
  if(!attached.empty()) throw std::runtime_error("DETACHED_RECOVERY_HAS_ATTACHMENT");
  const auto bodies=geometryBodies(scene,state);
  for(auto a=bodies.begin();a!=bodies.end();++a) for(auto b=std::next(a);b!=bodies.end();++b) {
    collision_detection::AllowedCollision::Type type;
    if(scene.getAllowedCollisionMatrix().getAllowedCollision(a->first,b->first,type)&&
       type==collision_detection::AllowedCollision::CONDITIONAL)
      throw std::runtime_error("DETACHED_CONDITIONAL_ACM_UNSUPPORTED");
  }
  const auto contacts=completeContacts(scene,state,cap);
  std::vector<DetachedPair> result;
  for(const auto& item:contacts.contacts) {
    const auto& a=bodies.at(item.first.first);const auto& b=bodies.at(item.first.second);
    const auto* robot=a.type==collision_detection::BodyTypes::ROBOT_LINK?&a:&b;
    const auto* world=robot==&a?&b:&a;
    if(robot->type!=collision_detection::BodyTypes::ROBOT_LINK||world->type!=collision_detection::BodyTypes::WORLD_OBJECT)
      throw std::runtime_error("DETACHED_INITIAL_SELF_OR_ATTACHED_CONTACT");
    if(world->shapes.size()!=1||world->shapes[0]->type!=shapes::BOX||
       !std::all_of(robot->shapes.begin(),robot->shapes.end(),[](const auto& s){return s->type==shapes::MESH;}))
      throw std::runtime_error("DETACHED_GEOMETRY_UNSUPPORTED");
    for(const auto& c:item.second) {
      if(pairKey(c.body_name_1,c.body_name_2)!=item.first||!std::isfinite(c.depth)||c.depth<0.||
         c.depth>support_contact_tolerance_m||!c.pos.allFinite()||!c.normal.allFinite())
        throw std::runtime_error("DETACHED_INITIAL_CONTACT_INVALID");
    }
    DetachedPair pair;pair.robot_link=robot->name;pair.world_object=world->name;pair.epoch=epoch;
    pair.geometry_identity=geometryIdentity(*robot)+"|"+geometryIdentity(*world);
    pair.robot_poses=robot->poses;pair.world_poses=world->poses;pair.initial_evidence=item.second;
    pair.initial_gap=-INFINITY;
    for(unsigned axis=0;axis<3;++axis) for(double sign:{-1.,1.}) {
      const Eigen::Vector3d n=sign*world->poses[0].linear().col(axis);
      const double support=projection(*world,n).upper();
      const double gap=(Interval(projection(*robot,n).lower())-support).lower();
      if(gap>pair.initial_gap) {pair.initial_gap=gap;pair.axis=n;pair.world_support=support;}
    }
    // Charge roundoff against the unchanged 100um bound, never enlarge it.
    if(!std::isfinite(pair.initial_gap)||
       ((Interval(pair.initial_gap)-detachedError(pair,scene,state,{}))/axisMagnitude(pair.axis)).lower() < -support_contact_tolerance_m)
      throw std::runtime_error("DETACHED_STABLE_PLANE_UNAVAILABLE");
    result.push_back(std::move(pair));
  }
  return result;
}

// Interval matrix arithmetic avoids Eigen's scalar-type assumptions. Taylor
// remainders enclose sin/cos without assuming libm respects directed rounding.
using IMatrix=std::array<std::array<Interval,4>,4>;
inline IMatrix zeroMatrix() {IMatrix r;for(auto& row:r) for(auto& x:row) x=Interval(0.);return r;}
inline IMatrix identityMatrix() {auto r=zeroMatrix();for(unsigned i=0;i<4;++i) r[i][i]=Interval(1.);return r;}
inline IMatrix intervalMatrix(const Eigen::Isometry3d& t) {
  auto r=zeroMatrix();for(unsigned i=0;i<4;++i) for(unsigned j=0;j<4;++j) r[i][j]=Interval(t.matrix()(i,j));return r;
}
inline IMatrix multiply(const IMatrix& a,const IMatrix& b) {
  auto r=zeroMatrix();
  for(unsigned i=0;i<4;++i) for(unsigned j=0;j<4;++j)
    for(unsigned k=0;k<4;++k) r[i][j]+=a[i][k]*b[k][j];
  return r;
}
inline IMatrix addMatrix(IMatrix a,const IMatrix& b) {
  for(unsigned i=0;i<4;++i) for(unsigned j=0;j<4;++j) a[i][j]+=b[i][j];
  return a;
}
inline std::pair<Interval,Interval> sincosBound(const Interval& q) {
  if(magnitude(q)>16.) throw std::runtime_error("DETACHED_ANGLE_BOUND_UNSUPPORTED");
  Interval s=q,c(1.),st=q,ct(1.);const auto square=boost::numeric::square(q);
  for(unsigned k=1;k<=48;++k) {
    st=-st*square/double((2*k)*(2*k+1));ct=-ct*square/double((2*k-1)*(2*k));s+=st;c+=ct;
  }
  const double se=magnitude(st*square/double(98*99)),ce=magnitude(ct*square/double(97*98));
  s+=Interval(-se,se);c+=Interval(-ce,ce);finite(s);finite(c);
  return {Interval(std::max(-1.,s.lower()),std::min(1.,s.upper())),
          Interval(std::max(-1.,c.lower()),std::min(1.,c.upper()))};
}
inline Interval mimicFactor(const moveit::core::JointModel* joint,const std::string& independent) {
  Interval factor(1.);
  while(joint->getMimic()) {factor*=joint->getMimicFactor();joint=joint->getMimic();}
  return joint->getVariableNames().at(0)==independent?factor:Interval(0.);
}
inline IMatrix linkDerivative(const moveit::core::LinkModel* link,const Polynomials& p,const std::string& variable) {
  std::vector<const moveit::core::LinkModel*> chain;
  for(auto* cursor=link;cursor;cursor=cursor->getParentLinkModel()) chain.push_back(cursor);
  auto t=identityMatrix(),d=zeroMatrix();
  for(auto it=chain.rbegin();it!=chain.rend();++it) {
    const auto* joint=(*it)->getParentJointModel();const auto origin=intervalMatrix((*it)->getJointOriginTransform());
    t=multiply(t,origin);d=multiply(d,origin);
    if(!joint->getVariableCount()) continue;
    if(joint->getVariableCount()!=1) throw std::runtime_error("DETACHED_JOINT_UNSUPPORTED");
    const auto q=hull(p.at(joint->getVariableNames()[0]));const auto factor=mimicFactor(joint,variable);
    auto motion=identityMatrix(),derivative=zeroMatrix();
    if(joint->getType()==moveit::core::JointModel::PRISMATIC) {
      const auto axis=static_cast<const moveit::core::PrismaticJointModel*>(joint)->getAxis();
      for(unsigned i=0;i<3;++i) {motion[i][3]=q*axis[i];derivative[i][3]=factor*axis[i];}
    } else if(joint->getType()==moveit::core::JointModel::REVOLUTE) {
      const auto axis=static_cast<const moveit::core::RevoluteJointModel*>(joint)->getAxis();
      const auto trig=sincosBound(q);const auto s=trig.first,c=trig.second;
      const double skew[3][3]={{0.,-axis.z(),axis.y()},{axis.z(),0.,-axis.x()},{-axis.y(),axis.x(),0.}};
      for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) {
        const Interval product=Interval(axis[i])*axis[j];const double identity=i==j?1.:0.;
        motion[i][j]=c*identity+(Interval(1.)-c)*product+s*skew[i][j];
        derivative[i][j]=factor*(-s*identity+s*product+c*skew[i][j]);
      }
    } else throw std::runtime_error("DETACHED_JOINT_UNSUPPORTED");
    d=addMatrix(multiply(d,motion),multiply(t,derivative));t=multiply(t,motion);
  }
  return d;
}
inline Polynomials derivativeControls(const Polynomials& p,int64_t duration_ns) {
  Polynomials result;
  for(const auto& item:p) {
    Controls values;const auto& c=item.second;
    for(std::size_t i=1;i<c.size();++i) values.push_back((c[i]-c[i-1])*double(c.size()-1)/double(duration_ns));
    if(values.empty()) values={Interval(0.)};
    result[item.first]=std::move(values);
  }
  return result;
}
inline double detachedError(const DetachedPair& pair,const planning_scene::PlanningScene& scene,
    const moveit::core::RobotState& anchor,const std::map<std::string,double>& errors,
    const Polynomials* full_controls) {
  const auto* link=scene.getRobotModel()->getLinkModel(pair.robot_link);
  double shape_radius=0.;
  for(std::size_t i=0;i<link->getShapes().size();++i)
    shape_radius=std::max(shape_radius,(Interval(normUpper(link->getCollisionOriginTransforms()[i].translation()))+
      radius(*link->getShapes()[i])).upper());
  Interval reach(shape_radius),travel(0.);unsigned chain_length=0;
  for(auto* cursor=link;cursor;cursor=cursor->getParentLinkModel()) {
    ++chain_length;const auto* joint=cursor->getParentJointModel();
    if(joint->getVariableCount()) {
      if(joint->getVariableCount()!=1) throw std::runtime_error("DETACHED_JOINT_UNSUPPORTED");
      const auto& name=joint->getVariableNames()[0];
      const auto q=full_controls?hull(full_controls->at(name)):Interval(anchor.getVariablePosition(name));
      const Interval error=Interval(256.*std::numeric_limits<double>::epsilon())*(Interval(1.)+magnitude(q))+
        (errors.count(name)?errors.at(name):0.);
      if(joint->getType()==moveit::core::JointModel::PRISMATIC) {
        travel+=error;
        reach+=Interval(std::max(magnitude(q+Interval(-error.upper(),error.upper())),std::abs(anchor.getVariablePosition(name))));
      } else if(joint->getType()==moveit::core::JointModel::REVOLUTE) travel+=error*reach;
      else throw std::runtime_error("DETACHED_JOINT_UNSUPPORTED");
    }
    reach+=Interval(normUpper(cursor->getJointOriginTransform().translation()));
  }
  // Error only, not nominal travel. Both lever arms and coordinate scales cover
  // the entire interval, including prismatic extrema away from the midpoint.
  const double neps=128.*double(chain_length+4)*std::numeric_limits<double>::epsilon();
  if(neps>=.5) throw std::runtime_error("DETACHED_ERROR_UNBOUNDED");
  travel+=reach*(Interval(neps)/(Interval(1.)-neps));
  const auto projected=(travel+1e-12)*axisMagnitude(pair.axis);finite(projected);return projected.upper();
}
inline bool stationaryLink(const moveit::core::LinkModel* link,const Polynomials& p) {
  for(auto* cursor=link;cursor;cursor=cursor->getParentLinkModel()) {
    const auto* joint=cursor->getParentJointModel();if(!joint->getVariableCount()) continue;
    const auto q=hull(p.at(joint->getVariableNames()[0]));if(q.lower()!=q.upper()) return false;
  }
  return true;
}
inline bool separatingInterval(const DetachedPair& pair,const planning_scene::PlanningScene& scene,
    const moveit::core::RobotState& anchor,const Polynomials& p,const Polynomials& derivatives,
    const std::map<std::string,double>& errors) {
  const auto* link=scene.getRobotModel()->getLinkModel(pair.robot_link);
  if(stationaryLink(link,p)) return true; // exact constant polynomial and identical deterministic FK
  std::vector<std::pair<IMatrix,Interval>> jacobians;
  for(const auto* joint:scene.getRobotModel()->getJointModels()) {
    if(!joint->getVariableCount()||joint->getMimic()) continue;
    const auto& name=joint->getVariableNames().at(0);const auto velocity=hull(derivatives.at(name));
    if(velocity.lower()==0.&&velocity.upper()==0.) continue;
    jacobians.emplace_back(linkDerivative(link,p,name),velocity);
  }
  const double guard=detachedError(pair,scene,anchor,errors,&p);
  for(std::size_t shape=0;shape<link->getShapes().size();++shape) {
    const auto& mesh=static_cast<const shapes::Mesh&>(*link->getShapes()[shape]);
    const auto origin=intervalMatrix(link->getCollisionOriginTransforms()[shape]);
    for(unsigned v=0;v<mesh.vertex_count;++v) {
      Interval speed(0.);
      for(const auto& jacobian:jacobians) {
        const auto d=multiply(jacobian.first,origin);Interval projected(0.);
        for(unsigned i=0;i<3;++i) {
          Interval value=d[i][3];for(unsigned j=0;j<3;++j) value+=d[i][j]*mesh.vertices[3*v+j];
          projected+=Interval(pair.axis[i])*value;
        }
        speed+=projected*jacobian.second;
      }
      finite(speed);
      // Proves the nominal continuous geometry and every evaluated JTC ns
      // increment, including FK error. A uniform tube alone is not monotonic.
      if(speed.lower()<=2.*guard) return false;
    }
  }
  return true;
}
inline double detachedGap(const DetachedPair& pair,const planning_scene::PlanningScene& scene,
    const moveit::core::RobotState& state,const std::map<std::string,double>& errors) {
  const auto bodies=geometryBodies(scene,state);
  return ((Interval(projection(bodies.at(pair.robot_link),pair.axis).lower())-
    pair.world_support-detachedError(pair,scene,state,errors))/axisMagnitude(pair.axis)).lower();
}
inline bool detachedBoundaryMonotone(const DetachedPair& pair,const planning_scene::PlanningScene& scene,
    const moveit::core::RobotState& prior,const moveit::core::RobotState& terminal,
    const std::map<std::string,double>& errors) {
  const auto a=geometryBodies(scene,prior).at(pair.robot_link),b=geometryBodies(scene,terminal).at(pair.robot_link);
  const double guard=detachedError(pair,scene,prior,errors)+detachedError(pair,scene,terminal,errors);
  for(std::size_t k=0;k<a.shapes.size();++k) {
    const auto& mesh=static_cast<const shapes::Mesh&>(*a.shapes[k]);
    for(unsigned v=0;v<mesh.vertex_count;++v) {
      Interval advance(0.);
      for(unsigned i=0;i<3;++i) {
        Interval x=Interval(b.poses[k].translation()[i])-a.poses[k].translation()[i];
        for(unsigned j=0;j<3;++j) x+=(Interval(b.poses[k].linear()(i,j))-a.poses[k].linear()(i,j))*mesh.vertices[3*v+j];
        advance+=Interval(pair.axis[i])*x;
      }
      if(advance.lower()<=guard) return false;
    }
  }
  return true;
}
} } // namespace workcell::controller_certificate
