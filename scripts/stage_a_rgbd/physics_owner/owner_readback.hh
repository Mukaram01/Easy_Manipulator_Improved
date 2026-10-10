// Only public DART/ign-physics APIs. Raw state never enters EPD geometry.
#pragma once
#include "bindings.hh"
#include <gz/physics/dartsim/World.hh>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/collision/CollisionResult.hpp>
#include <dart/collision/CollisionObject.hpp>
#include <jsoncpp/json/json.h>
#include <fstream>
#include <sstream>
#include <iomanip>
namespace workcell {
using Node=dart::dynamics::ShapeNode;
inline Json::Value Matrix(const Eigen::Isometry3d &t) {
  Json::Value rows(Json::arrayValue);
  for(int i=0;i<4;++i){Json::Value row(Json::arrayValue);for(int j=0;j<4;++j)row.append(t.matrix()(i,j));rows.append(row);}return rows;
}
inline std::string Hex(double value) {std::ostringstream s;s<<std::hexfloat<<value;return s.str();}
inline Json::Value MatrixHex(const Eigen::Isometry3d &t) {
  Json::Value rows(Json::arrayValue);
  for(int i=0;i<4;++i){Json::Value row(Json::arrayValue);for(int j=0;j<4;++j)row.append(Hex(t.matrix()(i,j)));rows.append(row);}return rows;
}
inline std::string Pointer(const void *p){std::ostringstream s;s<<p;return s.str();}
struct OwnerReadback {
  Bindings<Node> bindings;
  std::string backendPath,backendClass,output,session;
  std::uint64_t recordStep=0;
  std::set<std::uint64_t> recordSteps;
  std::map<std::uint64_t,Eigen::Isometry3d> preStepTransforms;
  int preStepFrames=-1;
  void Configure(const std::shared_ptr<const sdf::Element> &sdf) {
    output=sdf->Get<std::string>("owner_output", "").first;
    session=sdf->Get<std::string>("owner_session", "").first;
    recordStep=sdf->Get<std::uint64_t>("owner_record_step", 0).first;
    std::istringstream schedule(sdf->Get<std::string>("owner_record_steps", "").first);
    std::uint64_t step;while(schedule>>step)recordSteps.insert(step);
    if(recordSteps.empty())recordSteps.insert(recordStep);
  }
};
}
