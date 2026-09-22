#include "support_contact_policy.hpp"
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <geometric_shapes/shapes.h>
#include <pluginlib/class_list_macros.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <rclcpp/serialization.hpp>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>

namespace workcell {
// Opt-in evidence at the innermost adapter boundary; never changes a request.
class PlanningEvidence : public planning_request_adapter::PlanningRequestAdapter {
public:
  void initialize(const rclcpp::Node::SharedPtr&, const std::string&) override {}
  std::string getDescription() const override { return "Workcell effective planning evidence"; }
  template<class Message> static void save(const std::string& path, const Message& msg) {
    rclcpp::SerializedMessage bytes;
    rclcpp::Serialization<Message>().serialize_message(&msg, &bytes);
    std::ofstream out(path, std::ios::binary);
    out.exceptions(std::ios::badbit | std::ios::failbit);
    const auto& buffer=bytes.get_rcl_serialized_message();
    out.write(reinterpret_cast<const char*>(buffer.buffer), buffer.buffer_length);
  }
  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& scene,
      const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
      std::vector<std::size_t>&) const override {
    const char* directory=std::getenv("WORKCELL_PLANNING_TRACE_DIR");
    if (!directory || !*directory) return planner(scene,req,res);
    static std::atomic<unsigned long> sequence{0};
    std::filesystem::create_directories(directory);
    const auto stamp=std::chrono::system_clock::now().time_since_epoch();
    const auto prefix=std::string(directory)+"/"+std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(stamp).count())+"_"+std::to_string(sequence++);
    save(prefix+".request.cdr",req);
    moveit_msgs::msg::PlanningScene effective;
    scene->getPlanningSceneMsg(effective);
    save(prefix+".scene.cdr",effective);
    std::ofstream acm(prefix+".acm.txt");
    scene->getAllowedCollisionMatrix().print(acm);
    const auto begin=std::chrono::steady_clock::now();
    const bool ok=planner(scene,req,res);
    std::ofstream result(prefix+".result.txt");
    result << "success " << ok << "\nerror " << res.error_code_.val
           << "\nplanning_time " << res.planning_time_ << "\nwall_time "
           << std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count() << '\n';
    return ok;
  }
};

class StraightCartesianPath : public planning_request_adapter::PlanningRequestAdapter {
public:
  void initialize(const rclcpp::Node::SharedPtr&, const std::string&) override {}
  std::string getDescription() const override {
    return "Workcell straight Cartesian interpolation in the effective private planning scene";
  }

  static Eigen::Isometry3d parsePose(const YAML::Node& node) {
    if (!node || !node.IsSequence() || node.size()!=7)
      throw std::runtime_error("CARTESIAN_PATH_POSE_INVALID");
    for (std::size_t i=0;i<7;++i)
      if (!std::isfinite(node[i].as<double>()))
        throw std::runtime_error("CARTESIAN_PATH_POSE_NONFINITE");
    Eigen::Quaterniond q(node[6].as<double>(),node[3].as<double>(),
                         node[4].as<double>(),node[5].as<double>());
    if (q.norm()<1e-12) throw std::runtime_error("CARTESIAN_PATH_QUATERNION_INVALID");
    q.normalize();
    Eigen::Isometry3d result=Eigen::Isometry3d::Identity();
    result.linear()=q.toRotationMatrix();
    result.translation()=Eigen::Vector3d(
      node[0].as<double>(),node[1].as<double>(),node[2].as<double>());
    return result;
  }

  bool adaptAndPlan(const PlannerFn& planner,
      const planning_scene::PlanningSceneConstPtr& scene,
      const planning_interface::MotionPlanRequest& req,
      planning_interface::MotionPlanResponse& res,
      std::vector<std::size_t>&) const override {
    const std::string prefix="workcell_cartesian_path:";
    bool found_metadata=false;
    std::string metadata_text;
    for (const auto& constraint:req.trajectory_constraints.constraints) {
      if (constraint.name.compare(0,prefix.size(),prefix)==0) {
        if (found_metadata) {
          res.error_code_.val=moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
          res.trajectory_.reset();
          return false;
        }
        metadata_text=constraint.name.substr(prefix.size());
        found_metadata=true;
      }
    }
    if (!found_metadata) return planner(scene,req,res);

    const auto begin=std::chrono::steady_clock::now();
    auto fail=[&](const char* why) {
      RCLCPP_WARN(rclcpp::get_logger("workcell.cartesian_path"),"%s",why);
      res.error_code_.val=moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
      res.trajectory_.reset();
      res.planning_time_=std::chrono::duration<double>(
        std::chrono::steady_clock::now()-begin).count();
      return false;
    };

    try {
      const auto data=YAML::Load(metadata_text);
      if (!data["schema"] || data["schema"].as<std::string>()!="workcell_cartesian_path/v1")
        return fail("CARTESIAN_PATH_SCHEMA_INVALID");
      const auto link_name=data["tool_link"].as<std::string>();
      const double step=data["max_step_m"].as<double>();
      if (!std::isfinite(step) || step<=0. || step>.01)
        return fail("CARTESIAN_PATH_STEP_INVALID");

      moveit::core::RobotState start(scene->getRobotModel());
      start=scene->getCurrentState();
      moveit::core::robotStateMsgToRobotState(req.start_state,start);
      start.update();

      const auto* jmg=start.getJointModelGroup(req.group_name);
      const auto* link=start.getLinkModel(link_name);
      if (!jmg || !link) return fail("CARTESIAN_PATH_BINDING_INVALID");

      const Eigen::Isometry3d expected_start=parsePose(data["start_pose"]);
      const Eigen::Isometry3d goal=parsePose(data["goal_pose"]);
      const Eigen::Isometry3d actual_start=start.getGlobalLinkTransform(link);
      const double start_translation_error=
        (actual_start.translation()-expected_start.translation()).norm();
      const double start_angle_error=Eigen::AngleAxisd(
        actual_start.linear()*expected_start.linear().transpose()).angle();
      if (start_translation_error>1e-5 || start_angle_error>1e-5)
        return fail("CARTESIAN_PATH_START_CHANGED");

      auto kset=std::make_shared<kinematic_constraints::KinematicConstraintSet>(
        scene->getRobotModel());
      kset->add(req.path_constraints,scene->getTransforms());

      std::size_t collision_rejections=0;
      std::size_t constraint_rejections=0;
      moveit::core::GroupStateValidityCallbackFn valid=[
        scene,kset,&collision_rejections,&constraint_rejections](
          moveit::core::RobotState* state,
          const moveit::core::JointModelGroup* group,
          const double* solution) {
        state->setJointGroupPositions(group,solution);
        state->update();
        if (scene->isStateColliding(*state,group->getName())) {
          ++collision_rejections;
          return false;
        }
        if (!kset->empty() && !kset->decide(*state).satisfied) {
          ++constraint_rejections;
          return false;
        }
        return true;
      };

      const moveit::core::RobotState original_start(start);
      EigenSTL::vector_Isometry3d waypoints{goal};
      std::vector<moveit::core::RobotStatePtr> states;
      const double fraction=moveit::core::CartesianInterpolator::computeCartesianPath(
        &start,jmg,states,link,waypoints,true,
        moveit::core::MaxEEFStep(step),
        moveit::core::JumpThreshold(0.0),valid);
      if (!std::isfinite(fraction) || fraction<1.0-1e-9) {
        RCLCPP_WARN(
          rclcpp::get_logger("workcell.cartesian_path"),
          "CARTESIAN_PATH_INCOMPLETE fraction=%.9f collision_rejections=%zu constraint_rejections=%zu",
          fraction,collision_rejections,constraint_rejections);
        return fail("CARTESIAN_PATH_INCOMPLETE");
      }

      auto trajectory=std::make_shared<robot_trajectory::RobotTrajectory>(
        scene->getRobotModel(),req.group_name);
      trajectory->addSuffixWayPoint(original_start,0.0);
      for (const auto& state:states) {
        if (!state) return fail("CARTESIAN_PATH_STATE_MISSING");
        if (trajectory->getLastWayPoint().distance(*state)>1e-12)
          trajectory->addSuffixWayPoint(*state,0.0);
      }
      if (trajectory->getWayPointCount()<2)
        return fail("CARTESIAN_PATH_EMPTY");

      trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
      const double velocity=(req.max_velocity_scaling_factor>0. &&
                             req.max_velocity_scaling_factor<=1.)
                              ? req.max_velocity_scaling_factor : 0.2;
      const double acceleration=(req.max_acceleration_scaling_factor>0. &&
                                 req.max_acceleration_scaling_factor<=1.)
                                  ? req.max_acceleration_scaling_factor : 0.2;
      if (!time_parameterization.computeTimeStamps(
            *trajectory,velocity,acceleration))
        return fail("CARTESIAN_PATH_TIMING_FAILED");

      // Final fail-closed validation over every emitted state in the exact
      // effective private scene. The Python runtime independently performs a
      // denser FK corridor audit after this adapter returns.
      for (std::size_t i=0;i<trajectory->getWayPointCount();++i) {
        const auto& state=trajectory->getWayPoint(i);
        if (scene->isStateColliding(state,req.group_name))
          return fail("CARTESIAN_PATH_EMITTED_COLLISION");
        if (!kset->empty() && !kset->decide(state).satisfied)
          return fail("CARTESIAN_PATH_EMITTED_CONSTRAINT_FAILURE");
      }

      res.trajectory_=trajectory;
      res.error_code_.val=moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      res.planning_time_=std::chrono::duration<double>(
        std::chrono::steady_clock::now()-begin).count();
      RCLCPP_INFO(
        rclcpp::get_logger("workcell.cartesian_path"),
        "CARTESIAN_PATH_PASS points=%zu step=%.4f collision_rejections=%zu constraint_rejections=%zu",
        trajectory->getWayPointCount(),step,collision_rejections,constraint_rejections);
      return true;
    } catch (const std::exception& e) {
      RCLCPP_WARN(
        rclcpp::get_logger("workcell.cartesian_path"),
        "CARTESIAN_PATH_REJECTED: %s",e.what());
      return fail("CARTESIAN_PATH_REJECTED");
    } catch (...) {
      return fail("CARTESIAN_PATH_REJECTED_UNKNOWN_EXCEPTION");
    }
  }
};

class InitialSupportContact : public planning_request_adapter::PlanningRequestAdapter {
public:
  void initialize(const rclcpp::Node::SharedPtr&, const std::string&) override {}
  std::string getDescription() const override { return "Initial object/support numerical contact (<=0.1 mm)"; }
  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& scene,
      const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
      std::vector<std::size_t>&) const override {
    const std::string prefix="workcell_initial_support_contact:";
    if (req.path_constraints.name.compare(0,prefix.size(),prefix)!=0) return planner(scene,req,res);
    // MoveIt's adapter wrapper SKIPS throwing adapters. Activated policy must
    // return failure instead, never throw and silently fall back to raw planning.
    try {
      auto data=YAML::Load(req.path_constraints.name.substr(prefix.size()));
      SupportContact policy{data["object_id"].as<std::string>(),data["support_id"].as<std::string>(),
                            data["floor_z"].as<double>()};
      const auto tool=data["tool_link"].as<std::string>();
      auto fail=[&](const char* why) {
        RCLCPP_WARN(rclcpp::get_logger("workcell.support_contact"), "%s", why);
        res.error_code_.val=moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
        res.trajectory_.reset(); return false;
      };
      auto start=scene->getCurrentState();
      moveit::core::robotStateMsgToRobotState(req.start_state,start); start.update();
      const auto* body=start.getAttachedBody(policy.object);
      if (!body || !scene->getWorld()->hasObject(policy.support) || !start.knowsFrameTransform(tool) ||
          !start.satisfiesBounds() || !std::isfinite(policy.floor_z) ||
          body->getShapes().size()!=1 || body->getShapes()[0]->type!=shapes::BOX)
        return fail("SUPPORT_CONTACT_INVALID_BINDING");
      // Runtime currently accepts BOX observations. Establish the actual lowest
      // corner, not the bottom of a conservative usable-placement bounding box.
      auto box=static_cast<const shapes::Box*>(body->getShapes()[0].get());
      double bottom=std::numeric_limits<double>::infinity();
      const auto& transform=body->getGlobalCollisionBodyTransforms()[0];
      for (int x : {-1,1}) for (int y : {-1,1}) for (int z : {-1,1})
        bottom=std::min(bottom,(transform*Eigen::Vector3d(x*box->size[0]/2,y*box->size[1]/2,z*box->size[2]/2)).z());
      const auto owner=scene->getWorld()->getObject(policy.support);
      double certified_floor=std::numeric_limits<double>::infinity();
      for(int x : {-1,1}) for(int y : {-1,1}) for(int z : {-1,1}) {
        const Eigen::Vector3d corner=transform*Eigen::Vector3d(x*box->size[0]/2,y*box->size[1]/2,z*box->size[2]/2);
        const double height=floorAt(*owner,corner.head<2>());
        if(!std::isfinite(height) || (std::isfinite(certified_floor) && std::abs(height-certified_floor)>1e-8))
          return fail("SUPPORT_CONTACT_FLOOR_UNPROVEN");
        certified_floor=height;
      }
      if(std::abs(policy.floor_z-certified_floor)>1e-8) return fail("SUPPORT_CONTACT_NOT_FLOOR");
      policy.floor_z=certified_floor;
      if (std::abs(bottom-policy.floor_z)>support_contact_tolerance_m)
        return fail("SUPPORT_CONTACT_OUTSIDE_TOLERANCE");
      auto local=scene->diff(); local->decoupleParent();
      collision_detection::DecideContactFn predicate=[policy](collision_detection::Contact& c) { return policy(c); };
      local->getAllowedCollisionMatrixNonConst().setEntry(policy.object,policy.support,predicate);
      auto clean=req; clean.path_constraints.name.clear();
      if (!local->isStateValid(start,clean.path_constraints,"")) return fail("SUPPORT_CONTACT_INVALID_START");
      // All FCL contacts for this pair are evaluated by the conditional callback;
      // robot/environment pairs and every unrelated pair retain their original ACM.
      if (!planner(local,clean,res) || !res.trajectory_ || res.trajectory_->getWayPointCount()<2)
        return fail("SUPPORT_CONTACT_PLAN_FAILED");
      auto& trajectory=*res.trajectory_;
      if (start.distance(trajectory.getFirstWayPoint())>1e-9)
        return fail("SUPPORT_CONTACT_START_CHANGED");
      const Eigen::Isometry3d origin=body->getGlobalPose();
      bool separated=false;
      double last_height=0.;
      auto valid=[&](const moveit::core::RobotState& state, bool require_strict) {
        const auto* carried=state.getAttachedBody(policy.object);
        if (!carried || !state.satisfiesBounds()) return false;
        const auto motion=carried->getGlobalPose().translation()-origin.translation();
        if (motion.head<2>().norm()>.0025 || motion.z() < last_height-1e-9 || motion.z()>.01 ||
            Eigen::AngleAxisd(carried->getGlobalPose().rotation()*origin.rotation().transpose()).angle()>.01)
          return false;
        last_height=motion.z();
        const bool strict=scene->isStateValid(state,clean.path_constraints,"");
        if (strict) { separated=true; return true; }
        return !require_strict && !separated && local->isStateValid(state,clean.path_constraints,"");
      };
      if (!valid(trajectory.getFirstWayPoint(),false)) return fail("SUPPORT_CONTACT_INVALID_START");
      for (std::size_t i=1;i<trajectory.getWayPointCount();++i) {
        const auto& previous=trajectory.getWayPoint(i-1); const auto& next=trajectory.getWayPoint(i);
        const double radius=body->getPose().translation().norm()+body->getShapePoses()[0].translation().norm()+
                            Eigen::Vector3d(box->size[0],box->size[1],box->size[2]).norm()/2;
        const double travel=carriedTravelBound(previous,next,radius);
        if(!std::isfinite(travel) || travel>10000*support_contact_tolerance_m/4)
          return fail("SUPPORT_CONTACT_NONLOCAL_PATH");
        const auto count=std::max(1,static_cast<int>(std::ceil(travel/(support_contact_tolerance_m/4))));
        for (int j=1;j<=count;++j) {
          moveit::core::RobotState sample(previous); previous.interpolate(next,double(j)/count,sample); sample.update();
          // After the first stored point, the original scene must be strictly
          // valid. Interpolated initial separation may consume only this same
          // floor tolerance; recontact after separation is never permitted.
          if (!valid(sample,j==count)) return fail("SUPPORT_CONTACT_CARRIED_PATH_COLLISION");
        }
      }
      if (!separated) return fail("SUPPORT_CONTACT_NOT_SEPARATED");
      // No fabricated adapter-added indexes. Humble's pipeline independently
      // checks the original scene and permits solely invalid start index 0.
      // We have qualified that start more strictly above and every later state
      // uses normal carried-object/environment collision checking.
      return true;
    } catch (const std::exception& e) {
      RCLCPP_WARN(rclcpp::get_logger("workcell.support_contact"),"SUPPORT_CONTACT_REJECTED: %s",e.what());
      res.error_code_.val=moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
      res.trajectory_.reset(); return false;
    }
  }
};
}
PLUGINLIB_EXPORT_CLASS(workcell::StraightCartesianPath, planning_request_adapter::PlanningRequestAdapter)
PLUGINLIB_EXPORT_CLASS(workcell::InitialSupportContact, planning_request_adapter::PlanningRequestAdapter)
PLUGINLIB_EXPORT_CLASS(workcell::PlanningEvidence, planning_request_adapter::PlanningRequestAdapter)

// ABI for the simulator execution owner: exactly the planner's per-contact
// predicate, with the caller's measured identities and geometry. No ACM writes.
extern "C" bool workcell_support_contact_valid(const char* object,const char* support,
 const char* first,const char* second,double floor,const double* point) {
  collision_detection::Contact c;
  c.body_name_1=first;c.body_name_2=second;
  c.body_type_1=c.body_name_1==object ? collision_detection::BodyTypes::ROBOT_ATTACHED : collision_detection::BodyTypes::WORLD_OBJECT;
  c.body_type_2=c.body_name_2==object ? collision_detection::BodyTypes::ROBOT_ATTACHED : collision_detection::BodyTypes::WORLD_OBJECT;
  c.pos=Eigen::Vector3d(point[0],point[1],point[2]);
  c.normal=Eigen::Vector3d(point[3],point[4],point[5]);c.depth=point[6];
  return workcell::SupportContact{object,support,floor}(c);
}
