#include "support_contact_policy.hpp"
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <geometric_shapes/shapes.h>
#include <pluginlib/class_list_macros.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>

namespace workcell {
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
PLUGINLIB_EXPORT_CLASS(workcell::InitialSupportContact, planning_request_adapter::PlanningRequestAdapter)
