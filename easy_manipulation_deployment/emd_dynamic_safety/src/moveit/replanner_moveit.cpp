// Copyright 2021 ROS Industrial Consortium Asia Pacific
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <stdexcept>

#include "emd/dynamic_safety/replanner_moveit.hpp"
#include "moveit/robot_state/conversions.h"
#include "pluginlib/class_loader.hpp"
#include <rclcpp/parameter_client.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace dynamic_safety_moveit
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("dynamic_safety_moveit.replanner");

MoveitReplannerContext::MoveitReplannerContext(
  const std::string & robot_urdf,
  const std::string & robot_srdf,
  const dynamic_safety::ReplannerOption & option,
  const rclcpp::Node::SharedPtr & node)
{
  urdf::ModelSharedPtr umodel = std::make_shared<urdf::Model>();
  srdf::ModelSharedPtr smodel = std::make_shared<srdf::Model>();

  if (umodel->initString(robot_urdf)) {
    if (!smodel->initString(*umodel, robot_srdf)) {
      RCLCPP_ERROR(LOGGER, "Unable to parse SRDF");
      throw std::runtime_error("Unable to parse SRDF");
    }
  } else {
    RCLCPP_ERROR(LOGGER, "Unable to parse URDF");
    throw std::runtime_error("Unable to parse URDF");
  }

  // Construct robot model
  auto rm = std::make_shared<moveit::core::RobotModel>(umodel, smodel);

  // Joint Limit loader
  // new node for parameter loading
  auto joint_limits_node = std::make_shared<rclcpp::Node>(
    std::string(
      node->get_name()) + "_joint_limits_loader");

  auto joint_limit_parameters_client =
    std::make_shared<rclcpp::AsyncParametersClient>(
    joint_limits_node, option.joint_limits_parameter_server);
  while (!joint_limit_parameters_client->wait_for_service()) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        LOGGER, "Interrupted while waiting for %s service. Exiting.",
        option.joint_limits_parameter_server.c_str());
      throw std::runtime_error(
              "Failed to connect to joint limits service " +
              option.joint_limits_parameter_server);
    }
    RCLCPP_WARN(
      LOGGER, "%s service not available, waiting again...",
      option.joint_limits_parameter_server.c_str());
  }
  RCLCPP_INFO(
    LOGGER, "Connected to description server %s!!",
    option.joint_limits_parameter_server.c_str());

  auto joint_group = rm->getJointModelGroup(option.group);

  for (auto & name : joint_group->getVariableNames()) {
    auto joint = rm->getJointModel(name);
    moveit::core::VariableBounds bound;
    auto urdf_joint = umodel->getJoint(joint->getName());
    bound = joint->getVariableBounds(joint->getName());
    std::vector<rclcpp::Parameter> hal_p;
    auto hal_f = joint_limit_parameters_client->get_parameters(
      {option.joint_limits_parameter_namespace + "." +
        joint->getName() + ".has_acceleration_limits"});
    rclcpp::spin_until_future_complete(joint_limits_node, hal_f);
    if (hal_f.get()[0].as_bool()) {
      RCLCPP_INFO(LOGGER, "Found acceleration limits");
      auto al_f = joint_limit_parameters_client->get_parameters(
        {option.joint_limits_parameter_namespace + "." +
          joint->getName() + ".max_acceleration"});
      rclcpp::spin_until_future_complete(joint_limits_node, al_f);
      bound.acceleration_bounded_ = true;
      bound.max_acceleration_ = fabs(al_f.get()[0].as_double());
      bound.min_acceleration_ = -bound.max_acceleration_;
      joint->setVariableBounds(joint->getName(), bound);
    }
  }

  scene_ = std::make_shared<planning_scene::PlanningScene>(rm);

  // Construct a planning instance
  //
  // load the planning plugin
  auto planner_plugin_loader =
    std::make_unique<pluginlib::ClassLoader<planning_interface::PlannerManager>>(
    "moveit_core", "planning_interface::PlannerManager");
  auto to_all_lower = [](std::string in) -> std::string {
      std::transform(in.begin(), in.end(), in.begin(), ::tolower);
      return in;
    };

  std::string planner_name = to_all_lower(option.planner);
  std::string plugin_name("");
  if (planner_name == "ompl") {
    plugin_name = "ompl_interface/OMPLPlanner";
    planning_request_.planner_id = option.ompl_planner_id;
  }
  // TODO(anyone): Add in other planning methods.

  planning_manager_ = planner_plugin_loader->createUniqueInstance(plugin_name);

  // new node for planning config parameter loading
  auto planner_config_loader_node = std::make_shared<rclcpp::Node>(
    std::string(
      node->get_name()) + "_planner_config_loader");

  auto declare_and_set_planner_parameters =
    [&planner_config_loader_node](const std::vector<rclcpp::Parameter> & parameters) {
      for (const auto & parameter : parameters) {
        if (!planner_config_loader_node->has_parameter(parameter.get_name())) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.dynamic_typing = true;
          planner_config_loader_node->declare_parameter(
            parameter.get_name(),
            parameter.get_parameter_value(),
            descriptor);
        }
      }
      planner_config_loader_node->set_parameters(parameters);
    };

  // Load self parameters
  if (option.planner_parameter_server == node->get_name()) {
    auto planner_config =
      node->list_parameters({option.group, option.planner_parameter_namespace}, 5);
    if (!planner_config.names.empty()) {
      std::string result = "Parameters found:\n";
      for (auto & name : planner_config.names) {
        result += "\t" + name + "\n";
      }
      RCLCPP_WARN(LOGGER, "%s", result.c_str());
      auto planner_config_params = node->get_parameters(planner_config.names);
      declare_and_set_planner_parameters(planner_config_params);
    } else {
      RCLCPP_ERROR(
        LOGGER, "no planner configs defined");
    }
  } else {
    // Getting parameters from external node
    auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
      planner_config_loader_node, option.planner_parameter_server);
    while (!parameters_client->wait_for_service()) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          LOGGER, "Interrupted while waiting for %s service. Exiting.",
          option.planner_parameter_server.c_str());
        throw std::runtime_error(
                "Failed to connect to planner parameter service " +
                option.planner_parameter_server);
      }
      RCLCPP_WARN(
        LOGGER, "%s service not available, waiting again...",
        option.planner_parameter_server.c_str());
    }
    RCLCPP_INFO(
      LOGGER, "Connected to description server %s!!",
      option.planner_parameter_server.c_str());
    rcl_interfaces::msg::ListParametersResult planner_config;
    while (planner_config.names.empty()) {
      try {
        RCLCPP_INFO(LOGGER, "Get parameters");
        auto planner_config_future = parameters_client->list_parameters(
          {option.group, option.planner_parameter_namespace},
          5);
        rclcpp::spin_until_future_complete(
          planner_config_loader_node, planner_config_future);
        planner_config = planner_config_future.get();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(LOGGER, "%s", e.what());
      }
      if (!planner_config.names.empty()) {
        std::string result = "Parameters found:\n";
        for (auto & name : planner_config.names) {
          result += "\t" + name + "\n";
        }
        RCLCPP_WARN(LOGGER, "%s", result.c_str());
        break;
      } else {
        RCLCPP_ERROR(
          LOGGER, "dynamic_safety is waiting for planner_configs");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    auto f = parameters_client->get_parameters(planner_config.names);
    rclcpp::spin_until_future_complete(planner_config_loader_node, f);
    declare_and_set_planner_parameters(f.get());
  }

  if (!planning_manager_->initialize(
      scene_->getRobotModel(),
      planner_config_loader_node, option.planner_parameter_namespace))
  {
    throw std::runtime_error("Unable to initialize planning plugin");
  }

  planning_request_.group_name = option.group;
  planning_request_.allowed_planning_time = option.deadline;
  planning_request_.num_planning_attempts = 1;
  planning_request_.allowed_planning_time = 0.3;
  planning_request_.max_velocity_scaling_factor = 1;
  planning_request_.max_acceleration_scaling_factor = 1;

  time_parameterization_ = option.time_parameterization;
}

void MoveitReplannerContext::run(
  const std::vector<std::string> & joint_names,
  const trajectory_msgs::msg::JointTrajectoryPoint & start_point,
  const trajectory_msgs::msg::JointTrajectoryPoint & end_point,
  trajectory_msgs::msg::JointTrajectory & plan)
{
  // Take a lightweight diff-snapshot of the scene so the mutex can be released
  // before the potentially long solve() call, allowing update() to keep the
  // parent scene current while planning is in progress.
  planning_scene::PlanningScenePtr scene_snapshot;
  planning_interface::MotionPlanRequest request;
  {
    std::lock_guard<std::mutex> lk(scene_mtx_);
    scene_snapshot = scene_->diff();
    request = planning_request_;
  }

  auto & start_state = scene_snapshot->getCurrentStateNonConst();
  moveit::core::RobotState end_state = scene_snapshot->getCurrentState();

  // Update state — use a separate position index to correctly handle multi-axis
  // joints whose variable count may be greater than one.
  size_t pos_idx = 0;
  for (size_t i = 0;
    i < joint_names.size() && pos_idx < start_point.positions.size();
    i++)
  {
    const moveit::core::JointModel * jm = start_state.getJointModel(joint_names[i]);
    if (!jm) {
      // The joint_names / positions arrays are parallel (position[i] belongs to
      // joint_names[i]).  An unknown joint name still occupies one position slot
      // so we must advance pos_idx to keep subsequent joints correctly aligned.
      ++pos_idx;
      continue;
    }
    const size_t var_count = jm->getVariableCount();
    if (pos_idx + var_count > start_point.positions.size() ||
      pos_idx + var_count > end_point.positions.size())
    {
      RCLCPP_WARN(
        LOGGER,
        "Not enough positions for joint '%s' (need %zu, have %zu); skipping",
        joint_names[i].c_str(), var_count,
        start_point.positions.size() - pos_idx);
      break;
    }
    start_state.setJointPositions(joint_names[i], &start_point.positions[pos_idx]);
    end_state.setJointPositions(joint_names[i], &end_point.positions[pos_idx]);
    pos_idx += var_count;
  }

  moveit::core::robotStateToRobotStateMsg(start_state, request.start_state);
  request.goal_constraints = {
    kinematic_constraints::constructGoalConstraints(
      end_state,
      scene_snapshot->getRobotModel()->getJointModelGroup(
        request.group_name))
  };

  planning_interface::MotionPlanResponse planning_response;

  auto context =
    planning_manager_->getPlanningContext(
    scene_snapshot, request, planning_response.error_code_);
  if (context) {
    context->solve(planning_response);
    if (planning_response.error_code_.val == planning_response.error_code_.SUCCESS) {
      moveit_msgs::msg::RobotTrajectory moveit_traj_msg;
      planning_response.trajectory_->getRobotTrajectoryMsg(moveit_traj_msg);
      plan = std::move(moveit_traj_msg.joint_trajectory);
    } else {
      RCLCPP_ERROR(LOGGER, "Planning failed");
    }
  } else {
    RCLCPP_ERROR(LOGGER, "No planner found");
  }
}

void MoveitReplannerContext::update(
  const sensor_msgs::msg::JointState & joint_states)
{
  if (scene_mtx_.try_lock()) {
    auto & current_state = scene_->getCurrentStateNonConst();
    // JointState messages carry per-variable (not per-joint) entries, so use
    // setVariablePosition for a correct 1-to-1 variable update that handles
    // both single-axis and multi-axis joints.
    for (size_t i = 0;
      i < joint_states.name.size() && i < joint_states.position.size();
      i++)
    {
      if (current_state.getRobotModel()->hasVariableIndex(joint_states.name[i])) {
        current_state.setVariablePosition(joint_states.name[i], joint_states.position[i]);
      }
    }
    scene_mtx_.unlock();
  } else {
    RCLCPP_WARN(LOGGER, "Planning ongoing scene is not updated");
  }
}

void MoveitReplannerContext::update(
  const moveit_msgs::msg::PlanningScene & scene)
{
  if (scene_mtx_.try_lock()) {
    scene_->processPlanningSceneWorldMsg(scene.world);
    scene_mtx_.unlock();
  } else {
    RCLCPP_WARN(LOGGER, "Planning ongoing scene is not updated");
  }
}

bool MoveitReplannerContext::time_parameterize(
  trajectory_msgs::msg::JointTrajectory & plan,
  double scale)
{
  robot_trajectory::RobotTrajectory rt(scene_->getRobotModel(), planning_request_.group_name);
  moveit_msgs::msg::RobotTrajectory rt_msg;
  rt_msg.joint_trajectory = plan;
  auto state = scene_->getCurrentState();
  rt.setRobotTrajectoryMsg(scene_->getCurrentState(), rt_msg);
  if (!_time_parameterization(rt, scale)) {
    return false;
  }
  rt.getRobotTrajectoryMsg(rt_msg);
  plan = rt_msg.joint_trajectory;
  return true;
}

bool MoveitReplannerContext::_time_parameterization(
  robot_trajectory::RobotTrajectory & trajectory, double scale)
{
  if (time_parameterization_ == "totg") {
    return totg_.computeTimeStamps(trajectory, scale);
  } else if (time_parameterization_ == "iptp") {
    return iptp_.computeTimeStamps(trajectory, scale);
  } else if (time_parameterization_ == "isp") {
    return isp_.computeTimeStamps(trajectory, scale);
  }
  return false;
}

}  // namespace dynamic_safety_moveit
