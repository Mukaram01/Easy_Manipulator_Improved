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
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <stdexcept>

#include "emd/dynamic_safety/collision_checker_moveit.hpp"
#include "moveit/collision_detection_fcl/collision_detector_allocator_fcl.h"

// Commented out due to library confliction with tesseract
#ifndef END_DYNAMIC_SAFETY_TESSERACT
#include "moveit/collision_detection_bullet/collision_detector_allocator_bullet.h"
#endif

namespace dynamic_safety_moveit
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("dynamic_safety_moveit.collision_checker");

namespace
{
bool state_has_variable(const moveit::core::RobotState & state, const std::string & name)
{
  const auto & variable_names = state.getVariableNames();
  return std::find(variable_names.begin(), variable_names.end(), name) != variable_names.end();
}

void set_joint_positions_from_trajectory(
  moveit::core::RobotState & state,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & positions)
{
  size_t position_index = 0;
  for (const auto & joint_name : joint_names) {
    const moveit::core::JointModel * joint_model = state.getJointModel(joint_name);
    if (joint_model) {
      const size_t variable_count = joint_model->getVariableCount();
      if (position_index + variable_count > positions.size()) {
        RCLCPP_WARN(
          LOGGER, "Not enough joint positions for '%s' (needed %zu, have %zu)",
          joint_name.c_str(), variable_count, positions.size() - position_index);
        break;
      }
      std::vector<double> joint_positions(
        positions.begin() + position_index,
        positions.begin() + position_index + variable_count);
      state.setJointPositions(joint_model, joint_positions);
      state.enforceBounds(joint_model);
      position_index += variable_count;
      continue;
    }

    if (state_has_variable(state, joint_name)) {
      if (position_index >= positions.size()) {
        RCLCPP_WARN(LOGGER, "Not enough joint positions for variable '%s'", joint_name.c_str());
        break;
      }
      state.setVariablePosition(joint_name, positions[position_index]);
      state.enforceBounds();
      position_index += 1;
      continue;
    }

    RCLCPP_WARN(LOGGER, "Unknown joint or variable '%s' in trajectory", joint_name.c_str());
  }
}

void set_joint_positions_from_state_msg(
  moveit::core::RobotState & state,
  const sensor_msgs::msg::JointState & joint_states)
{
  if (joint_states.name.size() != joint_states.position.size()) {
    RCLCPP_WARN(
      LOGGER, "JointState name/position size mismatch (%zu vs %zu)",
      joint_states.name.size(), joint_states.position.size());
  }

  const size_t entry_count = std::min(joint_states.name.size(), joint_states.position.size());
  std::unordered_map<std::string, double> position_lookup;
  position_lookup.reserve(entry_count);
  for (size_t i = 0; i < entry_count; ++i) {
    position_lookup.emplace(joint_states.name[i], joint_states.position[i]);
  }

  const auto & joint_models = state.getRobotModel()->getJointModels();
  for (const auto * joint_model : joint_models) {
    const auto & variable_names = joint_model->getVariableNames();
    if (variable_names.empty()) {
      continue;
    }
    std::vector<double> joint_positions;
    joint_positions.reserve(variable_names.size());
    bool has_all_variables = true;
    for (const auto & variable_name : variable_names) {
      const auto it = position_lookup.find(variable_name);
      if (it == position_lookup.end()) {
        has_all_variables = false;
        break;
      }
      joint_positions.push_back(it->second);
    }
    if (has_all_variables) {
      state.setJointPositions(joint_model, joint_positions);
      state.enforceBounds(joint_model);
    }
  }

  for (size_t i = 0; i < entry_count; ++i) {
    const std::string & name = joint_states.name[i];
    const double value = joint_states.position[i];
    const moveit::core::JointModel * joint_model = state.getJointModel(name);
    if (joint_model && joint_model->getVariableCount() == 1) {
      state.setJointPositions(joint_model, {value});
      state.enforceBounds(joint_model);
      continue;
    }
    if (state_has_variable(state, name)) {
      state.setVariablePosition(name, value);
      state.enforceBounds();
    }
  }
}
}  // namespace

MoveitCollisionCheckerContext::MoveitCollisionCheckerContext(
  const std::string & robot_urdf,
  const std::string & robot_srdf,
  const std::string & collision_checking_plugin)
: dynamic_safety::CollisionCheckerContext(robot_urdf, robot_srdf, collision_checking_plugin)
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
  // Construct planning scene
  scene_ = std::make_shared<planning_scene::PlanningScene>(umodel, smodel);

  // Load collision_checking_plugin
  // TODO(anyone): use the mapping file
  // auto loader = pluginlib::ClassLoader<collision_detection::CollisionPlugin>(
  //     "moveit_core", "collision_detection::CollisionPlugin");
  auto to_all_lower = [](std::string in) -> std::string {
      std::transform(in.begin(), in.end(), in.begin(), ::tolower);
      return in;
    };

  std::string plugin_name;
  // TODO(anyone): use plugin loader after release of fix
  //               https://github.com/ros-planning/moveit2/pull/658
  if (to_all_lower(collision_checking_plugin) == "fcl") {
    scene_->allocateCollisionDetector(
      collision_detection::CollisionDetectorAllocatorFCL::create()
    );
  } else if (to_all_lower(collision_checking_plugin) == "bullet") {
#ifndef EMD_DYNAMIC_SAFETY_TESSERACT
    scene_->allocateCollisionDetector(
      collision_detection::CollisionDetectorAllocatorBullet::create()
    );
#endif
  }
}

void MoveitCollisionCheckerContext::configure(
  const dynamic_safety::CollisionCheckerOption & option)
{
  collision_request_.group_name = option.group;
  collision_request_.distance = option.distance;
  collision_request_.contacts = true;
  scene_->getCollisionEnvNonConst()->setPadding(option.padding);
}

void MoveitCollisionCheckerContext::run_discrete(
  std::vector<std::string> joint_names,
  trajectory_msgs::msg::JointTrajectoryPoint point,
  uint8_t & result, double & distance)
{
  moveit::core::RobotState current_state = scene_->getCurrentState();

  // Update state
  set_joint_positions_from_trajectory(current_state, joint_names, point.positions);
  // Check robot collision
  current_state.updateCollisionBodyTransforms();
  // scene_->checkCollision(collision_request_, collision_result_, state);
  result = false;
  collision_result_.clear();
  scene_->getCollisionEnv()->checkRobotCollision(
    collision_request_, collision_result_, current_state, scene_->getAllowedCollisionMatrix());
  distance = collision_result_.distance;
  result |= collision_result_.collision;
  // collision_result_.print();

  collision_result_.clear();

  scene_->getCollisionEnvUnpadded()->checkSelfCollision(
    collision_request_, collision_result_, current_state, scene_->getAllowedCollisionMatrix());

  distance = std::min<double>(distance, collision_result_.distance);
  result |= collision_result_.collision;
  // collision_result_.print();

  collision_result_.clear();
}

void MoveitCollisionCheckerContext::run_continuous(
  std::vector<std::string> joint_names,
  trajectory_msgs::msg::JointTrajectoryPoint point1,
  trajectory_msgs::msg::JointTrajectoryPoint point2,
  uint8_t & result, double & distance)
{
  moveit::core::RobotState start_state = scene_->getCurrentState();
  moveit::core::RobotState end_state = scene_->getCurrentState();

  // Update state
  set_joint_positions_from_trajectory(start_state, joint_names, point1.positions);
  set_joint_positions_from_trajectory(end_state, joint_names, point2.positions);

  // Check robot collision
  start_state.updateCollisionBodyTransforms();
  end_state.updateCollisionBodyTransforms();
  result = false;
  collision_result_.clear();
  scene_->getCollisionEnv()->checkRobotCollision(
    collision_request_, collision_result_, start_state, end_state,
    scene_->getAllowedCollisionMatrix());
  distance = collision_result_.distance;
  result |= collision_result_.collision;
  // collision_result_.print();

  collision_result_.clear();
  scene_->getCollisionEnvUnpadded()->checkSelfCollision(
    collision_request_, collision_result_, start_state, scene_->getAllowedCollisionMatrix());

  distance = std::min<double>(distance, collision_result_.distance);
  result |= collision_result_.collision;
  // collision_result_.print();

  collision_result_.clear();
}

void MoveitCollisionCheckerContext::update(const sensor_msgs::msg::JointState & joint_states)
{
  auto & current_state = scene_->getCurrentStateNonConst();
  // Update state
  set_joint_positions_from_state_msg(current_state, joint_states);
}

void MoveitCollisionCheckerContext::update(const moveit_msgs::msg::PlanningScene & scene_msgs)
{
  scene_->processPlanningSceneWorldMsg(scene_msgs.world);
}

}  // namespace dynamic_safety_moveit
