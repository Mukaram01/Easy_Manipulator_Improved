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
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "emd/dynamic_safety/collision_checker_tesseract.hpp"

#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_srdf/srdf_model.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_collision/core/types.h>

namespace dynamic_safety_tesseract
{

// Static member definitions (must live in exactly one translation unit).
tesseract_environment::Environment::Ptr TesseractCollisionCheckerContext::env;
int TesseractCollisionCheckerContext::instances = 0;

static const rclcpp::Logger LOGGER =
  rclcpp::get_logger("dynamic_safety_tesseract.collision_checker");

TesseractCollisionCheckerContext::TesseractCollisionCheckerContext(
  const std::string & robot_urdf,
  const std::string & robot_srdf,
  const std::string & /*collision_checking_plugin*/)
: dynamic_safety::CollisionCheckerContext(robot_urdf, robot_srdf, "")
{
  if (!env) {
    env = std::make_shared<tesseract_environment::Environment>();
    auto locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
    auto scene_graph = tesseract_urdf::parseURDFString(robot_urdf, *locator);
    if (!scene_graph) {
      throw std::runtime_error("TesseractCollisionChecker: failed to parse URDF string");
    }
    auto srdf_model = std::make_shared<tesseract_srdf::SRDFModel>();
    srdf_model->initString(*scene_graph, robot_srdf);
    if (!env->init(*scene_graph, srdf_model, locator)) {
      throw std::runtime_error("TesseractCollisionChecker: failed to initialise environment");
    }
  }

  discrete_manager_ = env->getDiscreteContactManager();
  if (!discrete_manager_) {
    throw std::runtime_error(
            "TesseractCollisionChecker: failed to obtain discrete contact manager");
  }

  continuous_manager_ = env->getContinuousContactManager();
  if (!continuous_manager_) {
    throw std::runtime_error(
            "TesseractCollisionChecker: failed to obtain continuous contact manager");
  }

  state_solver_ = env->getStateSolver();
  clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
}

void TesseractCollisionCheckerContext::configure(
  const dynamic_safety::CollisionCheckerOption & option)
{
  if (option.distance) {
    collision_check_config_.contact_request.type =
      tesseract_collision::ContactTestType::CLOSEST;
  } else {
    collision_check_config_.contact_request.type =
      tesseract_collision::ContactTestType::FIRST;
  }
}

void TesseractCollisionCheckerContext::run_discrete(
  std::vector<std::string> joint_names,
  trajectory_msgs::msg::JointTrajectoryPoint point,
  uint8_t & result, double & distance)
{
  if (point.positions.size() < joint_names.size()) {
    RCLCPP_WARN(LOGGER, "run_discrete: position vector shorter than joint name vector");
    result = 0;
    distance = -1.0;
    return;
  }

  std::unordered_map<std::string, double> joints;
  joints.reserve(joint_names.size());
  for (size_t i = 0; i < joint_names.size(); ++i) {
    joints[joint_names[i]] = point.positions[i];
  }

  auto state = state_solver_->getState(joints);

  discrete_manager_->setCollisionObjectsTransform(state.link_transforms);

  collision_result_.clear();
  discrete_manager_->contactTest(
    collision_result_,
    collision_check_config_.contact_request);

  result = static_cast<uint8_t>(!collision_result_.empty());

  distance = std::numeric_limits<double>::max();
  for (auto & pair : collision_result_) {
    for (auto & contact : pair.second) {
      if (contact.distance < distance) {
        distance = contact.distance;
      }
    }
  }
  if (distance == std::numeric_limits<double>::max()) {
    distance = -1.0;
  }

  collision_result_.clear();
}

void TesseractCollisionCheckerContext::run_continuous(
  std::vector<std::string> joint_names,
  trajectory_msgs::msg::JointTrajectoryPoint point1,
  trajectory_msgs::msg::JointTrajectoryPoint point2,
  uint8_t & result, double & distance)
{
  if (point1.positions.size() < joint_names.size() ||
    point2.positions.size() < joint_names.size())
  {
    RCLCPP_WARN(LOGGER, "run_continuous: position vector shorter than joint name vector");
    result = 0;
    distance = -1.0;
    return;
  }

  std::unordered_map<std::string, double> joints1, joints2;
  joints1.reserve(joint_names.size());
  joints2.reserve(joint_names.size());
  for (size_t i = 0; i < joint_names.size(); ++i) {
    joints1[joint_names[i]] = point1.positions[i];
    joints2[joint_names[i]] = point2.positions[i];
  }

  auto state1 = state_solver_->getState(joints1);
  auto state2 = state_solver_->getState(joints2);

  continuous_manager_->setCollisionObjectsTransform(
    state1.link_transforms, state2.link_transforms);

  collision_result_.clear();
  continuous_manager_->contactTest(
    collision_result_,
    collision_check_config_.contact_request);

  result = static_cast<uint8_t>(!collision_result_.empty());

  distance = std::numeric_limits<double>::max();
  for (auto & pair : collision_result_) {
    for (auto & contact : pair.second) {
      if (contact.distance < distance) {
        distance = contact.distance;
      }
    }
  }
  if (distance == std::numeric_limits<double>::max()) {
    distance = -1.0;
  }

  collision_result_.clear();
}

void TesseractCollisionCheckerContext::update(
  const sensor_msgs::msg::JointState & joint_states)
{
  const size_t count =
    std::min(joint_states.name.size(), joint_states.position.size());

  std::unordered_map<std::string, double> joints;
  joints.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    joints[joint_states.name[i]] = joint_states.position[i];
  }

  env->setState(joints);
  // Refresh the cached state solver so subsequent getState() calls reflect
  // any environment changes triggered by setState().
  state_solver_ = env->getStateSolver();
}

}  // namespace dynamic_safety_tesseract
