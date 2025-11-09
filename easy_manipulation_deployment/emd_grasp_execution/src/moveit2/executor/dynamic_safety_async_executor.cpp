// Copyright 2024 ROS Industrial Consortium Asia Pacific
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

#include <chrono>
#include <string>
#include <utility>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "emd/grasp_execution/moveit2/executor/dynamic_safety_async_executor.hpp"

namespace grasp_execution
{

namespace moveit2
{

using namespace std::chrono_literals;

DynamicSafetyAsyncExecutor::DynamicSafetyAsyncExecutor()
: logger_(rclcpp::get_logger("dynamic_safety_async_executor"))
{
}

bool DynamicSafetyAsyncExecutor::load(
  const moveit_cpp::MoveItCppPtr & moveit_cpp,
  const std::string & name)
{
  moveit_cpp_ = moveit_cpp;
  if (!moveit_cpp_) {
    RCLCPP_ERROR(logger_, "Invalid MoveItCpp pointer passed to DynamicSafetyAsyncExecutor");
    return false;
  }

  node_ = moveit_cpp_->getNode();
  if (!node_) {
    RCLCPP_ERROR(logger_, "Unable to retrieve rclcpp node from MoveItCpp instance");
    return false;
  }

  if (name.empty()) {
    RCLCPP_ERROR(logger_, "DynamicSafetyAsyncExecutor requires a controller name");
    return false;
  }

  controller_name_ = name;
  action_name_ = controller_name_ + "/follow_joint_trajectory";

  action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(node_, action_name_);
  if (!wait_for_server()) {
    RCLCPP_ERROR(
      logger_, "Failed to contact action server '%s' for dynamic safety execution", action_name_.c_str());
    action_client_.reset();
    return false;
  }

  RCLCPP_INFO(logger_, "Dynamic safety executor connected to action server '%s'", action_name_.c_str());
  return true;
}

bool DynamicSafetyAsyncExecutor::run(
  const robot_trajectory::RobotTrajectory & robot_trajectory)
{
  if (!action_client_) {
    RCLCPP_ERROR(logger_, "DynamicSafetyAsyncExecutor is not initialized");
    return false;
  }

  if (!wait_for_server()) {
    RCLCPP_ERROR(logger_, "Dynamic safety action server '%s' unavailable", action_name_.c_str());
    return false;
  }

  moveit_msgs::msg::RobotTrajectory robot_traj_msg;
  robot_trajectory.getRobotTrajectoryMsg(robot_traj_msg);

  if (robot_traj_msg.joint_trajectory.points.empty()) {
    RCLCPP_WARN(logger_, "Received empty joint trajectory. Nothing to execute.");
    return true;
  }

  FollowJointTrajectory::Goal goal;
  goal.trajectory = robot_traj_msg.joint_trajectory;

  auto goal_future = action_client_->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(node_, goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(logger_, "Failed to send trajectory goal to dynamic safety controller");
    return false;
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(logger_, "Dynamic safety controller rejected the trajectory goal");
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_goal_ = goal_handle;
  }

  auto result_future = action_client_->async_get_result(goal_handle);
  auto result_code = rclcpp::spin_until_future_complete(node_, result_future);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_goal_.reset();
  }

  if (result_code != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(logger_, "Failed to retrieve result from dynamic safety controller");
    return false;
  }

  auto wrapped_result = result_future.get();
  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(logger_, "Dynamic safety execution completed successfully");
      return true;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(logger_, "Dynamic safety execution was canceled");
      return false;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(logger_, "Dynamic safety execution aborted by controller");
      return false;
    default:
      RCLCPP_ERROR(logger_, "Dynamic safety execution failed with unknown result code");
      return false;
  }
}

void DynamicSafetyAsyncExecutor::cancel()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (action_client_ && active_goal_) {
    (void)action_client_->async_cancel_goal(active_goal_);
  }
}

bool DynamicSafetyAsyncExecutor::wait_for_server()
{
  if (!action_client_) {
    return false;
  }

  if (!action_client_->action_server_is_ready()) {
    if (!action_client_->wait_for_action_server(5s)) {
      RCLCPP_WARN(
        logger_, "Timed out waiting for dynamic safety action server '%s'", action_name_.c_str());
      return false;
    }
  }

  return true;
}

}  // namespace moveit2

}  // namespace grasp_execution

PLUGINLIB_EXPORT_CLASS(
  grasp_execution::moveit2::DynamicSafetyAsyncExecutor,
  grasp_execution::moveit2::Executor)
