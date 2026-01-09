// Copyright 2020 ROS Industrial Consortium Asia Pacific
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

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "emd/grasp_execution/moveit2/executor/default_executor.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"

namespace grasp_execution
{

namespace moveit2
{

bool DefaultExecutor::load(
  const moveit_cpp::MoveItCppPtr & moveit_cpp,
  const std::string & /*name*/)
{
  trajectory_execution_manager_ = moveit_cpp->getTrajectoryExecutionManagerNonConst();
  // Force explicit bool operator
  return trajectory_execution_manager_ ? true : false;
}

// Referenced from moveit_cpp.cpp
// https://github.com/ros-planning/moveit2/blob/2499a72f7388a371905eaef72685fcfaae04335a/moveit_ros/planning_interface/moveit_cpp/src/moveit_cpp.cpp#L270-L297
bool DefaultExecutor::run(
  const robot_trajectory::RobotTrajectory & robot_trajectory)
{
  const auto & group_name = robot_trajectory.getGroupName();

  // Check if there are controllers that can handle the execution
  if (!trajectory_execution_manager_->ensureActiveControllersForGroup(group_name)) {
    RCLCPP_ERROR(
      this->logger_,
      "Execution failed! No active controllers configured for group '%s'",
      group_name.c_str());
    return false;
  }

  // Execute trajectory
  moveit_msgs::msg::RobotTrajectory robot_trajectory_msg;
  robot_trajectory.getRobotTrajectoryMsg(robot_trajectory_msg);

  trajectory_execution_manager_->push(robot_trajectory_msg);

  // Watchdog thread that stops execution when ROS is shutting down
  std::atomic_bool finished(false);
  std::thread watchdog([this, &finished]() {
    rclcpp::Rate r(10);
    while (rclcpp::ok() && !finished) {
      r.sleep();
    }
    if (!rclcpp::ok()) {
      RCLCPP_WARN(logger_, "Shutting down - stopping execution");
      trajectory_execution_manager_->stopExecution();
    }
  });

  trajectory_execution_manager_->execute();

  const double expected_duration = robot_trajectory.getDuration();
  const double requested_timeout =
    expected_duration * trajectory_execution_manager_->allowedExecutionDurationScaling() +
    trajectory_execution_manager_->allowedGoalDurationMargin();
  std::atomic_bool execution_complete(false);
  std::atomic_bool timed_out(false);
  std::mutex status_mutex;
  moveit_controller_manager::ExecutionStatus status(
    moveit_controller_manager::ExecutionStatus::UNKNOWN);
  std::thread wait_thread([this, &execution_complete, &status_mutex, &status]() {
    const auto wait_status = trajectory_execution_manager_->waitForExecution();
    {
      std::lock_guard<std::mutex> lock(status_mutex);
      status = wait_status;
    }
    execution_complete = true;
  });

  rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  const auto start_time = steady_clock.now();
  rclcpp::Rate wait_rate(50);
  while (rclcpp::ok() && !execution_complete) {
    const auto elapsed = steady_clock.now() - start_time;
    const double elapsed_seconds = elapsed.seconds();
    if (requested_timeout > 0.0 && elapsed_seconds >= requested_timeout) {
      const auto last_status = trajectory_execution_manager_->getLastExecutionStatus();
      RCLCPP_WARN(
        logger_,
        "Execution timed out after %.3f s (requested %.3f s). Last action state: %s",
        elapsed_seconds, requested_timeout, last_status.asString().c_str());
      trajectory_execution_manager_->stopExecution();
      timed_out = true;
      break;
    }
    wait_rate.sleep();
  }

  wait_thread.join();

  finished = true;
  watchdog.join();

  // Allow timeout
  // TODO(anyone): fix doesn't finish in time problem.
  if (timed_out) {
    status = moveit_controller_manager::ExecutionStatus::TIMED_OUT;
  }
  return (status == moveit_controller_manager::ExecutionStatus::SUCCEEDED) ||
         (status == moveit_controller_manager::ExecutionStatus::TIMED_OUT);
}

void DefaultExecutor::cancel()
{
  if (trajectory_execution_manager_) {
    trajectory_execution_manager_->stopExecution();
  }
}

}  // namespace moveit2

}  // namespace grasp_execution

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  grasp_execution::moveit2::DefaultExecutor, grasp_execution::moveit2::Executor)
