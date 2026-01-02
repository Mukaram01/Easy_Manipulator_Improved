#ifndef EMD__GRASP_EXECUTION__MOVEIT2__EXECUTOR__DYNAMIC_SAFETY_ASYNC_EXECUTOR_HPP_
#define EMD__GRASP_EXECUTION__MOVEIT2__EXECUTOR__DYNAMIC_SAFETY_ASYNC_EXECUTOR_HPP_

#include <mutex>
#include <string>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "emd/grasp_execution/moveit2/executor.hpp"

namespace grasp_execution
{

namespace moveit2
{

/// Executor that streams trajectories to the dynamic safety aware controller.
class DynamicSafetyAsyncExecutor : public Executor
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(DynamicSafetyAsyncExecutor)

  DynamicSafetyAsyncExecutor();

  ~DynamicSafetyAsyncExecutor() override = default;

  bool load(
    const moveit_cpp::MoveItCppPtr & moveit_cpp,
    const std::string & name) override;

  bool run(
    const robot_trajectory::RobotTrajectory & robot_trajectory) override;

  void cancel() override;

private:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  bool wait_for_server();

  rclcpp::Logger logger_;
  moveit_cpp::MoveItCppPtr moveit_cpp_;
  rclcpp::Node::SharedPtr node_;
  std::string controller_name_;
  std::string action_name_;

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;

  std::mutex mutex_;
  GoalHandle::SharedPtr active_goal_;
};

}  // namespace moveit2

}  // namespace grasp_execution

#endif  // EMD__GRASP_EXECUTION__MOVEIT2__EXECUTOR__DYNAMIC_SAFETY_ASYNC_EXECUTOR_HPP_
