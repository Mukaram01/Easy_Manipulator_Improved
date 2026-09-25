// No robot, simulator, controller server or motion: exercise the real TEM's
// controller-plugin creation and shutdown, including its private executor.
#include <rclcpp/rclcpp.hpp>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <fstream>
#include <iostream>
#include <string>
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.parameter_overrides({
      rclcpp::Parameter("moveit_controller_manager", "moveit_simple_controller_manager/MoveItSimpleControllerManager"),
      rclcpp::Parameter("moveit_simple_controller_manager.controller_names", std::vector<std::string>{ "probe_arm" }),
      rclcpp::Parameter("moveit_simple_controller_manager.probe_arm.type", "FollowJointTrajectory"),
      rclcpp::Parameter("moveit_simple_controller_manager.probe_arm.action_ns", "follow_joint_trajectory"),
      rclcpp::Parameter("moveit_simple_controller_manager.probe_arm.joints", std::vector<std::string>{ "probe_joint" }) });
  auto node = std::make_shared<rclcpp::Node>("workcell_tem_teardown_probe", options);
  auto manager = std::make_unique<trajectory_execution_manager::TrajectoryExecutionManager>(node, nullptr, nullptr);
  std::vector<std::string> controllers;
  manager->getControllerManager()->getControllersList(controllers);
  if (controllers != std::vector<std::string>{ "probe_arm" }) return 2;
  std::cout << "TEM_READY" << std::endl;
  if (argc > 1 && std::string(argv[1]) == "--self-shutdown") rclcpp::shutdown();
  else rclcpp::spin(node);
  manager.reset();
  node.reset();
  if (rclcpp::ok()) rclcpp::shutdown();
  std::cout << "TEM_DESTROYED" << std::endl;
  return 0;
}
