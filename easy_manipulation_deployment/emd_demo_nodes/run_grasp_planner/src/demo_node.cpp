// Copyright 2020 Advanced Remanufacturing and Technology Centre
// Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
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

#include "rclcpp/rclcpp.hpp"
#include "emd/grasp_planner/grasp_scene.hpp"
#if EPD_ENABLED == 1
#include "emd/grasp_planner/epd_detection_adapter.hpp"
#endif


static const rclcpp::Logger & LOGGER_DEMO = rclcpp::get_logger("DemoNode");
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.declare_parameters_from_overrides(true);

  rclcpp::Node::SharedPtr node =
    rclcpp::Node::make_shared("grasp_planner_demo_node", "", node_options);

  node->declare_parameter<bool>("easy_perception_deployment.epd_enabled", false);
  node->declare_parameter<std::string>(
    "easy_perception_deployment.epd_detection_topic", "");
  node->declare_parameter<std::string>(
    "easy_perception_deployment.epd_detection_depth_topic", "/camera/depth/image_rect_raw");
  node->declare_parameter<std::string>(
    "easy_perception_deployment.epd_detection_camera_info_topic", "/camera/color/camera_info");
  node->declare_parameter<std::string>(
    "easy_perception_deployment.epd_localization_topic", "/processor/epd_localize_output");
  node->declare_parameter<bool>("easy_perception_deployment.tracking_enabled", false);
  node->declare_parameter<std::string>(
    "easy_perception_deployment.epd_tracking_topic", "/processor/epd_tracking_output");
  node->declare_parameter<double>("easy_perception_deployment.epd_msg_timeout_s", 5.0);
  node->declare_parameter<double>("easy_perception_deployment.epd_service_wait_timeout_s", 1.0);
  node->declare_parameter<std::string>("camera_parameters.point_cloud_topic", "/camera/pointcloud");
  node->declare_parameter<double>("camera_parameters.fx", 610.3740844726562);
  node->declare_parameter<double>("camera_parameters.fy", 609.8685913085938);
  node->declare_parameter<double>("camera_parameters.ppx", 323.3077697753906);
  node->declare_parameter<double>("camera_parameters.ppy", 235.43516540527344);

  rclcpp::executors::MultiThreadedExecutor executor;
  #if EPD_ENABLED == 1
  if (node->get_parameter("easy_perception_deployment.epd_enabled").as_bool()) {
    std::shared_ptr<grasp_planner::EpdDetectionAdapter> adapter_node;
    const auto detection_topic =
      node->get_parameter("easy_perception_deployment.epd_detection_topic").as_string();
    if (!detection_topic.empty()) {
      RCLCPP_INFO(LOGGER_DEMO, "EPD detection adapter enabled.");
      adapter_node = std::make_shared<grasp_planner::EpdDetectionAdapter>(node_options);
      adapter_node->set_parameters({
        rclcpp::Parameter(
          "camera_parameters.fx",
          node->get_parameter("camera_parameters.fx").as_double()),
        rclcpp::Parameter(
          "camera_parameters.fy",
          node->get_parameter("camera_parameters.fy").as_double()),
        rclcpp::Parameter(
          "camera_parameters.ppx",
          node->get_parameter("camera_parameters.ppx").as_double()),
        rclcpp::Parameter(
          "camera_parameters.ppy",
          node->get_parameter("camera_parameters.ppy").as_double())
      });
      adapter_node->setup(
        detection_topic,
        node->get_parameter(
          "easy_perception_deployment.epd_detection_depth_topic").as_string(),
        node->get_parameter(
          "easy_perception_deployment.epd_detection_camera_info_topic").as_string(),
        node->get_parameter(
          "easy_perception_deployment.epd_localization_topic").as_string());
      executor.add_node(adapter_node);
    }
    RCLCPP_INFO(LOGGER_DEMO, "EPD Workflow Enabled");
    const bool tracking_enabled =
      node->get_parameter("easy_perception_deployment.tracking_enabled").as_bool();
    if (tracking_enabled && detection_topic.empty()) {
      RCLCPP_INFO(LOGGER_DEMO, "EPD Tracking Enabled");
      grasp_planner::GraspScene<epd_msgs::msg::EPDObjectTracking> demo(node);
      demo.setup(node->get_parameter("easy_perception_deployment.epd_tracking_topic").as_string());
      executor.add_node(demo.node);
      executor.spin();
    } else {
      if (tracking_enabled && !detection_topic.empty()) {
        RCLCPP_WARN(
          LOGGER_DEMO,
          "Detection adapter publishes localization data; falling back to localization workflow.");
      }
      RCLCPP_INFO(LOGGER_DEMO, "EPD Localization Enabled");
      grasp_planner::GraspScene<epd_msgs::msg::EPDObjectLocalization> demo(node);
      demo.setup(
        node->get_parameter(
          "easy_perception_deployment.epd_localization_topic").as_string());
      executor.add_node(demo.node);
      executor.spin();
    }
  } else {
    RCLCPP_INFO(LOGGER_DEMO, "Direct Workflow Enabled");
    grasp_planner::GraspScene<sensor_msgs::msg::PointCloud2> demo(node);
    demo.setup(node->get_parameter("camera_parameters.point_cloud_topic").as_string());
    executor.add_node(demo.node);
    executor.spin();
  }
  #else
  RCLCPP_INFO(LOGGER_DEMO, "epd_msgs not found, Direct Workflow Enabled");
  grasp_planner::GraspScene<sensor_msgs::msg::PointCloud2> demo(node);
  demo.setup(node->get_parameter("camera_parameters.point_cloud_topic").as_string());
  executor.add_node(demo.node);
  executor.spin();
  #endif

  rclcpp::shutdown();
  std::cout << "Shutting Down" << std::endl;
  return 0;
}
