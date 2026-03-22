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

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "emd/grasp_planner/grasp_scene.hpp"
#if EPD_ENABLED == 1
#include "emd/grasp_planner/epd_detection_adapter.hpp"
#endif

namespace
{
template<typename ValueT>
ValueT get_parameter_or(
  const rclcpp::Node::SharedPtr & node, const std::string & name, const ValueT & default_value)
{
  ValueT value = default_value;
  node->get_parameter_or(name, value, default_value);
  return value;
}
}  // namespace

static const rclcpp::Logger & LOGGER_DEMO = rclcpp::get_logger("DemoNode");
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  rclcpp::Node::SharedPtr node =
    rclcpp::Node::make_shared("grasp_planner_demo_node", "", node_options);

  const auto epd_enabled =
    get_parameter_or(node, "easy_perception_deployment.epd_enabled", false);
  const auto detection_topic =
    get_parameter_or(node, "easy_perception_deployment.epd_detection_topic", std::string(""));
  const auto detection_depth_topic = get_parameter_or(
    node, "easy_perception_deployment.epd_detection_depth_topic",
    std::string("/camera/depth/image_rect_raw"));
  const auto detection_camera_info_topic = get_parameter_or(
    node, "easy_perception_deployment.epd_detection_camera_info_topic",
    std::string("/camera/color/camera_info"));
  const auto localization_topic = get_parameter_or(
    node, "easy_perception_deployment.epd_localization_topic",
    std::string("/processor/epd_localize_output"));
  const auto tracking_enabled =
    get_parameter_or(node, "easy_perception_deployment.tracking_enabled", false);
  const auto tracking_topic = get_parameter_or(
    node, "easy_perception_deployment.epd_tracking_topic",
    std::string("/processor/epd_tracking_output"));
  const auto point_cloud_topic =
    get_parameter_or(node, "camera_parameters.point_cloud_topic", std::string("/camera/pointcloud"));
  const auto camera_fx = get_parameter_or(node, "camera_parameters.fx", 610.3740844726562);
  const auto camera_fy = get_parameter_or(node, "camera_parameters.fy", 609.8685913085938);
  const auto camera_ppx = get_parameter_or(node, "camera_parameters.ppx", 323.3077697753906);
  const auto camera_ppy = get_parameter_or(node, "camera_parameters.ppy", 235.43516540527344);

  rclcpp::executors::MultiThreadedExecutor executor;
  #if EPD_ENABLED == 1
  if (epd_enabled) {
    std::shared_ptr<grasp_planner::EpdDetectionAdapter> adapter_node;
    if (!detection_topic.empty()) {
      RCLCPP_INFO(LOGGER_DEMO, "EPD detection adapter enabled.");
      adapter_node = std::make_shared<grasp_planner::EpdDetectionAdapter>(node_options);
      adapter_node->set_parameters({
        rclcpp::Parameter("camera_parameters.fx", camera_fx),
        rclcpp::Parameter("camera_parameters.fy", camera_fy),
        rclcpp::Parameter("camera_parameters.ppx", camera_ppx),
        rclcpp::Parameter("camera_parameters.ppy", camera_ppy)
      });
      adapter_node->setup(
        detection_topic,
        detection_depth_topic,
        detection_camera_info_topic,
        localization_topic);
      executor.add_node(adapter_node);
    }
    RCLCPP_INFO(LOGGER_DEMO, "EPD Workflow Enabled");
    if (tracking_enabled && detection_topic.empty()) {
      RCLCPP_INFO(LOGGER_DEMO, "EPD Tracking Enabled");
      grasp_planner::GraspScene<epd_msgs::msg::EPDObjectTracking> demo(node);
      demo.setup(tracking_topic);
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
      demo.setup(localization_topic);
      executor.add_node(demo.node);
      executor.spin();
    }
  } else {
    RCLCPP_INFO(LOGGER_DEMO, "Direct Workflow Enabled");
    grasp_planner::GraspScene<sensor_msgs::msg::PointCloud2> demo(node);
    demo.setup(point_cloud_topic);
    executor.add_node(demo.node);
    executor.spin();
  }
  #else
  RCLCPP_INFO(LOGGER_DEMO, "epd_msgs not found, Direct Workflow Enabled");
  grasp_planner::GraspScene<sensor_msgs::msg::PointCloud2> demo(node);
  demo.setup(point_cloud_topic);
  executor.add_node(demo.node);
  executor.spin();
  #endif

  rclcpp::shutdown();
  std::cout << "Shutting Down" << std::endl;
  return 0;
}
