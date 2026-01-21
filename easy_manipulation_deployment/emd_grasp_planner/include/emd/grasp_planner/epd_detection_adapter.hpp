// Copyright 2024 Advanced Remanufacturing and Technology Centre
// Copyright 2024 ROS-Industrial Consortium Asia Pacific Team
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

#ifndef EMD__GRASP_PLANNER__EPD_DETECTION_ADAPTER_HPP_
#define EMD__GRASP_PLANNER__EPD_DETECTION_ADAPTER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "rclcpp/rclcpp.hpp"

#if EPD_ENABLED == 1
#include <epd_msgs/msg/epd_object_localization.hpp>
#include <epd_msgs/msg/localized_object.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace grasp_planner
{

class EpdDetectionAdapter : public rclcpp::Node
{
public:
  explicit EpdDetectionAdapter(const rclcpp::NodeOptions & options);

  void setup(
    const std::string & detection_topic,
    const std::string & depth_topic,
    const std::string & camera_info_topic,
    const std::string & output_topic);

private:
  struct CameraIntrinsics
  {
    float fx{0.0F};
    float fy{0.0F};
    float ppx{0.0F};
    float ppy{0.0F};
    bool valid{false};
  };

  void detection_callback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr & msg);
  void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);

  bool get_depth_value(
    const cv::Mat & depth_image,
    const std::string & encoding,
    int u,
    int v,
    float & depth_m) const;
  CameraIntrinsics resolve_intrinsics(const sensor_msgs::msg::CameraInfo * camera_info) const;
  bool build_object_cloud(
    const vision_msgs::msg::BoundingBox2D & bbox,
    const CameraIntrinsics & intrinsics,
    const cv::Mat & depth_image,
    const std::string & depth_encoding,
    const std::string & depth_frame_id,
    const rclcpp::Time & depth_stamp,
    sensor_msgs::msg::PointCloud2 & cloud,
    geometry_msgs::msg::Point & centroid) const;

  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<epd_msgs::msg::EPDObjectLocalization>::SharedPtr epd_pub_;

  mutable std::mutex data_mutex_;
  cv::Mat last_depth_;
  std::string depth_encoding_;
  std::string depth_frame_id_;
  rclcpp::Time depth_stamp_;
  sensor_msgs::msg::CameraInfo last_camera_info_;
  bool has_camera_info_{false};
};

}  // namespace grasp_planner

#endif  // EPD_ENABLED

#endif  // EMD__GRASP_PLANNER__EPD_DETECTION_ADAPTER_HPP_
