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

#include "emd/grasp_planner/epd_detection_adapter.hpp"

#if EPD_ENABLED == 1

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace grasp_planner
{

EpdDetectionAdapter::EpdDetectionAdapter(const rclcpp::NodeOptions & options)
: rclcpp::Node("epd_detection_adapter", options)
{
  this->declare_parameter("camera_parameters.fx", 0.0);
  this->declare_parameter("camera_parameters.fy", 0.0);
  this->declare_parameter("camera_parameters.ppx", 0.0);
  this->declare_parameter("camera_parameters.ppy", 0.0);
}

void EpdDetectionAdapter::setup(
  const std::string & detection_topic,
  const std::string & depth_topic,
  const std::string & camera_info_topic,
  const std::string & output_topic)
{
  auto sensor_qos = rclcpp::SensorDataQoS();
  detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
    detection_topic,
    sensor_qos,
    std::bind(&EpdDetectionAdapter::detection_callback, this, std::placeholders::_1));

  depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    depth_topic,
    sensor_qos,
    std::bind(&EpdDetectionAdapter::depth_callback, this, std::placeholders::_1));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic,
    sensor_qos,
    std::bind(&EpdDetectionAdapter::camera_info_callback, this, std::placeholders::_1));

  epd_pub_ = this->create_publisher<epd_msgs::msg::EPDObjectLocalization>(
    output_topic,
    rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(
    this->get_logger(),
    "EPD detection adapter listening on %s with depth %s and camera info %s.",
    detection_topic.c_str(), depth_topic.c_str(), camera_info_topic.c_str());
}

void EpdDetectionAdapter::depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  } else {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Unsupported depth encoding '%s'.", msg->encoding.c_str());
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  last_depth_ = cv_ptr->image;
  depth_encoding_ = msg->encoding;
  depth_frame_id_ = msg->header.frame_id;
  depth_stamp_ = msg->header.stamp;
}

void EpdDetectionAdapter::camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_camera_info_ = *msg;
  has_camera_info_ = true;
}

EpdDetectionAdapter::CameraIntrinsics EpdDetectionAdapter::resolve_intrinsics(
  const sensor_msgs::msg::CameraInfo * camera_info) const
{
  CameraIntrinsics intrinsics;
  if (camera_info != nullptr) {
    intrinsics.fx = static_cast<float>(camera_info->k.at(0));
    intrinsics.fy = static_cast<float>(camera_info->k.at(4));
    intrinsics.ppx = static_cast<float>(camera_info->k.at(2));
    intrinsics.ppy = static_cast<float>(camera_info->k.at(5));
    intrinsics.valid = true;
    return intrinsics;
  }

  intrinsics.fx = static_cast<float>(this->get_parameter("camera_parameters.fx").as_double());
  intrinsics.fy = static_cast<float>(this->get_parameter("camera_parameters.fy").as_double());
  intrinsics.ppx = static_cast<float>(this->get_parameter("camera_parameters.ppx").as_double());
  intrinsics.ppy = static_cast<float>(this->get_parameter("camera_parameters.ppy").as_double());
  intrinsics.valid = intrinsics.fx > 0.0F && intrinsics.fy > 0.0F;
  return intrinsics;
}

bool EpdDetectionAdapter::get_depth_value(
  const cv::Mat & depth_image,
  const std::string & encoding,
  int u,
  int v,
  float & depth_m) const
{
  if (depth_image.empty()) {
    return false;
  }

  const int clamped_u = std::clamp(u, 0, depth_image.cols - 1);
  const int clamped_v = std::clamp(v, 0, depth_image.rows - 1);

  if (encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    const uint16_t depth_raw = depth_image.at<uint16_t>(clamped_v, clamped_u);
    if (depth_raw == 0U) {
      return false;
    }
    depth_m = static_cast<float>(depth_raw) * 0.001F;
    return std::isfinite(depth_m) && depth_m > 0.0F;
  }

  if (encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    const float depth_raw = depth_image.at<float>(clamped_v, clamped_u);
    if (!std::isfinite(depth_raw) || depth_raw <= 0.0F) {
      return false;
    }
    depth_m = depth_raw;
    return true;
  }

  return false;
}

bool EpdDetectionAdapter::build_object_cloud(
  const vision_msgs::msg::BoundingBox2D & bbox,
  const CameraIntrinsics & intrinsics,
  const cv::Mat & depth_image,
  const std::string & depth_encoding,
  const std::string & depth_frame_id,
  const rclcpp::Time & depth_stamp,
  sensor_msgs::msg::PointCloud2 & cloud,
  geometry_msgs::msg::Point & centroid) const
{
  if (!intrinsics.valid) {
    return false;
  }

  const int min_u = static_cast<int>(std::floor(bbox.center.position.x - bbox.size_x * 0.5));
  const int max_u = static_cast<int>(std::ceil(bbox.center.position.x + bbox.size_x * 0.5));
  const int min_v = static_cast<int>(std::floor(bbox.center.position.y - bbox.size_y * 0.5));
  const int max_v = static_cast<int>(std::ceil(bbox.center.position.y + bbox.size_y * 0.5));

  struct PointXYZ
  {
    float x;
    float y;
    float z;
  };

  std::vector<PointXYZ> points;
  points.reserve(static_cast<size_t>(std::max(1, (max_u - min_u) * (max_v - min_v))));

  constexpr int stride = 2;
  for (int v = min_v; v <= max_v; v += stride) {
    for (int u = min_u; u <= max_u; u += stride) {
      float depth_m = 0.0F;
      if (!get_depth_value(depth_image, depth_encoding, u, v, depth_m)) {
        continue;
      }
      PointXYZ point;
      point.x = (static_cast<float>(u) - intrinsics.ppx) / intrinsics.fx * depth_m;
      point.y = (static_cast<float>(v) - intrinsics.ppy) / intrinsics.fy * depth_m;
      point.z = depth_m;
      points.push_back(point);
    }
  }

  if (points.empty()) {
    float depth_m = 0.0F;
    if (!get_depth_value(
        depth_image,
        depth_encoding,
        static_cast<int>(std::lround(bbox.center.position.x)),
        static_cast<int>(std::lround(bbox.center.position.y)),
        depth_m))
    {
      return false;
    }
    PointXYZ point;
    point.x = (static_cast<float>(bbox.center.position.x) - intrinsics.ppx) / intrinsics.fx *
      depth_m;
    point.y = (static_cast<float>(bbox.center.position.y) - intrinsics.ppy) / intrinsics.fy *
      depth_m;
    point.z = depth_m;
    points.push_back(point);
  }

  geometry_msgs::msg::Point centroid_point;
  centroid_point.x = 0.0;
  centroid_point.y = 0.0;
  centroid_point.z = 0.0;
  for (const auto & point : points) {
    centroid_point.x += point.x;
    centroid_point.y += point.y;
    centroid_point.z += point.z;
  }
  centroid_point.x /= static_cast<double>(points.size());
  centroid_point.y /= static_cast<double>(points.size());
  centroid_point.z /= static_cast<double>(points.size());
  centroid = centroid_point;

  cloud = sensor_msgs::msg::PointCloud2();
  cloud.header.frame_id = depth_frame_id;
  cloud.header.stamp = depth_stamp;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  for (const auto & point : points) {
    *iter_x = point.x;
    *iter_y = point.y;
    *iter_z = point.z;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  return true;
}

void EpdDetectionAdapter::detection_callback(
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr & msg)
{
  cv::Mat depth_image;
  std::string depth_encoding;
  std::string depth_frame_id;
  rclcpp::Time depth_stamp;
  sensor_msgs::msg::CameraInfo camera_info;
  const sensor_msgs::msg::CameraInfo * camera_info_ptr = nullptr;
  CameraIntrinsics intrinsics;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (last_depth_.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "Depth image not available yet. Skipping detections.");
      return;
    }
    depth_image = last_depth_.clone();
    depth_encoding = depth_encoding_;
    depth_frame_id = depth_frame_id_;
    depth_stamp = depth_stamp_;
    if (has_camera_info_) {
      camera_info = last_camera_info_;
      camera_info_ptr = &camera_info;
    }
  }

  intrinsics = resolve_intrinsics(camera_info_ptr);
  if (!intrinsics.valid) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Camera intrinsics unavailable. Skipping detections.");
    return;
  }

  epd_msgs::msg::EPDObjectLocalization output;
  output.header = msg->header;

  for (const auto & detection : msg->detections) {
    epd_msgs::msg::LocalizedObject object;
    object.name = "object";
    if (!detection.results.empty()) {
      object.name = detection.results.front().hypothesis.class_id;
    }

    geometry_msgs::msg::Point centroid;
    sensor_msgs::msg::PointCloud2 cloud;
    if (!build_object_cloud(
        detection.bbox,
        intrinsics,
        depth_image,
        depth_encoding,
        depth_frame_id,
        depth_stamp,
        cloud,
        centroid))
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Skipping detection because depth could not be resolved.");
      continue;
    }

    object.centroid = centroid;
    object.segmented_pcl = cloud;
    output.objects.push_back(std::move(object));
  }

  if (!output.objects.empty()) {
    epd_pub_->publish(output);
  }
}

}  // namespace grasp_planner

#endif  // EPD_ENABLED
