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

#include <gtest/gtest.h>

#if EPD_ENABLED == 1

#define private public
#include "emd/grasp_planner/epd_detection_adapter.hpp"
#undef private

#include <sensor_msgs/image_encodings.hpp>

namespace
{
vision_msgs::msg::BoundingBox2D make_bbox(double cx, double cy, double sx, double sy)
{
  vision_msgs::msg::BoundingBox2D bbox;
  bbox.center.position.x = cx;
  bbox.center.position.y = cy;
  bbox.size_x = sx;
  bbox.size_y = sy;
  return bbox;
}
}

TEST(EpdDetectionAdapterTest, UnsupportedDepthEncodingIsRejected)
{
  auto adapter = std::make_shared<grasp_planner::EpdDetectionAdapter>(rclcpp::NodeOptions());

  sensor_msgs::msg::Image image;
  image.encoding = "rgb8";
  image.width = 4;
  image.height = 4;
  image.step = image.width * 3;
  image.data.resize(image.step * image.height, 0);

  adapter->depth_callback(std::make_shared<sensor_msgs::msg::Image>(image));

  std::lock_guard<std::mutex> lock(adapter->data_mutex_);
  EXPECT_TRUE(adapter->last_depth_.empty());
}

TEST(EpdDetectionAdapterTest, InvalidOrMissingIntrinsicsAreRejected)
{
  auto adapter = std::make_shared<grasp_planner::EpdDetectionAdapter>(rclcpp::NodeOptions());

  const auto missing_intrinsics = adapter->resolve_intrinsics(nullptr);
  EXPECT_FALSE(missing_intrinsics.valid);

  sensor_msgs::msg::CameraInfo invalid_camera_info;
  invalid_camera_info.k = {0.0, 0.0, 1.0, 0.0, 0.0, 2.0, 0.0, 0.0, 1.0};
  const auto invalid_intrinsics = adapter->resolve_intrinsics(&invalid_camera_info);
  EXPECT_FALSE(invalid_intrinsics.valid);
}

TEST(EpdDetectionAdapterTest, UsesBoundingBoxCenterDepthWhenRoiSamplesInvalid)
{
  auto adapter = std::make_shared<grasp_planner::EpdDetectionAdapter>(rclcpp::NodeOptions());

  cv::Mat depth_image(10, 10, CV_16UC1, cv::Scalar(0));
  depth_image.at<uint16_t>(5, 5) = 1500;

  grasp_planner::EpdDetectionAdapter::CameraIntrinsics intrinsics;
  intrinsics.fx = 100.0F;
  intrinsics.fy = 100.0F;
  intrinsics.ppx = 5.0F;
  intrinsics.ppy = 5.0F;
  intrinsics.valid = true;

  sensor_msgs::msg::PointCloud2 cloud;
  geometry_msgs::msg::Point centroid;
  const bool success = adapter->build_object_cloud(
    make_bbox(5.0, 5.0, 1.0, 1.0),
    intrinsics,
    depth_image,
    sensor_msgs::image_encodings::TYPE_16UC1,
    "camera_depth_optical_frame",
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    cloud,
    centroid);

  EXPECT_TRUE(success);
  EXPECT_EQ(cloud.width, 1u);
  EXPECT_NEAR(centroid.x, 0.0, 1e-6);
  EXPECT_NEAR(centroid.y, 0.0, 1e-6);
  EXPECT_NEAR(centroid.z, 1.5, 1e-6);
}

#endif
