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

#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "emd/grasp_execution/moveit2/moveit_cpp_if.hpp"

namespace grasp_execution
{
namespace moveit2
{
namespace
{

TEST(MoveitObjectPoseUtilsTest, ReturnsDefaultPoseWhenObjectHasNoShapePoses)
{
  auto object = std::make_shared<collision_detection::World::Object>("object_without_pose");

  EXPECT_TRUE(object->shape_poses_.empty());

  const auto pose = detail::get_object_pose_from_world_object(
    object, "object_without_pose", rclcpp::get_logger("test_logger"));

  EXPECT_DOUBLE_EQ(pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(pose.position.z, 0.0);
  EXPECT_DOUBLE_EQ(pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(pose.orientation.w, 0.0);
}

TEST(MoveitObjectPoseUtilsTest, ConvertsFirstShapePoseWhenAvailable)
{
  auto object = std::make_shared<collision_detection::World::Object>("object_with_pose");
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().x() = 0.1;
  transform.translation().y() = -0.2;
  transform.translation().z() = 0.3;
  object->shape_poses_.push_back(transform);

  const auto pose = detail::get_object_pose_from_world_object(
    object, "object_with_pose", rclcpp::get_logger("test_logger"));

  EXPECT_DOUBLE_EQ(pose.position.x, 0.1);
  EXPECT_DOUBLE_EQ(pose.position.y, -0.2);
  EXPECT_DOUBLE_EQ(pose.position.z, 0.3);
  EXPECT_DOUBLE_EQ(pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(pose.orientation.w, 1.0);
}

}  // namespace
}  // namespace moveit2
}  // namespace grasp_execution
