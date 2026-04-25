// Copyright 2026

#include <gtest/gtest.h>

#include <cmath>
#include <Eigen/Geometry>

#include "emd/grasp_execution/utils.hpp"

namespace
{

TEST(PoseTransformUtils, ConvertsDesiredGraspPoseBackToOriginalDesiredMoveitPose)
{
  Eigen::Isometry3d desired_tool0 = Eigen::Isometry3d::Identity();
  desired_tool0.translation() = Eigen::Vector3d(0.31, -0.12, 0.44);
  desired_tool0.linear() = (Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ())).toRotationMatrix();

  Eigen::Isometry3d tool0_to_gripper = Eigen::Isometry3d::Identity();
  tool0_to_gripper.translation() = Eigen::Vector3d(0.0, 0.0, 0.17);
  tool0_to_gripper.linear() =
    Eigen::AngleAxisd(-1.5707963267948966, Eigen::Vector3d::UnitY()).toRotationMatrix();

  const Eigen::Isometry3d desired_gripper = desired_tool0 * tool0_to_gripper;
  const auto desired_gripper_pose = tf2::toMsg(desired_gripper);

  const auto converted_tool0_pose =
    grasp_execution::convert_grasp_frame_pose_to_moveit_link_pose(
    desired_gripper_pose, tool0_to_gripper);

  Eigen::Isometry3d converted_tool0 = Eigen::Isometry3d::Identity();
  tf2::fromMsg(converted_tool0_pose, converted_tool0);

  EXPECT_TRUE(converted_tool0.isApprox(desired_tool0, 1e-9));
}

TEST(PoseTransformUtils, RejectsInvalidQuaternionNormalization)
{
  geometry_msgs::msg::Quaternion quat;
  quat.x = 0.0;
  quat.y = 0.0;
  quat.z = 0.0;
  quat.w = 0.0;
  EXPECT_FALSE(grasp_execution::normalize_quaternion(quat));
}

}  // namespace
