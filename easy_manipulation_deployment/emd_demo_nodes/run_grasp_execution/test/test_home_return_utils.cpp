// Copyright 2026 Mukaram
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

#include <rclcpp/parameter.hpp>

#include <vector>

#include "run_grasp_execution/home_return_utils.hpp"

TEST(TestHomeReturnUtils, SafeIntermediateEnabledOnlyWithValues)
{
  EXPECT_FALSE(run_grasp_execution::safe_intermediate_enabled(false, {0.1, 0.2}));
  EXPECT_FALSE(run_grasp_execution::safe_intermediate_enabled(true, {}));
  EXPECT_TRUE(run_grasp_execution::safe_intermediate_enabled(true, {0.1, 0.2}));
}

TEST(TestHomeReturnUtils, FailureReasonIncludesAttemptAndStep)
{
  const auto reason = run_grasp_execution::default_home_return_failure_reason(
    "Move back to home", 2, 5);
  EXPECT_NE(reason.find("Move back to home"), std::string::npos);
  EXPECT_NE(reason.find("2/5"), std::string::npos);
  EXPECT_NE(reason.find("no valid plan returned by MoveIt"), std::string::npos);
}

TEST(TestHomeReturnUtils, ParseSafeJointStateParamNotSetFallsBackToEmptyWithoutWarning)
{
  const rclcpp::Parameter missing_param("home_return.safe_joint_state");
  const auto resolution = run_grasp_execution::parse_safe_joint_state_parameter(
    missing_param,
    "home_return.safe_joint_state");

  EXPECT_TRUE(resolution.value.empty());
  EXPECT_FALSE(resolution.warning_message.has_value());
}

TEST(TestHomeReturnUtils, ParseSafeJointStateParamEmptyListIsAccepted)
{
  const rclcpp::Parameter param("home_return.safe_joint_state", std::vector<double>{});
  const auto resolution = run_grasp_execution::parse_safe_joint_state_parameter(
    param,
    "home_return.safe_joint_state");

  EXPECT_TRUE(resolution.value.empty());
  EXPECT_FALSE(resolution.warning_message.has_value());
}

TEST(TestHomeReturnUtils, ParseSafeJointStateParamWrongTypeStringFallsBackToEmptyWithWarning)
{
  const rclcpp::Parameter param("home_return.safe_joint_state", std::string("bad_type"));
  const auto resolution = run_grasp_execution::parse_safe_joint_state_parameter(
    param,
    "home_return.safe_joint_state");

  EXPECT_TRUE(resolution.value.empty());
  ASSERT_TRUE(resolution.warning_message.has_value());
  EXPECT_NE(resolution.warning_message->find("type 'string'"), std::string::npos);
}

TEST(TestHomeReturnUtils, ParseSafeJointStateParamWrongTypeIntegerFallsBackToEmptyWithWarning)
{
  const rclcpp::Parameter param("home_return.safe_joint_state", 42);
  const auto resolution = run_grasp_execution::parse_safe_joint_state_parameter(
    param,
    "home_return.safe_joint_state");

  EXPECT_TRUE(resolution.value.empty());
  ASSERT_TRUE(resolution.warning_message.has_value());
  EXPECT_NE(resolution.warning_message->find("type 'integer'"), std::string::npos);
}

TEST(TestHomeReturnUtils, SafeIntermediateSkipWarningForMismatchedDof)
{
  const auto warning = run_grasp_execution::safe_intermediate_skip_warning(
    true, {0.1, 0.2}, 6, "arm");
  ASSERT_TRUE(warning.has_value());
  EXPECT_NE(warning->find("has 2 values, expected 6"), std::string::npos);
  EXPECT_NE(warning->find("Skipping safe intermediate"), std::string::npos);
}

TEST(TestHomeReturnUtils, SafeIntermediateSkipWarningForEmptyList)
{
  const auto warning = run_grasp_execution::safe_intermediate_skip_warning(
    true, {}, 6, "arm");
  ASSERT_TRUE(warning.has_value());
  EXPECT_NE(warning->find("safe_joint_state is empty"), std::string::npos);
}

TEST(TestHomeReturnUtils, ParseSafeJointStateParamMissingFallsBackToEmptyWithoutWarning)
{
  const rclcpp::Parameter missing_param;
  const auto resolution = run_grasp_execution::parse_safe_joint_state_parameter(
    missing_param,
    "home_return.safe_joint_state");

  EXPECT_TRUE(resolution.value.empty());
  EXPECT_FALSE(resolution.warning_message.has_value());
}
