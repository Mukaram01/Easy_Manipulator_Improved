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

#include <fstream>
#include <string>

#include "run_grasp_execution/explicit_release_pose_utils.hpp"

namespace
{

std::string write_temp_payload(const std::string & content)
{
  const std::string path = "/tmp/test_explicit_release_pose_payload.json";
  std::ofstream out(path);
  out << content;
  out.close();
  return path;
}

}  // namespace

TEST(ExplicitReleasePoseUtilsTest, NormalizesObjectIdHashes)
{
  EXPECT_EQ(run_grasp_execution::normalize_object_id("##box_1"), "box_1");
  EXPECT_EQ(run_grasp_execution::normalize_object_id("box_1"), "box_1");
}

TEST(ExplicitReleasePoseUtilsTest, LoadsValidDestinationPose)
{
  const auto path = write_temp_payload(R"JSON({
    "grasp_task": {
      "grasp_targets": [
        {
          "object_id": "#box_red",
          "destination_id": "bin_red",
          "destination_name": "Red Bin",
          "destination_pose": {
            "frame_id": "world",
            "xyz": [0.4, 0.2, 0.3],
            "rpy": [0.0, 0.0, 1.57]
          }
        }
      ]
    }
  })JSON");

  const auto result = run_grasp_execution::load_explicit_release_pose_bridge_payload(path);
  ASSERT_TRUE(result.loaded);
  ASSERT_EQ(result.entries.size(), 1U);
  EXPECT_TRUE(result.entries.front().valid_pose);
  EXPECT_EQ(result.entries.front().normalized_object_id, "box_red");
  EXPECT_EQ(result.entries.front().frame_id, "world");
}

TEST(ExplicitReleasePoseUtilsTest, MalformedPoseDoesNotCrash)
{
  const auto path = write_temp_payload(R"JSON({
    "grasp_task": {
      "grasp_targets": [
        {
          "object_id": "box_bad",
          "destination_pose": {
            "frame_id": "world",
            "xyz": [0.4, 0.2]
          }
        }
      ]
    }
  })JSON");

  const auto result = run_grasp_execution::load_explicit_release_pose_bridge_payload(path);
  ASSERT_TRUE(result.loaded);
  ASSERT_EQ(result.entries.size(), 1U);
  EXPECT_FALSE(result.entries.front().valid_pose);
  EXPECT_FALSE(result.warnings.empty());
}

TEST(ExplicitReleasePoseUtilsTest, MissingFrameStillParsesPoseForPolicyHandling)
{
  const auto path = write_temp_payload(R"JSON({
    "grasp_task": {
      "grasp_targets": [
        {
          "object_id": "box_no_frame",
          "destination_id": "reject_bin",
          "destination_pose": {
            "xyz": [0.1, -0.2, 0.3],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]
          }
        }
      ]
    }
  })JSON");

  const auto result = run_grasp_execution::load_explicit_release_pose_bridge_payload(path);
  ASSERT_TRUE(result.loaded);
  ASSERT_EQ(result.entries.size(), 1U);
  EXPECT_TRUE(result.entries.front().valid_pose);
  EXPECT_EQ(result.entries.front().frame_id, "");
  EXPECT_EQ(result.entries.front().destination_id, "reject_bin");
}
