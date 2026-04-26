#include <gtest/gtest.h>

#include <set>
#include <string>
#include <utility>
#include <vector>

#include "run_grasp_execution/grasp_precheck_collision_filter.hpp"

TEST(GraspPrecheckCollisionFilter, AllowsOctomapToFingertipPair)
{
  const std::vector<std::pair<std::string, std::string>> pairs = {
    {"<octomap>", "gripper_finger1_finger_tip_link"}};
  const std::set<std::string> allowed_touch_links = {
    "gripper_finger1_finger_tip_link", "gripper_finger2_finger_tip_link"};
  const std::set<std::string> allowed_collision_ids = {"<octomap>", "#box-1"};
  const auto result = run_grasp_execution::filter_grasp_precheck_collision_pairs(
    pairs, allowed_touch_links, allowed_collision_ids);

  ASSERT_EQ(result.allowed_pairs.size(), 1u);
  EXPECT_TRUE(result.invalid_pairs.empty());
}

TEST(GraspPrecheckCollisionFilter, AllowsOctomapToSecondFingertipPair)
{
  const std::vector<std::pair<std::string, std::string>> pairs = {
    {"<octomap>", "gripper_finger2_finger_tip_link"}};
  const std::set<std::string> allowed_touch_links = {
    "gripper_finger1_finger_tip_link", "gripper_finger2_finger_tip_link"};
  const std::set<std::string> allowed_collision_ids = {"<octomap>", "#box-1"};
  const auto result = run_grasp_execution::filter_grasp_precheck_collision_pairs(
    pairs, allowed_touch_links, allowed_collision_ids);

  ASSERT_EQ(result.allowed_pairs.size(), 1u);
  EXPECT_TRUE(result.invalid_pairs.empty());
}

TEST(GraspPrecheckCollisionFilter, AllowsTargetToFingertipPair)
{
  const std::vector<std::pair<std::string, std::string>> pairs = {
    {"#box-123", "gripper_finger1_finger_tip_link"}};
  const std::set<std::string> allowed_touch_links = {
    "gripper_finger1_finger_tip_link", "gripper_finger2_finger_tip_link"};
  const std::set<std::string> allowed_collision_ids = {"<octomap>", "#box-123"};
  const auto result = run_grasp_execution::filter_grasp_precheck_collision_pairs(
    pairs, allowed_touch_links, allowed_collision_ids);

  ASSERT_EQ(result.allowed_pairs.size(), 1u);
  EXPECT_TRUE(result.invalid_pairs.empty());
}

TEST(GraspPrecheckCollisionFilter, RejectsForearmTablePair)
{
  const std::vector<std::pair<std::string, std::string>> pairs = {
    {"forearm_link", "table_"}};
  const std::set<std::string> allowed_touch_links = {
    "gripper_finger1_finger_tip_link", "gripper_finger2_finger_tip_link"};
  const std::set<std::string> allowed_collision_ids = {"<octomap>", "#box-123"};
  const auto result = run_grasp_execution::filter_grasp_precheck_collision_pairs(
    pairs, allowed_touch_links, allowed_collision_ids);

  EXPECT_TRUE(result.allowed_pairs.empty());
  ASSERT_EQ(result.invalid_pairs.size(), 1u);
  EXPECT_EQ(result.invalid_pairs.front(), "forearm_link<->table_");
}

TEST(GraspPrecheckCollisionFilter, MixedPairsKeepOnlyInvalidCollisions)
{
  const std::vector<std::pair<std::string, std::string>> pairs = {
    {"<octomap>", "gripper_finger1_finger_tip_link"},
    {"forearm_link", "table_"}};
  const std::set<std::string> allowed_touch_links = {
    "gripper_finger1_finger_tip_link", "gripper_finger2_finger_tip_link"};
  const std::set<std::string> allowed_collision_ids = {"<octomap>", "#box-123"};
  const auto result = run_grasp_execution::filter_grasp_precheck_collision_pairs(
    pairs, allowed_touch_links, allowed_collision_ids);

  ASSERT_EQ(result.allowed_pairs.size(), 1u);
  ASSERT_EQ(result.invalid_pairs.size(), 1u);
  EXPECT_EQ(result.invalid_pairs.front(), "forearm_link<->table_");
}
