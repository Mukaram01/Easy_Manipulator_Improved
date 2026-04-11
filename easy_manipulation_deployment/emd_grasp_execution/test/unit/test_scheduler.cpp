// Copyright 2021 ROS Industrial Consortium Asia Pacific
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

#include <limits>
#include <unordered_map>
#include <thread>
#include <chrono>
#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "emd/grasp_execution/core/scheduler.hpp"

namespace test_scheduler
{
bool test_unsafe_workflow(const std::string & id, std::vector<std::string> & flags)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  flags.push_back(id);
  return id.size() > 2;
}
}  // namespace test_scheduler

TEST(TestScheduler, SingleThreadedInit)
{
  grasp_execution::core::Scheduler * sch_ptr;
  try {
    sch_ptr = new grasp_execution::core::Scheduler(std::numeric_limits<size_t>::max());
    delete sch_ptr;
  } catch (const std::length_error & le) {
    std::cerr << "Length error: " << le.what() << '\n';
  }
  grasp_execution::core::Scheduler sch(1);
}

TEST(TestScheduler, SingleThreadedAddWorkflow)
{
  grasp_execution::core::Scheduler sch(1);
  using sch_status = grasp_execution::core::Workflow::Status;

  std::vector<std::string> flags;
  auto test_workflow = [&flags](const std::string & id) {
      return test_scheduler::test_unsafe_workflow(id, flags);
    };

  ASSERT_EQ(sch.add_workflow("w1", test_workflow), sch_status::ONGOING);
  EXPECT_EQ(sch.add_workflow("w1", test_workflow), sch_status::INVALID);

  auto w2_status = sch.add_workflow("w_2", test_workflow);
  EXPECT_TRUE(w2_status == sch_status::RUNNABLE || w2_status == sch_status::QUEUED);
  EXPECT_EQ(sch.add_workflow("w_2", test_workflow), sch_status::INVALID);

  bool result;
  EXPECT_EQ(sch.wait_till_complete("w1", result), sch_status::FAILED);
  EXPECT_FALSE(result);

  auto w3_status = sch.add_workflow("w_3", test_workflow);
  EXPECT_TRUE(w3_status == sch_status::RUNNABLE || w3_status == sch_status::QUEUED);
  EXPECT_EQ(sch.add_workflow("w1", test_workflow), sch_status::INVALID);
}

TEST(TestScheduler, LinearDependencyChainAtoBtoC)
{
  grasp_execution::core::Scheduler sch(2);
  using sch_status = grasp_execution::core::Workflow::Status;

  std::vector<std::string> execution_order;
  auto workflow = [&execution_order](const std::string & id) {
      execution_order.push_back(id);
      return true;
    };

  EXPECT_EQ(sch.add_workflow("A", workflow), sch_status::ONGOING);
  EXPECT_EQ(sch.add_workflow("B", workflow, {"A"}), sch_status::QUEUED);
  EXPECT_EQ(sch.add_workflow("C", workflow, {"B"}), sch_status::QUEUED);

  sch.wait_till_all_complete();

  bool result = false;
  EXPECT_EQ(sch.wait_till_complete("A", result), sch_status::COMPLETED);
  EXPECT_TRUE(result);
  EXPECT_EQ(sch.wait_till_complete("B", result), sch_status::COMPLETED);
  EXPECT_TRUE(result);
  EXPECT_EQ(sch.wait_till_complete("C", result), sch_status::COMPLETED);
  EXPECT_TRUE(result);

  ASSERT_EQ(execution_order.size(), 3U);
  EXPECT_EQ(execution_order[0], "A");
  EXPECT_EQ(execution_order[1], "B");
  EXPECT_EQ(execution_order[2], "C");
}

TEST(TestScheduler, DiamondFanInOrdering)
{
  grasp_execution::core::Scheduler sch(2);
  using sch_status = grasp_execution::core::Workflow::Status;

  std::vector<std::string> execution_order;
  auto workflow = [&execution_order](const std::string & id) {
      if (id == "A") {
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
      }
      execution_order.push_back(id);
      return true;
    };

  EXPECT_EQ(sch.add_workflow("A", workflow), sch_status::ONGOING);
  EXPECT_EQ(sch.add_workflow("B", workflow, {"A"}), sch_status::QUEUED);
  EXPECT_EQ(sch.add_workflow("C", workflow, {"A"}), sch_status::QUEUED);
  EXPECT_EQ(sch.add_workflow("D", workflow, {"B", "C"}), sch_status::QUEUED);

  sch.wait_till_all_complete();

  auto d_it = std::find(execution_order.begin(), execution_order.end(), "D");
  auto b_it = std::find(execution_order.begin(), execution_order.end(), "B");
  auto c_it = std::find(execution_order.begin(), execution_order.end(), "C");

  ASSERT_NE(d_it, execution_order.end());
  ASSERT_NE(b_it, execution_order.end());
  ASSERT_NE(c_it, execution_order.end());
  EXPECT_GT(std::distance(execution_order.begin(), d_it), std::distance(execution_order.begin(), b_it));
  EXPECT_GT(std::distance(execution_order.begin(), d_it), std::distance(execution_order.begin(), c_it));
}

TEST(TestScheduler, FailedPrerequisiteCancelsDependents)
{
  grasp_execution::core::Scheduler sch(2);
  using sch_status = grasp_execution::core::Workflow::Status;

  auto fail_workflow = [](const std::string &) { return false; };
  auto success_workflow = [](const std::string &) { return true; };

  EXPECT_EQ(sch.add_workflow("A", fail_workflow), sch_status::ONGOING);
  EXPECT_EQ(sch.add_workflow("B", success_workflow, {"A"}), sch_status::QUEUED);

  sch.wait_till_all_complete();

  bool result = true;
  EXPECT_EQ(sch.wait_till_complete("A", result), sch_status::FAILED);
  EXPECT_FALSE(result);

  EXPECT_EQ(sch.get_status("B"), sch_status::CANCELLED);
  EXPECT_EQ(sch.wait_till_complete("B", result), sch_status::CANCELLED);
  EXPECT_FALSE(result);
}

TEST(TestScheduler, CycleRejection)
{
  grasp_execution::core::Scheduler sch(1);
  using sch_status = grasp_execution::core::Workflow::Status;

  std::unordered_map<std::string, grasp_execution::core::Scheduler::WorkflowT> workflows{
    {"A", [](const std::string &) { return true; }},
    {"B", [](const std::string &) { return true; }},
    {"C", [](const std::string &) { return true; }}
  };

  std::unordered_map<std::string, std::vector<std::string>> prerequisites{
    {"A", {"C"}},
    {"B", {"A"}},
    {"C", {"B"}}
  };

  EXPECT_EQ(sch.add_workflows(workflows, prerequisites), sch_status::INVALID);
}
