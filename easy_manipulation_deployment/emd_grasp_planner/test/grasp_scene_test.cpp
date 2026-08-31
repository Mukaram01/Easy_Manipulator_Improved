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

#include <gtest/gtest.h>
#include "grasp_scene_test.hpp"

#include <chrono>
#include <future>

#if EPD_ENABLED == 1
#include <epd_msgs/msg/epd_object_localization.hpp>
#include <epd_msgs/srv/perception.hpp>
#endif

GraspSceneTest::GraspSceneTest()
{
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  node = rclcpp::Node::make_shared("grasp_scene_test", "", node_options);
}

namespace
{
class TestableDirectGraspScene : public grasp_planner::GraspScene<sensor_msgs::msg::PointCloud2>
{
public:
  explicit TestableDirectGraspScene(const rclcpp::Node::SharedPtr & node)
  : grasp_planner::GraspScene<sensor_msgs::msg::PointCloud2>(node)
  {
  }

  void set_gate_enabled(const bool enabled)
  {
    execution_gate_enabled = enabled;
  }

  void set_client_for_test(
    const rclcpp::Client<emd_msgs::srv::GraspRequest>::SharedPtr & client)
  {
    output_client = client;
  }

  void invoke_send_to_execution(const emd_msgs::msg::GraspTask & task)
  {
    send_to_execution(task);
  }

  std::atomic<int> send_count{0};
  bool service_available{true};
  std::shared_ptr<emd_msgs::srv::GraspRequest::Request> last_request;
  std::shared_ptr<std::promise<rclcpp::Client<emd_msgs::srv::GraspRequest>::SharedResponse>>
  delayed_promise;

protected:
  bool wait_for_output_service(const std::chrono::duration<double> &) override
  {
    return service_available;
  }

  std::shared_future<rclcpp::Client<emd_msgs::srv::GraspRequest>::SharedResponse>
  send_output_request(
    const std::shared_ptr<emd_msgs::srv::GraspRequest::Request> & request) override
  {
    ++send_count;
    last_request = request;
    if (delayed_promise) {
      return delayed_promise->get_future().share();
    }
    auto promise = std::make_shared<std::promise<
      rclcpp::Client<emd_msgs::srv::GraspRequest>::SharedResponse>>();
    auto response = std::make_shared<emd_msgs::srv::GraspRequest::Response>();
    response->success = true;
    response->message = "ok";
    promise->set_value(response);
    return promise->get_future().share();
  }
};
}  // namespace

TEST_F(GraspSceneTest, ConstructDirectGraspScene)
{
  grasp_planner::GraspScene<sensor_msgs::msg::PointCloud2> test_direct(node);
  SUCCEED();
}

TEST_F(GraspSceneTest, ExecutionGatePreventsRequestSpamWhileBusy)
{
  auto fake_client = node->create_client<emd_msgs::srv::GraspRequest>("grasp_requests");
  TestableDirectGraspScene scene(node);
  scene.set_client_for_test(fake_client);
  scene.set_gate_enabled(true);
  scene.delayed_promise = std::make_shared<std::promise<
    rclcpp::Client<emd_msgs::srv::GraspRequest>::SharedResponse>>();

  emd_msgs::msg::GraspTask task;
  task.task_id = "test-task";
  emd_msgs::msg::GraspTarget target;
  target.target_type = "mouse";
  task.grasp_targets.push_back(target);

  scene.invoke_send_to_execution(task);
  scene.invoke_send_to_execution(task);
  EXPECT_EQ(scene.send_count.load(), 1);

  auto response = std::make_shared<emd_msgs::srv::GraspRequest::Response>();
  response->success = true;
  response->message = "done";
  scene.delayed_promise->set_value(response);

  scene.invoke_send_to_execution(task);
  EXPECT_EQ(scene.send_count.load(), 2);
}

#if EPD_ENABLED == 1
namespace
{
class TestableEpdGraspScene : public grasp_planner::GraspScene<epd_msgs::msg::EPDObjectLocalization>
{
public:
  explicit TestableEpdGraspScene(const rclcpp::Node::SharedPtr & node)
  : grasp_planner::GraspScene<epd_msgs::msg::EPDObjectLocalization>(node)
  {
  }

  void evaluate_watchdog(const rclcpp::Time & now, const double timeout_s)
  {
    this->evaluate_epd_watchdog(now, timeout_s);
  }

  void set_last_epd_msg_time(const rclcpp::Time & time)
  {
    this->last_epd_msg_time = time;
  }

  rclcpp::Time get_last_epd_msg_time() const
  {
    return this->last_epd_msg_time;
  }

  void set_next_epd_trigger_time(const rclcpp::Time & time)
  {
    this->next_epd_trigger_time = time;
  }

  rclcpp::Time get_next_epd_trigger_time() const
  {
    return this->next_epd_trigger_time;
  }

  void call_trigger_epd_pipeline()
  {
    this->trigger_epd_pipeline();
  }

  void set_execution_busy(const bool busy)
  {
    execution_in_progress.store(busy, std::memory_order_release);
  }

  void set_execution_gate(const bool enabled)
  {
    execution_gate_enabled = enabled;
  }

  void set_pause_triggers_while_busy(const bool pause)
  {
    pause_epd_triggers_while_execution_in_progress = pause;
  }

  void set_now(const rclcpp::Time & now)
  {
    now_ = now;
  }

  int trigger_count{0};
  bool service_available{true};
  std::shared_ptr<epd_msgs::srv::Perception::Request> last_request;
  std::shared_ptr<std::promise<rclcpp::Client<epd_msgs::srv::Perception>::SharedResponse>>
  delayed_promise;

protected:
  rclcpp::Time get_current_time() const override
  {
    return now_;
  }

  bool wait_for_epd_service(const std::chrono::duration<double> &) override
  {
    return service_available;
  }

  std::shared_future<rclcpp::Client<epd_msgs::srv::Perception>::SharedResponse>
  send_epd_trigger_request(
    const std::shared_ptr<epd_msgs::srv::Perception::Request> & request) override
  {
    ++trigger_count;
    last_request = request;
    if (delayed_promise && trigger_count == 1) {
      return delayed_promise->get_future().share();
    }
    auto promise = std::make_shared<std::promise<
      rclcpp::Client<epd_msgs::srv::Perception>::SharedResponse>>();
    auto response = std::make_shared<epd_msgs::srv::Perception::Response>();
    response->success = true;
    promise->set_value(response);
    return promise->get_future().share();
  }

private:
  rclcpp::Time now_{0, 0, RCL_ROS_TIME};
};
}  // namespace

TEST_F(GraspSceneTest, EpdWatchdogTriggersOncePerTimeoutAndUpdatesTimestamp)
{
  node->declare_parameter("easy_perception_deployment.epd_service_wait_timeout_s", 0.0);
  TestableEpdGraspScene scene(node);

  const rclcpp::Time start(1000000000LL);
  scene.set_last_epd_msg_time(start);
  scene.set_next_epd_trigger_time(start);

  scene.evaluate_watchdog(start + rclcpp::Duration::from_seconds(1.2), 1.0);
  EXPECT_EQ(scene.trigger_count, 1);
  EXPECT_NE(scene.last_request, nullptr);
  EXPECT_DOUBLE_EQ(scene.get_last_epd_msg_time().seconds(), 2.2);
  EXPECT_DOUBLE_EQ(scene.get_next_epd_trigger_time().seconds(), 3.2);

  scene.evaluate_watchdog(start + rclcpp::Duration::from_seconds(2.5), 1.0);
  EXPECT_EQ(scene.trigger_count, 1);

  scene.evaluate_watchdog(start + rclcpp::Duration::from_seconds(3.3), 1.0);
  EXPECT_EQ(scene.trigger_count, 2);
  EXPECT_DOUBLE_EQ(scene.get_last_epd_msg_time().seconds(), 4.3);
}

TEST_F(GraspSceneTest, TriggerEpdPipelineReturnsWhenServiceUnavailable)
{
  node->declare_parameter("easy_perception_deployment.epd_service_wait_timeout_s", 0.0);
  TestableEpdGraspScene scene(node);
  scene.service_available = false;

  scene.call_trigger_epd_pipeline();

  EXPECT_EQ(scene.trigger_count, 0);
}

TEST_F(GraspSceneTest, BusyExecutionSuppressesOnlyConfiguredEpdTriggers)
{
  node->declare_parameter("easy_perception_deployment.epd_service_wait_timeout_s", 0.0);
  node->declare_parameter("easy_perception_deployment.epd_msg_timeout_s", 1.0);
  TestableEpdGraspScene scene(node);
  scene.set_execution_gate(true);
  scene.set_pause_triggers_while_busy(true);
  scene.set_execution_busy(true);

  scene.call_trigger_epd_pipeline();
  EXPECT_EQ(scene.trigger_count, 0);

  scene.set_pause_triggers_while_busy(false);
  scene.call_trigger_epd_pipeline();
  EXPECT_EQ(scene.trigger_count, 1);
}

TEST_F(GraspSceneTest, StaleInflightRequestIsAbandonedAfterEpdRestartWindow)
{
  node->declare_parameter("easy_perception_deployment.epd_service_wait_timeout_s", 0.0);
  node->declare_parameter("easy_perception_deployment.epd_msg_timeout_s", 1.0);
  TestableEpdGraspScene scene(node);
  scene.delayed_promise = std::make_shared<std::promise<
    rclcpp::Client<epd_msgs::srv::Perception>::SharedResponse>>();

  scene.set_now(rclcpp::Time(1000000000LL));
  scene.call_trigger_epd_pipeline();
  ASSERT_EQ(scene.trigger_count, 1);

  scene.set_now(rclcpp::Time(1500000000LL));
  scene.call_trigger_epd_pipeline();
  EXPECT_EQ(scene.trigger_count, 1);

  scene.set_now(rclcpp::Time(2500000000LL));
  scene.call_trigger_epd_pipeline();
  EXPECT_EQ(scene.trigger_count, 2);
}
#endif
