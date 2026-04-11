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

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <future>
#include <chrono>
#include <atomic>
#include <mutex>
#include <exception>

#include "emd/dynamic_safety/replanner.hpp"
#include "emd/dynamic_safety/replanner_flatten_utils.hpp"
#include "emd/interpolate.hpp"
#ifdef EMD_DYNAMIC_SAFETY_MOVEIT
#include "emd/dynamic_safety/replanner_moveit.hpp"
#endif
#ifdef EMD_DYNAMIC_SAFETY_TESSERACT
#include "emd/dynamic_safety/replanner_tesseract.hpp"
#endif

namespace dynamic_safety
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("dynamic_safety.replanner");

class Replanner::Impl
{
public:
  Impl() = default;
  virtual ~Impl() = default;

  enum class LifecycleState : uint8_t
  {
    IDLE = ReplannerStatus::IDLE,
    RUNNING = ReplannerStatus::RUNNING,
    SUCCEEDED = ReplannerStatus::SUCCEED,
    FAILED = ReplannerStatus::FAILED,
    TIMED_OUT = ReplannerStatus::TIMEOUT,
    TERMINATING = ReplannerStatus::TERMINATING
  };

  void configure(
    const ReplannerOption & option,
    const rclcpp::Node::SharedPtr & node,
    const std::string & robot_urdf,
    const std::string & robot_srdf)
  {
    if (option.framework == "moveit") {
#ifdef EMD_DYNAMIC_SAFETY_MOVEIT
      context_ = std::make_unique<dynamic_safety_moveit::MoveitReplannerContext>(
        robot_urdf, robot_srdf, option, node);
#endif
    } else if (option.framework == "tesseract") {
#ifdef EMD_DYNAMIC_SAFETY_TESSERACT
      context_ = std::make_unique<dynamic_safety_tesseract::TesseractReplannerContext>(
        robot_urdf, robot_srdf, option, node);
#endif
    }
    deadline_ = option.deadline;
  }

  void run_async(
    const std::vector<std::string> & joint_names,
    const trajectory_msgs::msg::JointTrajectoryPoint & start_point,
    const trajectory_msgs::msg::JointTrajectoryPoint & end_point)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    cleanup_terminate_future_locked();
    termination_requested_for_episode_ = false;
    future_owned_by_terminator_ = false;
    lifecycle_state_.store(LifecycleState::RUNNING);
    start_time_ = std::chrono::steady_clock::now();
    plan_future_ = std::async(
      std::launch::async, &Impl::_run, this,
      joint_names, start_point, end_point);
  }

  void terminate_async()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    cleanup_terminate_future_locked();
    const LifecycleState state = lifecycle_state_.load();
    if ((state != LifecycleState::TIMED_OUT && state != LifecycleState::FAILED &&
      state != LifecycleState::SUCCEEDED && state != LifecycleState::RUNNING) ||
      termination_requested_for_episode_)
    {
      return;
    }
    termination_requested_for_episode_ = true;
    lifecycle_state_.store(LifecycleState::TERMINATING);
    _start_async_termination_thread_locked();
  }

  void add_trajectory(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr & rt)
  {
    // Deep copy;
    reference_trajectory_ = *rt;
  }

  void run_async(
    double start_state_time, double end_state_time)
  {
    trajectory_msgs::msg::JointTrajectoryPoint start_state, end_state;
    // Always store the requested start state time to ensure deterministic
    // behaviour regardless of the end state option.  Previously the start
    // state time remained uninitialized when `end_state_time` was negative,
    // which could result in undefined behaviour during interpolation.
    start_state_time_ = start_state_time;
    if (end_state_time < 0) {
      end_state = reference_trajectory_.points.back();
      end_state_time_ = rclcpp::Duration(end_state.time_from_start).seconds();
    } else {
      if (start_state_time >= end_state_time) {
        return;
      }
      end_state_time_ = end_state_time;
    }

    // Acquire start_state
    size_t num_points = reference_trajectory_.points.size();
    size_t before, after;
    size_t i = 0;
    for (; i < num_points; i++) {
      if (rclcpp::Duration(reference_trajectory_.points[i].time_from_start).seconds() >=
        start_state_time_)
      {
        before = std::max<size_t>((i == 0) ? 0 : (i - 1), 0);  // Avoid unsigned int 0 minus 1
        after = std::min<size_t>(i, num_points - 1);
        emd::core::interpolate_between_points(
          reference_trajectory_.points[before].time_from_start,
          reference_trajectory_.points[before],
          reference_trajectory_.points[after].time_from_start,
          reference_trajectory_.points[after],
          rclcpp::Duration::from_seconds(start_state_time_), start_state);
        RCLCPP_ERROR(LOGGER, "start_state:");
        for (auto & position : start_state.positions) {
          RCLCPP_ERROR(LOGGER, "start_state: %f", position);
        }
        break;
      }
    }
    // Acquire end_state
    if (end_state_time_ > 0) {
      for (; i < num_points; i++) {
        if (rclcpp::Duration(reference_trajectory_.points[i].time_from_start).seconds() >=
          end_state_time_)
        {
          before = std::max<size_t>((i == 0) ? 0 : (i - 1), 0);  // Avoid unsigned int 0 minus 1
          after = std::min<size_t>(i, num_points - 1);
          emd::core::interpolate_between_points(
            reference_trajectory_.points[before].time_from_start,
            reference_trajectory_.points[before],
            reference_trajectory_.points[after].time_from_start,
            reference_trajectory_.points[after],
            rclcpp::Duration::from_seconds(end_state_time_), end_state);
          break;
        }
      }
      if (i == num_points) {
        end_state = reference_trajectory_.points.back();
      }
    }
    run_async(reference_trajectory_.joint_names, start_state, end_state);
  }

  trajectory_msgs::msg::JointTrajectory::SharedPtr flatten_result(
    double current_time,
    const std::vector<std::string> & joint_names,
    const trajectory_msgs::msg::JointTrajectoryPoint & current_state)
  {
    if (!have_same_joint_names(plan_.joint_names, joint_names)) {
      RCLCPP_ERROR(LOGGER, "flatten_result joint name mismatch against plan joint names");
      return std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    }

    size_t i = 0;
    size_t num_points = reference_trajectory_.points.size();
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> start_segment;
    std::vector<std::string> sorted_joint_names = joint_names;
    trajectory_msgs::msg::JointTrajectoryPoint sorted_current_states = current_state;
    if (plan_.joint_names != joint_names &&
      !reorder_joint(plan_.joint_names, sorted_joint_names, sorted_current_states))
    {
      RCLCPP_ERROR(LOGGER, "Failed to reorder current state to plan joint order");
      return std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    }

    if (current_time < start_state_time_) {
      for (; i < num_points; i++) {
        double time_from_start =
          rclcpp::Duration(reference_trajectory_.points[i].time_from_start).seconds();
        if (current_time <= time_from_start && start_state_time_ > time_from_start) {
          if (reference_trajectory_.joint_names != plan_.joint_names) {
            std::vector<std::string> sorted_ref_joint_names = reference_trajectory_.joint_names;
            trajectory_msgs::msg::JointTrajectoryPoint sorted_point =
              reference_trajectory_.points[i];
            if (!reorder_joint(plan_.joint_names, sorted_ref_joint_names, sorted_point)) {
              RCLCPP_ERROR(LOGGER, "Failed to reorder start segment point to plan joint order");
              return std::make_shared<trajectory_msgs::msg::JointTrajectory>();
            }
            start_segment.push_back(sorted_point);
          } else {
            start_segment.push_back(reference_trajectory_.points[i]);
          }
          start_segment.back().time_from_start = rclcpp::Duration::from_seconds(0.0);
        } else if (time_from_start >= start_state_time_) {
          break;
        }
      }
    }

    sorted_current_states.time_from_start = rclcpp::Duration::from_seconds(0.0);
    start_segment.push_back(sorted_current_states);
    plan_.points.insert(plan_.points.begin(), start_segment.begin(), start_segment.end());

    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> end_segment;
    for (; i < num_points; i++) {
      double time_from_start =
        rclcpp::Duration(reference_trajectory_.points[i].time_from_start).seconds();
      if (time_from_start > end_state_time_) {
        if (reference_trajectory_.joint_names != plan_.joint_names) {
          std::vector<std::string> sorted_ref_joint_names = reference_trajectory_.joint_names;
          trajectory_msgs::msg::JointTrajectoryPoint sorted_point = reference_trajectory_.points[i];
          if (!reorder_joint(plan_.joint_names, sorted_ref_joint_names, sorted_point)) {
            RCLCPP_ERROR(LOGGER, "Failed to reorder end segment point to plan joint order");
            return std::make_shared<trajectory_msgs::msg::JointTrajectory>();
          }
          end_segment.push_back(sorted_point);
        } else {
          end_segment.push_back(reference_trajectory_.points[i]);
        }
        end_segment.back().time_from_start = rclcpp::Duration::from_seconds(0.0);
      }
    }
    plan_.points.insert(plan_.points.end(), end_segment.begin(), end_segment.end());

    if (plan_.points.empty()) {
      RCLCPP_ERROR(LOGGER, "Flattened plan contains no points before time parameterization");
      return std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    }

    set_start_kinematics(plan_.points.front(), sorted_current_states);

    if (!start_segment.empty() && plan_.points.size() > start_segment.size()) {
      const size_t first_replan_idx = start_segment.size();
      if (plan_.points[first_replan_idx].velocities.size() == plan_.points.front().positions.size()) {
        plan_.points.front().velocities = plan_.points[first_replan_idx].velocities;
      }
      if (
        plan_.points[first_replan_idx].accelerations.size() ==
        plan_.points.front().positions.size())
      {
        plan_.points.front().accelerations = plan_.points[first_replan_idx].accelerations;
      }
    }

    if (!end_segment.empty()) {
      const size_t first_end_idx = plan_.points.size() - end_segment.size();
      if (first_end_idx > 0) {
        const size_t last_replan_idx = first_end_idx - 1;
        if (plan_.points[last_replan_idx].velocities.size() == plan_.points[first_end_idx].positions.size()) {
          plan_.points[first_end_idx].velocities = plan_.points[last_replan_idx].velocities;
        }
        if (
          plan_.points[last_replan_idx].accelerations.size() ==
          plan_.points[first_end_idx].positions.size())
        {
          plan_.points[first_end_idx].accelerations = plan_.points[last_replan_idx].accelerations;
        }
      }
    }

    if (context_->time_parameterize(plan_)) {
      if (!has_monotonic_timestamps(plan_.points)) {
        RCLCPP_ERROR(LOGGER, "Flattened plan has non-monotonic timestamps after insertion");
        return std::make_shared<trajectory_msgs::msg::JointTrajectory>();
      }
      return std::make_shared<trajectory_msgs::msg::JointTrajectory>(plan_);
    }

    return std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  }


  uint8_t get_status() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (lifecycle_state_.load() == LifecycleState::TERMINATING) {
      return ReplannerStatus::TERMINATING;
    }
    if (plan_future_.valid()) {
      auto status = plan_future_.wait_for(std::chrono::nanoseconds(0));
      double time_passed = std::chrono::duration<double, std::ratio<1>>(
        std::chrono::steady_clock::now() - start_time_).count();
      switch (status) {
        case std::future_status::deferred:
          lifecycle_state_.store(LifecycleState::IDLE);
          return ReplannerStatus::IDLE;
        case std::future_status::ready:
          if (lifecycle_state_.load() == LifecycleState::RUNNING) {
            lifecycle_state_.store(LifecycleState::SUCCEEDED);
          }
          return ReplannerStatus::SUCCEED;
        case std::future_status::timeout:
          if (time_passed > deadline_) {
            lifecycle_state_.store(LifecycleState::TIMED_OUT);
            return ReplannerStatus::TIMEOUT;
          }
          lifecycle_state_.store(LifecycleState::RUNNING);
          return ReplannerStatus::RUNNING;
        default:
          lifecycle_state_.store(LifecycleState::IDLE);
          return ReplannerStatus::IDLE;
      }
    } else {
      lifecycle_state_.store(LifecycleState::IDLE);
      return ReplannerStatus::IDLE;
    }
  }

  trajectory_msgs::msg::JointTrajectory::SharedPtr get_result()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!plan_future_.valid() || future_owned_by_terminator_) {
      return std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    }
    lifecycle_state_.store(LifecycleState::TERMINATING);
    try {
      plan_ = plan_future_.get();
      lifecycle_state_.store(LifecycleState::SUCCEEDED);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(LOGGER, "Replanner future failed: %s", e.what());
      plan_ = trajectory_msgs::msg::JointTrajectory();
      lifecycle_state_.store(LifecycleState::FAILED);
    } catch (...) {
      RCLCPP_ERROR(LOGGER, "Replanner future failed with unknown exception");
      plan_ = trajectory_msgs::msg::JointTrajectory();
      lifecycle_state_.store(LifecycleState::FAILED);
    }
    plan_future_ = std::shared_future<trajectory_msgs::msg::JointTrajectory>();
    future_owned_by_terminator_ = false;
    termination_requested_for_episode_ = false;
    lifecycle_state_.store(LifecycleState::IDLE);
    RCLCPP_INFO(
      LOGGER,
      "Total time take: %.5fs",
      std::chrono::duration<double, std::ratio<1>>(
        std::chrono::steady_clock::now() - start_time_).count());
    return std::make_shared<trajectory_msgs::msg::JointTrajectory>(plan_);
  }

  void update(
    const sensor_msgs::msg::JointState & joint_states)
  {
    context_->update(joint_states);
  }


  void update(
    const moveit_msgs::msg::PlanningScene & scene_msg)
  {
    context_->update(scene_msg);
  }

private:
  trajectory_msgs::msg::JointTrajectory _run(
    const std::vector<std::string> joint_names,
    const trajectory_msgs::msg::JointTrajectoryPoint start_point,
    const trajectory_msgs::msg::JointTrajectoryPoint end_point)
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    context_->run(joint_names, start_point, end_point, trajectory);
    return trajectory;
  }

  // Calling blocking get in a new async until it ended
  // This will result the future to invalid
  void _start_async_termination_thread_locked()
  {
    if (!plan_future_.valid()) {
      lifecycle_state_.store(LifecycleState::IDLE);
      termination_requested_for_episode_ = false;
      return;
    }
    future_owned_by_terminator_ = true;
    auto plan_future = plan_future_;
    plan_future_ = std::shared_future<trajectory_msgs::msg::JointTrajectory>();
    terminate_future_ = std::async(
      [this, plan_future]() mutable -> void {
        try {
          plan_future.get();
        } catch (...) {
          // best effort termination
        }
        RCLCPP_ERROR(
          LOGGER,
          "Unfortunately, Total time take: %.5fs",
          std::chrono::duration<double, std::ratio<1>>(
            std::chrono::steady_clock::now() - start_time_).count());
        std::lock_guard<std::mutex> lock(state_mutex_);
        future_owned_by_terminator_ = false;
        termination_requested_for_episode_ = false;
        lifecycle_state_.store(LifecycleState::IDLE);
      });
  }
  void cleanup_terminate_future_locked()
  {
    if (!terminate_future_.valid()) {
      return;
    }
    auto status = terminate_future_.wait_for(std::chrono::nanoseconds(0));
    if (status == std::future_status::ready) {
      terminate_future_.get();
      terminate_future_ = std::future<void>();
    }
  }

  std::unique_ptr<ReplannerContext> context_;
  std::shared_future<trajectory_msgs::msg::JointTrajectory> plan_future_;
  std::future<void> terminate_future_;
  mutable std::mutex state_mutex_;
  mutable std::atomic<LifecycleState> lifecycle_state_{LifecycleState::IDLE};
  bool termination_requested_for_episode_{false};
  bool future_owned_by_terminator_{false};

  // Specialized planning feature
  trajectory_msgs::msg::JointTrajectory reference_trajectory_;
  double start_state_time_, end_state_time_;
  trajectory_msgs::msg::JointTrajectory plan_;

  double deadline_;
  std::chrono::steady_clock::time_point start_time_;
};

Replanner::Replanner()
: impl_ptr_(std::make_unique<Impl>())
{
}

Replanner::~Replanner()
{
}

void Replanner::configure(
  const ReplannerOption & option,
  const rclcpp::Node::SharedPtr & node,
  const std::string & robot_urdf,
  const std::string & robot_srdf)
{
  impl_ptr_->configure(option, node, robot_urdf, robot_srdf);
}

void Replanner::run_async(
  const std::vector<std::string> & joint_names,
  const trajectory_msgs::msg::JointTrajectoryPoint & start_point,
  const trajectory_msgs::msg::JointTrajectoryPoint & end_point)
{
  impl_ptr_->run_async(joint_names, start_point, end_point);
}

void Replanner::add_trajectory(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr & rt)
{
  impl_ptr_->add_trajectory(rt);
}

void Replanner::run_async(
  double start_state_time,
  double end_state_time)
{
  impl_ptr_->run_async(start_state_time, end_state_time);
}

void Replanner::terminate_async()
{
  impl_ptr_->terminate_async();
}

trajectory_msgs::msg::JointTrajectory::SharedPtr Replanner::flatten_result(
  double current_time,
  const std::vector<std::string> & joint_names,
  const trajectory_msgs::msg::JointTrajectoryPoint & current_state)
{
  return impl_ptr_->flatten_result(current_time, joint_names, current_state);
}

void Replanner::update(
  const sensor_msgs::msg::JointState & msg)
{
  impl_ptr_->update(msg);
}

void Replanner::update(
  const moveit_msgs::msg::PlanningScene & msg)
{
  impl_ptr_->update(msg);
}

uint8_t Replanner::get_status() const
{
  return impl_ptr_->get_status();
}

trajectory_msgs::msg::JointTrajectory::SharedPtr Replanner::get_result()
{
  return impl_ptr_->get_result();
}
}  // namespace dynamic_safety
