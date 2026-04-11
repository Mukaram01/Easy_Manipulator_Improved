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
#include <chrono>
#include <fstream>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>
#include <stdexcept>
#include <sstream>
#include <limits>
#include <cstdint>

#include "emd/dynamic_safety/dynamic_safety.hpp"
#include <rclcpp/parameter_client.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

namespace dynamic_safety
{

static const rclcpp::Logger & LOGGER = rclcpp::get_logger("dynamic_safety");
static const double LOG_RATE = 1;  // This is duration

template<typename NodeT>
const Option & Option::load(const std::shared_ptr<NodeT> & node)
{
  // Load dynamic safety parameters
  emd::declare_or_get_param<bool>(
    dynamic_parameterization,
    "dynamic_safety.dynamic_parameterization",
    node, LOGGER, true);

  emd::declare_or_get_param<bool>(
    use_description_server,
    "dynamic_safety.use_description_server",
    node, LOGGER, true);

  if (use_description_server) {
    emd::declare_or_get_param<std::string>(
      description_server,
      "dynamic_safety.description_server",
      node, LOGGER);

    using namespace std::chrono_literals;
    auto description_loader_node = std::make_shared<rclcpp::Node>(
      std::string(
        node->get_name()) + "_description_loader");
    auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
      description_loader_node, description_server);
    while (!parameters_client->wait_for_service()) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          LOGGER, "Interrupted while waiting for %s service. Exiting.",
          description_server.c_str());
        throw std::runtime_error(
                "Failed to connect to description service " + description_server);
      }
      RCLCPP_WARN(
        LOGGER, "%s service not available, waiting again...",
        description_server.c_str());
    }
    RCLCPP_INFO(LOGGER, "Connected to description server %s!!", description_server.c_str());
    // search and wait for robot_description on param server
    while (robot_description.empty()) {
      try {
        RCLCPP_INFO(LOGGER, "Get parameters");
        auto f = parameters_client->get_parameters(
          {"robot_description",
            "robot_description_semantic"});
        RCLCPP_INFO(LOGGER, "Wait for parameters");
        rclcpp::spin_until_future_complete(description_loader_node, f);
        RCLCPP_INFO(LOGGER, "parameter received");
        std::vector<rclcpp::Parameter> values = f.get();
        RCLCPP_INFO(LOGGER, "parameter gotten");
        robot_description = values[0].as_string();
        robot_description_semantic = values[1].as_string();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(LOGGER, "%s", e.what());
      }

      if (!robot_description.empty()) {
        break;
      } else {
        RCLCPP_ERROR(
          LOGGER, "dynamic_safety is waiting for model"
          " URDF in parameter [robot_description] on the ROS param server.");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(
      LOGGER, "Received urdf & srdf from param server, parsing...");
  } else {
    emd::declare_or_get_param<std::string>(
      robot_description,
      "dynamic_safety.robot_description",
      node, LOGGER);  // default: "second"
    emd::declare_or_get_param<std::string>(
      robot_description_semantic,
      "dynamic_safety.robot_description_semantic",
      node, LOGGER);  // default: "second"
  }

  // Load dynamic safety parameters
  emd::declare_or_get_param<bool>(
    allow_replan,
    "dynamic_safety.allow_replan",
    node, LOGGER, false);

  // Load dynamic safety parameters
  emd::declare_or_get_param<bool>(
    benchmark,
    "dynamic_safety.benchmark",
    node, LOGGER, false);

  // Load dynamic safety parameters
  emd::declare_or_get_param<bool>(
    visualize,
    "dynamic_safety.visualize",
    node, LOGGER, false);

  // -------------- Detailed parameters that needs to set manually --------------

  emd::declare_or_get_param<double>(
    safety_zone_options.look_ahead_time,
    "dynamic_safety.look_ahead_time",
    node, LOGGER);

  emd::declare_or_get_param<std::string>(
    environment_joint_states_topic,
    "dynamic_safety.environment_joint_states_topic",
    node, LOGGER);

  emd::declare_or_get_param<std::string>(
    moveit_scene_topic,
    "dynamic_safety.moveit_scene_topic",
    node, LOGGER);
  // Load collision checker parameters
  emd::declare_or_get_param<std::string>(
    collision_checker_options.framework,
    "dynamic_safety.collision_checker.framework",
    node, LOGGER, collision_checker_options.framework);  // default: moveit

  emd::declare_or_get_param<std::string>(
    collision_checker_options.collision_checking_plugin,
    "dynamic_safety.collision_checker.collision_checking_plugin",
    node, LOGGER, collision_checker_options.collision_checking_plugin);  // default: fcl


  // Load collision checker parameters
  emd::declare_or_get_param<bool>(
    collision_checker_options.distance,
    "dynamic_safety.collision_checker.distance",
    node, LOGGER, false);  // default: false

  emd::declare_or_get_param<bool>(
    collision_checker_options.continuous,
    "dynamic_safety.collision_checker.continuous",
    node, LOGGER, false);  // default: false

  emd::declare_or_get_param<double>(
    collision_checker_options.step,
    "dynamic_safety.collision_checker.step",
    node, LOGGER);

  emd::declare_or_get_param<std::string>(
    collision_checker_options.group,
    "dynamic_safety.collision_checker.group",
    node, LOGGER);  // needed for continuous collision checking

  emd::declare_or_get_param<double>(
    collision_checker_options.padding,
    "dynamic_safety.collision_checker.padding",
    node, LOGGER, 0.0);

  // -------------- Load overwritable parameters -------------------
  // If the following parameters are defined and greater than zero,
  // dynamic parameterization will not change these parameters
  emd::declare_or_get_param<double>(
    rate,
    "dynamic_safety.rate",
    node, LOGGER, 0);

  emd::declare_or_get_param<double>(
    safety_zone_options.slow_down_time,
    "dynamic_safety.slow_down_time",
    node, LOGGER, 0);

  // Unable to overwrite currently, it is detected by default
  // emd::declare_or_get_param<bool>(
  //   collision_checker_options.realtime,
  //   "dynamic_safety.collision_checker.realtime",
  //   node, LOGGER, true);  // default: true

  emd::declare_or_get_param<int>(
    collision_checker_options.thread_count,
    "dynamic_safety.collision_checker.thread_count",
    node, LOGGER);

  emd::declare_or_get_param<double>(
    zone_policy.min_replan_interval,
    "dynamic_safety.policy.min_replan_interval",
    node, LOGGER, zone_policy.min_replan_interval);
  emd::declare_or_get_param<double>(
    zone_policy.emergency_hysteresis,
    "dynamic_safety.policy.hysteresis.emergency",
    node, LOGGER, zone_policy.emergency_hysteresis);
  emd::declare_or_get_param<double>(
    zone_policy.slowdown_hysteresis,
    "dynamic_safety.policy.hysteresis.slowdown",
    node, LOGGER, zone_policy.slowdown_hysteresis);
  emd::declare_or_get_param<double>(
    zone_policy.replan_hysteresis,
    "dynamic_safety.policy.hysteresis.replan",
    node, LOGGER, zone_policy.replan_hysteresis);
  emd::declare_or_get_param<double>(
    zone_policy.safe_hysteresis,
    "dynamic_safety.policy.hysteresis.safe",
    node, LOGGER, zone_policy.safe_hysteresis);
  emd::declare_or_get_param<double>(
    zone_policy.scale_floor,
    "dynamic_safety.policy.scale.floor",
    node, LOGGER, zone_policy.scale_floor);
  emd::declare_or_get_param<double>(
    zone_policy.scale_ceiling,
    "dynamic_safety.policy.scale.ceiling",
    node, LOGGER, zone_policy.scale_ceiling);
  emd::declare_or_get_param<double>(
    zone_policy.scale_ramp_down_rate,
    "dynamic_safety.policy.scale.ramp_down_rate",
    node, LOGGER, zone_policy.scale_ramp_down_rate);
  emd::declare_or_get_param<double>(
    zone_policy.scale_ramp_up_rate,
    "dynamic_safety.policy.scale.ramp_up_rate",
    node, LOGGER, zone_policy.scale_ramp_up_rate);
  emd::declare_or_get_param<double>(
    zone_policy.collision_persistence_window,
    "dynamic_safety.policy.collision_persistence_window",
    node, LOGGER, zone_policy.collision_persistence_window);
  emd::declare_or_get_param<double>(
    zone_policy.replan_start_time_epsilon,
    "dynamic_safety.policy.replan_start_time_epsilon",
    node, LOGGER, zone_policy.replan_start_time_epsilon);

  // -------------- Static parameters -------------------
  if (!dynamic_parameterization) {
    // emd::declare_or_get_param<std::string>(
    //   safety_zone_options.unit_type,
    //   "dynamic_safety.safety_zone.unit_type",
    //   node, LOGGER, "second");  // default: "second"

    // // Only second is enabled
    // if (safety_zone_options.unit_type != "second") {
    //   RCLCPP_WARN(
    //     LOGGER, "Wrong safety zone unit type: [%s], default to [second]",
    //     safety_zone_options.unit_type.c_str());
    //   safety_zone_options.unit_type = "second";
    // }
    // TODO(anyone): distance based collision checking
    // I don't think distance makes sense here.

  } else {
    // Dynamically detect realtime OS
    std::ifstream realtime_file("/sys/kernel/realtime", std::ios::in);
    collision_checker_options.realtime = false;
    if (realtime_file.is_open()) {
      realtime_file >> collision_checker_options.realtime;
    }

    // Set thread count to max capability
    if (collision_checker_options.thread_count == 0) {
      collision_checker_options.thread_count =
        static_cast<int>(std::thread::hardware_concurrency()) / 2;
    }

    // Joint limit parameters needed for dynamic parameterization
    // of slow down time
    if (safety_zone_options.slow_down_time <= 0) {
      emd::declare_or_get_param<std::string>(
        joint_limits_parameter_server,
        "dynamic_safety.joint_limits_parameter_server",
        node, LOGGER);
      emd::declare_or_get_param<std::string>(
        joint_limits_parameter_namespace,
        "dynamic_safety.joint_limits_parameter_namespace",
        node, LOGGER);

      // Joint Limit loader
      // new node for parameter loading
      auto joint_limits_node = std::make_shared<rclcpp::Node>(
        std::string(
          node->get_name()) + "_joint_limits_loader");
      auto joint_limits_parameters_client =
        std::make_shared<rclcpp::AsyncParametersClient>(
        joint_limits_node, joint_limits_parameter_server);

      while (!joint_limits_parameters_client->wait_for_service()) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(
            LOGGER, "Interrupted while waiting for %s service. Exiting.",
            joint_limits_parameter_server.c_str());
          throw std::runtime_error(
                  "Failed to connect to joint limits service " +
                  joint_limits_parameter_server);
        }
        RCLCPP_WARN(
          LOGGER, "%s service not available, waiting again...",
          joint_limits_parameter_server.c_str());
      }
      RCLCPP_INFO(
        LOGGER, "Connected to joint limits server %s!!",
        joint_limits_parameter_server.c_str());
      rcl_interfaces::msg::ListParametersResult joint_limits_parameters;
      try {
        RCLCPP_INFO(LOGGER, "Get parameters");
        auto joint_limits_future = joint_limits_parameters_client->list_parameters(
          {joint_limits_parameter_namespace},
          5);
        rclcpp::spin_until_future_complete(
          joint_limits_node, joint_limits_future);
        joint_limits_parameters = joint_limits_future.get();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(LOGGER, "%s", e.what());
      }
      size_t start_idx = joint_limits_parameter_namespace.size() + 1;
      for (auto & name : joint_limits_parameters.prefixes) {
        name.erase(name.begin(), name.begin() + static_cast<int>(start_idx));
        auto jl_f = joint_limits_parameters_client->get_parameters(
          {
            joint_limits_parameter_namespace + "." + name + ".has_velocity_limits",
            joint_limits_parameter_namespace + "." + name + ".max_velocity",
            joint_limits_parameter_namespace + "." + name + ".has_acceleration_limits",
            joint_limits_parameter_namespace + "." + name + ".max_acceleration"
          });
        rclcpp::spin_until_future_complete(joint_limits_node, jl_f);
        auto jl = jl_f.get();
        try {
          if (jl[0].as_bool()) {
            joint_limits[name].first = jl[1].as_double();
          }
          if (jl[2].as_bool()) {
            joint_limits[name].second = jl[3].as_double();
          }
        } catch (const rclcpp::ParameterTypeException & e) {
          RCLCPP_ERROR(LOGGER, e.what());
          continue;
        }
      }
      for (auto & limit : joint_limits) {
        RCLCPP_ERROR(
          LOGGER, "joint: %s max_vel: %f max_accel: %f",
          limit.first.c_str(),
          limit.second.first,
          limit.second.second);
      }
    }
  }

  // Replanner parameters
  if (allow_replan) {
    emd::declare_or_get_param<std::string>(
      replanner_options.framework,
      "dynamic_safety.replanner.framework",
      node, LOGGER, replanner_options.framework);

    emd::declare_or_get_param<std::string>(
      replanner_options.planner,
      "dynamic_safety.replanner.planner",
      node, LOGGER, replanner_options.planner);

    emd::declare_or_get_param<std::string>(
      replanner_options.time_parameterization,
      "dynamic_safety.replanner.time_parameterization",
      node, LOGGER, replanner_options.time_parameterization);

    emd::declare_or_get_param<std::string>(
      replanner_options.group,
      "dynamic_safety.replanner.group",
      node, LOGGER);

    emd::declare_or_get_param<double>(
      replanner_options.deadline,
      "dynamic_safety.replanner.deadline",
      node, LOGGER);

    emd::declare_or_get_param<std::string>(
      replanner_options.joint_limits_parameter_server,
      "dynamic_safety.replanner.joint_limit_parameter_server",
      node, LOGGER, joint_limits_parameter_server);
    emd::declare_or_get_param<std::string>(
      replanner_options.joint_limits_parameter_namespace,
      "dynamic_safety.replanner.joint_limit_parameter_namespace",
      node, LOGGER, joint_limits_parameter_namespace);

    if (replanner_options.framework == "moveit") {
      if (replanner_options.planner == "ompl") {
        emd::declare_or_get_param<std::string>(
          replanner_options.ompl_planner_id,
          "dynamic_safety.replanner.ompl_planner_id",
          node, LOGGER);
      }

      emd::declare_or_get_param<std::string>(
        replanner_options.planner_parameter_server,
        "dynamic_safety.replanner.planner_parameter_server",
        node, LOGGER);
      emd::declare_or_get_param<std::string>(
        replanner_options.planner_parameter_namespace,
        "dynamic_safety.replanner.planner_parameter_namespace",
        node, LOGGER);
    }
  }

  if (visualize) {
    emd::declare_or_get_param<bool>(
      visualizer_options.publish_scene,
      "dynamic_safety.visualizer.publish_scene",
      node, LOGGER, false);

    emd::declare_or_get_param<double>(
      visualizer_options.publish_frequency,
      "dynamic_safety.visualizer.publish_frequency",
      node, LOGGER, 10);

    emd::declare_or_get_param<double>(
      visualizer_options.step,
      "dynamic_safety.visualizer.step",
      node, LOGGER, 0.1);

    emd::declare_or_get_param<std::string>(
      visualizer_options.topic,
      "dynamic_safety.visualizer.topic",
      node, LOGGER);

    emd::declare_or_get_param<std::string>(
      visualizer_options.tcp_link,
      "dynamic_safety.visualizer.tcp_link",
      node, LOGGER);
    emd::declare_or_get_param<std::string>(
      visualizer_options.scene_topic,
      "dynamic_safety.visualizer.scene_topic",
      node, LOGGER, std::string());
  }

  // Return idiom
  return *this;
}

class DynamicSafety::Impl
{
public:
  Impl();
  explicit Impl(const Option & option)
  : option_(option), activated_(false), min_distance_(-1.0)
  {
    // Reset Cache
    env_state_cache_.initRT(sensor_msgs::msg::JointState());
    moveit_scene_cache_.initRT(moveit_msgs::msg::PlanningScene());
    current_state_cache_.initRT(CurrentState());
    current_time_cache_.initRT(0);
    scale_cache_.initRT(1);
  }

  ~Impl()
  {
    stop();
  }

  struct CurrentState
  {
    CurrentState() = default;

    CurrentState(
      const std::vector<std::string> & _joint_names,
      const trajectory_msgs::msg::JointTrajectoryPoint & _state)
    {
      joint_names = _joint_names;
      state = _state;
    }
    std::vector<std::string> joint_names;
    trajectory_msgs::msg::JointTrajectoryPoint state;
  };

  void configure(
    const rclcpp::Node::SharedPtr & node);

  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & lifecycle_node);

  void add_trajectory(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr & rt);

  void update_time(double current_time);

  void update_state(const sensor_msgs::msg::JointState::SharedPtr & state);

  void update_state(
    const std::vector<std::string> & joint_names,
    const trajectory_msgs::msg::JointTrajectoryPoint & current_state);

  void update_scene(const moveit_msgs::msg::PlanningScene::SharedPtr & scene_msg);

  double get_scale();

  void start();

  void wait();

  void stop();

  std::function<void(const trajectory_msgs::msg::JointTrajectory::SharedPtr &)> NewTrajectoryCB;

protected:
  void _deadline_cb(rclcpp::QOSDeadlineRequestedInfo &);

  void _main_loop();

  double _cal_scale_time(
    const CurrentState & current_state,
    double current_scale,
    double target_scale);

  void _handle_replanner(double start_state_time);
  static const char * _zone_to_str(uint8_t zone);

  // Temporary functions to be moved into collision checker
  double _back_track_last_collision();
private:
  template <typename NodePtrT>
  void configure_common(const NodePtrT & node);
  double full_duration_;
  // Temporary map better handling needed
  std::unordered_set<std::string> joint_names;

  Option option_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr env_state_sub_;
  rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr moveit_scene_sub_;
  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::CallbackGroup::SharedPtr env_state_callback_group_;
  rclcpp::CallbackGroup::SharedPtr moveit_scene_callback_group_;
  rclcpp::CallbackGroup::SharedPtr main_callback_group_;

  CollisionChecker collision_checker_;
  SafetyZone safety_zone_;
  // NextPointPublisher next_point_publisher_;
  Replanner replanner_;
  Visualizer visualizer_;

  double collision_time_point_;
  // double replan_time_point_;

  double min_distance_;

  // uint8_t zone;

  std::atomic_bool activated_;
  std::atomic_bool started;

  std::vector<double> benchmark_stats;

  std::promise<void> sig_;
  std::future<void> future_;

  emd::TimeProfiler<> * pf_;

  // realtime_tools::RealtimeBuffer<trajectory_msgs::msg::JointTrajectoryPoint> state_cache_;
  realtime_tools::RealtimeBuffer<sensor_msgs::msg::JointState> env_state_cache_;
  realtime_tools::RealtimeBuffer<moveit_msgs::msg::PlanningScene> moveit_scene_cache_;
  realtime_tools::RealtimeBuffer<CurrentState> current_state_cache_;
  realtime_tools::RealtimeBuffer<double> current_time_cache_;
  realtime_tools::RealtimeBuffer<double> scale_cache_;
  ZoneDecisionPolicy zone_decision_policy_{option_.zone_policy};
  std::atomic<uint64_t> timeout_count_{0};
  std::atomic<uint64_t> terminate_count_{0};
  std::atomic<uint64_t> retry_count_{0};
  std::atomic<uint64_t> success_after_retry_count_{0};
  size_t max_retries_per_window_{3};
  double retry_cooldown_s_{0.5};
  size_t retries_in_window_{0};
  double last_retry_time_{-std::numeric_limits<double>::infinity()};
  bool retry_pending_{false};
  double pending_retry_start_state_time_{0.0};
  bool timeout_episode_active_{false};
  bool terminate_requested_for_timeout_episode_{false};
  bool last_attempt_was_retry_{false};
  bool conservative_fallback_active_{false};
  // realtime_tools::RealtimeBuffer<octomap::OcTree> env_state_cache_;
};

void DynamicSafety::Impl::configure(
  const rclcpp::Node::SharedPtr & node)
{
  configure_common(node);
}

void DynamicSafety::Impl::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & lifecycle_node)
{
  configure_common(lifecycle_node);
}

template <typename NodePtrT>
void DynamicSafety::Impl::configure_common(const NodePtrT & node)
{
  // Initialize flags
  started = false;
  activated_ = false;

  collision_checker_.configure(
    option_.collision_checker_options,
    option_.robot_description,
    option_.robot_description_semantic);

  if (option_.dynamic_parameterization && option_.rate == 0) {
    // Use the new polling function to estimate
    // * Running rate
    option_.rate = (2.0 * collision_checker_.polling(option_.safety_zone_options.look_ahead_time));
  }
  RCLCPP_INFO(
    LOGGER, "Dynamic safety will run at %dHz.",
    static_cast<int>(option_.rate));

  // Blind zone is the iteration period
  option_.safety_zone_options.collision_checking_deadline = 1.0 / option_.rate;

  if (option_.allow_replan) {
    auto replanner_node = std::make_shared<rclcpp::Node>(
      node->get_name() + std::string("_replanner"));
    replanner_.configure(
      option_.replanner_options,
      replanner_node,
      option_.robot_description,
      option_.robot_description_semantic);
    option_.safety_zone_options.replan_deadline = option_.replanner_options.deadline;
    NewTrajectoryCB = std::bind(&DynamicSafety::Impl::add_trajectory, this, std::placeholders::_1);
  }

  if (!safety_zone_.set(option_.safety_zone_options)) {
    throw std::runtime_error("Wrong safety zone parameters");
  } else {
    safety_zone_.print();
  }

  if (option_.visualize) {
    auto visualizer_node = std::make_shared<rclcpp::Node>(
      node->get_name() + std::string("_visualizer"));
    visualizer_.configure(
      visualizer_node, option_.visualizer_options,
      option_.safety_zone_options,
      option_.robot_description,
      option_.robot_description_semantic);
  }

  benchmark_stats.clear();
  zone_decision_policy_.reset();
  timeout_count_ = 0;
  terminate_count_ = 0;
  retry_count_ = 0;
  success_after_retry_count_ = 0;
  retries_in_window_ = 0;
  last_retry_time_ = -std::numeric_limits<double>::infinity();
  retry_pending_ = false;
  timeout_episode_active_ = false;
  terminate_requested_for_timeout_episode_ = false;
  last_attempt_was_retry_ = false;
  conservative_fallback_active_ = false;

  // Reset Cache
  env_state_cache_.initRT(sensor_msgs::msg::JointState());
  current_time_cache_.initRT(0);
  scale_cache_.initRT(1);

  // double period = 1 / option_.rate;
  env_state_callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions env_state_sub_option;
  env_state_sub_option.callback_group = env_state_callback_group_;
  auto qos = rclcpp::QoS(2);

  if (!option_.environment_joint_states_topic.empty()) {
    env_state_sub_ = node->template create_subscription<sensor_msgs::msg::JointState>(
      option_.environment_joint_states_topic,
      qos,
      [ = ](sensor_msgs::msg::JointState::UniquePtr joint_state_msg) -> void
      {
        if (started) {
          update_state(std::move(joint_state_msg));
        }
      },
      env_state_sub_option
    );
  }

  moveit_scene_callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions moveit_scene_sub_option;
  moveit_scene_sub_option.callback_group = moveit_scene_callback_group_;
  if (!option_.moveit_scene_topic.empty()) {
    moveit_scene_sub_ = node->template create_subscription<moveit_msgs::msg::PlanningScene>(
      option_.moveit_scene_topic,
      qos,
      [ = ](moveit_msgs::msg::PlanningScene::UniquePtr scene_msg) -> void
      {
        if (started) {
          update_scene(std::move(scene_msg));
        }
      },
      moveit_scene_sub_option
    );
  }

  main_callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  main_timer_ = node->create_wall_timer(
    rclcpp::Duration::from_seconds(1.0 / option_.rate).to_chrono<std::chrono::nanoseconds>(),
    [ = ]() -> void {
      if (started) {
        if (option_.benchmark) {
          if (pf_) {
            pf_->reset();
          }
        }
        _main_loop();
        if (option_.benchmark) {
          if (pf_) {
            pf_->lapse_and_record();
          }
        }
      }
    },
    main_callback_group_);
}

void DynamicSafety::Impl::add_trajectory(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr & rt)
{
  joint_names.clear();
  for (auto & joint : rt->joint_names) {
    joint_names.insert(joint);
  }
  collision_checker_.add_trajectory(rt);
  full_duration_ = rclcpp::Duration(rt->points.back().time_from_start).seconds();
  if (option_.visualize) {
    visualizer_.add_trajectory(rt);
  }
  if (option_.allow_replan) {
    replanner_.add_trajectory(rt);
    retries_in_window_ = 0;
    last_retry_time_ = -std::numeric_limits<double>::infinity();
    retry_pending_ = false;
    timeout_episode_active_ = false;
    terminate_requested_for_timeout_episode_ = false;
    last_attempt_was_retry_ = false;
    conservative_fallback_active_ = false;
  }
  activated_ = true;
  zone_decision_policy_.reset();
}


void DynamicSafety::Impl::update_time(double current_time)
{
  current_time_cache_.writeFromNonRT(current_time);
}

void DynamicSafety::Impl::update_state(const sensor_msgs::msg::JointState::SharedPtr & state)
{
  env_state_cache_.writeFromNonRT(*state);
}

void DynamicSafety::Impl::update_state(
  const std::vector<std::string> & joint_names,
  const trajectory_msgs::msg::JointTrajectoryPoint & current_state)
{
  current_state_cache_.writeFromNonRT(CurrentState(joint_names, current_state));
}

void DynamicSafety::Impl::update_scene(const moveit_msgs::msg::PlanningScene::SharedPtr & scene_msg)
{
  moveit_scene_cache_.writeFromNonRT(*scene_msg);
}


double DynamicSafety::Impl::get_scale()
{
  return *scale_cache_.readFromRT();
}

void DynamicSafety::Impl::start()
{
  // next_point_publisher_.start();
  // RCLCPP_INFO(LOGGER, "Next point publisher started!");
  // collision_time_point_ = -1;

  if (activated_) {
    if (!started) {
      if (option_.benchmark) {
        pf_ = new emd::TimeProfiler<>(5000);
      }

      started = true;

      if (option_.visualize) {
        visualizer_.start();
      }
      sig_ = std::promise<void>();
      future_ = sig_.get_future();
      RCLCPP_INFO(LOGGER, "All started");
    } else {
      RCLCPP_WARN(LOGGER, "Already started");
    }
  } else {
    RCLCPP_ERROR(LOGGER, "Not configured!! Please call the configure() first");
  }
}

void DynamicSafety::Impl::wait()
{
  RCLCPP_INFO(LOGGER, "Waiting...");
  future_.wait();
  RCLCPP_INFO(LOGGER, "Successfully exit.");
}

void DynamicSafety::Impl::stop()
{
  // visualizer_.reset();
  // RCLCPP_INFO(
  //   LOGGER, "Next Point Publisher ended with %s status",
  //   (next_point_publisher_.get_status() == NextPointPublisher::SUCCEEDED) ?
  //   "SUCCEEDED" : "FAILED");
  // next_point_publisher_.reset();
  if (option_.visualize) {
    visualizer_.stop();
  }
  started = false;
  activated_ = false;

  // Print out result
  if (option_.benchmark) {
    if (pf_) {
      std::ostringstream oss;
      pf_->print(oss);
      RCLCPP_INFO_STREAM(
        LOGGER,
        "Time stats:\n" << oss.str());
      delete pf_;
    }
  }
  RCLCPP_INFO(
    LOGGER,
    "dynamic_safety_metrics timeout_count=%lu terminate_count=%lu retry_count=%lu "
    "success_after_retry_count=%lu",
    timeout_count_.load(),
    terminate_count_.load(),
    retry_count_.load(),
    success_after_retry_count_.load());
  sig_.set_value();
}

void DynamicSafety::Impl::_deadline_cb(rclcpp::QOSDeadlineRequestedInfo &)
{
}

void DynamicSafety::Impl::_main_loop()
{
  // Update joint state
  if (!option_.environment_joint_states_topic.empty()) {
    collision_checker_.update(*env_state_cache_.readFromRT());
    if (option_.allow_replan) {
      replanner_.update(*env_state_cache_.readFromRT());
    }
  }

  // Scene with MoveIt Scene
  if (!option_.moveit_scene_topic.empty()) {
    collision_checker_.update(*moveit_scene_cache_.readFromRT());
    if (option_.allow_replan) {
      replanner_.update(*moveit_scene_cache_.readFromRT());
    }
  }

  // get scaled time point
  double current_time_point = *current_time_cache_.readFromRT();
  // RCLCPP_INFO(LOGGER, "Current time: %f", current_time_point);
  collision_time_point_ = -1;

  // Check collision once
  collision_checker_.run_once(
    current_time_point,
    option_.safety_zone_options.look_ahead_time,
    collision_time_point_
  );

  if (option_.collision_checker_options.distance) {
    min_distance_ = collision_checker_.get_min_distance();
    RCLCPP_DEBUG(LOGGER, "Minimum distance to obstacle: %f", min_distance_);
  }

  double scale = *scale_cache_.readFromRT();

  // Dynamically adjust slow down time
  if (option_.safety_zone_options.slow_down_time <= 0 && option_.dynamic_parameterization) {
    SafetyZone::Option dynamic_option = option_.safety_zone_options;
    if (scale != 0.0001) {
      dynamic_option.slow_down_time =
        _cal_scale_time(*current_state_cache_.readFromRT(), scale, 0.0001);
    } else {
      dynamic_option.slow_down_time = 0;
    }

    // Dynamically set scale step
    if (dynamic_option.slow_down_time > 0) {
      safety_zone_.set(dynamic_option);

      // Update visualizer as well
      if (option_.visualize) {
        visualizer_.update(dynamic_option);
      }
    } else if (scale != 0.0001) {
      RCLCPP_ERROR(
        LOGGER,
        "There is no velocity state feedback from the robot. "
        "Please check /joint_states for velocity values!!"
        "Hard set the slow down time to 0.5s instead");
      option_.safety_zone_options.slow_down_time = 0.5;
      safety_zone_.set(option_.safety_zone_options);
      // Update visualizer as well
      if (option_.visualize) {
        visualizer_.update(option_.safety_zone_options);
      }
    }
  } else {
    // preconfifured slow down time reduce it based on current scale
    SafetyZone::Option dynamic_option = option_.safety_zone_options;
    dynamic_option.slow_down_time = option_.safety_zone_options.slow_down_time * scale;
    safety_zone_.set(dynamic_option);
    // Update visualizer as well
    if (option_.visualize) {
      visualizer_.update(dynamic_option);
    }
  }

  const bool collision_detected = collision_time_point_ >= current_time_point;
  const uint8_t raw_zone = collision_detected ?
    safety_zone_.get_zone(collision_time_point_ - current_time_point) : SafetyZone::SAFE;
  const uint8_t replanner_status = option_.allow_replan ? replanner_.get_status() : static_cast<uint8_t>(ReplannerStatus::IDLE);
  const ZoneDecision decision = zone_decision_policy_.decide(
    ZoneDecisionInput{
      *current_time_cache_.readFromRT(),
      1.0 / option_.rate,
      current_time_point,
      full_duration_,
      collision_time_point_,
      raw_zone,
      collision_detected,
      option_.allow_replan,
      scale,
      safety_zone_.get_zone_limit(SafetyZone::EMERGENCY),
      safety_zone_.get_zone_limit(SafetyZone::SLOWDOWN),
      replanner_status
    });

  if (decision.zone != raw_zone || decision.action != "none" ||
    std::fabs(decision.next_scale - scale) > 1e-6)
  {
    RCLCPP_INFO_STREAM(
      LOGGER,
      "zone_transition raw=" << _zone_to_str(raw_zone) <<
        " effective=" << _zone_to_str(decision.zone) <<
        " action=" << decision.action <<
        " start_state_time=" << decision.start_state_time <<
        " collision_time=" << collision_time_point_ <<
        " scale:" << scale << "->" << decision.next_scale);
  }

  scale = decision.next_scale;
  if (option_.allow_replan) {
    if (decision.start_replanner) {
      _handle_replanner(decision.start_state_time);
    }
    if (decision.terminate_replanner) {
      replanner_.terminate_async();
    }
    if (decision.consume_replan_result) {
      replanner_.get_result();
    }
  }
  if (conservative_fallback_active_) {
    scale = std::min(scale, 0.0001);
  }

  scale_cache_.writeFromNonRT(scale);

  if (option_.visualize) {
    visualizer_.update(current_time_point, collision_time_point_);
  }
}

const char * DynamicSafety::Impl::_zone_to_str(uint8_t zone)
{
  switch (zone) {
    case SafetyZone::BLIND: return "BLIND";
    case SafetyZone::EMERGENCY: return "EMERGENCY";
    case SafetyZone::SLOWDOWN: return "SLOWDOWN";
    case SafetyZone::REPLAN: return "REPLAN";
    case SafetyZone::SAFE: return "SAFE";
    default: return "UNKNOWN";
  }
}

double DynamicSafety::Impl::_cal_scale_time(
  const CurrentState & current_state,
  double current_scale,
  double target_scale)
{
  double scale_time = -1.0;
  if (!current_state.state.velocities.empty()) {
    for (size_t i = 0; i < current_state.state.velocities.size(); i++) {
      // Skip joints that is not controlled by this controller
      if (joint_names.find(current_state.joint_names[i]) == joint_names.end()) {
        continue;
      }
      // Skip joints with no limits
      if (option_.joint_limits[current_state.joint_names[i]].first == 0) {
        continue;
      }
      // Skip joints with no limits
      if (option_.joint_limits[current_state.joint_names[i]].second == 0) {
        continue;
      }
      // Slow down
      if (current_scale >= target_scale) {
        double temp_scale_time =
          ::fabs(current_state.state.velocities[i] * (current_scale - target_scale)) /
          current_scale / option_.joint_limits[current_state.joint_names[i]].second;
        if (temp_scale_time > scale_time) {
          scale_time = temp_scale_time;
        }
      } else {
        // Speed up
        double temp_scale_time =
          (option_.joint_limits[current_state.joint_names[i]].first -
          ::fabs(current_state.state.velocities[i])) /
          option_.joint_limits[current_state.joint_names[i]].second;
        if (temp_scale_time > scale_time) {
          scale_time = temp_scale_time;
        }
      }
    }
  }
  return scale_time;
}

void DynamicSafety::Impl::_handle_replanner(double start_state_time)
{
  start_state_time = std::clamp(start_state_time, *current_time_cache_.readFromRT(), full_duration_);
  const double now = *current_time_cache_.readFromRT();
  auto maybe_retry = [&](double restart_time) {
      if (retries_in_window_ >= max_retries_per_window_) {
        return false;
      }
      if (now - last_retry_time_ < retry_cooldown_s_) {
        return false;
      }
      double end_state_time = _back_track_last_collision();
      replanner_.run_async(restart_time, end_state_time);
      retries_in_window_++;
      retry_count_++;
      last_retry_time_ = now;
      last_attempt_was_retry_ = true;
      retry_pending_ = false;
      conservative_fallback_active_ = false;
      RCLCPP_WARN(
        LOGGER, "Retrying replanner (%zu/%zu)", retries_in_window_, max_retries_per_window_);
      return true;
    };
  // Replanner not started
  auto status = replanner_.get_status();
  if (status == ReplannerStatus::IDLE) {
    if (retry_pending_) {
      if (!maybe_retry(pending_retry_start_state_time_)) {
        RCLCPP_ERROR(
          LOGGER,
          "Retry budget exhausted or cooldown active; entering conservative emergency behavior");
        conservative_fallback_active_ = true;
      }
      return;
    }
    // Replanner Started
    RCLCPP_WARN_ONCE(
      LOGGER,
      "Starting replanner [%s] with starting time point",
      option_.replanner_options.planner.c_str());
    double end_state_time = _back_track_last_collision();
    replanner_.run_async(start_state_time, end_state_time);
    last_attempt_was_retry_ = false;

  } else if (status == ReplannerStatus::RUNNING) {
    // Just let it run baby.
    // TODO(anyone): Better handling?
  } else if (status == ReplannerStatus::TIMEOUT) {
    if (!timeout_episode_active_) {
      timeout_episode_active_ = true;
      timeout_count_++;
    }
    if (!terminate_requested_for_timeout_episode_) {
      replanner_.terminate_async();
      terminate_count_++;
      terminate_requested_for_timeout_episode_ = true;
      retry_pending_ = true;
      pending_retry_start_state_time_ = start_state_time;
    }
  } else if (status == ReplannerStatus::TERMINATING) {
    // Wait until replanner reaches IDLE after termination.
  } else if (status == ReplannerStatus::FAILED) {
    retry_pending_ = true;
    pending_retry_start_state_time_ = start_state_time;
  } else if (status == ReplannerStatus::SUCCEED) {
    // Let's see if you really finished, or just failed and gaveup
    auto result = replanner_.get_result();
    if (result->points.empty()) {
      retry_pending_ = true;
      pending_retry_start_state_time_ = start_state_time;
      if (!maybe_retry(start_state_time)) {
        RCLCPP_ERROR(
          LOGGER,
          "Retry budget exhausted or cooldown active; entering conservative emergency behavior");
        conservative_fallback_active_ = true;
      }
    } else {
      // Good job replanner, let's add a starting point and
      // do time_parameterization.
      auto joint_names = current_state_cache_.readFromRT()->joint_names;
      auto current_state = current_state_cache_.readFromRT()->state;
      double current_time = *current_time_cache_.readFromRT();
      // Get time parameterized result
      auto new_traj = replanner_.flatten_result(current_time, joint_names, current_state);
      if (!new_traj->points.empty()) {
        NewTrajectoryCB(new_traj);
        if (last_attempt_was_retry_) {
          success_after_retry_count_++;
          last_attempt_was_retry_ = false;
        }
        timeout_episode_active_ = false;
        terminate_requested_for_timeout_episode_ = false;
        retry_pending_ = false;
        conservative_fallback_active_ = false;
      }
    }
  }
}
double DynamicSafety::Impl::_back_track_last_collision()
{
  // Use collision checker to backtrack collision
  // This is not nearly as efficient right now to be improved.
  // TODO(anyone): Enable this in collision checker

  // Extra time buffer (seconds) added when using Tesseract, which does not handle
  // very short trajectory segments well. Keeps the returned time point safely
  // ahead of the detected collision point.
  static constexpr double kTesseractShortSegmentBuffer = 0.8;

  double time_from_start = full_duration_;
  double step = option_.collision_checker_options.step;
  double collision_time = -1;
  while (time_from_start >= 0) {
    time_from_start -= step;
    collision_checker_.run_once(time_from_start, 0.0, collision_time);
    if (collision_time > 0) {
      // Tesseract doesn't see to work well with short segment
      if (option_.replanner_options.framework == "tesseract") {
        return std::min<double>(time_from_start + kTesseractShortSegmentBuffer, full_duration_);
      }
      return time_from_start + step;
    }
  }
  return full_duration_;
}

DynamicSafety::DynamicSafety(
  rclcpp::Node::SharedPtr node)
: DynamicSafety(Option().load(node))
{
}

DynamicSafety::DynamicSafety(
  rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node)
: DynamicSafety(Option().load(lifecycle_node))
{
}

DynamicSafety::~DynamicSafety()
{
}

DynamicSafety::DynamicSafety(
  const Option & option)
: impl_ptr_(std::make_unique<Impl>(option))
{
}

void DynamicSafety::configure(
  const rclcpp::Node::SharedPtr & node)
{
  impl_ptr_->configure(node);
}

void DynamicSafety::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & lifecycle_node)
{
  impl_ptr_->configure(lifecycle_node);
}

void DynamicSafety::add_trajectory(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr & rt)
{
  impl_ptr_->add_trajectory(rt);
}

void DynamicSafety::set_new_trajectory_callback(
  std::function<void(const trajectory_msgs::msg::JointTrajectory::SharedPtr &)> cb)
{
  impl_ptr_->NewTrajectoryCB = cb;
}


void DynamicSafety::update_time(double current_time)
{
  impl_ptr_->update_time(current_time);
}

void DynamicSafety::update_state(const sensor_msgs::msg::JointState::SharedPtr & state)
{
  impl_ptr_->update_state(state);
}

void DynamicSafety::update_state(
  const std::vector<std::string> & joint_names,
  const trajectory_msgs::msg::JointTrajectoryPoint & current_state)
{
  impl_ptr_->update_state(joint_names, current_state);
}

void DynamicSafety::update_state(
  const std::vector<std::string> & joint_names,
  const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr & state)
{
  impl_ptr_->update_state(joint_names, *state);
}

double DynamicSafety::get_scale()
{
  return impl_ptr_->get_scale();
}

void DynamicSafety::start()
{
  impl_ptr_->start();
}

void DynamicSafety::wait()
{
  impl_ptr_->wait();
}

void DynamicSafety::stop()
{
  impl_ptr_->stop();
}

}  // namespace dynamic_safety
