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

// Main PCL files
#include "emd/grasp_planner/grasp_scene.hpp"
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <type_traits>
#include <utility>
#include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>

namespace
{
template<typename T, typename = void>
struct has_camera_info : std::false_type
{
};

template<typename T>
struct has_camera_info<T, std::void_t<decltype(std::declval<T>().camera_info)>> : std::true_type
{
};

template<typename RequestT, typename = void>
struct has_ready_flag : std::false_type
{
};

template<typename RequestT>
struct has_ready_flag<RequestT, std::void_t<decltype(std::declval<RequestT &>().ready)>>
  : std::true_type
{
};

template<typename RequestT, typename = void>
struct has_start_flag : std::false_type
{
};

template<typename RequestT>
struct has_start_flag<RequestT, std::void_t<decltype(std::declval<RequestT &>().start)>>
  : std::true_type
{
};

template<typename RequestT, typename = void>
struct has_trigger_flag : std::false_type
{
};

template<typename RequestT>
struct has_trigger_flag<RequestT, std::void_t<decltype(std::declval<RequestT &>().trigger)>>
  : std::true_type
{
};

template<typename RequestT>
void set_epd_trigger_flag_if_available(RequestT & request)
{
  if constexpr (has_ready_flag<RequestT>::value) {
    request.ready = true;
  } else if constexpr (has_start_flag<RequestT>::value) {
    request.start = true;
  } else if constexpr (has_trigger_flag<RequestT>::value) {
    request.trigger = true;
  }
}

// Conversion factor: raw 16-bit depth value (millimetres) → metres.
constexpr double kDepthMmToMeters = 0.001;

std::string to_lower_copy(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
  return value;
}

rclcpp::QoS build_perception_qos(
  const rclcpp::Node::SharedPtr & node,
  const std::string & reliability_parameter,
  std::string & selected_reliability)
{
  node->get_parameter_or(reliability_parameter, selected_reliability, std::string("best_effort"));
  selected_reliability = to_lower_copy(selected_reliability);

  auto perception_qos = rclcpp::SensorDataQoS().keep_last(10);
  if (selected_reliability == "reliable") {
    perception_qos.reliable();
    selected_reliability = "reliable";
  } else {
    perception_qos.best_effort();
    selected_reliability = "best_effort";
  }

  return perception_qos;
}
}  // namespace

template<typename T>
void grasp_planner::GraspScene<T>::send_to_execution(
  const emd_msgs::msg::GraspTask & grasp_task)
{
  if (grasp_task.grasp_targets.empty()) {
    RCLCPP_ERROR(LOGGER, "No grasp tasks generated, Skipping request to grasp execution...");
    this->grasp_objects.clear();
    return;
  }

  if (execution_gate_enabled && execution_in_progress.load(std::memory_order_acquire)) {
    RCLCPP_WARN(
      LOGGER,
      "Grasp execution still running; dropping new request with %zu targets due to "
      "'execution_in_progress_gate_enabled'.",
      grasp_task.grasp_targets.size());
    this->grasp_objects.clear();
    return;
  }

  if (result_future.valid()) {
    const auto status = result_future.wait_for(std::chrono::nanoseconds(0));
    if (status == std::future_status::ready) {
      auto result = result_future.get();
      const bool success = static_cast<bool>(result && result->success);
      RCLCPP_INFO(
        LOGGER,
        "Previous grasp execution completed: success=%s message='%s'",
        success ? "true" : "false",
        result ? result->message.c_str() : "<null response>");
      execution_in_progress.store(false, std::memory_order_release);
    } else {
      execution_in_progress.store(true, std::memory_order_release);
      if (execution_gate_enabled) {
        RCLCPP_INFO(LOGGER, "Previous grasp execution request still running; skipping send.");
        this->grasp_objects.clear();
        return;
      }
      RCLCPP_WARN(
        LOGGER,
        "Previous grasp execution request still running but gate disabled; sending another request.");
    }
  }

  RCLCPP_INFO(LOGGER, "Sending grasp request to grasp execution module.");
  auto req = std::make_shared<emd_msgs::srv::GraspRequest::Request>();
  req->grasp_targets = grasp_task.grasp_targets;
  RCLCPP_INFO(LOGGER, "Waiting for grasp execution service.");

  constexpr int kMaxWaitAttempts = 5;
  int attempts = 0;
  while (!wait_for_output_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_WARN(
        node->get_logger(), "Grasp execution service wait interrupted. Skipping request.");
      this->grasp_objects.clear();
      return;
    }
    if (++attempts >= kMaxWaitAttempts) {
      RCLCPP_ERROR(
        LOGGER,
        "Grasp execution service unavailable after %d attempts. Dropping task.",
        kMaxWaitAttempts);
      this->grasp_objects.clear();
      return;
    }
    RCLCPP_WARN(
      LOGGER, "Grasp execution service unavailable, retrying (%d/%d)...",
      attempts, kMaxWaitAttempts);
  }

  this->result_future = send_output_request(req);
  execution_in_progress.store(true, std::memory_order_release);
  RCLCPP_INFO(
    LOGGER, "Grasp execution request accepted/sent (%zu targets).",
    req->grasp_targets.size());
  this->grasp_objects.clear();
}

template<typename T>
bool grasp_planner::GraspScene<T>::wait_for_output_service(const std::chrono::duration<double> & timeout)
{
  return output_client->wait_for_service(timeout);
}

template<typename T>
std::shared_future<rclcpp::Client<emd_msgs::srv::GraspRequest>::SharedResponse>
grasp_planner::GraspScene<T>::send_output_request(
  const std::shared_ptr<emd_msgs::srv::GraspRequest::Request> & request)
{
  return output_client->async_send_request(request).future.share();
}

template<typename T>
emd_msgs::msg::GraspTask grasp_planner::GraspScene<T>::generate_grasp_task()
{
  emd_msgs::msg::GraspTask grasp_task;
  grasp_task.task_id = MathFunctions::generate_task_id();

  if (this->grasp_objects.size() == 0) {return grasp_task;}

  load_end_effectors();
  for (auto object : this->grasp_objects) {
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    for (auto & gripper : this->end_effectors) {
      std::chrono::steady_clock::time_point grasp_begin = std::chrono::steady_clock::now();

      emd_msgs::msg::GraspMethod grasp_method;
      grasp_method.ee_id = gripper->get_id();
      grasp_method.grasp_ranks.insert(
        grasp_method.grasp_ranks.begin(), std::numeric_limits<float>::lowest());

      gripper->plan_grasps(
        object, grasp_method, world_collision_object,
        node->get_parameter("camera_parameters.camera_frame").as_string());
      grasp_method.grasp_ranks.pop_back();

      if (grasp_method.grasp_ranks.size() > 0) {
        object.grasp_target.grasp_methods.push_back(grasp_method);
      } else {
        RCLCPP_ERROR_STREAM(
          LOGGER, "For Object " << object.grasp_target.target_type.c_str() <<
            ", no grasp methods can be found with end effector " << gripper->get_id());
        // continue;
      }

      std::chrono::steady_clock::time_point grasp_end = std::chrono::steady_clock::now();

      RCLCPP_INFO_STREAM(
        LOGGER, "Grasp planning time for " << grasp_method.ee_id << " " <<
          std::to_string(
          std::chrono::duration_cast<std::chrono::milliseconds>(grasp_end - grasp_begin).count()) +
          " [ms] ");

      bool point_cloud_visualization = false;
      node->get_parameter_or(
        "visualization_params.point_cloud_visualization", point_cloud_visualization, false);
      if (point_cloud_visualization) {
        if (ensure_visualization_runtime()) {
          RCLCPP_INFO(
            LOGGER, "Visualizing grasps for object '%s' (cloud points: %zu).",
            object.object_name.c_str(), object.cloud ? object.cloud->size() : 0UL);
          viewer->removeAllPointClouds();
          viewer->removeAllShapes();
          viewer->removeAllCoordinateSystems();
          if (viewer->wasStopped()) {
            viewer->resetStoppedFlag();
          }
        gripper->visualize_grasps(viewer, object);
        } else {
          RCLCPP_WARN(
            LOGGER, "Skipping visualization for '%s': %s",
            object.object_name.c_str(), visualization_unavailable_reason.c_str());
          maybe_save_visualization_debug_artifacts(object, visualization_unavailable_reason);
        }
      }
    }

    if (object.grasp_target.grasp_methods.size() > 0) {
      grasp_task.grasp_targets.push_back(object.grasp_target);
    } else {
      RCLCPP_ERROR_STREAM(
        LOGGER, "For Object " << object.grasp_target.target_type <<
          ", no grasp methods can be found with any given "
          " end effectors provided. ");
      continue;
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    RCLCPP_INFO_STREAM(
      LOGGER, "Grasp planning time for object " << object.object_name << " " <<
        std::to_string(
        std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()) +
        " [ms] ");
  }

  object_pose_rectification(grasp_task);

  return grasp_task;
}

template<typename T>
bool grasp_planner::GraspScene<T>::is_display_available() const
{
  const char * display = std::getenv("DISPLAY");
  const char * wayland = std::getenv("WAYLAND_DISPLAY");
  return (display != nullptr && std::string(display).size() > 0U) ||
         (wayland != nullptr && std::string(wayland).size() > 0U);
}

template<typename T>
bool grasp_planner::GraspScene<T>::ensure_visualization_runtime()
{
  if (visualization_initialized) {
    return visualization_available;
  }
  visualization_initialized = true;
  std::string backend{"pcl"};
  node->get_parameter_or(
    "visualization_params.point_cloud_visualization_backend", backend, std::string("pcl"));
  RCLCPP_INFO(LOGGER, "Visualization backend selected: %s", backend.c_str());
  if (backend != "pcl") {
    visualization_unavailable_reason = "unsupported backend '" + backend + "'";
    return false;
  }
  if (!is_display_available()) {
    visualization_unavailable_reason = "no display available (DISPLAY/WAYLAND_DISPLAY unset)";
    return false;
  }
  try {
    viewer = std::make_shared<pcl::visualization::PCLVisualizer>("EMD Grasp Planner Viewer");
    visualization_available = static_cast<bool>(viewer);
    if (!visualization_available) {
      visualization_unavailable_reason = "PCL viewer allocation returned null";
      return false;
    }
    RCLCPP_INFO(LOGGER, "Point cloud visualization viewer initialized successfully.");
  } catch (const std::exception & ex) {
    visualization_unavailable_reason = std::string("viewer init error: ") + ex.what();
    visualization_available = false;
  }
  return visualization_available;
}

template<typename T>
void grasp_planner::GraspScene<T>::maybe_save_visualization_debug_artifacts(
  const GraspObject & object,
  const std::string & reason) const
{
  bool save_debug = false;
  node->get_parameter_or(
    "visualization_params.point_cloud_visualization_save_debug_pcd", save_debug, false);
  if (!save_debug) {return;}
  std::string debug_dir{"/tmp/emd_grasp_visualization"};
  node->get_parameter_or(
    "visualization_params.point_cloud_visualization_debug_dir", debug_dir, debug_dir);
  std::filesystem::create_directories(debug_dir);
  const auto stamp = std::to_string(node->now().nanoseconds());
  if (object.cloud && !object.cloud->empty()) {
    pcl::io::savePCDFileBinary(
      debug_dir + "/object_cloud_" + stamp + ".pcd",
      *object.cloud);
  }
  if (cloud && !cloud->empty()) {
    pcl::io::savePCDFileBinary(
      debug_dir + "/scene_cloud_" + stamp + ".pcd",
      *cloud);
  }
  YAML::Node summary;
  summary["reason"] = reason;
  summary["object_name"] = object.object_name;
  summary["object_cloud_size"] = object.cloud ? object.cloud->size() : 0U;
  summary["scene_cloud_size"] = cloud ? cloud->size() : 0U;
  std::ofstream fout(debug_dir + "/grasp_debug_summary_" + stamp + ".yaml");
  fout << summary;
  RCLCPP_WARN(LOGGER, "Visualization debug artifacts written to '%s'", debug_dir.c_str());
}

template<typename T>
void grasp_planner::GraspScene<T>::load_end_effectors()
{
  this->end_effectors.clear();
  std::vector<std::string> end_effector_array = node->get_parameter(
    "end_effectors.end_effector_names").as_string_array();
  for (std::string end_effector : end_effector_array) {
    try {
      std::string end_effector_type =
        node->get_parameter("end_effectors." + end_effector + ".type").as_string();
      RCLCPP_INFO_STREAM(LOGGER, "Loading " << end_effector_type << " gripper " << end_effector);
      if (end_effector_type.compare("finger") == 0) {
        bool allow_target_fingertip_contact = true;
        node->get_parameter_or(
          "grasp_planner_allow_target_fingertip_contact",
          allow_target_fingertip_contact,
          true);
        std::vector<std::string> allowed_target_touch_links{
          "gripper_finger1_finger_tip_link",
          "gripper_finger2_finger_tip_link"};
        node->get_parameter_or(
          "grasp_planner_allowed_target_touch_links",
          allowed_target_touch_links,
          allowed_target_touch_links);
        std::vector<std::string> allowed_target_touch_link_patterns;
        node->get_parameter_or(
          "grasp_planner_allowed_target_touch_link_patterns",
          allowed_target_touch_link_patterns,
          allowed_target_touch_link_patterns);
        FingerGripper gripper(
          end_effector,
          node->get_parameter("end_effectors." + end_effector + ".num_fingers_side_1").as_int(),
          node->get_parameter("end_effectors." + end_effector + ".num_fingers_side_2").as_int(),
          static_cast<float>(node->get_parameter(
            "end_effectors." + end_effector +
            ".distance_between_fingers_1").as_double()),
          static_cast<float>(node->get_parameter(
            "end_effectors." + end_effector +
            ".distance_between_fingers_2").as_double()),
          static_cast<float>(node->get_parameter(
            "end_effectors." + end_effector +
            ".finger_thickness").as_double()),
          static_cast<float>(node->get_parameter(
            "end_effectors." + end_effector +
            ".gripper_stroke").as_double()),
          static_cast<float>(node->get_parameter(
            "end_effectors." + end_effector +
            ".grasp_planning_params.voxel_size").as_double()),
          static_cast<float>(node->get_parameter(
            "end_effectors." + end_effector +
            ".grasp_planning_params.grasp_rank_weight_1").as_double()),
          static_cast<float>(node->get_parameter(
            "end_effectors." + end_effector +
            ".grasp_planning_params.grasp_rank_weight_2").as_double()),
          static_cast<float>(node->get_parameter(
            "end_effectors." + end_effector +
            ".grasp_planning_params.centroid_dist_penalty_weight").as_double()),
          static_cast<float>(node->get_parameter(
            "end_effectors." + end_effector +
            ".grasp_planning_params.wrist_rotation_penalty_weight").as_double()),
          static_cast<float>(node->get_parameter(
            "end_effectors." + end_effector +
            ".grasp_planning_params.preferred_wrist_roll").as_double()),
          static_cast<float>(node->get_parameter(
            "end_effectors." + end_effector +
            ".grasp_planning_params.preferred_wrist_pitch").as_double()),
          static_cast<float>(node->get_parameter(
            "end_effectors." + end_effector +
            ".grasp_planning_params.preferred_wrist_yaw").as_double()),
          static_cast<float>(node->get_parameter(
            "end_effectors." + end_effector +
            ".grasp_planning_params.grasp_plane_dist_limit").as_double()),
          static_cast<float>(node->get_parameter(
            "point_cloud_params.cloud_normal_radius").as_double()),
          static_cast<float>(node->get_parameter(
            "end_effectors." + end_effector +
            ".grasp_planning_params.world_x_angle_threshold").as_double()),
          static_cast<float>(node->get_parameter(
            "end_effectors." + end_effector +
            ".grasp_planning_params.world_y_angle_threshold").as_double()),
          static_cast<float>(node->get_parameter(
            "end_effectors." + end_effector +
            ".grasp_planning_params.world_z_angle_threshold").as_double()),
          node->get_parameter(
            "end_effectors." + end_effector +
            ".gripper_coordinate_system.grasp_stroke_direction").as_string(),
          node->get_parameter(
            "end_effectors." + end_effector +
            ".gripper_coordinate_system.grasp_stroke_normal_direction").as_string(),
          node->get_parameter(
            "end_effectors." + end_effector +
            ".gripper_coordinate_system.grasp_approach_direction").as_string(),
          allow_target_fingertip_contact,
          allowed_target_touch_links,
          allowed_target_touch_link_patterns
        );
        this->end_effectors.push_back(std::make_shared<FingerGripper>(std::move(gripper)));
      } else if (end_effector_type.compare("suction") == 0) {
        SuctionGripper gripper(
          end_effector,
          node->get_parameter("end_effectors." + end_effector + ".num_cups_length").as_int(),
          node->get_parameter("end_effectors." + end_effector + ".num_cups_breadth").as_int(),
          static_cast<float>(node->get_parameter(
            "end_effectors." + end_effector + ".dist_between_cups_length").as_double()),
          static_cast<float>(node->get_parameter(
            "end_effectors." + end_effector + ".dist_between_cups_breadth").as_double()),
          static_cast<float>(node->get_parameter(
            "end_effectors." + end_effector + ".cup_radius").as_double()),
          static_cast<float>(node->get_parameter(
            "end_effectors." + end_effector + ".cup_height").as_double()),
          node->get_parameter(
            "end_effectors." + end_effector +
            ".grasp_planning_params.num_sample_along_axis").as_int(),
          static_cast<float>(node->get_parameter(
            "end_effectors." + end_effector +
            ".grasp_planning_params.search_resolution").as_double()),
          node->get_parameter(
            "end_effectors." + end_effector +
            ".grasp_planning_params.search_angle_resolution").as_int(),
          static_cast<float>(node->get_parameter(
            "point_cloud_params.cloud_normal_radius").as_double()),
          static_cast<float>(node->get_parameter(
            "end_effectors." + end_effector +
            ".grasp_planning_params.weights.curvature").as_double()),
          static_cast<float>(node->get_parameter(
            "end_effectors." + end_effector +
            ".grasp_planning_params.weights.grasp_distance_to_center").as_double()),
          static_cast<float>(node->get_parameter(
            "end_effectors." + end_effector +
            ".grasp_planning_params.weights.number_contact_points").as_double()),
          node->get_parameter(
            "end_effectors." + end_effector +
            ".gripper_coordinate_system.length_direction").as_string(),
          node->get_parameter(
            "end_effectors." + end_effector +
            ".gripper_coordinate_system.breadth_direction").as_string(),
          node->get_parameter(
            "end_effectors." + end_effector +
            ".gripper_coordinate_system.grasp_approach_direction").as_string());

        this->end_effectors.push_back(std::make_shared<SuctionGripper>(std::move(gripper)));
      } else {
        RCLCPP_ERROR_STREAM(
          LOGGER,
          "Unknown end effector type '" << end_effector_type << "' for end effector '"
                                        << end_effector << "'. Skipping.");
      }
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException & ex) {
      RCLCPP_ERROR(
        LOGGER,
        "Missing parameter while loading end effector '%s': %s. Skipping.",
        end_effector.c_str(), ex.what());
    } catch (const std::invalid_argument & ex) {
      RCLCPP_ERROR(
        LOGGER,
        "Invalid configuration for end effector '%s': %s. Skipping.",
        end_effector.c_str(), ex.what());
    }
  }
  RCLCPP_INFO(LOGGER, "All End Effectors Loaded");
}

template<>
void grasp_planner::GraspScene<sensor_msgs::msg::PointCloud2>::extract_objects_direct(
  const builtin_interfaces::msg::Time & stamp)
{
  RCLCPP_INFO(LOGGER, "Extracting Objects from point cloud");

  std::string camera_frame = node->get_parameter("camera_parameters.camera_frame").as_string();

  int min_cluster_size = node->get_parameter(
    "point_cloud_params.min_cluster_size").as_int();

  float cloud_normal_radius = static_cast<float>(node->get_parameter(
      "point_cloud_params.cloud_normal_radius").as_double());

  int normal_estimation_threads = node->get_parameter(
    "point_cloud_params.normal_estimation_threads").as_int();

  float cluster_tolerance = static_cast<float>(node->get_parameter(
      "point_cloud_params.cluster_tolerance").as_double());

  // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  // std::vector<pcl::PointIndices> clusterIndices;
  // pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ecExtractor;

  // tree->setInputCloud(cloud);
  // ecExtractor.setClusterTolerance(cluster_tolerance);
  // ecExtractor.setMinClusterSize(min_cluster_size);
  // ecExtractor.setSearchMethod(tree);
  // ecExtractor.setInputCloud(cloud);
  // ecExtractor.extract(clusterIndices);

  std::vector<pcl::PointIndices> clusterIndices = PCLFunctions::extract_pointcloud_clusters(
    this->cloud_plane_removed, cluster_tolerance, min_cluster_size);

  if (clusterIndices.empty()) {
    RCLCPP_ERROR(LOGGER, "No Objects can be extracted");
  } else {
    std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin();
    for (it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
      for (std::vector<int>::const_iterator pit = it->indices.begin();
        pit != it->indices.end(); ++pit)
      {
        objectCloud->points.push_back(this->cloud_plane_removed->points[*pit]);
      }

      objectCloud->width = objectCloud->points.size();
      objectCloud->height = 1;
      objectCloud->is_dense = true;

      // Get the centroid of the point cloud
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*objectCloud, centroid);
      GraspObject object(
        camera_frame,
        objectCloud,
        centroid);
      PCLFunctions::compute_cloud_normal(
        objectCloud,
        object.cloud_normal,
        cloud_normal_radius,
        normal_estimation_threads);
      object.get_object_bb();
      object.get_object_world_angles();
      object.grasp_target.target_shape = object.get_object_shape();
      object.grasp_target.target_pose = object.get_object_pose(camera_frame, stamp);
      this->grasp_objects.push_back(object);
    }
  }
  RCLCPP_INFO_STREAM(
    LOGGER, "Extracted " << std::to_string(
      this->grasp_objects.size()) << " from point cloud");
}


#if EPD_ENABLED == 1
template<typename T>
void grasp_planner::GraspScene<T>::extract_objects_epd(
  const std::vector<epd_msgs::msg::LocalizedObject> & objects,
  const builtin_interfaces::msg::Time & stamp)
{
  RCLCPP_INFO(LOGGER, "Processing Objects detected by EPD...");

  std::string camera_frame = node->get_parameter(
    "camera_parameters.camera_frame").as_string();

  float cloud_normal_radius = static_cast<float>(node->get_parameter(
      "point_cloud_params.cloud_normal_radius").as_double());
  int normal_estimation_threads = node->get_parameter(
    "point_cloud_params.normal_estimation_threads").as_int();

  for (const auto & raw_object : objects) {
    // Standard deviation multiplier threshold for statistical outlier removal.
    static constexpr float kStatOutlierStddevThreshold = 0.5f;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PCLPointCloud2 pcl_pc2;
    PCLFunctions::sensor_msg_to_pcl_pointcloud2((raw_object.segmented_pcl), pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *(objectCloud));
    PCLFunctions::remove_statistical_outlier(objectCloud, kStatOutlierStddevThreshold);

    objectCloud->width = objectCloud->points.size();
    objectCloud->height = 1;
    objectCloud->is_dense = true;

    // Get the centroid of the point cloud
    Eigen::Vector4f centroid;
    centroid(0) = raw_object.centroid.x;
    centroid(1) = raw_object.centroid.y;
    centroid(2) = raw_object.centroid.z;

    GraspObject object(
      raw_object.name,
      camera_frame, objectCloud,
      centroid);
    PCLFunctions::compute_cloud_normal(
      objectCloud,
      object.cloud_normal,
      cloud_normal_radius,
      normal_estimation_threads);
    object.get_object_bb();
    object.get_object_world_angles();
    object.grasp_target.target_shape = object.get_object_shape();
    object.grasp_target.target_pose = object.get_object_pose(camera_frame, stamp);
    this->grasp_objects.push_back(object);
  }
  RCLCPP_INFO(
    LOGGER,
    "EPD detected %zu objects.",
    this->grasp_objects.size());
}
#endif

template<>
void grasp_planner::GraspScene<sensor_msgs::msg::PointCloud2>::extract_objects(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  extract_objects_direct(msg->header.stamp);
}

#if EPD_ENABLED == 1
template<>
void grasp_planner::GraspScene<epd_msgs::msg::EPDObjectLocalization>::extract_objects(
  const epd_msgs::msg::EPDObjectLocalization::ConstSharedPtr & msg)
{
  extract_objects_epd(msg->objects, msg->header.stamp);
}

template<>
void grasp_planner::GraspScene<epd_msgs::msg::EPDObjectTracking>::extract_objects(
  const epd_msgs::msg::EPDObjectTracking::ConstSharedPtr & msg)
{
  extract_objects_epd(msg->objects, msg->header.stamp);
}
#endif

template<>
void grasp_planner::GraspScene<sensor_msgs::msg::PointCloud2>::create_world_collision(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  const std::string robot_base_frame =
    node->get_parameter("camera_parameters.robot_base_frame").as_string();
  geometry_msgs::msg::TransformStamped sensorToWorldTf =
    this->buffer_->lookupTransform(
    robot_base_frame, msg->header.frame_id,
    msg->header.stamp);
  octomap::point3d sensor_origin = octomap::point_tf_to_octomap(
    sensorToWorldTf.transform.translation);
  this->world_collision_object = FCLFunctions::create_collision_object_from_pointcloud_rgb(
    this->org_cloud, sensor_origin,
    static_cast<float>(node->get_parameter("point_cloud_params.octomap_resolution").as_double()));
}

template<typename T>
void grasp_planner::GraspScene<T>::create_world_collision(
  const typename T::ConstSharedPtr & msg)
{
  float ppx = 0.0F;
  float fx = 0.0F;
  float ppy = 0.0F;
  float fy = 0.0F;

  if constexpr (has_camera_info<T>::value) {
    const auto & camera_info = msg->camera_info;
    ppx = static_cast<float>(camera_info.k.at(2));
    fx = static_cast<float>(camera_info.k.at(0));
    ppy = static_cast<float>(camera_info.k.at(5));
    fy = static_cast<float>(camera_info.k.at(4));
  } else {
    ppx = static_cast<float>(node->get_parameter("camera_parameters.ppx").as_double());
    fx = static_cast<float>(node->get_parameter("camera_parameters.fx").as_double());
    ppy = static_cast<float>(node->get_parameter("camera_parameters.ppy").as_double());
    fy = static_cast<float>(node->get_parameter("camera_parameters.fy").as_double());
  }
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat depth_img;
  const std::string & depth_encoding = msg->depth_image.encoding;
  if (depth_encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    cv_ptr = cv_bridge::toCvCopy(msg->depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
    depth_img = cv_ptr->image;
  } else if (depth_encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    cv_ptr = cv_bridge::toCvCopy(msg->depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
    depth_img = cv_ptr->image;
  } else {
    RCLCPP_ERROR(
      LOGGER,
      "Unsupported depth encoding '%s' in create_world_collision. Skipping.",
      depth_encoding.c_str());
    return;
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  for (size_t i = 0; i < msg->depth_image.width; i++) {
    for (size_t j = 0; j < msg->depth_image.height; j++) {
      pcl::PointXYZRGB temp_point;
      double depth = 0.0;
      if (depth_encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        depth = depth_img.at<uint16_t>(j, i) * kDepthMmToMeters;
      } else {
        depth = static_cast<double>(depth_img.at<float>(j, i));
      }
      temp_point.x = (i - ppx) / fx * depth;
      temp_point.y = (j - ppy) / fy * depth;
      temp_point.z = depth;
      scene_cloud->points.push_back(temp_point);
    }
  }

  PCLFunctions::passthrough_filter(
    scene_cloud,
    static_cast<float>(node->get_parameter("point_cloud_params.passthrough_filter_limits_x").
    as_double_array()[1]),
    static_cast<float>(node->get_parameter("point_cloud_params.passthrough_filter_limits_x").
    as_double_array()[0]),
    static_cast<float>(node->get_parameter("point_cloud_params.passthrough_filter_limits_y").
    as_double_array()[1]),
    static_cast<float>(node->get_parameter("point_cloud_params.passthrough_filter_limits_y").
    as_double_array()[0]),
    static_cast<float>(node->get_parameter("point_cloud_params.passthrough_filter_limits_z").
    as_double_array()[1]),
    static_cast<float>(node->get_parameter("point_cloud_params.passthrough_filter_limits_z").
    as_double_array()[0]));
  PCLFunctions::remove_statistical_outlier(scene_cloud, 1.0);

  geometry_msgs::msg::TransformStamped sensorToWorldTf =
    this->buffer_->lookupTransform(
    node->get_parameter("camera_parameters.robot_base_frame").as_string(),
    msg->header.frame_id,
    msg->header.stamp);
  octomap::point3d sensor_origin = octomap::point_tf_to_octomap(
    sensorToWorldTf.transform.translation);

  PCLFunctions::voxelize_cloud
  <pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::VoxelGrid<pcl::PointXYZRGB>>(
    scene_cloud,
    static_cast<float>(node->get_parameter(
      "point_cloud_params.fcl_voxel_size").as_double()),
    this->org_cloud);

  this->world_collision_object = FCLFunctions::create_collision_object_from_pointcloud_rgb(
    this->org_cloud, sensor_origin,
    static_cast<float>(node->get_parameter("point_cloud_params.octomap_resolution").as_double()));
}

template<typename T>
bool grasp_planner::GraspScene<T>::process_pointcloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  RCLCPP_INFO(LOGGER, "Processing Point Cloud... ");
  pcl::PCLPointCloud2 pcl_pc2;
  PCLFunctions::sensor_msg_to_pcl_pointcloud2(*msg, pcl_pc2);
  bool has_rgb_field = false;
  for (const auto & field : pcl_pc2.fields) {
    if (field.name == "rgb" || field.name == "rgba") {
      has_rgb_field = true;
      break;
    }
  }
  if (has_rgb_field) {
    pcl::fromPCLPointCloud2(pcl_pc2, *(this->cloud));
  } else {
    pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, xyz_cloud);
    this->cloud->clear();
    this->cloud->reserve(xyz_cloud.size());
    for (const auto & point : xyz_cloud.points) {
      pcl::PointXYZRGB point_rgb;
      point_rgb.x = point.x;
      point_rgb.y = point.y;
      point_rgb.z = point.z;
      point_rgb.r = 255;
      point_rgb.g = 255;
      point_rgb.b = 255;
      this->cloud->push_back(point_rgb);
    }
    RCLCPP_WARN_THROTTLE(
      LOGGER, *node->get_clock(), 10000,
      "Input cloud has no rgb/rgba field; converted XYZ points with default color. "
      "This is harmless for collision/grasp planning.");
  }
  RCLCPP_INFO(LOGGER, "Applying Passthrough filters");
  PCLFunctions::passthrough_filter(
    this->cloud,
    static_cast<float>(node->get_parameter("point_cloud_params.passthrough_filter_limits_x").
    as_double_array()[1]),
    static_cast<float>(node->get_parameter("point_cloud_params.passthrough_filter_limits_x").
    as_double_array()[0]),
    static_cast<float>(node->get_parameter("point_cloud_params.passthrough_filter_limits_y").
    as_double_array()[1]),
    static_cast<float>(node->get_parameter("point_cloud_params.passthrough_filter_limits_y").
    as_double_array()[0]),
    static_cast<float>(node->get_parameter("point_cloud_params.passthrough_filter_limits_z").
    as_double_array()[1]),
    static_cast<float>(node->get_parameter("point_cloud_params.passthrough_filter_limits_z").
    as_double_array()[0]));
  if (this->cloud->empty()) {
    RCLCPP_WARN(LOGGER, "Point cloud is empty after passthrough filtering. "
      "Check passthrough_filter_limits in config or verify the point cloud source.");
    return false;
  }
  RCLCPP_INFO(LOGGER, "Removing Statistical Outlier");
  PCLFunctions::remove_statistical_outlier(this->cloud, 1.0);
  RCLCPP_INFO(LOGGER, "Downsampling Point Cloud");
  PCLFunctions::voxelize_cloud
  <pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::VoxelGrid<pcl::PointXYZRGB>>(
    this->cloud,
    static_cast<float>(node->get_parameter(
      "point_cloud_params.fcl_voxel_size").as_double()),
    this->org_cloud);
  RCLCPP_INFO(LOGGER, "Segmenting plane");
  PCLFunctions::plane_segmentation(
    this->cloud, this->cloud_plane_removed, this->cloud_table,
    node->get_parameter(
      "point_cloud_params.segmentation_max_iterations").as_int(),
    static_cast<float>(node->get_parameter(
      "point_cloud_params.segmentation_distance_threshold").as_double()));
  RCLCPP_INFO(LOGGER, "Point cloud successfully processed!");
  return true;
}

template<>
void grasp_planner::GraspScene<sensor_msgs::msg::PointCloud2>::start_planning(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  if (execution_gate_enabled && execution_in_progress.load(std::memory_order_acquire)) {
    RCLCPP_WARN_THROTTLE(
      LOGGER,
      *node->get_clock(),
      2000,
      "Execution is in progress; skipping direct point-cloud planning callback.");
    return;
  }
  if (planning_in_progress.exchange(true, std::memory_order_acq_rel)) {
    RCLCPP_WARN_THROTTLE(
      LOGGER,
      *node->get_clock(),
      2000,
      "Skipping incoming point cloud frame because grasp planning is still in progress.");
    return;
  }

  int min_processing_period_ms = 0;
  node->get_parameter_or(
    "point_cloud_params.min_processing_period_ms",
    min_processing_period_ms,
    0);

  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  const auto last_processed_ns = last_processed_time_ns.load(std::memory_order_acquire);

  if (min_processing_period_ms > 0 && last_processed_ns > 0) {
    const auto min_period_ns = static_cast<int64_t>(min_processing_period_ms) * 1000000LL;
    const auto elapsed_ns = now_ns - last_processed_ns;
    if (elapsed_ns >= 0 && elapsed_ns < min_period_ns) {
      RCLCPP_WARN_THROTTLE(
        LOGGER,
        *node->get_clock(),
        2000,
        "Skipping incoming point cloud frame due to minimum processing period (%d ms, elapsed %lld ms).",
        min_processing_period_ms,
        static_cast<long long>(elapsed_ns / 1000000LL));
      planning_in_progress.store(false, std::memory_order_release);
      return;
    }
  }

  last_processed_time_ns.store(now_ns, std::memory_order_release);

  RCLCPP_INFO(LOGGER, "Perception input received!");
  if (!process_pointcloud(msg)) {
    RCLCPP_INFO(LOGGER, "Grasp planning skipped: point cloud was empty after filtering.");
    planning_in_progress.store(false, std::memory_order_release);
    return;
  }
  create_world_collision(msg);
  extract_objects(msg);
  // load_end_effectors();
  emd_msgs::msg::GraspTask grasp_task = generate_grasp_task();
  send_to_execution(grasp_task);
  RCLCPP_INFO(LOGGER, "Grasp Planning complete.");
  planning_in_progress.store(false, std::memory_order_release);
}

#if EPD_ENABLED == 1
template<typename T>
void grasp_planner::GraspScene<T>::start_planning(const typename T::ConstSharedPtr & msg)
{
  if (execution_gate_enabled && execution_in_progress.load(std::memory_order_acquire)) {
    RCLCPP_WARN_THROTTLE(
      LOGGER,
      *node->get_clock(),
      2000,
      "Execution is in progress; skipping EPD planning callback.");
    return;
  }
  RCLCPP_INFO(LOGGER, "Perception input received!");
  last_epd_msg_time = node->get_clock()->now();
  create_world_collision(msg);
  extract_objects(msg);
  // load_end_effectors();
  emd_msgs::msg::GraspTask grasp_task = generate_grasp_task();
  send_to_execution(grasp_task);
  RCLCPP_INFO(LOGGER, "Grasp Planning complete.");
  trigger_epd_pipeline();
}
#endif

template<>
void grasp_planner::GraspScene<sensor_msgs::msg::PointCloud2>::setup(std::string topic_name)
{
  const int64_t raw_queue_size =
    node->get_parameter("camera_parameters.tf_filter_queue_size").as_int();
  const int64_t bounded_queue_size = std::max<int64_t>(1, raw_queue_size);
  const uint32_t tf_filter_queue_size = static_cast<uint32_t>(bounded_queue_size);
  const std::string robot_base_frame =
    node->get_parameter("camera_parameters.robot_base_frame").as_string();

  this->output_client =
    this->node->create_client<emd_msgs::srv::GraspRequest>(
    this->node->get_parameter("grasp_output_service").as_string());
  node->get_parameter_or(
    "execution_in_progress_gate_enabled",
    execution_gate_enabled,
    true);
  node->get_parameter_or(
    "easy_perception_deployment.pause_epd_triggers_while_execution_in_progress",
    pause_epd_triggers_while_execution_in_progress,
    true);

  std::string selected_reliability;
  const auto perception_qos = build_perception_qos(
    node,
    "camera_parameters.point_cloud_subscription_reliability",
    selected_reliability);

  RCLCPP_INFO_STREAM(LOGGER, "Listening to: " << topic_name << "...");
  RCLCPP_INFO_STREAM(
    LOGGER,
    "Perception subscription QoS: reliability=" << selected_reliability <<
      ", depth=10");

  this->perception_sub =
    std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>();
  this->perception_sub->subscribe(node, topic_name, perception_qos.get_rmw_qos_profile());

  this->tf_perception_sub =
    std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
    *buffer_, robot_base_frame, tf_filter_queue_size,
    node->get_node_logging_interface(),
    node->get_node_clock_interface(),
    std::chrono::seconds(1));

  this->tf_perception_sub->connectInput(*perception_sub);

  this->tf_perception_sub->registerCallback(
    &grasp_planner::GraspScene<sensor_msgs::msg::PointCloud2>::start_planning, this);

  RCLCPP_INFO(LOGGER, "waiting....");
}

#if EPD_ENABLED == 1
template<typename T>
void grasp_planner::GraspScene<T>::setup(std::string topic_name)
{
  const int64_t raw_queue_size =
    node->get_parameter("camera_parameters.tf_filter_queue_size").as_int();
  const int64_t bounded_queue_size = std::max<int64_t>(1, raw_queue_size);
  const uint32_t tf_filter_queue_size = static_cast<uint32_t>(bounded_queue_size);
  const std::string robot_base_frame =
    node->get_parameter("camera_parameters.robot_base_frame").as_string();

  this->output_client =
    this->node->create_client<emd_msgs::srv::GraspRequest>(
    this->node->get_parameter("grasp_output_service").as_string());
  node->get_parameter_or(
    "execution_in_progress_gate_enabled",
    execution_gate_enabled,
    true);

  this->epd_client =
    this->node->create_client<epd_msgs::srv::Perception>(
    this->node->get_parameter("easy_perception_deployment.epd_service").as_string());

  std::string selected_reliability;
  const auto perception_qos = build_perception_qos(
    node,
    "easy_perception_deployment.epd_subscription_reliability",
    selected_reliability);

  RCLCPP_INFO(
    LOGGER,
    "Listening to: %s...",
    topic_name.c_str());
  RCLCPP_INFO(
    LOGGER,
    "Perception subscription QoS: reliability=%s, depth=10",
    selected_reliability.c_str());

  this->perception_sub = std::make_shared<message_filters::Subscriber<T>>();
  this->perception_sub->subscribe(node, topic_name, perception_qos.get_rmw_qos_profile());

  this->tf_perception_sub = std::make_shared<tf2_ros::MessageFilter<T>>(
    *buffer_, robot_base_frame, tf_filter_queue_size,
    node->get_node_logging_interface(),
    node->get_node_clock_interface(),
    std::chrono::seconds(1));

  this->tf_perception_sub->connectInput(*perception_sub);

  this->tf_perception_sub->registerCallback(
    &grasp_planner::GraspScene<T>::start_planning, this);

  last_epd_msg_time = get_current_time();
  next_epd_trigger_time = last_epd_msg_time;
  const double epd_msg_timeout_s =
    node->get_parameter("easy_perception_deployment.epd_msg_timeout_s").as_double();
  if (epd_msg_timeout_s > 0.0) {
    const auto check_period =
      std::chrono::duration<double>(std::min(epd_msg_timeout_s, 1.0));
    this->epd_msg_timer = node->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(check_period),
      [this, epd_msg_timeout_s]() {
        evaluate_epd_watchdog(get_current_time(), epd_msg_timeout_s);
      });
  }

  //First trigger to start EPD
  trigger_epd_pipeline();

  RCLCPP_INFO(LOGGER, "waiting....");
}

template<typename T>
void grasp_planner::GraspScene<T>::trigger_epd_pipeline()
{
  if (
    pause_epd_triggers_while_execution_in_progress &&
    execution_gate_enabled &&
    execution_in_progress.load(std::memory_order_acquire))
  {
    RCLCPP_INFO_THROTTLE(
      LOGGER,
      *node->get_clock(),
      2000,
      "Skipping EPD trigger while grasp execution is in progress.");
    return;
  }

  const double epd_service_wait_timeout_s = std::max(
    0.0, node->get_parameter("easy_perception_deployment.epd_service_wait_timeout_s").as_double());
  const auto wait_timeout = std::chrono::duration<double>(epd_service_wait_timeout_s);

  RCLCPP_INFO(
    LOGGER, "Checking EPD service availability (timeout %.2f s).", epd_service_wait_timeout_s);
  if (!wait_for_epd_service(wait_timeout)) {
    RCLCPP_WARN(
      LOGGER,
      "EPD service unavailable after %.2f s wait. Skipping trigger request.",
      epd_service_wait_timeout_s);
    return;
  }

  auto req = std::make_shared<epd_msgs::srv::Perception::Request>();
  set_epd_trigger_flag_if_available(*req);

  if (!this->epd_result_future.valid()) {
    RCLCPP_INFO(LOGGER, "Sending EPD trigger request (first request).");
    this->epd_result_future = send_epd_trigger_request(req);
    epd_request_sent_time = get_current_time();
    return;
  }

  if (this->epd_result_future.wait_for(std::chrono::nanoseconds(0)) == std::future_status::timeout) {
    const double request_age = (get_current_time() - epd_request_sent_time).seconds();
    const double recovery_timeout = std::max(
      0.1, node->get_parameter("easy_perception_deployment.epd_msg_timeout_s").as_double());
    if (request_age <= recovery_timeout) {
      RCLCPP_INFO(LOGGER, "EPD trigger request already in-flight; skipping duplicate trigger.");
      return;
    }
    RCLCPP_WARN(
      LOGGER,
      "EPD trigger request has been in-flight for %.2f s; abandoning it so EPD restart can recover.",
      request_age);
    this->epd_result_future = {};
  }

  if (this->epd_result_future.valid()) {
    auto result = this->epd_result_future.get();
    if (result->success) {
      RCLCPP_INFO(LOGGER, "Previous EPD trigger request succeeded. Sending next trigger request.");
    } else {
      RCLCPP_WARN(LOGGER, "Previous EPD trigger request failed. Sending retry trigger request.");
    }
  }

  this->epd_result_future = send_epd_trigger_request(req);
  epd_request_sent_time = get_current_time();
}

template<typename T>
void grasp_planner::GraspScene<T>::evaluate_epd_watchdog(
  const rclcpp::Time & now,
  double epd_msg_timeout_s)
{
  if (epd_msg_timeout_s <= 0.0) {
    return;
  }
  if (
    pause_epd_triggers_while_execution_in_progress &&
    execution_gate_enabled &&
    execution_in_progress.load(std::memory_order_acquire))
  {
    return;
  }
  if (now < next_epd_trigger_time) {
    return;
  }

  const double elapsed = (now - last_epd_msg_time).seconds();
  if (elapsed > epd_msg_timeout_s) {
    RCLCPP_WARN(
      LOGGER,
      "No EPD message received for %.2f s (timeout %.2f s). Triggering EPD pipeline.",
      elapsed, epd_msg_timeout_s);
    trigger_epd_pipeline();
    last_epd_msg_time = now;
    next_epd_trigger_time = now + rclcpp::Duration::from_seconds(epd_msg_timeout_s);
  }
}

template<typename T>
rclcpp::Time grasp_planner::GraspScene<T>::get_current_time() const
{
  return node->get_clock()->now();
}

template<typename T>
bool grasp_planner::GraspScene<T>::wait_for_epd_service(
  const std::chrono::duration<double> & timeout)
{
  return epd_client->wait_for_service(timeout);
}

template<typename T>
std::shared_future<rclcpp::Client<epd_msgs::srv::Perception>::SharedResponse>
grasp_planner::GraspScene<T>::send_epd_trigger_request(
  const std::shared_ptr<epd_msgs::srv::Perception::Request> & request)
{
  auto response = epd_client->async_send_request(request);
  return response.future.share();
}

#endif

// LCOV_EXCL_START

template<typename T>
void grasp_planner::GraspScene<T>::print_pose(
  const geometry_msgs::msg::Pose & _pose)
{
  RCLCPP_DEBUG(LOGGER, "Position:");
  RCLCPP_DEBUG(LOGGER, "X: %f", _pose.position.x);
  RCLCPP_DEBUG(LOGGER, "Y: %f", _pose.position.y);
  RCLCPP_DEBUG(LOGGER, "Z: %f", _pose.position.z);

  RCLCPP_DEBUG(LOGGER, "Orientation:");
  RCLCPP_DEBUG(LOGGER, "X: %f", _pose.orientation.x);
  RCLCPP_DEBUG(LOGGER, "Y: %f", _pose.orientation.y);
  RCLCPP_DEBUG(LOGGER, "Z: %f", _pose.orientation.z);
  RCLCPP_DEBUG(LOGGER, "W: %f", _pose.orientation.w);
}

template<typename T>
void grasp_planner::GraspScene<T>::print_pose(const geometry_msgs::msg::PoseStamped & _pose)
{
  RCLCPP_DEBUG(LOGGER, "Frame ID: %s", _pose.header.frame_id.c_str());
  print_pose(_pose.pose);
}

template<typename T>
void grasp_planner::GraspScene<T>::object_pose_rectification(
  emd_msgs::msg::GraspTask & grasp_task)
{
  for (auto & grasp_target : grasp_task.grasp_targets) {
    grasp_target.target_shape.dimensions[0] =
      std::abs(
      grasp_target.target_pose.pose.position.z -
      static_cast<float>(node->get_parameter("table_to_camera_height").as_double()));
    grasp_target.target_pose.pose.position.z += grasp_target.target_shape.dimensions[0] / 2;
  }
}

// LCOV_EXCL_STOP

template class grasp_planner::GraspScene<sensor_msgs::msg::PointCloud2>;
#if EPD_ENABLED == 1
template class grasp_planner::GraspScene<epd_msgs::msg::EPDObjectLocalization>;
template class grasp_planner::GraspScene<epd_msgs::msg::EPDObjectTracking>;
#endif
