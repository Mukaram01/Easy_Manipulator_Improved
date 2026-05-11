#pragma once
#include <string>
#include <vector>

#include "object_placement_model.hpp"

namespace workcell_builder
{
struct WorkspaceBounds { double x_min{-1.5}, x_max{1.5}, y_min{-1.5}, y_max{1.5}, z_min{0.0}, z_max{2.0}; std::string source{"default"}; };
struct ReachEnvelope { double base_x{0.0}, base_y{0.0}, inner_radius_m{0.15}, outer_radius_m{0.85}; bool known{false}; };
struct SafetyZone { std::string name; std::string type; std::string shape; double center_x{0.0}, center_y{0.0}, radius{0.0}; double origin_x{0.0}, origin_y{0.0}, size_x{0.0}, size_y{0.0}; };
struct ObjectFootprint { std::string name; double x{0.0}, y{0.0}, width{0.6}, height{0.4}, z{0.0}; };
struct CameraFootprint { std::string camera_id; double x{0.0}, y{0.0}, z{0.0}; bool enabled{false}; };
struct ReadinessOverlayIssue { std::string status; std::string message; std::string object_name; };
struct ReadinessOverlayResult { std::string readiness_overlay_status{"UNKNOWN"}; std::vector<ReadinessOverlayIssue> issues; int blocker_count{0}; int warning_count{0}; };
ReadinessOverlayResult evaluate_offline_readiness_overlay(
  const std::vector<ObjectFootprint> & objects,
  const ReachEnvelope & reach,
  const WorkspaceBounds & workspace,
  const std::vector<SafetyZone> & safety_zones,
  const std::vector<CameraFootprint> & cameras,
  const std::string & pick_source,
  const std::string & place_target,
  const std::string & compatibility_status,
  const std::string & tcp_frame,
  const std::string & tool_mount_link,
  const std::string & camera_topic);

ReachEnvelope estimate_robot_reach_envelope(const std::string & robot_id);
ObjectFootprint estimate_object_footprint(const PlacedObject & object);
CameraFootprint estimate_camera_footprint(const std::string & camera_id, double x, double y, double z, bool enabled);
std::vector<ReadinessOverlayIssue> evaluate_reach_warnings(const std::vector<ObjectFootprint> &, const ReachEnvelope &);
std::vector<ReadinessOverlayIssue> evaluate_workspace_bounds(const std::vector<ObjectFootprint> &, const WorkspaceBounds &);
std::vector<ReadinessOverlayIssue> evaluate_simple_overlap_warnings(const std::vector<ObjectFootprint> &);
std::vector<ReadinessOverlayIssue> evaluate_camera_placement_warnings(const CameraFootprint &, const WorkspaceBounds &);
std::vector<ReadinessOverlayIssue> evaluate_task_pick_place_reach(const std::string &, const std::string &, const std::vector<ObjectFootprint> &, const ReachEnvelope &);
std::string readiness_overlay_status_label(const ReadinessOverlayResult & result);
}
