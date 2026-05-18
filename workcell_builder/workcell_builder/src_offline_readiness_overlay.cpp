#include "offline_readiness_overlay.hpp"
#include <algorithm>
#include <cmath>

namespace workcell_builder
{
static ReadinessOverlayIssue make_issue(const std::string & status, const std::string & message, const std::string & object_name, const std::string & code, const std::string & severity, const std::vector<std::string> & asset_ids = {})
{
  ReadinessOverlayIssue issue;
  issue.status = status;
  issue.message = message;
  issue.object_name = object_name;
  issue.code = code;
  issue.severity = severity;
  issue.asset_ids = asset_ids;
  return issue;
}

static bool overlaps(const ObjectFootprint & a, const ObjectFootprint & b) {
  return std::fabs(a.x - b.x) < ((a.width + b.width) / 2.0) && std::fabs(a.y - b.y) < ((a.height + b.height) / 2.0);
}

static double overlap_ratio(const ObjectFootprint & a, const ObjectFootprint & b)
{
  const double ax0 = a.x - (a.width / 2.0), ax1 = a.x + (a.width / 2.0);
  const double ay0 = a.y - (a.height / 2.0), ay1 = a.y + (a.height / 2.0);
  const double bx0 = b.x - (b.width / 2.0), bx1 = b.x + (b.width / 2.0);
  const double by0 = b.y - (b.height / 2.0), by1 = b.y + (b.height / 2.0);
  const double ix = std::max(0.0, std::min(ax1, bx1) - std::max(ax0, bx0));
  const double iy = std::max(0.0, std::min(ay1, by1) - std::max(ay0, by0));
  const double intersection = ix * iy;
  const double min_area = std::max(1e-6, std::min(a.width * a.height, b.width * b.height));
  return intersection / min_area;
}
ReachEnvelope estimate_robot_reach_envelope(const std::string & robot_id){ ReachEnvelope e; if (robot_id=="ur5") {e.known=true; e.outer_radius_m=0.85; e.inner_radius_m=0.15;} return e; }
ObjectFootprint estimate_object_footprint(const PlacedObject & o){ ObjectFootprint f; f.name=o.name; f.x=o.x; f.y=o.y; f.z=o.z; return f; }
CameraFootprint estimate_camera_footprint(const std::string & id,double x,double y,double z,bool enabled){ CameraFootprint c; c.camera_id=id; c.x=x; c.y=y; c.z=z; c.enabled=enabled; return c; }
std::vector<ReadinessOverlayIssue> evaluate_reach_warnings(const std::vector<ObjectFootprint> & objs, const ReachEnvelope & env){ std::vector<ReadinessOverlayIssue> out; if(!env.known){ out.push_back(make_issue("UNKNOWN","Robot reach envelope unknown","","READINESS_REACH_UNKNOWN","warning")); return out;} for (const auto & o:objs){ double r=std::hypot(o.x-env.base_x,o.y-env.base_y); if(r>env.outer_radius_m||r<env.inner_radius_m) out.push_back(make_issue("OUTSIDE_REACH","Reach Warning",o.name,"OBJECT_OUTSIDE_REACH","warning",{o.name})); } return out; }
std::vector<ReadinessOverlayIssue> evaluate_workspace_bounds(const std::vector<ObjectFootprint> & objs, const WorkspaceBounds & ws){ std::vector<ReadinessOverlayIssue> out; for (const auto & o:objs){ if(o.x<ws.x_min||o.x>ws.x_max||o.y<ws.y_min||o.y>ws.y_max||o.z<ws.z_min||o.z>ws.z_max) out.push_back(make_issue("OUTSIDE_WORKSPACE","Workspace Warning",o.name,"OBJECT_OUTSIDE_WORKSPACE","warning",{o.name})); if(o.z < -0.005) out.push_back(make_issue("WARN","object z below floor/support threshold",o.name,"OBJECT_Z_BELOW_SUPPORT","warning",{o.name})); if(o.z>3.0) out.push_back(make_issue("WARN","suspicious object z height",o.name,"OBJECT_Z_SUSPICIOUS","warning",{o.name})); } return out; }
std::vector<ReadinessOverlayIssue> evaluate_simple_overlap_warnings(const std::vector<ObjectFootprint> & objs){ std::vector<ReadinessOverlayIssue> out; constexpr double kMinOverlapRatio = 0.20; for(size_t i=0;i<objs.size();++i) for(size_t j=i+1;j<objs.size();++j) if(overlaps(objs[i],objs[j]) && overlap_ratio(objs[i], objs[j]) >= kMinOverlapRatio) out.push_back(make_issue("OVERLAP_WARNING","Overlap Warning",objs[i].name+" vs "+objs[j].name,"ASSET_FOOTPRINT_OVERLAP","warning",{objs[i].name, objs[j].name})); return out; }
std::vector<ReadinessOverlayIssue> evaluate_camera_placement_warnings(const CameraFootprint & c, const WorkspaceBounds & ws){ std::vector<ReadinessOverlayIssue> out; if(!c.enabled)return out; if(c.x<ws.x_min||c.x>ws.x_max||c.y<ws.y_min||c.y>ws.y_max) out.push_back(make_issue("CAMERA_WARNING","Camera Warning outside workspace",c.camera_id,"CAMERA_OUTSIDE_WORKSPACE","warning",{c.camera_id})); if(c.z<0.2||c.z>3.0) out.push_back(make_issue("CAMERA_WARNING","Camera Warning suspicious z",c.camera_id,"CAMERA_Z_SUSPICIOUS","warning",{c.camera_id})); return out; }
std::vector<ReadinessOverlayIssue> evaluate_task_pick_place_reach(const std::string & pick, const std::string & place, const std::vector<ObjectFootprint> & objs, const ReachEnvelope & env){ std::vector<ReadinessOverlayIssue> out; if(pick.empty()||place.empty()) out.push_back(make_issue("TASK_TARGET_WARNING","task pick/place missing","","TASK_TARGET_MISSING","warning")); for (const auto & o:objs){ if(o.name==pick||o.name==place){ double r=std::hypot(o.x-env.base_x,o.y-env.base_y); if(env.known && r>env.outer_radius_m) out.push_back(make_issue("TASK_TARGET_WARNING","task target outside reach",o.name,"TASK_TARGET_OUTSIDE_REACH","warning",{o.name})); }} return out; }
static std::vector<ReadinessOverlayIssue> evaluate_safety_zone_warnings(const std::vector<ObjectFootprint> & objs, const std::vector<SafetyZone> & zones){ std::vector<ReadinessOverlayIssue> out; for (const auto & o:objs){ for (const auto & z:zones){ if(z.type=="robot_base_exclusion" && z.shape=="circle"){ const double r=std::hypot(o.x-z.center_x,o.y-z.center_y); if(r<z.radius){ out.push_back(make_issue("SAFETY_ZONE_WARNING","object overlaps robot base exclusion zone",o.name,"SAFETY_ZONE_CONFLICT","severe",{o.name})); } } } } return out; }
ReadinessOverlayResult evaluate_offline_readiness_overlay(const std::vector<ObjectFootprint> & objects, const ReachEnvelope & reach, const WorkspaceBounds & workspace, const std::vector<SafetyZone> & safety_zones, const std::vector<CameraFootprint> & cameras, const std::string & pick_source, const std::string & place_target, const std::string & compatibility_status, const std::string & tcp_frame, const std::string & tool_mount_link, const std::string & camera_topic){ ReadinessOverlayResult result; auto append=[&result](const std::vector<ReadinessOverlayIssue>& issues, bool blocker=false){ for(const auto & i:issues){ result.issues.push_back(i); if(blocker || i.severity=="severe" || i.status=="BLOCKER") result.blocker_count++; else result.warning_count++; } }; append(evaluate_reach_warnings(objects, reach)); append(evaluate_workspace_bounds(objects, workspace)); append(evaluate_simple_overlap_warnings(objects)); append(evaluate_task_pick_place_reach(pick_source, place_target, objects, reach)); append(evaluate_safety_zone_warnings(objects, safety_zones)); for (const auto & c : cameras) append(evaluate_camera_placement_warnings(c, workspace)); if((pick_source.empty() || place_target.empty()) && !cameras.empty()) { for (const auto & c : cameras) { if (!c.enabled) continue; result.issues.push_back(make_issue("CAMERA_WARNING", "camera has no target/work area association", c.camera_id, "CAMERA_NO_TARGET_ASSOCIATION", "warning", {c.camera_id})); result.warning_count++; } } if(camera_topic.empty()) { result.issues.push_back(make_issue("CAMERA_WARNING","missing camera topic warnings","camera_topic","CAMERA_TOPIC_MISSING","warning",{"camera_topic"})); result.warning_count++; } if(tcp_frame.empty()) { result.issues.push_back(make_issue("TASK_TARGET_WARNING","unknown TCP/mount link warnings","MISSING_TCP","TCP_FRAME_MISSING","warning",{"MISSING_TCP"})); result.warning_count++; } if(tool_mount_link.empty()) { result.issues.push_back(make_issue("TASK_TARGET_WARNING","unknown TCP/mount link warnings","MISSING_MOUNT_LINK","TOOL_MOUNT_LINK_MISSING","warning",{"MISSING_MOUNT_LINK"})); result.warning_count++; } if(compatibility_status=="INCOMPATIBLE") { result.issues.push_back(make_issue("BLOCKER","incompatible robot/tool blocker",compatibility_status,"ROBOT_TOOL_INCOMPATIBLE","severe",{compatibility_status})); result.blocker_count++; }
  result.readiness_overlay_status = readiness_overlay_status_label(result);
  return result;
}
// SAFETY_ZONE_WARNING
std::string readiness_overlay_status_label(const ReadinessOverlayResult & r){ if(r.blocker_count>0) return "BLOCKED"; if(r.warning_count>0) return "WARN"; return r.issues.empty()?"READY":"UNKNOWN"; }
}
