#include "offline_readiness_overlay.hpp"
#include <cmath>

namespace workcell_builder
{
static bool overlaps(const ObjectFootprint & a, const ObjectFootprint & b) {
  return std::fabs(a.x - b.x) < ((a.width + b.width) / 2.0) && std::fabs(a.y - b.y) < ((a.height + b.height) / 2.0);
}
ReachEnvelope estimate_robot_reach_envelope(const std::string & robot_id){ ReachEnvelope e; if (robot_id=="ur5") {e.known=true; e.outer_radius_m=0.85; e.inner_radius_m=0.15;} return e; }
ObjectFootprint estimate_object_footprint(const PlacedObject & o){ ObjectFootprint f; f.name=o.name; f.x=o.x; f.y=o.y; f.z=o.z; return f; }
CameraFootprint estimate_camera_footprint(const std::string & id,double x,double y,double z,bool enabled){ CameraFootprint c; c.camera_id=id; c.x=x; c.y=y; c.z=z; c.enabled=enabled; return c; }
std::vector<ReadinessOverlayIssue> evaluate_reach_warnings(const std::vector<ObjectFootprint> & objs, const ReachEnvelope & env){ std::vector<ReadinessOverlayIssue> out; if(!env.known){ out.push_back({"UNKNOWN","Robot reach envelope unknown",""}); return out;} for (const auto & o:objs){ double r=std::hypot(o.x-env.base_x,o.y-env.base_y); if(r>env.outer_radius_m||r<env.inner_radius_m) out.push_back({"OUTSIDE_REACH","Reach Warning",o.name}); } return out; }
std::vector<ReadinessOverlayIssue> evaluate_workspace_bounds(const std::vector<ObjectFootprint> & objs, const WorkspaceBounds & ws){ std::vector<ReadinessOverlayIssue> out; for (const auto & o:objs){ if(o.x<ws.x_min||o.x>ws.x_max||o.y<ws.y_min||o.y>ws.y_max||o.z<ws.z_min||o.z>ws.z_max) out.push_back({"OUTSIDE_WORKSPACE","Workspace Warning",o.name}); if(o.z<-0.01||o.z>3.0) out.push_back({"WARN","suspicious object z height",o.name}); } return out; }
std::vector<ReadinessOverlayIssue> evaluate_simple_overlap_warnings(const std::vector<ObjectFootprint> & objs){ std::vector<ReadinessOverlayIssue> out; for(size_t i=0;i<objs.size();++i) for(size_t j=i+1;j<objs.size();++j) if(overlaps(objs[i],objs[j])) out.push_back({"OVERLAP_WARNING","Overlap Warning",objs[i].name+" vs "+objs[j].name}); return out; }
std::vector<ReadinessOverlayIssue> evaluate_camera_placement_warnings(const CameraFootprint & c, const WorkspaceBounds & ws){ std::vector<ReadinessOverlayIssue> out; if(!c.enabled)return out; if(c.x<ws.x_min||c.x>ws.x_max||c.y<ws.y_min||c.y>ws.y_max) out.push_back({"CAMERA_WARNING","Camera Warning outside workspace",c.camera_id}); if(c.z<0.2||c.z>3.0) out.push_back({"CAMERA_WARNING","Camera Warning suspicious z",c.camera_id}); return out; }
std::vector<ReadinessOverlayIssue> evaluate_task_pick_place_reach(const std::string & pick, const std::string & place, const std::vector<ObjectFootprint> & objs, const ReachEnvelope & env){ std::vector<ReadinessOverlayIssue> out; if(pick.empty()||place.empty()) out.push_back({"TASK_TARGET_WARNING","task pick/place missing",""}); for (const auto & o:objs){ if(o.name==pick||o.name==place){ double r=std::hypot(o.x-env.base_x,o.y-env.base_y); if(env.known && r>env.outer_radius_m) out.push_back({"TASK_TARGET_WARNING","task target outside reach",o.name}); }} return out; }
// SAFETY_ZONE_WARNING
std::string readiness_overlay_status_label(const ReadinessOverlayResult & r){ if(r.blocker_count>0) return "BLOCKED"; if(r.warning_count>0) return "WARN"; return r.issues.empty()?"READY":"UNKNOWN"; }
}
