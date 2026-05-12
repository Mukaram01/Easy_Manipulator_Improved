#include "grasp_strategy_model.hpp"

#include <boost/filesystem.hpp>
#include <fstream>
#include <sstream>
#include <yaml-cpp/yaml.h>

namespace fs = boost::filesystem;
namespace workcell_builder {
GraspStrategyTable parse_grasp_strategy_yaml(const std::string & yaml_text) {
  GraspStrategyTable t;
  const YAML::Node root = YAML::Load(yaml_text)["grasp_strategy"];
  if (!root) return t;
  if (root["schema_version"]) t.schema_version = root["schema_version"].as<int>();
  if (root["runtime_mode"]) t.runtime_mode = root["runtime_mode"].as<std::string>();
  if (root["default_strategy"]) t.default_strategy = root["default_strategy"].as<std::string>();
  if (root["strategies"]) for (const auto & s : root["strategies"]) {
    GraspStrategyRule r;
    if (s["class_label"]) r.class_label = s["class_label"].as<std::string>();
    if (s["end_effector_type"]) r.end_effector_type = s["end_effector_type"].as<std::string>();
    if (s["strategy"]) r.strategy = s["strategy"].as<std::string>();
    if (s["approach_axis"]) r.approach_axis = s["approach_axis"].as<std::string>();
    if (s["approach_distance_m"]) r.approach_distance_m = s["approach_distance_m"].as<double>();
    if (s["retreat_axis"]) r.retreat_axis = s["retreat_axis"].as<std::string>();
    if (s["retreat_distance_m"]) r.retreat_distance_m = s["retreat_distance_m"].as<double>();
    if (s["preferred_pose_source"]) r.preferred_pose_source = s["preferred_pose_source"].as<std::string>();
    if (s["required_inputs"]) for (const auto & i : s["required_inputs"]) r.required_inputs.push_back(i.as<std::string>());
    t.strategies.push_back(r);
  }
  return t;
}
std::string serialize_grasp_strategy_yaml(const GraspStrategyTable & t) {
  YAML::Emitter out;
  out << YAML::BeginMap << YAML::Key << "grasp_strategy" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "schema_version" << YAML::Value << t.schema_version;
  out << YAML::Key << "runtime_mode" << YAML::Value << t.runtime_mode;
  out << YAML::Key << "default_strategy" << YAML::Value << t.default_strategy;
  out << YAML::Key << "strategies" << YAML::Value << YAML::BeginSeq;
  for (const auto & r : t.strategies) {
    out << YAML::BeginMap << YAML::Key << "class_label" << YAML::Value << r.class_label
        << YAML::Key << "end_effector_type" << YAML::Value << r.end_effector_type
        << YAML::Key << "strategy" << YAML::Value << r.strategy
        << YAML::Key << "approach_axis" << YAML::Value << r.approach_axis
        << YAML::Key << "approach_distance_m" << YAML::Value << r.approach_distance_m
        << YAML::Key << "retreat_axis" << YAML::Value << r.retreat_axis
        << YAML::Key << "retreat_distance_m" << YAML::Value << r.retreat_distance_m
        << YAML::Key << "preferred_pose_source" << YAML::Value << r.preferred_pose_source;
    if (!r.required_inputs.empty()) out << YAML::Key << "required_inputs" << YAML::Value << r.required_inputs;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq << YAML::EndMap << YAML::EndMap;
  return out.c_str();
}
GraspStrategyReadinessResult validate_grasp_strategy_table(const GraspStrategyTable & t){GraspStrategyReadinessResult r; if(t.strategies.empty()) r.errors.push_back("ERROR: no grasp strategies configured"); if(t.default_strategy.empty()) r.warnings.push_back("WARN: no default strategy set"); r.infos={"EPD provides perception only","EMD owns grasp planning","request artifact only","no planner call yet","no execution call","fake hardware recommended"}; r.readiness_status=r.errors.empty()?(!r.warnings.empty()?"WARN":"OK"):"ERROR"; return r;}
GraspStrategyRule select_grasp_strategy_for_detection(const GraspStrategyTable & t,const std::string & object_class,const std::string & end_effector_type,bool * used_default){ if (used_default) *used_default=false; for(const auto & r:t.strategies){ if(r.class_label==object_class && r.end_effector_type==end_effector_type) return r; } if (used_default) *used_default=true; GraspStrategyRule fallback; fallback.class_label=object_class; fallback.end_effector_type=end_effector_type; fallback.strategy=t.default_strategy.empty()?"generic_top_down":t.default_strategy; return fallback; }
GraspStrategyReadinessResult validate_grasp_strategy_compatibility(const GraspStrategyRule & rule, const std::string & end_effector_type){ GraspStrategyReadinessResult r; r.selected_strategy=rule.strategy; r.end_effector_compatible=(rule.end_effector_type.empty()||rule.end_effector_type==end_effector_type); if(!r.end_effector_compatible) r.errors.push_back("ERROR: selected strategy incompatible with end effector type"); r.readiness_status=r.errors.empty()?"OK":"ERROR"; return r; }
EmdGraspPlannerRequest build_emd_grasp_planner_request(const GraspStrategyRule & rule,const std::string & robot,const std::string & end_effector,const std::string & end_effector_type,const std::string & object_class,const std::string & object_pose_source,const std::string & pick_zone,const std::string & place_zone,bool segmented_cloud_available,bool has_point_cloud_source){EmdGraspPlannerRequest req; req.robot=robot; req.end_effector=end_effector; req.end_effector_type=end_effector_type; req.object_class=object_class; req.object_pose_source=object_pose_source; req.pick_zone=pick_zone; req.place_zone=place_zone; req.grasp_strategy=rule.strategy; req.approach_distance_m=rule.approach_distance_m; req.retreat_distance_m=rule.retreat_distance_m; req.segmented_cloud_available=segmented_cloud_available; req.can_call_grasp_planner_later=!robot.empty()&&!end_effector.empty()&&!object_class.empty()&&!pick_zone.empty()&&!object_pose_source.empty()&&has_point_cloud_source; if (end_effector_type=="finger") req.emd_planner_config_file=(end_effector.find("3f")!=std::string::npos)?"params_3f.yaml":"params_2f.yaml"; else if (end_effector_type=="suction") req.emd_planner_config_file=(end_effector.find("airpick")!=std::string::npos)?"params_airpick4.yaml":"params_suction.yaml"; return req; }
std::string serialize_emd_grasp_planner_request_yaml(const EmdGraspPlannerRequest & r){ YAML::Emitter out; out<<YAML::BeginMap<<YAML::Key<<"emd_grasp_planner_request"<<YAML::Value<<YAML::BeginMap; out<<YAML::Key<<"schema_version"<<YAML::Value<<r.schema_version<<YAML::Key<<"runtime_mode"<<YAML::Value<<r.runtime_mode<<YAML::Key<<"source"<<YAML::Value<<r.source<<YAML::Key<<"planner_backend"<<YAML::Value<<r.planner_backend<<YAML::Key<<"perception_source"<<YAML::Value<<r.perception_source<<YAML::Key<<"robot"<<YAML::Value<<r.robot<<YAML::Key<<"end_effector"<<YAML::Value<<r.end_effector<<YAML::Key<<"end_effector_type"<<YAML::Value<<r.end_effector_type<<YAML::Key<<"object_class"<<YAML::Value<<r.object_class<<YAML::Key<<"object_pose_source"<<YAML::Value<<r.object_pose_source<<YAML::Key<<"pick_zone"<<YAML::Value<<r.pick_zone<<YAML::Key<<"place_zone"<<YAML::Value<<r.place_zone<<YAML::Key<<"grasp_strategy"<<YAML::Value<<r.grasp_strategy<<YAML::Key<<"approach_distance_m"<<YAML::Value<<r.approach_distance_m<<YAML::Key<<"retreat_distance_m"<<YAML::Value<<r.retreat_distance_m<<YAML::Key<<"point_cloud_required"<<YAML::Value<<true<<YAML::Key<<"segmented_cloud_available"<<YAML::Value<<r.segmented_cloud_available<<YAML::Key<<"can_call_grasp_planner_later"<<YAML::Value<<r.can_call_grasp_planner_later<<YAML::Key<<"robot_motion_commanded"<<YAML::Value<<false<<YAML::Key<<"gripper_command_sent"<<YAML::Value<<false<<YAML::Key<<"moveit_execute_called"<<YAML::Value<<false<<YAML::Key<<"grasp_execution_called"<<YAML::Value<<false<<YAML::Key<<"emd_planner_config_file"<<YAML::Value<<r.emd_planner_config_file; out<<YAML::EndMap<<YAML::EndMap; return out.c_str(); }
std::string serialize_emd_grasp_planner_request_json(const EmdGraspPlannerRequest & r){ std::ostringstream s; s<<"{\n  \"emd_grasp_planner_request\": {\n    \"planner_backend\": \""<<r.planner_backend<<"\",\n    \"robot\": \""<<r.robot<<"\",\n    \"end_effector\": \""<<r.end_effector<<"\",\n    \"grasp_strategy\": \""<<r.grasp_strategy<<"\",\n    \"robot_motion_commanded\": false,\n    \"gripper_command_sent\": false,\n    \"moveit_execute_called\": false,\n    \"grasp_execution_called\": false\n  }\n}"; return s.str(); }
void write_emd_grasp_planner_request_artifacts(const std::string & out_dir,const GraspStrategyTable & table,const EmdGraspPlannerRequest & request,const GraspStrategyReadinessResult & readiness){ fs::create_directories(out_dir); std::ofstream(out_dir+"/grasp_strategy.yaml")<<serialize_grasp_strategy_yaml(table); std::ofstream(out_dir+"/emd_grasp_planner_request.yaml")<<serialize_emd_grasp_planner_request_yaml(request); std::ofstream(out_dir+"/emd_grasp_planner_request.json")<<serialize_emd_grasp_planner_request_json(request); YAML::Emitter y; y<<YAML::BeginMap<<YAML::Key<<"grasp_strategy_readiness_report"<<YAML::Value<<YAML::BeginMap<<YAML::Key<<"readiness_status"<<YAML::Value<<readiness.readiness_status<<YAML::Key<<"selected_grasp_strategy"<<YAML::Value<<readiness.selected_strategy<<YAML::Key<<"end_effector_compatible"<<YAML::Value<<readiness.end_effector_compatible<<YAML::Key<<"errors"<<YAML::Value<<readiness.errors<<YAML::Key<<"warnings"<<YAML::Value<<readiness.warnings<<YAML::Key<<"infos"<<YAML::Value<<readiness.infos<<YAML::EndMap<<YAML::EndMap; std::ofstream(out_dir+"/grasp_strategy_readiness_report.yaml")<<y.c_str(); std::ofstream(out_dir+"/grasp_strategy_readiness_report.json")<<"{\"grasp_strategy_readiness_report\":{\"readiness_status\":\""<<readiness.readiness_status<<"\"}}"; }
}
