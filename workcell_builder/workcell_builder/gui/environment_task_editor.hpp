#ifndef WORKCELL_BUILDER__ENVIRONMENT_TASK_EDITOR_HPP_
#define WORKCELL_BUILDER__ENVIRONMENT_TASK_EDITOR_HPP_

#include "layout_item_serializer.hpp"
#include <cmath>
#include <stdexcept>

namespace workcell_builder
{
// Editable intent only. Live observations, geometry and robot execution state never enter this model.
struct EnvironmentTaskEdits
{
  std::string target_class, grasp_intent, home_pose;
  double min_confidence{0.0}, max_age_seconds{2.0};
  double approach_distance_m{0.12}, retreat_distance_m{0.10};
  std::array<double, 3> placement_rpy{};
};

inline EnvironmentTaskEdits read_environment_task_edits(const YAML::Node & environment)
{
  const YAML::Node task = environment["task"];
  if (!task || !task.IsMap()) throw std::runtime_error("environment.yaml has no authored task map");
  const YAML::Node filter = task["object_filter"], grasp = task["grasp"];
  EnvironmentTaskEdits edits;
  if (filter && filter.IsMap()) {
    edits.target_class = filter["class_id"].as<std::string>("");
    edits.min_confidence = filter["min_confidence"].as<double>(0.0);
    edits.max_age_seconds = filter["max_age_seconds"].as<double>(2.0);
  }
  if (grasp && grasp.IsMap()) {
    edits.grasp_intent = grasp["intent"].as<std::string>(grasp["strategy"].as<std::string>(""));
    edits.approach_distance_m = grasp["approach_distance_m"].as<double>(0.12);
    edits.retreat_distance_m = grasp["retreat_distance_m"].as<double>(0.10);
  }
  edits.home_pose = task["home_pose"].as<std::string>("");
  const YAML::Node destinations = task["destinations"];
  const std::string destination_id = task["place"] && task["place"].IsMap() ?
    task["place"]["target_ref"].as<std::string>("") : "";
  if (destinations && destinations.IsSequence()) {
    for (const YAML::Node & destination : destinations) {
      if (destination["id"].as<std::string>("") != destination_id) continue;
      const YAML::Node rpy = destination["pose_rpy"];
      if (rpy && rpy.IsSequence() && rpy.size() == 3)
        for (std::size_t i = 0; i < 3; ++i) edits.placement_rpy[i] = rpy[i].as<double>();
    }
  }
  return edits;
}

inline YAML::Node apply_environment_task_edits(
  const YAML::Node & environment, const EnvironmentTaskEdits & edits)
{
  if (edits.target_class.empty() || edits.home_pose.empty() || edits.grasp_intent.empty())
    throw std::runtime_error("Target class, grasp intent and home pose reference are required");
  for (double value : {edits.min_confidence, edits.max_age_seconds,
      edits.approach_distance_m, edits.retreat_distance_m,
      edits.placement_rpy[0], edits.placement_rpy[1], edits.placement_rpy[2]})
    if (!std::isfinite(value)) throw std::runtime_error("Task values must be finite");
  if (edits.min_confidence < 0.0 || edits.min_confidence > 1.0 ||
      edits.max_age_seconds <= 0.0 || edits.approach_distance_m <= 0.0 || edits.retreat_distance_m <= 0.0)
    throw std::runtime_error("Confidence must be 0–1; freshness, approach and retreat must be positive");
  YAML::Node out = YAML::Clone(environment);
  YAML::Node task = out["task"];
  if (!task || !task.IsMap()) throw std::runtime_error("environment.yaml has no authored task map");
  const std::string destination_id = task["place"]["target_ref"].as<std::string>("");
  bool destination_found = false;
  if (task["destinations"] && task["destinations"].IsSequence()) {
    for (YAML::Node destination : task["destinations"]) {
      if (destination["id"].as<std::string>("") != destination_id) continue;
      destination["pose_rpy"] = layout_sequence3(edits.placement_rpy);
      destination_found = true;
    }
  }
  if (!destination_found) throw std::runtime_error("Task place.target_ref must name an authored destination before editing its orientation");
  task["object_filter"]["class_id"] = edits.target_class;
  // A zero value in the compact editor means that the source does not expose
  // confidence scores. Persist that as YAML null so the runtime gate does not
  // reject otherwise valid EPD localization messages.
  task["object_filter"]["min_confidence"] =
    edits.min_confidence > 0.0 ? YAML::Node(edits.min_confidence) : YAML::Node();
  task["object_filter"]["max_age_seconds"] = edits.max_age_seconds;
  task["grasp"]["intent"] = edits.grasp_intent;
  task["grasp"]["approach_distance_m"] = edits.approach_distance_m;
  task["grasp"]["retreat_distance_m"] = edits.retreat_distance_m;
  task["home_pose"] = edits.home_pose;
  return out;
}
}  // namespace workcell_builder
#endif
