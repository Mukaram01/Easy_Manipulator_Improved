#include "robot_tool_compatibility.hpp"

#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

namespace {

std::string normalize(std::string value)
{
  for (auto & c : value) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return value;
}

std::string infer_robot_id(const Scene & scene)
{
  if (!scene.robot_loaded || scene.robot_vector.empty()) { return "generic_unknown_robot"; }
  const std::string key = normalize(scene.robot_vector[0].name);
  if (key.find("ur5") != std::string::npos || key.find("ur_") != std::string::npos) { return "ur5"; }
  return "generic_unknown_robot";
}

std::string infer_tool_id(const Scene & scene)
{
  if (!scene.ee_loaded || scene.ee_vector.empty()) { return "generic_unknown_tool"; }

  const EndEffector & ee = scene.ee_vector[0];
  std::string key = ee.name;
  key += " " + ee.brand;
  key += " " + ee.ee_type;
  key += " " + ee.gripper_type;
  key += " " + ee.planner_id;
  key += " " + ee.base_link;
  key += " " + ee.robot_link;
  key += " " + ee.grasp_frame;
  key += " " + ee.tcp_link;
  key = normalize(key);

  if (key.find("robotiq") != std::string::npos ||
    key.find("2f") != std::string::npos ||
    key.find("finger") != std::string::npos ||
    key.find("85") != std::string::npos)
  {
    return "robotiq_2f85";
  }
  if (key.find("airpick") != std::string::npos ||
    key.find("suction") != std::string::npos ||
    key.find("vacuum") != std::string::npos ||
    key.find("cup") != std::string::npos)
  {
    return "onrobot_airpick";
  }
  return "generic_unknown_tool";
}

QJsonObject read_json(const QString & path)
{
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) { return QJsonObject(); }
  const QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
  return doc.isObject() ? doc.object() : QJsonObject();
}

}  // namespace

std::map<std::string, RobotProfile> load_robot_profiles(const std::string & config_root)
{
  std::map<std::string, RobotProfile> out;
  const QDir dir(QString::fromStdString(config_root + "/robots"));
  for (const QString & file : dir.entryList(QStringList() << "*.json", QDir::Files)) {
    const QJsonObject obj = read_json(dir.absoluteFilePath(file));
    RobotProfile p;
    p.robot_id = obj.value("robot_id").toString().toStdString();
    p.label = obj.value("label").toString().toStdString();
    p.description_package = obj.value("description_package").toString().toStdString();
    p.moveit_config_package = obj.value("moveit_config_package").toString().toStdString();
    p.base_link = obj.value("base_link").toString().toStdString();
    p.planning_group = obj.value("planning_group").toString().toStdString();
    p.default_tool_mount_link = obj.value("default_tool_mount_link").toString().toStdString();
    p.controller_family = obj.value("controller_family").toString().toStdString();
    p.robot_family = obj.value("robot_family").toString("unknown").toStdString();
    p.real_driver_required = obj.value("real_driver_required").toBool(true);
    p.driver_package_hint = obj.value("driver_package_hint").toString().toStdString();
    p.controller_type_hint = obj.value("controller_type_hint").toString().toStdString();
    p.ros2_control_required = obj.value("ros2_control_required").toBool(true);
    p.network_required = obj.value("network_required").toBool(true);
    p.calibration_required = obj.value("calibration_required").toBool(false);
    p.supported_in_simulation = obj.value("supported_in_simulation").toBool(true);
    p.supported_on_real_hardware = obj.value("supported_on_real_hardware").toString("unknown").toStdString();
    p.deployment_notes = obj.value("deployment_notes").toString().toStdString();
    for (const auto & v : obj.value("supported_tool_types").toArray()) { p.supported_tool_types.push_back(v.toString().toStdString()); }
    out[p.robot_id] = p;
  }
  return out;
}

std::map<std::string, ToolProfile> load_tool_profiles(const std::string & config_root)
{
  std::map<std::string, ToolProfile> out;
  const QDir dir(QString::fromStdString(config_root + "/tools"));
  for (const QString & file : dir.entryList(QStringList() << "*.json", QDir::Files)) {
    const QJsonObject obj = read_json(dir.absoluteFilePath(file));
    ToolProfile p;
    p.tool_id = obj.value("tool_id").toString().toStdString();
    p.label = obj.value("label").toString().toStdString();
    p.tool_type = obj.value("tool_type").toString().toStdString();
    p.mount_link = obj.value("mount_link").toString().toStdString();
    p.tcp_frame = obj.value("tcp_frame").toString().toStdString();
    p.controller_hint = obj.value("controller_hint").toString().toStdString();
    p.grasp_strategy_default = obj.value("grasp_strategy_default").toString().toStdString();
    p.release_strategy_default = obj.value("release_strategy_default").toString().toStdString();
    p.requires_io = obj.value("requires_io").toBool(false);
    p.io_required = obj.value("io_required").toBool(p.requires_io);
    p.io_type = obj.value("io_type").toString("unknown").toStdString();
    p.open_command_hint = obj.value("open_command_hint").toString().toStdString();
    p.close_command_hint = obj.value("close_command_hint").toString().toStdString();
    p.release_command_hint = obj.value("release_command_hint").toString().toStdString();
    p.real_hardware_io_mapping_required = obj.value("real_hardware_io_mapping_required").toBool(false);
    p.deployment_notes = obj.value("deployment_notes").toString().toStdString();
    out[p.tool_id] = p;
  }
  return out;
}

RobotToolCompatibilityResult evaluate_robot_tool_compatibility(const Scene & scene, const std::string & config_root)
{
  RobotToolCompatibilityResult r;
  r.status = "UNKNOWN_COMPATIBILITY";
  auto robots = load_robot_profiles(config_root);
  auto tools = load_tool_profiles(config_root);
  r.robot_id = infer_robot_id(scene);
  r.tool_id = infer_tool_id(scene);
  if (!robots.count(r.robot_id)) { r.status = "MISSING_ROBOT_PROFILE"; r.issues.push_back({"missing robot profile", false}); }
  if (!tools.count(r.tool_id)) { r.status = "MISSING_TOOL_PROFILE"; r.issues.push_back({"missing tool profile", false}); return r; }
  const auto & tp = tools[r.tool_id];
  if (r.status == "MISSING_ROBOT_PROFILE") {
    r.tool_mount_link = tp.mount_link;
    r.tcp_frame = tp.tcp_frame;
    infer_tool_defaults_from_profile(&r);
    return r;
  }
  const auto & rp = robots[r.robot_id];
  r.tool_mount_link = tp.mount_link.empty() ? rp.default_tool_mount_link : tp.mount_link;
  r.tcp_frame = tp.tcp_frame;
  r.planning_group = rp.planning_group;
  r.controller_hint = tp.controller_hint.empty() ? rp.controller_family : tp.controller_hint;
  r.tool_type = tp.tool_type;
  r.grasp_strategy_default = tp.grasp_strategy_default;
  r.release_strategy_default = tp.release_strategy_default;
  if (r.tcp_frame.empty()) { r.status = "MISSING_TCP"; r.issues.push_back({"missing tcp_frame", true}); }
  else if (r.tool_mount_link.empty()) { r.status = "MISSING_MOUNT_LINK"; r.issues.push_back({"missing mount_link", true}); }
  else if (r.controller_hint.empty()) { r.status = "MISSING_CONTROLLER_METADATA"; r.issues.push_back({"missing controller metadata", false}); }
  else if (r.robot_id == "ur5" && r.tool_id == "onrobot_airpick") { r.status = "COMPATIBLE_WITH_WARNINGS"; r.issues.push_back({"requires_io=true for suction controller", false}); }
  else if (r.robot_id == "ur5" && r.tool_id == "robotiq_2f85") { r.status = "COMPATIBLE"; }
  else if (r.robot_id == "generic_unknown_robot" || r.tool_id == "generic_unknown_tool") { r.status = "UNKNOWN_COMPATIBILITY"; }
  else { r.status = "INCOMPATIBLE"; r.issues.push_back({"pair marked INCOMPATIBLE", true}); }
  return r;
}

void infer_tool_defaults_from_profile(RobotToolCompatibilityResult * r)
{
  if (!r) { return; }
  if (r->tool_type == "suction") {
    r->grasp_strategy_default = "suction_top";
    r->release_strategy_default = "vacuum_off";
  } else if (r->tool_type == "finger") {
    r->grasp_strategy_default = "finger_top";
    r->release_strategy_default = "open_gripper";
  }
}

std::string compatibility_status_label(const RobotToolCompatibilityResult & result)
{
  return result.status;
}
