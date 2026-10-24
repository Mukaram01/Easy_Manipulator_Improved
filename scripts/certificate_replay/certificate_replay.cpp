// Offline replay of a traced controller interval certificate.
//
// Reads the frozen PlanningScene and emitted RobotTrajectory that the planning
// adapter saves under WORKCELL_PLANNING_TRACE_DIR (certificate_*.scene.cdr /
// certificate_*.trajectory.cdr), rebuilds the robot model from the session's
// qualification URDF/SRDF and runs the same header-only certificate. Optional
// scene edits (allow one collision pair, drop one world object) isolate which
// pair makes an interval inconclusive. Diagnostic only: nothing here changes
// the certificate or any scene policy used by the runtime.
#include "controller_interval_certificate.hpp"
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>

template<class Message> Message load(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("cannot read " + path);
  const std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  rclcpp::SerializedMessage serialized(bytes.size());
  auto& raw = serialized.get_rcl_serialized_message();
  std::memcpy(raw.buffer, bytes.data(), bytes.size());
  raw.buffer_length = bytes.size();
  Message message;
  rclcpp::Serialization<Message>().deserialize_message(&serialized, &message);
  return message;
}

int main(int argc, char** argv) {
  if (argc < 6) {
    std::cerr << "usage: certificate_replay <urdf> <srdf> <group> <trace prefix> <max_depth> [--allow A B]... [--drop OBJECT]... [--distance A B t0 t1 step] [--grid A B jointA jointB lo hi n]\n";
    return 2;
  }
  const std::string urdf_path = argv[1], srdf_path = argv[2], group = argv[3], prefix = argv[4];
  workcell::ControllerAuditOptions options;
  options.max_depth = static_cast<unsigned>(std::stoul(argv[5]));
  std::vector<std::pair<std::string, std::string>> allow;
  std::vector<std::string> drop;
  std::string sweep_a, sweep_b; double sweep_t0 = 0., sweep_t1 = 0., sweep_step = 1.;
  std::string grid_joint_a, grid_joint_b; double grid_lo = 0., grid_hi = 0.; int grid_n = 0;
  for (int i = 6; i < argc; ++i) {
    const std::string flag = argv[i];
    if (flag == "--allow" && i + 2 < argc) { allow.emplace_back(argv[i + 1], argv[i + 2]); i += 2; }
    else if (flag == "--drop" && i + 1 < argc) { drop.emplace_back(argv[++i]); }
    else if (flag == "--grid" && i + 7 < argc) {
      sweep_a = argv[i + 1]; sweep_b = argv[i + 2]; grid_joint_a = argv[i + 3]; grid_joint_b = argv[i + 4];
      grid_lo = std::stod(argv[i + 5]); grid_hi = std::stod(argv[i + 6]); grid_n = std::stoi(argv[i + 7]); i += 7;
    }
    else if (flag == "--distance" && i + 5 < argc) {
      sweep_a = argv[i + 1]; sweep_b = argv[i + 2]; sweep_t0 = std::stod(argv[i + 3]); sweep_t1 = std::stod(argv[i + 4]);
      sweep_step = std::stod(argv[i + 5]); i += 5;
    }
    else { std::cerr << "unknown argument " << flag << "\n"; return 2; }
  }
  auto urdf = urdf::parseURDFFile(urdf_path);
  if (!urdf) { std::cerr << "URDF parse failed\n"; return 2; }
  auto srdf = std::make_shared<srdf::Model>();
  if (!srdf->initFile(*urdf, srdf_path)) { std::cerr << "SRDF parse failed\n"; return 2; }
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);

  auto scene_msg = load<moveit_msgs::msg::PlanningScene>(prefix + ".scene.cdr");
  auto& objects = scene_msg.world.collision_objects;
  objects.erase(std::remove_if(objects.begin(), objects.end(), [&](const auto& o) {
    return std::find(drop.begin(), drop.end(), o.id) != drop.end(); }), objects.end());
  planning_scene::PlanningScene scene(model);
  scene.setPlanningSceneMsg(scene_msg);
  for (const auto& pair : allow) scene.getAllowedCollisionMatrixNonConst().setEntry(pair.first, pair.second, true);

  const auto trajectory_msg = load<moveit_msgs::msg::RobotTrajectory>(prefix + ".trajectory.cdr");
  robot_trajectory::RobotTrajectory trajectory(model, group);
  trajectory.setRobotTrajectoryMsg(scene.getCurrentState(), trajectory_msg);

  if (grid_n > 0) {
    // Unpadded FCL distance of one self pair over a (wrist_1, wrist_2) grid at the
    // scene's current state: the pair's clearance landscape for route design.
    collision_detection::AllowedCollisionMatrix only(scene.getAllowedCollisionMatrix());
    for (const auto& a : model->getLinkModelNames()) for (const auto& b : model->getLinkModelNames())
      if (!(a == sweep_a && b == sweep_b) && !(a == sweep_b && b == sweep_a)) only.setEntry(a, b, true);
    for (int i = 0; i < grid_n; ++i) for (int j = 0; j < grid_n; ++j) {
      moveit::core::RobotState state(scene.getCurrentState());
      const double w1 = grid_lo + (grid_hi - grid_lo) * i / (grid_n - 1), w2 = grid_lo + (grid_hi - grid_lo) * j / (grid_n - 1);
      state.setVariablePosition(grid_joint_a, w1); state.setVariablePosition(grid_joint_b, w2); state.update();
      collision_detection::DistanceRequest req; req.type = collision_detection::DistanceRequestType::SINGLE;
      req.acm = &only; req.enable_signed_distance = true;
      collision_detection::DistanceResult res;
      scene.getCollisionEnvUnpadded()->distanceSelf(req, res, state);
      std::cout << "grid " << grid_joint_a << ' ' << w1 << ' ' << grid_joint_b << ' ' << w2 << " distance " << res.minimum_distance.distance << '\n';
    }
    return 0;
  }
  if (!sweep_a.empty()) {
    // Exact unpadded FCL distance of one self pair along the waypoint interpolation
    // (identical to the controller spline inside constant-velocity segments).
    collision_detection::AllowedCollisionMatrix only(scene.getAllowedCollisionMatrix());
    for (const auto& a : model->getLinkModelNames()) for (const auto& b : model->getLinkModelNames())
      if (!(a == sweep_a && b == sweep_b) && !(a == sweep_b && b == sweep_a)) only.setEntry(a, b, true);
    for (double t = sweep_t0; t <= sweep_t1 + 1e-12; t += sweep_step) {
      auto shared = std::make_shared<moveit::core::RobotState>(scene.getCurrentState());
      trajectory.getStateAtDurationFromStart(t, shared);
      shared->update();
      collision_detection::DistanceRequest req; req.type = collision_detection::DistanceRequestType::SINGLE;
      req.acm = &only; req.enable_nearest_points = true; req.enable_signed_distance = true;
      collision_detection::DistanceResult res;
      scene.getCollisionEnvUnpadded()->distanceSelf(req, res, *shared);
      const auto& d = res.minimum_distance;
      std::cout << "t " << t << " distance " << d.distance << " collision " << res.collision
                << " links " << d.link_names[0] << ' ' << d.link_names[1]
                << " p1 " << d.nearest_points[0].transpose() << " p2 " << d.nearest_points[1].transpose()
                << " wrist_1 " << shared->getVariablePosition("wrist_1_joint") << '\n';
    }
    return 0;
  }

  const auto report = workcell::controller_certificate::certify(trajectory, scene, options);
  std::cout << "result " << static_cast<int>(report.result) << " reason " << report.reason
            << " interval " << report.failure_begin_ns << ' ' << report.failure_end_ns
            << " inspected " << report.inspected << " certified " << report.certified
            << " subdivided " << report.subdivided << " deepest " << report.deepest
            << " seconds " << report.wall_seconds << '\n';
  return report.result == workcell::ControllerCertificate::CERTIFIED_CLEAR ? 0 : 1;
}
