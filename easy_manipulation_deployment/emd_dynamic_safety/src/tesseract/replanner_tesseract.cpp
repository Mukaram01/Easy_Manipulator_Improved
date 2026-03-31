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
#include <stdexcept>
#include <string>
#include <vector>

#include "emd/dynamic_safety/replanner_tesseract.hpp"

#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_srdf/srdf_model.h>
#include <tesseract_common/resource_locator.h>

// Instruction / waypoint types used to build the planning program.
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/utils.h>

// Time parameterization helpers.
#include <tesseract_time_parameterization/instructions_trajectory.h>

namespace dynamic_safety_tesseract
{

static const rclcpp::Logger LOGGER =
  rclcpp::get_logger("dynamic_safety_tesseract.replanner");

// ---------------------------------------------------------------------------
// Helper: build a joint-name → position map from parallel vectors
// ---------------------------------------------------------------------------
static std::unordered_map<std::string, double>
make_joint_map(
  const std::vector<std::string> & names,
  const std::vector<double> & positions)
{
  std::unordered_map<std::string, double> out;
  out.reserve(names.size());
  const size_t n = std::min(names.size(), positions.size());
  for (size_t i = 0; i < n; ++i) {
    out[names[i]] = positions[i];
  }
  return out;
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
TesseractReplannerContext::TesseractReplannerContext(
  const std::string & robot_urdf,
  const std::string & robot_srdf,
  const dynamic_safety::ReplannerOption & option,
  const rclcpp::Node::SharedPtr & /*node*/)
: dynamic_safety::ReplannerContext(robot_urdf, robot_srdf, option, nullptr)
{
  // ---- Initialise environment ----
  env_ = std::make_shared<tesseract_environment::Environment>();
  auto locator = std::make_shared<tesseract_common::GeneralResourceLocator>();

  auto scene_graph = tesseract_urdf::parseURDFString(robot_urdf, *locator);
  if (!scene_graph) {
    throw std::runtime_error("TesseractReplanner: failed to parse URDF string");
  }

  auto srdf_model = std::make_shared<tesseract_srdf::SRDFModel>();
  srdf_model->initString(*scene_graph, robot_srdf);

  if (!env_->init(*scene_graph, srdf_model, locator)) {
    throw std::runtime_error("TesseractReplanner: failed to initialise environment");
  }

  // ---- Kinematics ----
  fwd_kin_ = env_->getManipulatorManager()->getFwdKinematicSolver(option.group);
  inv_kin_ = env_->getManipulatorManager()->getInvKinematicSolver(option.group);

  if (!fwd_kin_) {
    throw std::runtime_error(
            "TesseractReplanner: no forward kinematics solver for group '" +
            option.group + "'");
  }
  if (!inv_kin_) {
    throw std::runtime_error(
            "TesseractReplanner: no inverse kinematics solver for group '" +
            option.group + "'");
  }

  // ---- Manipulator info ----
  manip_.manipulator = option.group;

  // ---- Joint limits ----
  const auto & limits = env_->getKinematicsInformation()
    .getKinematicsGroup(option.group)->getLimits();
  const Eigen::MatrixX2d & vel_limits = limits.velocity_limits;
  const Eigen::MatrixX2d & accel_limits = limits.acceleration_limits;

  joint_vel_limits_.resize(static_cast<size_t>(vel_limits.rows()));
  joint_accel_limits_.resize(static_cast<size_t>(accel_limits.rows()));

  for (Eigen::Index i = 0; i < vel_limits.rows(); ++i) {
    joint_vel_limits_[static_cast<size_t>(i)] = vel_limits(i, 1);
  }
  for (Eigen::Index i = 0; i < accel_limits.rows(); ++i) {
    joint_accel_limits_[static_cast<size_t>(i)] = accel_limits(i, 1);
  }

  // ---- Planner / time-parameterization selection ----
  planner_ = option.planner;
  time_parameterization_ = option.time_parameterization;

  // ---- Common planning request fields ----
  planning_request_.env = env_;
  planning_request_.env_state = env_->getState();
  planning_request_.manip_info = manip_;
}

// ---------------------------------------------------------------------------
// run()
// ---------------------------------------------------------------------------
void TesseractReplannerContext::run(
  const std::vector<std::string> & joint_names,
  const trajectory_msgs::msg::JointTrajectoryPoint & start_point,
  const trajectory_msgs::msg::JointTrajectoryPoint & end_point,
  trajectory_msgs::msg::JointTrajectory & plan)
{
  // ---- Build start state ----
  Eigen::VectorXd start_positions(static_cast<Eigen::Index>(joint_names.size()));
  Eigen::VectorXd end_positions(static_cast<Eigen::Index>(joint_names.size()));

  for (size_t i = 0; i < joint_names.size(); ++i) {
    start_positions[static_cast<Eigen::Index>(i)] = start_point.positions[i];
    end_positions[static_cast<Eigen::Index>(i)] = end_point.positions[i];
  }

  tesseract_planning::StateWaypointPoly start_wp{
    tesseract_planning::StateWaypoint(joint_names, start_positions)};
  tesseract_planning::StateWaypointPoly end_wp{
    tesseract_planning::StateWaypoint(joint_names, end_positions)};

  // ---- Build composite instruction (program) ----
  tesseract_planning::CompositeInstruction program(
    "DEFAULT",
    tesseract_planning::CompositeInstructionOrder::ORDERED,
    manip_);

  tesseract_planning::MoveInstruction start_instr(
    start_wp, tesseract_planning::MoveInstructionType::START);
  tesseract_planning::MoveInstruction end_instr(
    end_wp, tesseract_planning::MoveInstructionType::FREESPACE);

  program.push_back(start_instr);
  program.push_back(end_instr);

  // ---- Fill in the request ----
  planning_request_.env_state = env_->getState();
  planning_request_.instructions = program;

  tesseract_planning::PlannerResponse response;
  bool success = false;

  auto to_lower = [](std::string s) {
      std::transform(s.begin(), s.end(), s.begin(), ::tolower);
      return s;
    };

  const std::string planner_lower = to_lower(planner_);

  if (planner_lower == "ompl") {
    auto status = ompl_planner_.solve(planning_request_, response);
    success = static_cast<bool>(status);
  } else if (planner_lower == "trajopt") {
    auto status = trajopt_planner_.solve(planning_request_, response);
    success = static_cast<bool>(status);
  } else if (planner_lower == "trajopt_ifopt") {
    auto status = trajopt_ifopt_planner_.solve(planning_request_, response);
    success = static_cast<bool>(status);
  } else {
    RCLCPP_ERROR(LOGGER, "Unknown planner: %s", planner_.c_str());
    return;
  }

  if (!success) {
    RCLCPP_ERROR(LOGGER, "Tesseract planning failed");
    return;
  }

  // ---- Convert result to JointTrajectory ----
  auto result_ci =
    response.results.as<tesseract_planning::CompositeInstruction>();
  plan = to_joint_trajectory_msg(result_ci);
  plan.joint_names = joint_names;
}

// ---------------------------------------------------------------------------
// time_parameterize()
// ---------------------------------------------------------------------------
bool TesseractReplannerContext::time_parameterize(
  trajectory_msgs::msg::JointTrajectory & plan,
  double scale)
{
  // Re-build a CompositeInstruction from the plan, apply time parameterization,
  // and write back.
  tesseract_planning::CompositeInstruction program;
  to_composite_instructions(plan, program);

  tesseract_planning::InstructionsTrajectory traj(program);

  auto to_lower = [](std::string s) {
      std::transform(s.begin(), s.end(), s.begin(), ::tolower);
      return s;
    };

  const std::string tp_lower = to_lower(time_parameterization_);
  bool ok = false;

  if (tp_lower == "totg") {
    ok = totg_.compute(traj, joint_vel_limits_, joint_accel_limits_, scale, scale);
  } else if (tp_lower == "isp") {
    ok = isp_.compute(traj, joint_vel_limits_, joint_accel_limits_, scale, scale);
  } else {
    RCLCPP_ERROR(
      LOGGER, "Unknown time parameterization: %s", time_parameterization_.c_str());
    return false;
  }

  if (!ok) {
    RCLCPP_ERROR(LOGGER, "Time parameterization failed");
    return false;
  }

  // Write the time-stamped result back to plan
  plan = to_joint_trajectory_msg(program);
  return true;
}

// ---------------------------------------------------------------------------
// update()
// ---------------------------------------------------------------------------
void TesseractReplannerContext::update(
  const sensor_msgs::msg::JointState & joint_states)
{
  const size_t count =
    std::min(joint_states.name.size(), joint_states.position.size());

  std::unordered_map<std::string, double> joints;
  joints.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    joints[joint_states.name[i]] = joint_states.position[i];
  }

  env_->setState(joints);
  planning_request_.env_state = env_->getState();
}

// ---------------------------------------------------------------------------
// to_joint_trajectory_msg() — CompositeInstruction → JointTrajectory
// ---------------------------------------------------------------------------
trajectory_msgs::msg::JointTrajectory
TesseractReplannerContext::to_joint_trajectory_msg(
  const tesseract_planning::CompositeInstruction & result)
{
  trajectory_msgs::msg::JointTrajectory traj;
  double time_from_start = 0.0;

  for (const auto & instr : result) {
    if (!instr.isMoveInstruction()) {
      continue;
    }
    const auto & move_instr = instr.as<tesseract_planning::MoveInstruction>();
    if (!move_instr.getWaypoint().isStateWaypoint()) {
      continue;
    }
    const auto & swp =
      move_instr.getWaypoint().as<tesseract_planning::StateWaypoint>();

    if (traj.joint_names.empty()) {
      traj.joint_names = swp.joint_names;
    }

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.assign(swp.position.data(), swp.position.data() + swp.position.size());

    if (swp.velocity.size() > 0) {
      pt.velocities.assign(
        swp.velocity.data(),
        swp.velocity.data() + swp.velocity.size());
    }
    if (swp.acceleration.size() > 0) {
      pt.accelerations.assign(
        swp.acceleration.data(),
        swp.acceleration.data() + swp.acceleration.size());
    }

    // Use the waypoint's time field if set, else accumulate.
    if (swp.time > 0.0) {
      time_from_start = swp.time;
    }
    pt.time_from_start = rclcpp::Duration::from_seconds(time_from_start);

    traj.points.push_back(pt);
  }

  return traj;
}

// ---------------------------------------------------------------------------
// to_composite_instructions() — JointTrajectory → CompositeInstruction
// ---------------------------------------------------------------------------
void TesseractReplannerContext::to_composite_instructions(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  tesseract_planning::CompositeInstruction & program)
{
  program = tesseract_planning::CompositeInstruction(
    "DEFAULT",
    tesseract_planning::CompositeInstructionOrder::ORDERED,
    manip_);

  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & pt = trajectory.points[i];
    Eigen::VectorXd positions(static_cast<Eigen::Index>(trajectory.joint_names.size()));
    for (size_t j = 0; j < trajectory.joint_names.size(); ++j) {
      positions[static_cast<Eigen::Index>(j)] = pt.positions[j];
    }

    tesseract_planning::StateWaypoint swp(trajectory.joint_names, positions);
    swp.time = rclcpp::Duration(pt.time_from_start).seconds();

    if (!pt.velocities.empty()) {
      Eigen::VectorXd vel(static_cast<Eigen::Index>(pt.velocities.size()));
      for (size_t j = 0; j < pt.velocities.size(); ++j) {
        vel[static_cast<Eigen::Index>(j)] = pt.velocities[j];
      }
      swp.velocity = vel;
    }
    if (!pt.accelerations.empty()) {
      Eigen::VectorXd acc(static_cast<Eigen::Index>(pt.accelerations.size()));
      for (size_t j = 0; j < pt.accelerations.size(); ++j) {
        acc[static_cast<Eigen::Index>(j)] = pt.accelerations[j];
      }
      swp.acceleration = acc;
    }

    tesseract_planning::StateWaypointPoly swp_poly{swp};
    const auto instr_type = (i == 0)
      ? tesseract_planning::MoveInstructionType::START
      : tesseract_planning::MoveInstructionType::FREESPACE;

    program.push_back(tesseract_planning::MoveInstruction(swp_poly, instr_type));
  }
}

}  // namespace dynamic_safety_tesseract
