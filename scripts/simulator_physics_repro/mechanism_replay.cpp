// Faithful standalone replay of a recorded Stage-A grasp hold (simulated Robotiq 2F-85 on the UR5).
// No ROS, controller manager or MoveIt; the physics is the simulator's own:
//  - gripper: all six joints, inertias and pad boxes from the session's runtime robot.sdf;
//  - the overlay's passive mimic rows (JointCouplingConstraint.hpp) and contact detector
//    (TouchingBulletContacts.hpp: BOX pairs through double-precision ODE dBoxBox), compiled in verbatim
//    from passive-joint-coupling.patch;
//  - finger1 hold: gz_ros2_control position law v = 0.1 * 100 * (hold_command - q) on a SERVO capped at the
//    joint effort; the five followers PASSIVE;
//  - arm=: the UR5 chain as velocity servos tracking the recorded joint positions (default: the wrist
//    replayed kinematically from its recorded pose);
//  - 1 ms step, Dantzig LCP, gravity, 20 g part, pick support; DART contact defaults unless allowance= is given.
// LENLEG 2026-10-25, session stage_a1_targeted_native_pads_fc1_20261025_000126: with arm= the replay matches the
// recorded run step for step (pad-1 manifold 3 3 3 2 3 2 2 2 1 4 4 3 2 3 1 4 4 2 3 0 over 183277-183296, rotation
// steps 0.300/0.596 mrad, pad-1 contact absent at 183296 with tip forces 0.19/0.21 N). The part walks across the
// pinch at 7-33 um/s during transfer: the same with arm_actuator=velocity, ~10x less with the wrist replayed
// as one free body, scaling with substeps, i.e. 1 ms integration of the articulated arm against the free part.
// Default contact compliance holds the pinch on ~2 um; cfm=1e-4 on ~20 um, and the asymmetry stays bounded.
// Inputs: mechanism_replay_inputs.py.
//
// usage: mechanism_replay <mechanism.txt> <replay.txt> [arm=<arm.txt>] [hold=position|stationary]
//                         [cfm=<c>] [allowance=<m>] [trace=<it0>:<it1>]
//   substeps=<n>: n physics steps per recorded millisecond; arm_actuator=velocity: arm joints kinematic.
//   rebase_hold=<steps>: after that many steps take the settled finger position as the closure and command
//   closure + 0.002 rad (the 2F-85 hold reserve), as the executor does after its closing search.
//   cfm sets DART's world-wide contact constraint force mixing (contact compliance; the workcell physics
//   overlay uses 1e-4, DART's default is 1e-5); allowance sets the contact error allowance (diagnostic).
#include <dart/dart.hpp>
#include <dart/constraint/ContactConstraint.hpp>
#include "JointCouplingConstraint.hpp"
#include "TouchingBulletContacts.hpp"
#include <fstream>
#include <limits>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace dart;
using Eigen::Isometry3d; using Eigen::Vector3d;

static Isometry3d xyzrpy(const std::vector<double>& v, size_t o) {
  Isometry3d T = Isometry3d::Identity();
  T.translation() = Vector3d(v[o], v[o + 1], v[o + 2]);
  T.linear() = (Eigen::AngleAxisd(v[o + 5], Vector3d::UnitZ()) * Eigen::AngleAxisd(v[o + 4], Vector3d::UnitY()) *
                Eigen::AngleAxisd(v[o + 3], Vector3d::UnitX())).toRotationMatrix();
  return T;
}
static Isometry3d xyzq(const std::vector<double>& v, size_t o) {   // x y z qx qy qz qw
  Isometry3d T = Isometry3d::Identity();
  T.translation() = Vector3d(v[o], v[o + 1], v[o + 2]);
  T.linear() = Eigen::Quaterniond(v[o + 6], v[o + 3], v[o + 4], v[o + 5]).normalized().toRotationMatrix();
  return T;
}
static std::vector<double> nums(std::istringstream& ss) { std::vector<double> r; double v; while (ss >> v) r.push_back(v); return r; }

int main(int argc, char** argv) {
  if (argc < 3) { std::cerr << "usage: mech_replay <mechanism.txt> <replay.txt> [key=value ...]\n"; return 2; }
  std::map<std::string, std::string> opt;
  for (int i = 3; i < argc; ++i) { std::string a = argv[i]; auto e = a.find('='); opt[a.substr(0, e)] = a.substr(e + 1); }
  const std::string hold = opt.count("hold") ? opt["hold"] : "position";
  long trace0 = -1, trace1 = -1;
  if (opt.count("trace")) { auto s = opt["trace"]; trace0 = std::stol(s.substr(0, s.find(':'))); trace1 = std::stol(s.substr(s.find(':') + 1)); }
  double kHoldCommand = std::numeric_limits<double>::quiet_NaN();   // summary.json hold_command_rad
  const double kGain = 0.1, kUpdateRate = 100.0;       // launch.log position_proportional_gain; controllers.yaml update_rate

  const long substeps = opt.count("substeps") ? std::stol(opt["substeps"]) : 1;   // physics steps per recorded 1 ms
  auto world = simulation::World::create(); world->setTimeStep(0.001 / substeps); world->setGravity(Vector3d(0, 0, -9.81));
  world->getConstraintSolver()->setCollisionDetector(std::make_shared<workcell::TouchingBulletContacts>());
  if (opt.count("allowance")) constraint::ContactConstraint::setErrorAllowance(std::stod(opt["allowance"]));
  if (opt.count("cfm")) constraint::ContactConstraint::setConstraintForceMixing(std::stod(opt["cfm"]));

  auto robot = dynamics::Skeleton::create("workcell_robot");
  std::map<std::string, dynamics::BodyNode*> body;
  std::map<std::string, dynamics::Joint*> joint;
  std::vector<std::string> armJoints; std::vector<std::pair<long, std::vector<double>>> armTraj;
  dynamics::BodyNode* palm = nullptr;
  auto revolute = [&](std::istringstream& ss, std::string& name, std::string& driver, double& mult) {
    std::string parent, child; ss >> name >> parent >> child;
    std::vector<double> pose(6); for (auto& v : pose) ss >> v;
    double lo, hi, effort; ss >> lo >> hi >> effort >> driver >> mult;
    auto in = nums(ss);   // mass cx cy cz ixx iyy izz ixy ixz iyz
    dynamics::RevoluteJoint::Properties rp; rp.mName = name; rp.mAxis = Vector3d::UnitZ();
    rp.mT_ParentBodyToJoint = xyzrpy(pose, 0);
    auto pair = robot->createJointAndBodyNodePair<dynamics::RevoluteJoint>(body.at(parent), rp, dynamics::BodyNode::AspectProperties(child));
    auto* b = pair.second; b->setMass(in[0]); b->setLocalCOM(Vector3d(in[1], in[2], in[3]));
    b->setMomentOfInertia(in[4], in[5], in[6], in[7], in[8], in[9]);
    pair.first->setPositionLowerLimit(0, lo); pair.first->setPositionUpperLimit(0, hi);
    pair.first->setForceLowerLimit(0, -effort); pair.first->setForceUpperLimit(0, effort);
    body[child] = b; joint[name] = pair.first;
    return pair.first;
  };
  if (opt.count("arm")) {   // dynamic UR5: base welded at base_joint, arm joints velocity servos with their effort caps
    std::ifstream af(opt["arm"]); std::string l;
    while (std::getline(af, l)) {
      std::istringstream ss(l); std::string tag; ss >> tag;
      if (tag == "BASE") {
        auto v = nums(ss); dynamics::WeldJoint::Properties wp; wp.mName = "base_joint"; wp.mT_ParentBodyToJoint = xyzrpy(v, 0);
        body["base_link"] = robot->createJointAndBodyNodePair<dynamics::WeldJoint>(nullptr, wp, dynamics::BodyNode::AspectProperties("base_link")).second;
        body["base_link"]->setMass(4.0);
      } else if (tag == "A") {
        std::string name, d; double m; auto* j = revolute(ss, name, d, m);
        j->setActuatorType(opt.count("arm_actuator") && opt["arm_actuator"] == "velocity" ? dynamics::Joint::VELOCITY
                                                                                     : dynamics::Joint::SERVO);
        armJoints.push_back(name);
      } else if (tag == "R") {
        long it; ss >> it; armTraj.emplace_back(it, nums(ss));
      }
    }
    palm = body.at("wrist_3_link");
  } else {
    dynamics::FreeJoint::Properties fp; fp.mName = "wrist_3_root";
    palm = robot->createJointAndBodyNodePair<dynamics::FreeJoint>(nullptr, fp, dynamics::BodyNode::AspectProperties("wrist_3_link")).second;
    palm->setMass(0.824851); palm->setLocalCOM(Vector3d(0, 0, -0.0002640187136828349));
    palm->setMomentOfInertia(0.00119081966386744, 0.0012518196638674399, 0.00051211718749999999);
    palm->getParentJoint()->setActuatorType(dynamics::Joint::VELOCITY);
    body["wrist_3_link"] = palm;
  }
  std::vector<std::tuple<std::string, std::string, double>> mimics;
  std::map<std::string, std::vector<dynamics::ShapeNode*>> pads;
  std::ifstream mf(argv[1]); std::string line;
  while (std::getline(mf, line)) {
    std::istringstream ss(line); std::string tag; ss >> tag;
    if (tag == "J") {
      std::string name, driver; double mult; auto* j = revolute(ss, name, driver, mult);
      if (driver != "-") { mimics.emplace_back(name, driver, mult); j->setActuatorType(dynamics::Joint::PASSIVE); }
    } else if (tag == "B") {
      std::string link; ss >> link; auto v = nums(ss);
      auto sn = body.at(link)->createShapeNodeWith<dynamics::CollisionAspect, dynamics::DynamicsAspect>(
          std::make_shared<dynamics::BoxShape>(Vector3d(v[6], v[7], v[8])));
      sn->setRelativeTransform(xyzrpy(v, 0));
      sn->getDynamicsAspect()->setFrictionCoeff(v[9]); sn->getDynamicsAspect()->setSecondaryFrictionCoeff(v[9]);
      pads[link].push_back(sn);
    }
  }
  auto* driver = joint.at("gripper_finger1_joint");
  driver->setActuatorType(dynamics::Joint::SERVO);
  robot->disableSelfCollisionCheck();

  // replay inputs
  std::ifstream rf(argv[2]); std::vector<double> start; std::map<std::string, std::pair<double, double>> q0;
  std::vector<std::pair<long, Isometry3d>> traj;
  while (std::getline(rf, line)) {
    std::istringstream ss(line); std::string tag; ss >> tag;
    if (tag == "S") start = nums(ss);
    else if (tag == "H") ss >> kHoldCommand;
    else if (tag == "Q") { std::string n; double p, v; ss >> n >> p >> v; q0[n] = {p, v}; }
    else if (tag == "T") { long it; ss >> it; auto v = nums(ss); traj.emplace_back(it, xyzq(v, 0)); }
  }
  if (armJoints.empty()) dynamics::FreeJoint::setTransform(robot.get(), xyzq(start, 0));
  else for (size_t i = 0; i < armJoints.size(); ++i) {
    joint.at(armJoints[i])->setPosition(0, armTraj.front().second[2 * i]); joint.at(armJoints[i])->setVelocity(0, armTraj.front().second[2 * i + 1]);
  }
  for (auto& [n, pv] : q0) { joint.at(n)->setPosition(0, pv.first); joint.at(n)->setVelocity(0, pv.second); }

  auto part = dynamics::Skeleton::create("part_04");
  auto pb = part->createJointAndBodyNodePair<dynamics::FreeJoint>().second;
  pb->setMass(0.020); pb->setMomentOfInertia(2.083333333e-6, 2.083333333e-6, 2.083333333e-6);
  auto psn = pb->createShapeNodeWith<dynamics::CollisionAspect, dynamics::DynamicsAspect>(std::make_shared<dynamics::BoxShape>(Vector3d::Constant(0.025)));
  psn->getDynamicsAspect()->setFrictionCoeff(0.65); psn->getDynamicsAspect()->setSecondaryFrictionCoeff(0.65);
  if (!std::isfinite(kHoldCommand) || traj.size() < 2) { std::cerr << "replay inputs incomplete\n"; return 2; }
  dynamics::FreeJoint::setTransform(part.get(), xyzq(start, 7));
  auto support = dynamics::Skeleton::create("pick_support");
  auto sb = support->createJointAndBodyNodePair<dynamics::WeldJoint>().second;
  auto ssn = sb->createShapeNodeWith<dynamics::CollisionAspect, dynamics::DynamicsAspect>(
      std::make_shared<dynamics::BoxShape>(Vector3d(1.2084002380371095, 0.81340008544921871, 0.9062377070188522)));
  ssn->getDynamicsAspect()->setFrictionCoeff(0.8); ssn->getDynamicsAspect()->setSecondaryFrictionCoeff(0.8);
  Isometry3d st = Isometry3d::Identity(); st.translation() = Vector3d(0.51, 0.0022999572753906183, -0.4531188535094261);
  sb->getParentJoint()->setTransformFromParentBodyNode(st);
  world->addSkeleton(robot); world->addSkeleton(part); world->addSkeleton(support);
  for (auto& [f, d, m] : mimics)
    world->getConstraintSolver()->addConstraint(std::make_shared<workcell::JointCouplingConstraint>(joint.at(f), joint.at(d), m));

  const std::string tip1 = "gripper_finger1_finger_tip_link", tip2 = "gripper_finger2_finger_tip_link";
  struct PadState { int points = 0; double normal = 0, mindepth = 1e9, maxdepth = -1e9; };
  auto pad = [&](const std::string& link) {
    PadState s; const auto& res = world->getLastCollisionResult();
    for (const auto& c : res.getContacts()) {
      auto* a = c.collisionObject1->getShapeFrame(); auto* b = c.collisionObject2->getShapeFrame();
      for (auto* sn : pads[link]) if ((a == sn && b == psn) || (b == sn && a == psn)) {
        ++s.points; s.normal += std::abs(c.force.dot(c.normal));
        s.mindepth = std::min(s.mindepth, c.penetrationDepth); s.maxdepth = std::max(s.maxdepth, c.penetrationDepth);
      }
    }
    return s;
  };
  auto tipForce = [&](const std::string& link) { return body.at(link)->getBodyForce().tail<3>().norm(); };
  auto relative = [&]() { return Isometry3d(palm->getWorldTransform().inverse() * pb->getWorldTransform()); };

  // hold=stationary: command the recorded closure position instead of closure + reserve (no closing demand).
  const double stationaryTarget = q0.at("gripper_finger1_joint").first;
  const long rebaseAt = opt.count("rebase_hold") ? traj.front().first + std::stol(opt["rebase_hold"]) : -1;
  Isometry3d reference; bool haveReference = false;
  double peakRot = 0, peakTrans = 0, minF = 1e9; long dropouts = 0, partial = 0, firstDropout = -1, lift = -1;
  for (size_t k = 0; k + 1 < traj.size(); ++k) {
    const long it = traj[k].first; const long gap = traj[k + 1].first - it; const long now0 = it;
    const Eigen::Vector6d twist = math::logMap(palm->getWorldTransform().inverse() * traj[k + 1].second) / (gap * 0.001);
    for (long g = 0; g < gap * substeps; ++g) {
      if (armJoints.empty()) palm->getParentJoint()->setCommands(twist);
      else for (size_t i = 0; i < armJoints.size(); ++i) {   // track the measured joint positions (velocity servo)
        auto* aj = joint.at(armJoints[i]);
        aj->setCommand(0, (armTraj[k + 1].second[2 * i] - aj->getPosition(0)) / ((gap * substeps - g) * world->getTimeStep()));
      }
      const double q = driver->getPosition(0);
      if (rebaseAt >= 0 && g % substeps == 0 && now0 + g / substeps == rebaseAt) kHoldCommand = q + 0.002;
      double v = kGain * kUpdateRate * (kHoldCommand - q);
      if (hold == "stationary") v = kGain * kUpdateRate * (stationaryTarget - q);
      driver->setCommand(0, v);
      world->step();
      if ((g + 1) % substeps) continue;                   // evaluate on recorded 1 ms boundaries
      const long now = it + (g + 1) / substeps;
      const auto p1 = pad(tip1), p2 = pad(tip2);
      const double f1 = tipForce(tip1), f2 = tipForce(tip2);
      if (lift < 0 && pb->getWorldTransform().translation().z() > 0.0126) lift = now;
      if (!haveReference && now >= traj.front().first + 100) { reference = relative(); haveReference = true; }
      if (haveReference) {
        const Isometry3d rel = reference.inverse() * relative();
        peakRot = std::max(peakRot, Eigen::AngleAxisd(rel.linear()).angle()); peakTrans = std::max(peakTrans, rel.translation().norm());
        minF = std::min({minF, f1, f2});
        if (p1.points == 0 || p2.points == 0) { ++dropouts; if (firstDropout < 0) firstDropout = now; }
        if (p1.points < 4 || p2.points < 4) ++partial;
      }
      if (now >= trace0 && now <= trace1)
        std::cout << "TRACE it=" << now << " pts1=" << p1.points << " pts2=" << p2.points << " N1=" << p1.normal << " N2=" << p2.normal
                  << " depth1=[" << (p1.points ? p1.mindepth : 0) << "," << (p1.points ? p1.maxdepth : 0) << "] depth2=[" << (p2.points ? p2.mindepth : 0) << "," << (p2.points ? p2.maxdepth : 0)
                  << "] tipF=" << f1 << "," << f2 << " q=" << q << " qd=" << driver->getVelocity(0) << " vcmd=" << v
                  << " servoF=" << driver->getForce(0) << " rot=" << (haveReference ? Eigen::AngleAxisd((reference.inverse() * relative()).linear()).angle() : 0) << "\n";
    }
  }
  std::cout << "RESULT hold=" << hold << " allowance=" << constraint::ContactConstraint::getErrorAllowance() << " cfm=" << constraint::ContactConstraint::getConstraintForceMixing() << " lift_it=" << lift
            << " peak_rotation_rad=" << peakRot << " peak_translation_m=" << peakTrans << " min_tip_force_n=" << minF
            << " dropout_samples=" << dropouts << " first_dropout_it=" << firstDropout << " partial_manifold_samples=" << partial << "\n";
  return 0;
}
