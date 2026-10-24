// Minimal pinch-contact reproduction (Workstream B, step 1): no ROS, no controller, no MoveIt.
// Build: g++ -O2 $(pkg-config --cflags dart) -std=c++17 pinch_replay.cpp -o pinch_replay -I/usr/include/eigen3 \
//   -I/usr/include/bullet -ldart -ldart-collision-bullet -lfcl -lassimp -L$OVERLAY/lib -lBulletCollision -lLinearMath
// Run with LD_LIBRARY_PATH=$OVERLAY/lib (the qualified simulator physics overlay), as the benchmark does.
// Input: first line tool/tip1/tip2/part poses at grasp establishment, then "t x y z qx qy qz qw" tool samples.
// A palm replays a recorded tool trajectory (velocity-commanded); two pads on prismatic joints squeeze
// a free 20 g box with a constant force. One variable at a time: pad collision geometry, replay speed,
// collision detector. Output: in-grip rotation/translation, pad force collapses, pad steps.
#include <dart/dart.hpp>
#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>

using namespace dart;
using Eigen::Isometry3d; using Eigen::Vector3d;

Isometry3d pose(const std::vector<double>& p, size_t o = 0) {
  Isometry3d T = Isometry3d::Identity();
  T.translation() = Vector3d(p[o], p[o+1], p[o+2]);
  T.linear() = Eigen::Quaterniond(p[o+6], p[o+3], p[o+4], p[o+5]).normalized().toRotationMatrix();  // x y z w
  return T;
}

int main(int argc, char** argv) {
  if (argc < 7) { std::cerr << "usage: pinch_replay <replay.txt> <mesh|box> <speed> <force_N> <bullet|fcl> <stl>\n"; return 2; }
  const std::string geometry = argv[2]; const double speed = std::stod(argv[3]); const double force = std::stod(argv[4]);
  const std::string detector = argv[5]; const std::string stl = argv[6];
  // replay.txt: line 1: tool t1 t2 part poses (4 x 7); then: t x y z qx qy qz qw (tool trajectory)
  std::ifstream in(argv[1]); std::string line; std::getline(in, line);
  std::vector<double> s; { std::istringstream ss(line); double v; while (ss >> v) s.push_back(v); }
  std::vector<std::vector<double>> traj;
  while (std::getline(in, line)) { std::istringstream ss(line); std::vector<double> r; double v; while (ss >> v) r.push_back(v); if (r.size() == 8) traj.push_back(r); }
  const Isometry3d tool0 = pose(s, 0), tip1 = pose(s, 7), tip2 = pose(s, 14), part0 = pose(s, 21);

  auto world = simulation::World::create(); world->setTimeStep(0.001); world->setGravity(Vector3d(0, 0, -9.81));
  if (detector == "fcl") world->getConstraintSolver()->setCollisionDetector(collision::FCLCollisionDetector::create());
  else world->getConstraintSolver()->setCollisionDetector(collision::BulletCollisionDetector::create());

  // Gripper: palm (velocity-commanded free joint, no collision) + two pads on prismatic joints.
  auto gripper = dynamics::Skeleton::create("gripper");
  dynamics::FreeJoint::Properties fj; fj.mName = "palm_joint";
  auto palm = gripper->createJointAndBodyNodePair<dynamics::FreeJoint>(nullptr, fj).second;
  palm->setMass(1.0); palm->getParentJoint()->setActuatorType(dynamics::Joint::VELOCITY);
  dynamics::FreeJoint::setTransform(gripper.get(), tool0);
  auto mesh = std::make_shared<dynamics::MeshShape>(Vector3d::Ones(), dynamics::MeshShape::loadMesh(stl));
  std::vector<dynamics::BodyNode*> pads;
  for (int k = 0; k < 2; ++k) {
    const Isometry3d tip = k ? tip2 : tip1;
    dynamics::PrismaticJoint::Properties pj; pj.mName = k ? "pad2" : "pad1";
    pj.mT_ParentBodyToJoint = tool0.inverse() * tip;
    // squeeze direction: from the pad toward the part, expressed in the pad (tip) frame
    // the pinch axis is the tool x axis; each pad pushes along it toward the part
    const double sign = (tool0.linear().transpose() * (part0.translation() - tip.translation())).x() > 0 ? 1.0 : -1.0;
    Vector3d axis = tip.linear().transpose() * (tool0.linear() * Vector3d(sign, 0, 0));
    pj.mAxis = axis.normalized();
    // The real fingers are coupled to one actuator (mimic linkage): the pinch cannot translate as a pair.
    // Pad 1 is force-driven along the pinch axis; pad 2 reacts rigidly (welded at its grasp pose).
    dynamics::BodyNode* pad;
    if (k == 0) {
      pad = gripper->createJointAndBodyNodePair<dynamics::PrismaticJoint>(palm, pj).second;
      pad->getParentJoint()->setActuatorType(dynamics::Joint::FORCE);
    } else {
      dynamics::WeldJoint::Properties wj; wj.mName = "pad2"; wj.mT_ParentBodyToJoint = tool0.inverse() * tip;
      pad = gripper->createJointAndBodyNodePair<dynamics::WeldJoint>(palm, wj).second;
    }
    pad->setMass(0.019555); pad->setMomentOfInertia(2e-6, 5e-6, 6e-6);
    dynamics::ShapePtr shape = mesh; Isometry3d offset = Isometry3d::Identity();
    if (geometry == "bbox") {          // the fingertip mesh's bounding box: encloses the mesh, +y face = pad plane
      shape = std::make_shared<dynamics::BoxShape>(Vector3d(0.0570, 0.0317, 0.0220));
      offset.translation() = Vector3d(0.0160, -0.00805, 0.0);
    }
    if (geometry == "box") {           // pad contact face (tip frame y = 0.0078 m), 6 mm thick, spanning the measured contact region
      shape = std::make_shared<dynamics::BoxShape>(Vector3d(0.0173, 0.006, 0.022));
      offset.translation() = Vector3d(0.03585, 0.0048, 0.0);
    }
    auto sn = pad->createShapeNodeWith<dynamics::CollisionAspect, dynamics::DynamicsAspect>(shape);
    sn->setRelativeTransform(offset); sn->getDynamicsAspect()->setFrictionCoeff(100000.0);
    pads.push_back(pad);
  }
  palm->getSkeleton()->enableSelfCollisionCheck(); gripper->disableSelfCollisionCheck();
  // Part: free 20 g, 25 mm box, mu 0.65 (min with the pad's).
  auto part = dynamics::Skeleton::create("part");
  auto body = part->createJointAndBodyNodePair<dynamics::FreeJoint>().second;
  body->setMass(0.020); body->setMomentOfInertia(2.083333333e-6, 2.083333333e-6, 2.083333333e-6);
  auto box = body->createShapeNodeWith<dynamics::CollisionAspect, dynamics::DynamicsAspect>(std::make_shared<dynamics::BoxShape>(Vector3d::Constant(0.025)));
  box->getDynamicsAspect()->setFrictionCoeff(0.65);
  dynamics::FreeJoint::setTransform(part.get(), part0);
  world->addSkeleton(gripper); world->addSkeleton(part);

  auto command = [&](const Isometry3d& target) {   // body-frame twist that moves the palm to target in one step
    const Isometry3d now = palm->getWorldTransform();
    Eigen::Vector6d twist = math::logMap(now.inverse() * target) / world->getTimeStep();
    palm->getParentJoint()->setCommands(twist);
  };
  auto squeeze = [&]() { pads[0]->getParentJoint()->setCommand(0, force); };
  auto relative = [&]() { return Isometry3d(palm->getWorldTransform().inverse() * body->getWorldTransform()); };
  auto padload = [&](int k) {      // normal force on one pad from the last solve (sum over its contacts)
    double sum = 0; const auto& res = world->getLastCollisionResult();
    for (const auto& c : res.getContacts()) {
      auto* a = c.collisionObject1->getShapeFrame(); auto* b = c.collisionObject2->getShapeFrame();
      auto* pn = pads[k]->getShapeNode(0);
      if (a == pn || b == pn) sum += c.force.norm();
    }
    return sum;
  };
  // Settle stationary for 1 s under the squeeze, then take the held reference.
  for (int i = 0; i < 1000; ++i) { command(tool0); squeeze(); world->step(); }
  const Isometry3d reference = relative();
  double peak_rot = 0, peak_trans = 0, settle_rot = Eigen::AngleAxisd((reference.inverse() * relative()).linear()).angle();
  int collapses = 0, samples = 0, steps = 0; double minload = 1e9;
  std::vector<double> lastq = {pads[0]->getParentJoint()->getPosition(0), 0.0};
  const double t0 = traj.front()[0], T = traj.back()[0] - t0;
  size_t idx = 0;
  for (double t = 0; t <= T / speed; t += world->getTimeStep()) {
    const double ts = t0 + t * speed;
    while (idx + 1 < traj.size() && traj[idx + 1][0] <= ts) ++idx;
    command(pose(traj[idx], 1)); squeeze(); world->step();
    if (std::getenv("TRACE2") && t > 5.95 && t < 6.03 && samples % 5 == 0)
      std::cerr << "t=" << t << " palm_v=" << palm->getLinearVelocity().norm() << " palm_w=" << palm->getAngularVelocity().norm()
                << " q1=" << pads[0]->getParentJoint()->getPosition(0)
                << " contacts=" << world->getLastCollisionResult().getNumContacts() << " load1=" << padload(0) << " load2=" << padload(1)
                << " part_v=" << body->getLinearVelocity().norm() << "\n";
    if (std::getenv("TRACE") && (samples % 1000 == 0))
      std::cerr << "t=" << t << " palm_err_m=" << (palm->getWorldTransform().translation() - pose(traj[idx], 1).translation()).norm()
                << " part_from_palm_m=" << relative().translation().norm() << "\n";
    const Isometry3d rel = reference.inverse() * relative();
    const double rot_now = Eigen::AngleAxisd(rel.linear()).angle(), trans_now = rel.translation().norm();
    if (std::getenv("TRACE") && trans_now > 0.002 && peak_trans <= 0.002)
      std::cerr << "first translation > 2 mm at t=" << t << " rot=" << rot_now << " trans=" << trans_now << "\n";
    peak_rot = std::max(peak_rot, rot_now);
    peak_trans = std::max(peak_trans, trans_now);
    for (int k = 0; k < 2; ++k) {
      const double load = padload(k); minload = std::min(minload, load); collapses += load < 0.5 * force;
      if (k == 0) { const double q = pads[0]->getParentJoint()->getPosition(0); steps += std::abs(q - lastq[0]) >= 2e-6; lastq[0] = q; }
    }
    ++samples;
  }
  std::cout << "geometry=" << geometry << " detector=" << detector << " speed=" << speed << " force=" << force
            << " samples=" << samples << " peak_rotation_rad=" << peak_rot << " peak_translation_m=" << peak_trans
            << " pad_collapse_samples=" << collapses << " min_pad_load_n=" << minload << " pad_steps=" << steps << "\n";
  return 0;
}
