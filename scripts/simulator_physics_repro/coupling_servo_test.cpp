#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/constraint/BoxedLcpConstraintSolver.hpp>
#include <dart/simulation/World.hpp>
#include <iostream>
#include <stdexcept>
#include "JointCouplingConstraint.hpp"

void require(bool condition, const char* reason) {
  if (!condition) throw std::runtime_error(reason);
}

struct GroupSolver : dart::constraint::BoxedLcpConstraintSolver {
  using ConstraintSolver::updateConstraints;
  using ConstraintSolver::buildConstrainedGroups;
  using ConstraintSolver::solveConstrainedGroups;
  std::size_t groupCount() const { return mConstrainedGroups.size(); }
};

// Replay the nested union topology captured during physical contact. This is
// a group/impulse regression; the runtime calibration proves actual retention.
void nestedContactUnion(double ratio, bool deeper) {
  using namespace dart::dynamics;
  auto robot=Skeleton::create("coupled_robot");
  RevoluteJoint::Properties props;props.mAxis=Eigen::Vector3d::UnitZ();
  props.mName="primary";
  auto drive=robot->createJointAndBodyNodePair<RevoluteJoint>(nullptr,props);
  props.mName="follower";
  auto follow=robot->createJointAndBodyNodePair<RevoluteJoint>(nullptr,props);
  drive.first->setActuatorType(Joint::SERVO);
  follow.first->setActuatorType(Joint::PASSIVE);
  drive.first->setForceLowerLimit(0,-10.);drive.first->setForceUpperLimit(0,10.);
  follow.first->setForceLowerLimit(0,-.1);follow.first->setForceUpperLimit(0,.1);
  robot->setTimeStep(.001);robot->setGravity(Eigen::Vector3d::Zero());
  drive.first->setCommand(0,.1);
  robot->computeForwardDynamics();robot->integrateVelocities(.001);
  GroupSolver solver;solver.setTimeStep(.001);solver.addSkeleton(robot);
  solver.addConstraint(std::make_shared<workcell::JointCouplingConstraint>(
      follow.first,drive.first,ratio));
  solver.updateConstraints();
  auto cluster=Skeleton::create("contact_cluster");
  auto next=Skeleton::create("additional_contact_cluster");
  auto root=Skeleton::create("larger_contact_cluster");
  robot->mUnionRootSkeleton=cluster;
  cluster->mUnionRootSkeleton=deeper?next:root;
  next->mUnionRootSkeleton=root;
  solver.buildConstrainedGroups();
  const auto groups=solver.groupCount();
  solver.solveConstrainedGroups();robot->computeImpulseForwardDynamics();
  const double error=follow.first->getVelocity(0)-ratio*drive.first->getVelocity(0);
  std::cout<<"nested union ratio="<<ratio<<" deeper="<<deeper<<" groups="<<groups
           <<" coupling velocity error="<<error<<'\n';
  require(groups==1,"contact union split mimic and native servo rows");
  require(std::abs(error)<1e-8,"later group discarded the passive coupling impulse");
  require(drive.first->getVelocity(0)>0.,"coupled servo did not move");
  require(follow.first->getForceUpperLimit(0)==.1,"follower actuator cap changed");
}

int main() try {
  for (double ratio : {1., -1.}) {
    for (bool deeper : {false,true}) nestedContactUnion(ratio,deeper);
    for (bool gear : {false, true}) {
      auto skeleton=dart::dynamics::Skeleton::create("coupled_rotors");
      dart::dynamics::RevoluteJoint::Properties props;
      props.mAxis=Eigen::Vector3d::UnitZ();
      auto drive=skeleton->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(nullptr,props);
      auto follow=skeleton->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(nullptr,props);
      for (auto joint : {drive.first,follow.first}) {
        joint->getChildBodyNode()->setMomentOfInertia(1.,1.,1.);
        joint->setForceLowerLimit(0,-.1);joint->setForceUpperLimit(0,.1);
      }
      drive.first->setForceLowerLimit(0,-10.);drive.first->setForceUpperLimit(0,10.);
      drive.first->setActuatorType(dart::dynamics::Joint::SERVO);
      follow.first->setActuatorType(dart::dynamics::Joint::SERVO);
      auto world=dart::simulation::World::create();world->setGravity(Eigen::Vector3d::Zero());
      world->setTimeStep(.001);world->addSkeleton(skeleton);
      if (gear)world->getConstraintSolver()->addConstraint(
          std::make_shared<workcell::JointCouplingConstraint>(follow.first,drive.first,ratio));
      for (int tick=0;tick<300;++tick) {
        drive.first->setCommand(0,1.);
        follow.first->setCommand(0,100.*(ratio*drive.first->getPosition(0)-follow.first->getPosition(0)));
        world->step();
      }
      const double error=follow.first->getPosition(0)-ratio*drive.first->getPosition(0);
      std::cout << "ratio="<<ratio<<" gear="<<gear<<" error="<<error
                <<" drive="<<drive.first->getPosition(0)<<"\n";
      require(follow.first->getForceUpperLimit(0)==.1,"follower actuator cap changed");
      if (gear) {
        require(std::abs(error)<1e-6,"passive relation not preserved under drive load");
        require(drive.first->getPosition(0)>.05,"servo driver stalled under passive rows");
      } else require(std::abs(error)>.01,"bounded servo did not reproduce loaded relation error");
    }
  }
  return 0;
} catch (const std::exception& error) {
  std::cerr<<error.what()<<'\n';
  return 1;
}
