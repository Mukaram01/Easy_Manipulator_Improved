#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/simulation/World.hpp>
#include <iostream>
#include <stdexcept>
#include "JointCouplingConstraint.hpp"

void require(bool condition, const char* reason) {
  if (!condition) throw std::runtime_error(reason);
}

int main() {
  for (double ratio : {1., -1.}) {
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
      drive.first->setActuatorType(dart::dynamics::Joint::FORCE);
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
        require(drive.first->getPosition(0)<.03,"follower load did not react on drive");
      } else require(std::abs(error)>.01,"bounded servo did not reproduce loaded relation error");
    }
  }
}
