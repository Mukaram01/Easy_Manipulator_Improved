#include <gz/plugin/Loader.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Joint.hh>
#include <limits>
#include <gz/physics/RemoveEntities.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructLink.hh>
#include <gz/physics/sdf/ConstructJoint.hh>
#include <gz/physics/dartsim/World.hh>
#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <sdf/Root.hh>
#include <sdf/Model.hh>
#include <sdf/Link.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/World.hh>
#include <iostream>
using namespace ignition;
struct Features:physics::FeatureList<physics::GetEntities,physics::ForwardStep,
 physics::RemoveModelFromWorld,physics::dartsim::RetrieveWorld,physics::SetJointVelocityCommandFeature,physics::SetBasicJointState,
 physics::sdf::ConstructSdfWorld,physics::sdf::ConstructSdfModel,
 physics::sdf::ConstructSdfLink,physics::sdf::ConstructSdfJoint>{};
void require(bool b,const char* message){if(!b)throw std::runtime_error(message);}
int run(int argc,char**argv){
 require(argc==4,"need candidate library, authored robot SDF, mode");
 int mode=std::stoi(argv[3]);
 plugin::Loader loader;loader.LoadLib(argv[1]);
 auto plugin=loader.Instantiate("ignition::physics::dartsim::Plugin");
 auto engine=physics::RequestEngine3d<Features>::From(plugin);require(engine!=nullptr,"engine unavailable");
 sdf::Root root;root.Load(argv[2]);const auto* authored=root.Model();require(authored,"model absent");
 for(bool piecemeal:{false,true}){
  sdf::World worldSdf;worldSdf.SetName(piecemeal?"piecemeal":"whole");worldSdf.SetGravity(math::Vector3d::Zero);
  auto world=engine->ConstructWorld(worldSdf);require(world!=nullptr,"world construction");
  physics::Model3dPtr<Features> model;
  if(!piecemeal)model=world->ConstructModel(*authored);
  else{
   sdf::Model blank;blank.SetName(authored->Name());blank.SetRawPose(authored->RawPose());blank.SetStatic(false);
   model=world->ConstructModel(blank);
   for(uint64_t i=0;i<authored->LinkCount();++i)model->ConstructLink(*authored->LinkByIndex(i));
   for(uint64_t i=0;i<authored->JointCount();++i){
    const auto* original=authored->JointByIndex(i);sdf::Joint copy;
    copy.SetName(original->Name());copy.SetType(original->Type());copy.SetRawPose(original->RawPose());
    copy.SetParentLinkName(original->ParentLinkName());copy.SetChildLinkName(original->ChildLinkName());
    if(original->Axis(0))copy.SetAxis(0,*original->Axis(0));
    model->ConstructJoint(copy);
   }
  }
  require(model!=nullptr,"model construction");
  physics::ForwardStep::Input in;physics::ForwardStep::Output out;physics::ForwardStep::State state;
  world->Step(out,state,in);
  auto native=world->GetDartsimWorld();auto solver=native->getConstraintSolver();
  require(solver->getNumConstraints()==5,"canonical five passive relations not constructed");
  auto sk=native->getSkeleton(authored->Name());require(sk!=nullptr,"native model absent");
  auto* drive=sk->getJoint("gripper_finger1_joint");require(drive,"leader absent");
  for(uint64_t k=0;k<authored->JointCount();++k){auto sj=authored->JointByIndex(k);if(!sj->Axis(0)||!sj->Axis(0)->Element()->HasElement("mimic"))continue;
   auto api=model->GetJoint(sj->Name());auto follower=sk->getJoint(sj->Name());require(api!=nullptr,"public follower API missing");
   auto q=follower->getPosition(0),v=follower->getVelocity(0);
   api->SetVelocityCommand(0,1.);require(follower->getActuatorType()==dart::dynamics::Joint::PASSIVE,"publicvelocity command actuated passive follower");
   require(follower->getPosition(0)==q&&follower->getVelocity(0)==v,"virtual follower command overwrote measured state");
   api->SetForce(0,.01);require(follower->getActuatorType()==dart::dynamics::Joint::PASSIVE,"publicforce command actuated passive follower");
   require(follower->getPosition(0)==q&&follower->getVelocity(0)==v,"publicforce command overwrote measured follower state");
   api->SetForce(0,std::numeric_limits<double>::quiet_NaN());api->SetVelocityCommand(0,std::numeric_limits<double>::quiet_NaN());
   require(follower->getActuatorType()==dart::dynamics::Joint::PASSIVE,"invalid command changed passive mode");
  }
  for(int i=0;i<200;++i){
   for(auto j:sk->getJoints())if(j->getNumDofs())model->GetJoint(j->getName())->SetVelocityCommand(0,0.);
   if(mode==1){drive->setActuatorType(dart::dynamics::Joint::FORCE);drive->setCommand(0,1.);}
   else model->GetJoint("gripper_finger1_joint")->SetVelocityCommand(0,.5);
   for(uint64_t k=0;k<authored->JointCount();++k){
    auto sj=authored->JointByIndex(k);if(!sj->Axis(0)||!sj->Axis(0)->Element()->HasElement("mimic"))continue;
    auto meta=sj->Axis(0)->Element()->FindElement("mimic");double m=meta->Get<double>("multiplier",1.).first;
    auto* follower=sk->getJoint(sj->Name());
    if(mode==2){model->GetJoint(sj->Name())->SetVelocityCommand(0,100.*(m*drive->getPosition(0)-follower->getPosition(0)));}
    else follower->setCommand(0,100.*(m*drive->getPosition(0)-follower->getPosition(0)));
   }
   world->Step(out,state,in);
  }
  std::cout<<"actualrobot servo driver_q="<<drive->getPosition(0)<<" velocity="<<drive->getVelocity(0)<<" motorcap="<<drive->getForceUpperLimit(0)<<" constraintimpulse="<<drive->getConstraintImpulse(0)<<std::endl;
  bool moved=drive->getPosition(0)>.01;
  for(uint64_t i=0;i<authored->JointCount();++i){
   auto j=authored->JointByIndex(i);if(!j->Axis(0)||!j->Axis(0)->Element()->HasElement("mimic"))continue;
   auto m=j->Axis(0)->Element()->FindElement("mimic");double multiplier=m->Get<double>("multiplier",1.).first;
   auto* follow=sk->getJoint(j->Name());
   require(std::abs(follow->getPosition(0)-multiplier*drive->getPosition(0))<1e-6,"loaded mimic violated");
   require(follow->getForceUpperLimit(0)==j->Axis(0)->Effort(),"authored follower effort limit changed");
  }
  require(model->Remove(),"native model removal failed");
  require(solver->getNumConstraints()==0,"passive rows survived model removal");
  sk.reset();model=nullptr;for(int i=0;i<10;++i)world->Step(out,state,in);
  require(moved,"native servo gripper stalled");
  std::cout<<"piecemeal="<<piecemeal<<" authoredrows=5 limits=unchanged removedrows=0 subsequentsteps=10 PASS\n";
 }
 return 0;
}
int main(int argc,char**argv){try{return run(argc,argv);}catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}}
