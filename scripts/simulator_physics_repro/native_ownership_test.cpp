// Actual engine API regression. This is unit evidence, not Stage-A runtime acceptance.
#include <gz/plugin/Loader.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/FixedJoint.hh>
#include <gz/physics/RevoluteJoint.hh>
#include <gz/physics/PrismaticJoint.hh>
#include <gz/physics/RemoveEntities.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructLink.hh>
#include <gz/physics/sdf/ConstructJoint.hh>
#include <gz/physics/dartsim/World.hh>
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
 physics::RemoveModelFromWorld,physics::dartsim::RetrieveWorld,physics::DetachJointFeature,
 physics::AttachFixedJointFeature,physics::AttachRevoluteJointFeature,physics::AttachPrismaticJointFeature,
 physics::sdf::ConstructSdfWorld,physics::sdf::ConstructSdfModel,
 physics::sdf::ConstructSdfLink,physics::sdf::ConstructSdfJoint>{};
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
template<class Fn>void rejected(Fn operation){bool caught=false;try{operation();}catch(const std::logic_error&){caught=true;}require(caught,"coupled topology mutation was not rejected before mutation");}
int run(int argc,char**argv){
 require(argc==4,"need exact engine, actual robot SDF, mode");
 plugin::Loader loader;require(!loader.LoadLib(argv[1]).empty(),"engine load failed");
 auto plugin=loader.Instantiate("ignition::physics::dartsim::Plugin");
 auto engine=physics::RequestEngine3d<Features>::From(plugin);require(engine!=nullptr,"requested actual API features unavailable");
 sdf::Root root;root.Load(argv[2]);require(root.Model(),"authored robot missing");
 const std::string mode=argv[3];
 sdf::World sdfWorld;sdfWorld.SetName("ownership");sdfWorld.SetGravity(math::Vector3d::Zero);
 auto world=engine->ConstructWorld(sdfWorld);require(world!=nullptr,"world missing");
 sdf::Model modelSdf(*root.Model());modelSdf.SetName("coupled");
 if(mode=="nested"){
   for(int i=0;i<3;++i){sdf::Model child(*root.Model());child.SetName("child"+std::to_string(i));child.SetRawPose(math::Pose3d(5*(i+1),0,0,0,0,0));require(modelSdf.AddModel(child),"nested fixture child rejected");}
 }
 auto model=world->ConstructModel(modelSdf);require(model!=nullptr,"actual authored model missing");
 physics::ForwardStep::Input in;physics::ForwardStep::Output out;physics::ForwardStep::State state;
 world->Step(out,state,in);
 auto native=world->GetDartsimWorld();auto solver=native->getConstraintSolver();
 require(solver->getNumConstraints()==(mode=="nested"?20u:5u),"expected actual authored rows absent");
 if(mode=="detach"){
   for(const auto& name:{"gripper_finger1_finger_tip_joint","gripper_finger1_joint","shoulder_pan_joint"}){
     auto joint=model->GetJoint(name);require(joint!=nullptr,"detach fixture joint absent");rejected([&]{joint->Detach();});
     require(solver->getNumConstraints()==5,"rejected detach changed rows");world->Step(out,state,in);
   }
 }else if(mode=="attach"){
   sdf::Model freeModel;freeModel.SetName("free_model");sdf::Link freeLink;freeLink.SetName("free_link");require(freeModel.AddLink(freeLink),"free fixture link rejected");
   auto outsider=world->ConstructModel(freeModel);require(outsider!=nullptr,"outside fixture absent");
   auto free=outsider->GetLink("free_link");auto coupled=model->GetLink("gripper_finger1_finger_tip_link");require(free&&coupled,"attach fixture link absent");
   rejected([&]{coupled->AttachFixedJoint(nullptr);});rejected([&]{coupled->AttachRevoluteJoint(nullptr);});rejected([&]{coupled->AttachPrismaticJoint(nullptr);});
   rejected([&]{free->AttachFixedJoint(coupled);});rejected([&]{free->AttachRevoluteJoint(coupled);});rejected([&]{free->AttachPrismaticJoint(coupled);});
   require(solver->getNumConstraints()==5,"rejected attach changed rows");world->Step(out,state,in);require(outsider->Remove(),"outside removal failed");
 }else if(mode=="sdf"){
   // An unrelated free-rooted ancestor can contain both coupled joints. Named
   // follower/driver replacement checks alone miss moving this subtree.
   auto joint=modelSdf.JointByName("shoulder_pan_joint");require(joint,"SDF fixture ancestor missing");
   sdf::Joint replacement(*joint);replacement.SetName("replacement_ancestor");
   rejected([&]{model->ConstructJoint(replacement);});
   sdf::Joint freeRoot;freeRoot.SetName("replacement_free_root");freeRoot.SetType(sdf::JointType::REVOLUTE);freeRoot.SetParentLinkName("world");freeRoot.SetChildLinkName("base_link");
   sdf::JointAxis axis;axis.SetXyz(math::Vector3d::UnitZ);freeRoot.SetAxis(0,axis);
   rejected([&]{model->ConstructJoint(freeRoot);});
   require(solver->getNumConstraints()==5,"rejected SDF mutation changed rows");world->Step(out,state,in);
 }else require(mode=="nested","unknown mode");
 require(model->Remove(),"parent model removal failed");
 require(solver->getNumConstraints()==0,"coupled child rows survived parent removal");
 require(native->getNumSkeletons()==0,"nested child skeleton survived parent removal");
 for(int i=0;i<10;++i)world->Step(out,state,in);
 std::cout<<"actual API "<<mode<<" rows_removed=all subsequentsteps=10 PASS\n";return 0;
}
int main(int argc,char**argv){try{return run(argc,argv);}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
