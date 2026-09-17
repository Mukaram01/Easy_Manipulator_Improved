#include <gtest/gtest.h>
#include "support_contact_policy.hpp"

using workcell::SupportContact;
collision_detection::Contact contact(double depth) {
  collision_detection::Contact c;
  c.body_name_1="fixture"; c.body_type_1=collision_detection::BodyTypes::WORLD_OBJECT;
  c.body_name_2="part"; c.body_type_2=collision_detection::BodyTypes::ROBOT_ATTACHED;
  c.pos=Eigen::Vector3d(0,0,0); c.normal=Eigen::Vector3d::UnitZ(); c.depth=depth;
  return c;
}
TEST(SupportContact, InclusiveNumericalBoundary) {
  SupportContact policy{"part", "fixture", 0.0};
  for (double depth : {0.0, 1.44e-8, 0.000099, 0.0001}) EXPECT_TRUE(policy(contact(depth)));
  EXPECT_FALSE(policy(contact(0.000100001)));
  EXPECT_FALSE(policy(contact(0.001)));
}
TEST(SupportContact, OnlyIdentifiedPairAndUpwardFloor) {
  SupportContact policy{"part", "fixture", 0.0};
  auto c=contact(1e-8);
  c.body_name_1="unrelated"; EXPECT_FALSE(policy(c));
  c=contact(1e-8); c.body_name_2="other-part"; EXPECT_FALSE(policy(c));
  c=contact(1e-8); c.body_name_2="finger-tip"; c.body_type_2=collision_detection::BodyTypes::ROBOT_LINK;
  EXPECT_FALSE(policy(c));
  c=contact(1e-8); c.normal=Eigen::Vector3d::UnitX(); EXPECT_FALSE(policy(c));
  c=contact(1e-8); c.pos.z()=.2; EXPECT_FALSE(policy(c));
  c=contact(1e-8); c.normal=-Eigen::Vector3d::UnitZ(); EXPECT_FALSE(policy(c));
  c=contact(1e-8); std::swap(c.body_name_1,c.body_name_2); std::swap(c.body_type_1,c.body_type_2);
  c.normal=-c.normal; EXPECT_TRUE(policy(c));
}
TEST(SupportContact, InvalidNumericsFailClosed) {
  SupportContact policy{"part", "fixture", 0.0};
  EXPECT_FALSE(policy(contact(NAN))); EXPECT_FALSE(policy(contact(-1e-8)));
  auto c=contact(0); c.pos.x()=NAN; EXPECT_FALSE(policy(c));
}

// Exercise the actual adapter and MoveIt/FCL; only the OMPL response is supplied
// by the test. No alternative collision engine or policy is used.
#include "src_support_contact_adapter.cpp"
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

struct SupportFixture {
  planning_scene::PlanningScenePtr scene;
  planning_interface::MotionPlanRequest request;
  explicit SupportFixture(double penetration) {
    const std::string urdf=R"(<robot name="test"><link name="base"/><link name="tool"/>
      <joint name="lift" type="prismatic"><parent link="base"/><child link="tool"/>
      <axis xyz="0 0 1"/><limit lower="-.1" upper=".2" effort="10" velocity="1"/></joint></robot>)";
    auto robot=urdf::parseURDF(urdf); auto srdf=std::make_shared<srdf::Model>();
    srdf->initString(*robot,"<robot name='test'><group name='arm'><joint name='lift'/></group></robot>");
    auto model=std::make_shared<moveit::core::RobotModel>(robot,srdf);
    scene=std::make_shared<planning_scene::PlanningScene>(model);
    Eigen::Isometry3d floor=Eigen::Isometry3d::Identity(); floor.translation().z()=-.05;
    scene->getWorldNonConst()->addToObject("fixture",shapes::ShapeConstPtr(new shapes::Box(1,1,.1)),floor);
    auto& state=scene->getCurrentStateNonConst(); state.setToDefaultValues();
    Eigen::Isometry3d object=Eigen::Isometry3d::Identity(); object.translation().z()=.0125-penetration;
    EigenSTL::vector_Isometry3d poses{object};
    state.attachBody("part",Eigen::Isometry3d::Identity(),{shapes::ShapeConstPtr(new shapes::Box(.025,.025,.025))},
                     poses,std::set<std::string>{},"tool");
    state.update(); moveit::core::robotStateToRobotStateMsg(state,request.start_state);
    request.group_name="arm";
    request.path_constraints.name="workcell_initial_support_contact:{object_id: part, support_id: fixture, floor_z: 0, tool_link: tool}";
  }
  bool run(std::vector<double> heights={0.,.005}, bool* called=nullptr) {
    workcell::InitialSupportContact adapter;
    planning_interface::MotionPlanResponse response; std::vector<std::size_t> indexes;
    bool result=adapter.adaptAndPlan([&](const auto& private_scene,const auto&,auto& out) {
      if(called) *called=true;
      EXPECT_FALSE(private_scene->isStateColliding());
      out.trajectory_=std::make_shared<robot_trajectory::RobotTrajectory>(scene->getRobotModel(),"arm");
      for(double h:heights) {
        auto state=scene->getCurrentState(); state.setVariablePosition("lift",h); state.update();
        out.trajectory_->addSuffixWayPoint(state,.1);
      }
      out.error_code_.val=1; return true;
    },scene,request,response,indexes);
    EXPECT_TRUE(indexes.empty()); // No fabricated pipeline validation exceptions.
    return result;
  }
};
TEST(SupportAdapter, NumericalSupportStartAndStrictLift) {
  for(double depth : {0.,1.44e-8,0.00009}) {
    SupportFixture f(depth); bool called=false; EXPECT_TRUE(f.run({0.,.005},&called)); EXPECT_TRUE(called);
    EXPECT_TRUE(f.scene->getAllowedCollisionMatrix().getSize()==0); // private policy never leaks
  }
}
TEST(SupportAdapter, AboveToleranceNeverReachesPlanner) {
  SupportFixture f(.00011); bool called=false; EXPECT_FALSE(f.run({0.,.005},&called)); EXPECT_FALSE(called);
}
TEST(SupportAdapter, WrongSupportNeverReachesPlanner) {
  SupportFixture f(1.44e-8);
  f.scene->getWorldNonConst()->addToObject("wrong",shapes::ShapeConstPtr(new shapes::Box(1,1,.1)),Eigen::Isometry3d::Identity());
  f.request.path_constraints.name="workcell_initial_support_contact:{object_id: part, support_id: wrong, floor_z: 0, tool_link: tool}";
  bool called=false; EXPECT_FALSE(f.run({0.,.005},&called)); EXPECT_FALSE(called);
}
TEST(SupportAdapter, UnrelatedTinyObstacleRemainsCollision) {
  SupportFixture f(1.44e-8); Eigen::Isometry3d p=Eigen::Isometry3d::Identity(); p.translation()=Eigen::Vector3d(.025-1e-8,0,.0125);
  f.scene->getWorldNonConst()->addToObject("other",shapes::ShapeConstPtr(new shapes::Box(.025,.025,.025)),p);
  bool called=false; EXPECT_FALSE(f.run({0.,.005},&called)); EXPECT_FALSE(called);
}
TEST(SupportAdapter, CarriedCollisionAndReturnToSupportFail) {
  SupportFixture f(1.44e-8); EXPECT_FALSE(f.run({0.,-.002})); EXPECT_FALSE(f.run({0.,.005,0.}));
  EXPECT_FALSE(f.run({0.,0.,.005})); // Tolerance cannot persist at later stored waypoints.
}
TEST(SupportAdapter, MalformedRequestFailsClosed) {
  SupportFixture f(1.44e-8); f.request.path_constraints.name="workcell_initial_support_contact:{";
  bool called=false; EXPECT_FALSE(f.run({0.,.005},&called)); EXPECT_FALSE(called);
}

TEST(SupportAdapter, SameOwnerRaisedLipIsNotFloor) {
  SupportFixture f(1.44e-8-.02);
  Eigen::Isometry3d lip=Eigen::Isometry3d::Identity();lip.translation().z()=.015;
  f.scene->getWorldNonConst()->addToObject("fixture",shapes::ShapeConstPtr(new shapes::Box(.2,.2,.01)),lip);
  f.request.path_constraints.name="workcell_initial_support_contact:{object_id: part, support_id: fixture, floor_z: .02, tool_link: tool}";
  bool called=false;EXPECT_FALSE(f.run({0.,.005},&called));EXPECT_FALSE(called);
}
TEST(SupportAdapter, RevoluteSamplingIsBoundedInMetres) {
  const std::string xml=R"(<robot name="lever"><link name="base"/><link name="arm"/><link name="tool"/>
    <joint name="rotation" type="revolute"><parent link="base"/><child link="arm"/><axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" velocity="1" effort="1"/></joint>
    <joint name="offset" type="fixed"><parent link="arm"/><child link="tool"/><origin xyz="0 0 -2"/></joint></robot>)";
  auto u=urdf::parseURDF(xml);auto semantic=std::make_shared<srdf::Model>();semantic->initString(*u,"<robot name='lever'/>");
  auto model=std::make_shared<moveit::core::RobotModel>(u,semantic);
  moveit::core::RobotState a(model),b(model),mid(model);a.setToDefaultValues();b.setToDefaultValues();
  a.setVariablePosition("rotation",-.0005);b.setVariablePosition("rotation",.0005);a.update();b.update();
  a.interpolate(b,.5,mid);mid.update();
  // Equal endpoint heights conceal a downward dip. Joint-only .001-rad
  // subdivision samples endpoints; the metre bound forces interior samples.
  EXPECT_NEAR(a.getGlobalLinkTransform("tool").translation().z(),b.getGlobalLinkTransform("tool").translation().z(),1e-12);
  EXPECT_LT(mid.getGlobalLinkTransform("tool").translation().z(),a.getGlobalLinkTransform("tool").translation().z());
  const double bound=workcell::carriedTravelBound(a,b,.1);
  EXPECT_GE(bound,(a.getGlobalLinkTransform("tool").translation()-mid.getGlobalLinkTransform("tool").translation()).norm()*2);
  EXPECT_GT(std::ceil(bound/(workcell::support_contact_tolerance_m/4)),40);
}
TEST(SupportAdapter, TravelBoundIncludesMimicAmplification) {
  const std::string xml=R"(<robot name="mimic"><link name="base"/><link name="driver"/><link name="arm"/><link name="tool"/>
    <joint name="drive" type="revolute"><parent link="base"/><child link="driver"/><axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" velocity="1" effort="1"/></joint>
    <joint name="follower" type="revolute"><parent link="base"/><child link="arm"/><axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" velocity="1" effort="1"/><mimic joint="drive" multiplier="8"/></joint>
    <joint name="offset" type="fixed"><parent link="arm"/><child link="tool"/><origin xyz="2 0 0"/></joint></robot>)";
  auto u=urdf::parseURDF(xml);auto semantic=std::make_shared<srdf::Model>();semantic->initString(*u,"<robot name='mimic'/>");
  auto model=std::make_shared<moveit::core::RobotModel>(u,semantic);
  moveit::core::RobotState a(model),b(model);a.setToDefaultValues();b.setToDefaultValues();
  b.setVariablePosition("drive",.001);a.update();b.update();
  const double actual=(a.getGlobalLinkTransform("tool").translation()-b.getGlobalLinkTransform("tool").translation()).norm();
  EXPECT_GT(actual,.01);EXPECT_GE(workcell::carriedTravelBound(a,b,.1),actual);
}
