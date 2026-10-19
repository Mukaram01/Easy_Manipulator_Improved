#include <gtest/gtest.h>
#include <array>
#include <cfenv>
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


TEST(PileContact, ExactIdentitiesTypesAndFiniteGeometry) {
  workcell::PileContact policy{"part", "fixture"};
  for (double depth : {0., 0.0001}) EXPECT_TRUE(policy(contact(depth)));
  EXPECT_FALSE(policy(contact(0.000100001)));
  EXPECT_FALSE(policy(contact(-1e-8)));
  EXPECT_FALSE(policy(contact(NAN)));
  auto c=contact(0.); c.normal.x()=NAN; EXPECT_FALSE(policy(c));
  c=contact(0.); c.pos.z()=NAN; EXPECT_FALSE(policy(c));
  c=contact(0.); c.body_name_1="fixture_extra"; EXPECT_FALSE(policy(c));
  c=contact(0.); c.body_type_2=collision_detection::BodyTypes::ROBOT_LINK;
  EXPECT_FALSE(policy(c));
  c=contact(0.); std::swap(c.body_name_1,c.body_name_2);
  std::swap(c.body_type_1,c.body_type_2); c.normal=-c.normal;
  EXPECT_TRUE(policy(c));
}

TEST(PileContact, RuntimeAbiUsesExactPairAndSharedDepthBound) {
  double point[7]{0,0,0,1,0,0,.0001};
  EXPECT_TRUE(workcell_pile_contact_valid("part","neighbor","part","neighbor",point));
  EXPECT_TRUE(workcell_pile_contact_valid("part","neighbor","neighbor","part",point));
  EXPECT_FALSE(workcell_pile_contact_valid("part","neighbor","part","neighbor_extra",point));
  EXPECT_FALSE(workcell_pile_contact_valid("part","neighbor","finger","neighbor",point));
  EXPECT_FALSE(workcell_pile_contact_valid("part","part","part","part",point));
  EXPECT_FALSE(workcell_pile_contact_valid(nullptr,"neighbor","part","neighbor",point));
  point[6]=.000100001;
  EXPECT_FALSE(workcell_pile_contact_valid("part","neighbor","part","neighbor",point));
}

struct MeasuredBoxes {
  double size[3]{.025,.025,.025};
  double target[7]{0,0,0,0,0,0,1};
  double neighbor[7]{.025,0,0,0,0,0,1};
  double points[6]{.0125,0,0,.0125,.005,0};
  double evidence[8];
  bool run(std::size_t count=2) {
    return workcell_measured_pile_contact(size,target,size,neighbor,points,count,evidence);
  }
};
TEST(MeasuredPileContact, ShallowContactUsesActualFclDepthAndNormal) {
  MeasuredBoxes b;
  b.neighbor[0]=.025-.00005;
  ASSERT_TRUE(b.run());
  EXPECT_NEAR(b.evidence[0],.00005,1e-12);
  EXPECT_DOUBLE_EQ(b.evidence[1],0.);
  EXPECT_NEAR(b.evidence[2],1.,1e-12);
  for (double value:b.evidence) EXPECT_TRUE(std::isfinite(value));
  b.neighbor[0]=.025-.000099999999;
  EXPECT_TRUE(b.run());
  b.neighbor[0]=.025-.000100001;
  EXPECT_FALSE(b.run());
  EXPECT_GT(b.evidence[0],.0001);
}
TEST(MeasuredPileContact, SeparatedGeometryUsesFclNearestPoints) {
  MeasuredBoxes b;
  b.neighbor[0]=.025+.00005;
  ASSERT_TRUE(b.run());
  EXPECT_DOUBLE_EQ(b.evidence[0],0.);
  EXPECT_NEAR(b.evidence[1],.00005,1e-12);
  EXPECT_NEAR(b.evidence[2],1.,1e-9);
  b.neighbor[0]=.025+.000100001;
  EXPECT_FALSE(b.run());
  EXPECT_GT(b.evidence[1],.0001);
  EXPECT_FALSE(b.run(0));
  EXPECT_GT(b.evidence[1],.0001);
  b.neighbor[0]=.2;
  EXPECT_FALSE(b.run());
}
TEST(MeasuredPileContact, EveryPhysicalPointMustMatchBothBoxSurfaces) {
  MeasuredBoxes b;
  b.neighbor[0]=.025-.00005;
  ASSERT_TRUE(b.run());
  b.points[3]=.011;
  EXPECT_FALSE(b.run());
  b.points[3]=.0125; b.points[4]=.1;
  EXPECT_FALSE(b.run());
  b.points[4]=NAN;
  EXPECT_FALSE(b.run());
}
TEST(MeasuredPileContact, RotatedMeasuredBoxesUseFullPose) {
  MeasuredBoxes b;
  const Eigen::AngleAxisd rotation(.71,Eigen::Vector3d(1,2,3).normalized());
  const Eigen::Quaterniond q(rotation);
  Eigen::Map<Eigen::Vector3d>(b.neighbor)=rotation*Eigen::Vector3d(.025-.00005,0,0);
  for (double* pose:{b.target,b.neighbor}) {
    pose[3]=q.x(); pose[4]=q.y(); pose[5]=q.z(); pose[6]=q.w();
  }
  for (unsigned int i=0;i<2;++i)
    Eigen::Map<Eigen::Vector3d>(b.points+3*i)=rotation*Eigen::Vector3d(.0125,.005*i,0);
  ASSERT_TRUE(b.run());
  EXPECT_NEAR(b.evidence[0],.00005,1e-12);
  EXPECT_NEAR((Eigen::Map<Eigen::Vector3d>(b.evidence+2)-rotation*Eigen::Vector3d::UnitX()).norm(),0.,1e-10);
}
TEST(MeasuredPileContact, InvalidInputsFailClosedWithDeterministicEvidence) {
  MeasuredBoxes b;
  EXPECT_FALSE(b.run(0));
  EXPECT_DOUBLE_EQ(b.evidence[1],0.);
  b.size[0]=-1.; EXPECT_FALSE(b.run());
  for (double value:b.evidence) EXPECT_TRUE(std::isnan(value));
  b.size[0]=INFINITY; EXPECT_FALSE(b.run());
  b.size[0]=.025; b.target[6]=0.; EXPECT_FALSE(b.run());
  b.target[6]=2.; EXPECT_FALSE(b.run());
  b.target[6]=1.; b.target[0]=NAN; EXPECT_FALSE(b.run());
  EXPECT_FALSE(workcell_measured_pile_contact(nullptr,b.target,b.size,b.neighbor,b.points,2,b.evidence));
  EXPECT_FALSE(workcell_measured_pile_contact(b.size,b.target,b.size,b.neighbor,b.points,2,nullptr));
}

// Literal measured BOX poses and physical points from the preserved stationary
// failure evidence, iteration 41672 (2026-09-23). This protects the escaped
// predicted-pose mismatch: the physical target touches all five neighbors.
TEST(MeasuredPileContact, RecordedFivePhysicalNeighborsUseMeasuredPoses) {
  const double size[3]{.025,.025,.025};
  const double target[7]{0.3847929167274054,-0.22440961888336086,0.03749985893218837,2.137887752146462e-07,6.314741267587348e-07,0.12332964599661471,0.9923657583864456};
  struct Pair { const char* name; std::array<double,7> pose; std::vector<double> points; };
  const std::vector<Pair> pairs{
    {"part_00",{0.3699988381564545,-0.2500016810888486,0.01249997994770509,-2.4633481557677397e-06,2.0680720280557986e-06,0.07496885809228575,0.9971858755046592},{0.3757328446599582,-0.2395890099707328,0.02499986318150591,0.3749798463463771,-0.23660633331083447,0.024999865917407035,0.38048941764208033,-0.23577320105809152,0.024999857618738457,0.3808703022046534,-0.23829202162490176,0.024999855712364953}},
    {"part_01",{0.3999455376089018,-0.25000172020418737,0.012499314673824792,2.5486464821529496e-05,-3.486050750188227e-06,-0.09970726486117532,0.995016814266066},{0.39165272378933497,-0.2355699212444393,0.02499988853788514}},
    {"part_03",{0.3700000002546573,-0.2000000024884246,0.01249992871028953,3.579333205517817e-07,1.016065358196973e-06,-0.17410826120679215,0.9847265170484579},{0.3785249467357782,-0.21309974535742393,0.02499991347448979}},
    {"part_04",{0.39999891896792333,-0.19999992561312946,0.01249996098772407,1.307694556660767e-06,-5.755825687317762e-07,0.04996377346971057,0.9987510307071658},{0.38849760152292684,-0.2105820764669521,0.024999857960547642,0.39385289477595187,-0.20923009391969913,0.024999850174704186,0.39482523651048873,-0.21308160453365457,0.024999846641853195,0.3888088408427896,-0.21368506715530255,0.024999855862355914}},
    {"part_07",{0.4152177426999811,-0.22541668722725056,0.03749878236489574,3.317809853697e-05,7.195735808274394e-06,-0.13646492327064116,0.9906449028608451},{0.39997232774344743,-0.23346957700321053,0.024999827940705502}},
  };
  for (const auto& pair:pairs) {
    SCOPED_TRACE(pair.name);
    double evidence[8];
    EXPECT_TRUE(workcell_measured_pile_contact(size,target,size,pair.pose.data(),
      pair.points.data(),pair.points.size()/3,evidence));
    EXPECT_LE(evidence[0],.0001);
    EXPECT_LE(evidence[1],.0001);
  }
}


class PrismaticTestIK : public kinematics::KinematicsBase {
public:
  PrismaticTestIK() {
    setValues("test_robot", "arm", "base", {"tool"}, 0.001);
  }

  bool solve(const geometry_msgs::msg::Pose& pose,
             std::vector<double>& solution,
             moveit_msgs::msg::MoveItErrorCodes& error_code,
             const IKCallbackFn& callback = IKCallbackFn()) const {
    const double qnorm =
      pose.orientation.x*pose.orientation.x +
      pose.orientation.y*pose.orientation.y +
      pose.orientation.z*pose.orientation.z +
      pose.orientation.w*pose.orientation.w;
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
        !std::isfinite(pose.position.z) || !std::isfinite(qnorm) ||
        std::abs(pose.position.x)>1e-9 || std::abs(pose.position.y)>1e-9 ||
        pose.position.z < -0.1-1e-12 || pose.position.z > 0.2+1e-12 ||
        std::abs(pose.orientation.x)>1e-9 ||
        std::abs(pose.orientation.y)>1e-9 ||
        std::abs(pose.orientation.z)>1e-9 ||
        std::abs(std::abs(pose.orientation.w)-1.0)>1e-9) {
      error_code.val=moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
      return false;
    }
    solution={pose.position.z};
    error_code.val=moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    if (callback) {
      callback(pose,solution,error_code);
      return error_code.val==moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    }
    return true;
  }

  bool getPositionIK(
      const geometry_msgs::msg::Pose& pose,
      const std::vector<double>&,
      std::vector<double>& solution,
      moveit_msgs::msg::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions&) const override {
    return solve(pose,solution,error_code);
  }

  bool searchPositionIK(
      const geometry_msgs::msg::Pose& pose,
      const std::vector<double>&,
      double,
      std::vector<double>& solution,
      moveit_msgs::msg::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions&) const override {
    return solve(pose,solution,error_code);
  }

  bool searchPositionIK(
      const geometry_msgs::msg::Pose& pose,
      const std::vector<double>&,
      double,
      const std::vector<double>&,
      std::vector<double>& solution,
      moveit_msgs::msg::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions&) const override {
    return solve(pose,solution,error_code);
  }

  bool searchPositionIK(
      const geometry_msgs::msg::Pose& pose,
      const std::vector<double>&,
      double,
      std::vector<double>& solution,
      const IKCallbackFn& callback,
      moveit_msgs::msg::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions&) const override {
    return solve(pose,solution,error_code,callback);
  }

  bool searchPositionIK(
      const geometry_msgs::msg::Pose& pose,
      const std::vector<double>&,
      double,
      const std::vector<double>&,
      std::vector<double>& solution,
      const IKCallbackFn& callback,
      moveit_msgs::msg::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions&) const override {
    return solve(pose,solution,error_code,callback);
  }

  bool getPositionFK(
      const std::vector<std::string>& link_names,
      const std::vector<double>& joint_angles,
      std::vector<geometry_msgs::msg::Pose>& poses) const override {
    if (joint_angles.size()!=1) return false;
    poses.clear();
    poses.reserve(link_names.size());
    for (const auto& link:link_names) {
      geometry_msgs::msg::Pose pose;
      pose.orientation.w=1.0;
      if (link=="tool") pose.position.z=joint_angles[0];
      else if (link!="base") return false;
      poses.push_back(pose);
    }
    return true;
  }

  const std::vector<std::string>& getJointNames() const override {
    return joint_names_;
  }

  const std::vector<std::string>& getLinkNames() const override {
    return link_names_;
  }

private:
  const std::vector<std::string> joint_names_{"lift"};
  const std::vector<std::string> link_names_{"base","tool"};
};

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
    model->setKinematicsAllocators({
      {"arm", [](const moveit::core::JointModelGroup*) {
        return std::make_shared<PrismaticTestIK>();
      }}
    });
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
  void addPileNeighbor(double overlap) {
    const auto& state=scene->getCurrentState();
    const auto* body=state.getAttachedBody("part");
    ASSERT_NE(body,nullptr);
    ASSERT_EQ(body->getGlobalCollisionBodyTransforms().size(),1U);
    // AttachedBody::getGlobalPose() is the attached-frame transform, while the
    // BOX itself is offset by shape_poses. Build the neighbour against the
    // actual global collision geometry so this fixture really exercises
    // carried-object/world contact depth.
    const double bottom=
      body->getGlobalCollisionBodyTransforms()[0].translation().z()-.0125;
    Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();
    pose.translation().z()=bottom-.0125+overlap;
    scene->getWorldNonConst()->addToObject(
      "pile_neighbor",shapes::ShapeConstPtr(new shapes::Box(.025,.025,.025)),pose);

    collision_detection::CollisionRequest request;
    request.contacts=true;
    request.max_contacts=32;
    request.max_contacts_per_pair=16;
    request.group_name="arm";
    collision_detection::CollisionResult result;
    scene->checkCollision(request,result,state);
    ASSERT_TRUE(result.collision);
    ASSERT_FALSE(result.contacts.empty());
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
        out.trajectory_->addSuffixWayPoint(state,out.trajectory_->getWayPointCount() ? .1 : 0.);
      }
      out.error_code_.val=1; return true;
    },scene,request,response,indexes);
    EXPECT_TRUE(indexes.empty()); // No fabricated pipeline validation exceptions.
    return result;
  }
};
TEST(CartesianAdapter, StraightLiftUsesEffectivePrivateSupportScene) {
  SupportFixture f(1.44e-8);
  moveit_msgs::msg::Constraints marker;
  marker.name=R"(workcell_cartesian_path:{"allow_initial_attached_world_separation":false,"goal_pose":[0,0,0.005,0,0,0,1],"initial_separation_object_ids":[],"max_step_m":0.001,"schema":"workcell_cartesian_path/v1","stage":"PREPLAN_LIFT","start_pose":[0,0,0,0,0,0,1],"tool_link":"tool"})";
  f.request.trajectory_constraints.constraints={marker};

  workcell::InitialSupportContact support;
  workcell::StraightCartesianPath cartesian;
  planning_interface::MotionPlanResponse response;
  std::vector<std::size_t> indexes;
  bool downstream_called=false;
  const bool result=support.adaptAndPlan(
    [&](const auto& private_scene,const auto& clean,auto& out) {
      std::vector<std::size_t> nested_indexes;
      return cartesian.adaptAndPlan(
        [&](const auto&,const auto&,auto&) {
          downstream_called=true;
          return false;
        },
        private_scene,clean,out,nested_indexes);
    },
    f.scene,f.request,response,indexes);

  ASSERT_TRUE(result);
  EXPECT_FALSE(downstream_called);
  ASSERT_TRUE(response.trajectory_);
  ASSERT_GE(response.trajectory_->getWayPointCount(),2U);
  EXPECT_NEAR(response.trajectory_->getFirstWayPoint().getVariablePosition("lift"),0.,1e-12);
  EXPECT_NEAR(response.trajectory_->getLastWayPoint().getVariablePosition("lift"),.005,1e-6);
  EXPECT_EQ(response.error_code_.val,moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}

TEST(CartesianAdapter, CertifiedInitialPileContactSeparatesDuringLift) {
  SupportFixture f(1.44e-8);
  f.addPileNeighbor(0.00005);
  moveit_msgs::msg::Constraints marker;
  marker.name=R"(workcell_cartesian_path:{"allow_initial_attached_world_separation":true,"goal_pose":[0,0,0.005,0,0,0,1],"initial_separation_object_ids":["pile_neighbor"],"max_step_m":0.001,"schema":"workcell_cartesian_path/v1","stage":"PREPLAN_LIFT","start_pose":[0,0,0,0,0,0,1],"tool_link":"tool"})";
  f.request.trajectory_constraints.constraints={marker};

  workcell::InitialSupportContact support;
  workcell::StraightCartesianPath cartesian;
  planning_interface::MotionPlanResponse response;
  std::vector<std::size_t> indexes;
  bool downstream_called=false;
  const bool result=support.adaptAndPlan(
    [&](const auto& private_scene,const auto& clean,auto& out) {
      std::vector<std::size_t> nested_indexes;
      return cartesian.adaptAndPlan(
        [&](const auto&,const auto&,auto&) {
          downstream_called=true;
          return false;
        },
        private_scene,clean,out,nested_indexes);
    },
    f.scene,f.request,response,indexes);

  ASSERT_TRUE(result);
  EXPECT_FALSE(downstream_called);
  ASSERT_TRUE(response.trajectory_);
  EXPECT_NEAR(response.trajectory_->getLastWayPoint().getVariablePosition("lift"),.005,1e-6);
}

TEST(CartesianAdapter, InitialPileContactAboveNumericalToleranceFailsClosed) {
  SupportFixture f(1.44e-8);
  f.addPileNeighbor(0.00011);
  moveit_msgs::msg::Constraints marker;
  marker.name=R"(workcell_cartesian_path:{"allow_initial_attached_world_separation":true,"goal_pose":[0,0,0.005,0,0,0,1],"initial_separation_object_ids":["pile_neighbor"],"max_step_m":0.001,"schema":"workcell_cartesian_path/v1","stage":"PREPLAN_LIFT","start_pose":[0,0,0,0,0,0,1],"tool_link":"tool"})";
  f.request.trajectory_constraints.constraints={marker};

  workcell::InitialSupportContact support;
  workcell::StraightCartesianPath cartesian;
  planning_interface::MotionPlanResponse response;
  std::vector<std::size_t> indexes;
  bool downstream_called=false;
  EXPECT_FALSE(support.adaptAndPlan(
    [&](const auto& private_scene,const auto& clean,auto& out) {
      std::vector<std::size_t> nested_indexes;
      return cartesian.adaptAndPlan(
        [&](const auto&,const auto&,auto&) {
          downstream_called=true;
          return true;
        },
        private_scene,clean,out,nested_indexes);
    },
    f.scene,f.request,response,indexes));
  EXPECT_FALSE(downstream_called);
  EXPECT_EQ(response.error_code_.val,moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
}

TEST(CartesianAdapter, MalformedMetadataFailsClosedWithoutPlannerFallback) {
  SupportFixture f(0.);
  f.request.path_constraints.name.clear();
  moveit_msgs::msg::Constraints marker;
  marker.name="workcell_cartesian_path:{";
  f.request.trajectory_constraints.constraints={marker};
  workcell::StraightCartesianPath cartesian;
  planning_interface::MotionPlanResponse response;
  std::vector<std::size_t> indexes;
  bool downstream_called=false;
  EXPECT_FALSE(cartesian.adaptAndPlan(
    [&](const auto&,const auto&,auto&) {
      downstream_called=true;
      return true;
    },
    f.scene,f.request,response,indexes));
  EXPECT_FALSE(downstream_called);
  EXPECT_EQ(response.error_code_.val,moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
  EXPECT_FALSE(response.trajectory_);
}

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

// Ordinary requests must audit the same p/v/a interpolation that the controller
// executes, even when both MoveIt response waypoints are collision-free.
robot_trajectory::RobotTrajectoryPtr ordinaryQuintic(
    const SupportFixture& fixture, double duration=1., double velocity=.2,
    double acceleration=0., double first_delay=0.) {
  auto trajectory=std::make_shared<robot_trajectory::RobotTrajectory>(
    fixture.scene->getRobotModel(),"arm");
  for (unsigned i=0;i<2;++i) {
    auto state=fixture.scene->getCurrentState();
    state.setVariablePosition("lift",0.);
    state.setVariableVelocity("lift",i ? -velocity : velocity);
    state.setVariableAcceleration("lift",acceleration);
    state.update();
    trajectory->addSuffixWayPoint(state,i ? duration : first_delay);
  }
  return trajectory;
}

void prepareOrdinaryRequest(SupportFixture& fixture, double obstacle_z) {
  fixture.scene->getWorldNonConst()->removeObject("fixture");
  fixture.request.path_constraints.name.clear();
  Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();
  pose.translation().z()=obstacle_z;
  fixture.scene->getWorldNonConst()->addToObject(
    "spline_obstacle",shapes::ShapeConstPtr(new shapes::Box(.025,.025,.005)),pose);
}

TEST(CartesianAdapter, OrdinaryQuinticOvershootCollisionRejectsClearWaypoints) {
  SupportFixture fixture(0.); prepareOrdinaryRequest(fixture,.075);
  auto planned=ordinaryQuintic(fixture);
  ASSERT_FALSE(fixture.scene->isStateColliding(planned->getFirstWayPoint(),"arm"));
  ASSERT_FALSE(fixture.scene->isStateColliding(planned->getLastWayPoint(),"arm"));
  // These p/v/a endpoints produce q(.5)=.0625, so the carried BOX intersects
  // the obstacle even though q(0)=q(1)=0 are both clear and within bounds.
  auto interior=fixture.scene->getCurrentState();
  interior.setVariablePosition("lift",.0625); interior.update();
  ASSERT_TRUE(fixture.scene->isStateColliding(interior,"arm"));
  workcell::StraightCartesianPath adapter;
  planning_interface::MotionPlanResponse response; std::vector<std::size_t> indexes;
  unsigned calls=0;
  EXPECT_FALSE(adapter.adaptAndPlan([&](const auto&,const auto&,auto& out) {
    ++calls;out.trajectory_=planned;
    out.error_code_.val=moveit_msgs::msg::MoveItErrorCodes::SUCCESS;return true;
  },fixture.scene,fixture.request,response,indexes));
  EXPECT_EQ(calls,1U); EXPECT_TRUE(indexes.empty());
  EXPECT_EQ(response.error_code_.val,moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
  EXPECT_FALSE(response.trajectory_);
}

TEST(CartesianAdapter, RejectedCertificateTracePreservesExactInputWithoutPromotingResult) {
  SupportFixture fixture(0.);prepareOrdinaryRequest(fixture,.075);
  auto trajectory=ordinaryQuintic(fixture);
  moveit_msgs::msg::RobotTrajectory expected_trajectory;trajectory->getRobotTrajectoryMsg(expected_trajectory);
  moveit_msgs::msg::PlanningScene expected_scene;fixture.scene->getPlanningSceneMsg(expected_scene);
  char name[]="/tmp/workcell-certificate-trace-XXXXXX";
  const char* directory=mkdtemp(name);ASSERT_NE(directory,nullptr);
  struct RestoreTrace {
    std::string directory,prior;
    bool had_prior;
    ~RestoreTrace() {
      if(had_prior) setenv("WORKCELL_PLANNING_TRACE_DIR",prior.c_str(),1);
      else unsetenv("WORKCELL_PLANNING_TRACE_DIR");
      std::filesystem::remove_all(directory);
    }
  } restore{directory,std::getenv("WORKCELL_PLANNING_TRACE_DIR")?std::getenv("WORKCELL_PLANNING_TRACE_DIR"):"",
            std::getenv("WORKCELL_PLANNING_TRACE_DIR")!=nullptr};
  ASSERT_EQ(setenv("WORKCELL_PLANNING_TRACE_DIR",directory,1),0);
  workcell::StraightCartesianPath adapter;
  planning_interface::MotionPlanResponse response;std::vector<std::size_t> indexes;
  auto plan=[&](const auto&,const auto&,auto& out) {
    out.trajectory_=trajectory;out.error_code_.val=moveit_msgs::msg::MoveItErrorCodes::SUCCESS;return true;
  };
  EXPECT_FALSE(adapter.adaptAndPlan(plan,fixture.scene,fixture.request,response,indexes));
  EXPECT_FALSE(response.trajectory_);
  EXPECT_EQ(response.error_code_.val,moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
  std::filesystem::path trajectory_path,scene_path,result_path;std::size_t files=0;
  for(const auto& entry:std::filesystem::directory_iterator(directory)) {
    ++files;const auto filename=entry.path().filename().string();
    if(filename.find(".trajectory.cdr")!=std::string::npos) trajectory_path=entry.path();
    else if(filename.find(".scene.cdr")!=std::string::npos) scene_path=entry.path();
    else if(filename.find(".result.txt")!=std::string::npos) result_path=entry.path();
  }
  ASSERT_EQ(files,3U);ASSERT_FALSE(trajectory_path.empty());ASSERT_FALSE(scene_path.empty());ASSERT_FALSE(result_path.empty());
  auto deserialize=[](const std::filesystem::path& path,auto& message) {
    std::ifstream input(path,std::ios::binary);
    const std::string bytes{std::istreambuf_iterator<char>(input),std::istreambuf_iterator<char>()};
    rclcpp::SerializedMessage serialized(bytes.size());auto& raw=serialized.get_rcl_serialized_message();
    std::memcpy(raw.buffer,bytes.data(),bytes.size());raw.buffer_length=bytes.size();
    rclcpp::Serialization<std::decay_t<decltype(message)>>().deserialize_message(&serialized,&message);
  };
  moveit_msgs::msg::RobotTrajectory recorded_trajectory;deserialize(trajectory_path,recorded_trajectory);
  moveit_msgs::msg::PlanningScene recorded_scene;deserialize(scene_path,recorded_scene);
  EXPECT_EQ(recorded_trajectory,expected_trajectory);
  EXPECT_EQ(recorded_scene.robot_state,expected_scene.robot_state);
  EXPECT_EQ(recorded_scene.world,expected_scene.world);
  EXPECT_EQ(recorded_scene.allowed_collision_matrix,expected_scene.allowed_collision_matrix);
  std::ifstream result(result_path);const std::string text{std::istreambuf_iterator<char>(result),std::istreambuf_iterator<char>()};
  EXPECT_NE(text.find("result 1\nreason FCL_COLLISION\n"),std::string::npos);
  unsetenv("WORKCELL_PLANNING_TRACE_DIR");
  EXPECT_FALSE(adapter.adaptAndPlan(plan,fixture.scene,fixture.request,response,indexes));
  EXPECT_EQ(std::distance(std::filesystem::directory_iterator(directory),std::filesystem::directory_iterator()),3);
}

TEST(CartesianAdapter, OrdinaryClearQuinticRetainsSuccessfulPlannerResult) {
  SupportFixture fixture(0.); prepareOrdinaryRequest(fixture,.15);
  auto planned=ordinaryQuintic(fixture);
  workcell::StraightCartesianPath adapter;
  planning_interface::MotionPlanResponse response; std::vector<std::size_t> indexes;
  unsigned calls=0;
  EXPECT_TRUE(adapter.adaptAndPlan([&](const auto&,const auto&,auto& out) {
    ++calls;out.trajectory_=planned;
    out.error_code_.val=moveit_msgs::msg::MoveItErrorCodes::SUCCESS;return true;
  },fixture.scene,fixture.request,response,indexes));
  EXPECT_EQ(calls,1U); EXPECT_TRUE(indexes.empty());
  EXPECT_EQ(response.error_code_.val,moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  EXPECT_EQ(response.trajectory_,planned);
}

TEST(CartesianAdapter, OrdinaryNonfiniteStateAndInvalidTimingFailClosed) {
  struct Input { double duration,velocity,acceleration; double first_delay=0.; };
  for (const auto& input:std::vector<Input>{{0.,.2,0.},{-1.,.2,0.},
      {NAN,.2,0.},{1.,NAN,0.},{1.,.2,INFINITY},{1.,.2,0.,.1}}) {
    SCOPED_TRACE(::testing::Message()<<"duration="<<input.duration
      <<" velocity="<<input.velocity<<" acceleration="<<input.acceleration
      <<" first_delay="<<input.first_delay);
    SupportFixture fixture(0.); prepareOrdinaryRequest(fixture,.15);
    auto planned=ordinaryQuintic(fixture,input.duration,input.velocity,input.acceleration,input.first_delay);
    workcell::StraightCartesianPath adapter;
    planning_interface::MotionPlanResponse response; std::vector<std::size_t> indexes;
    EXPECT_FALSE(adapter.adaptAndPlan([&](const auto&,const auto&,auto& out) {
      out.trajectory_=planned;out.error_code_.val=moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      return true;
    },fixture.scene,fixture.request,response,indexes));
    EXPECT_TRUE(indexes.empty());
    EXPECT_EQ(response.error_code_.val,moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
    EXPECT_FALSE(response.trajectory_);
  }
}

TEST(CartesianAdapter, OrdinaryInnerPlannerFailureIsNotReplacedWithSuccess) {
  SupportFixture fixture(0.); prepareOrdinaryRequest(fixture,.15);
  workcell::StraightCartesianPath adapter;
  planning_interface::MotionPlanResponse response; std::vector<std::size_t> indexes;
  unsigned calls=0;
  EXPECT_FALSE(adapter.adaptAndPlan([&](const auto&,const auto&,auto& out) {
    ++calls;out.error_code_.val=moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT;return false;
  },fixture.scene,fixture.request,response,indexes));
  EXPECT_EQ(calls,1U);EXPECT_TRUE(indexes.empty());EXPECT_FALSE(response.trajectory_);
  EXPECT_EQ(response.error_code_.val,moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT);
}

TEST(CartesianAdapter, SavedOneMillisecondQuinticCollisionRejects) {
  SupportFixture fixture(0.);
  prepareOrdinaryRequest(fixture,.027500010);
  auto trajectory=ordinaryQuintic(fixture,.001,.0002,0.);
  ASSERT_FALSE(fixture.scene->isStateColliding(trajectory->getFirstWayPoint(),""));
  ASSERT_FALSE(fixture.scene->isStateColliding(trajectory->getLastWayPoint(),""));
  moveit_msgs::msg::RobotTrajectory msg; trajectory->getRobotTrajectoryMsg(msg);
  joint_trajectory_controller::Trajectory jtc;
  trajectory_msgs::msg::JointTrajectoryPoint midpoint;
  jtc.interpolate_between_points(rclcpp::Time(0),msg.joint_trajectory.points[0],
    rclcpp::Time(1000000),msg.joint_trajectory.points[1],rclcpp::Time(500000),midpoint);
  auto state=trajectory->getFirstWayPoint(); state.setVariablePosition("lift",midpoint.positions[0]); state.update();
  ASSERT_NEAR(midpoint.positions[0],6.25e-8,1e-14);
  ASSERT_TRUE(fixture.scene->isStateColliding(state,""));
  workcell::StraightCartesianPath adapter;
  planning_interface::MotionPlanResponse response; std::vector<std::size_t> indexes;
  EXPECT_FALSE(adapter.adaptAndPlan([&](const auto&,const auto&,auto& out) {
    out.trajectory_=trajectory;out.error_code_.val=1;return true;
  },fixture.scene,fixture.request,response,indexes));
}

TEST(ControllerCertificate, SameSubmillisecondGeometryWithClearanceCertifies) {
  SupportFixture f(0.);prepareOrdinaryRequest(f,.028);
  const auto report=workcell::controller_certificate::certify(*ordinaryQuintic(f,.001,.0002),*f.scene);
  EXPECT_EQ(report.result,workcell::ControllerCertificate::CERTIFIED_CLEAR);
  EXPECT_EQ(report.certified,1U);EXPECT_EQ(report.subdivided,0U);
}
TEST(ControllerCertificate, LargeMotionWithLargeClearanceCertifies) {
  SupportFixture f(0.);prepareOrdinaryRequest(f,2.);
  const auto report=workcell::controller_certificate::certify(*ordinaryQuintic(f,1.,.4),*f.scene);
  EXPECT_EQ(report.result,workcell::ControllerCertificate::CERTIFIED_CLEAR);
  EXPECT_EQ(report.inspected,1U);
}

TEST(CartesianAdapter, ShutdownDuringPlanningCannotContinueCertification) {
  // The captured failure left PLAN_ONLY active through both result waits and
  // then through launch's shutdown grace. Exercise that same adapter boundary
  // with a valid path: shutdown is a lifecycle failure, never path clearance.
  SupportFixture f(0.);prepareOrdinaryRequest(f,2.);
  auto context=std::make_shared<rclcpp::Context>();context->init(0,nullptr);
  auto node=std::make_shared<rclcpp::Node>("audit_shutdown_regression",
      rclcpp::NodeOptions().context(context));
  workcell::StraightCartesianPath adapter;adapter.initialize(node,"");
  planning_interface::MotionPlanResponse response;std::vector<std::size_t> indexes;
  unsigned calls=0;
  const bool success=adapter.adaptAndPlan([&](const auto&,const auto&,auto& out) {
    ++calls;out.trajectory_=ordinaryQuintic(f,1.,.4);out.error_code_.val=1;
    context->shutdown("owned planning context stopped");return true;
  },f.scene,f.request,response,indexes);
  EXPECT_EQ(calls,1U);
  EXPECT_FALSE(success);
  EXPECT_FALSE(response.trajectory_);
  EXPECT_NE(response.error_code_.val,moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}
TEST(CartesianAdapter, NestedAdaptersShareExpiredAttempt) {
  SupportFixture f(0.);prepareOrdinaryRequest(f,2.);
  workcell::InitialSupportContact outer;workcell::StraightCartesianPath inner;
  planning_interface::MotionPlanResponse response;std::vector<std::size_t> indexes;
  response.trajectory_=ordinaryQuintic(f,1.,.4);response.error_code_.val=1;
  unsigned calls=0;
  // Exercise the actual steady-clock deadline without a 120 s unit-test sleep.
  workcell::PlanningAttempt expired(nullptr,
      workcell::PlanningAttempt::Clock::now()-std::chrono::seconds(120));
  EXPECT_FALSE(outer.adaptAndPlan([&](const auto& scene,const auto& request,auto& out) {
    return inner.adaptAndPlan([&](const auto&,const auto&,auto&) {
      ++calls;return true;
    },scene,request,out,indexes);
  },f.scene,f.request,response,indexes));
  EXPECT_EQ(calls,0U);EXPECT_FALSE(response.trajectory_);
  EXPECT_EQ(response.error_code_.val,moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT);
}
TEST(CartesianAdapter, NestedAdapterCannotReturnSuccessAfterOwnerShutdown) {
  SupportFixture f(0.);prepareOrdinaryRequest(f,2.);
  auto context=std::make_shared<rclcpp::Context>();context->init(0,nullptr);
  auto node=std::make_shared<rclcpp::Node>("nested_audit_shutdown_regression",
      rclcpp::NodeOptions().context(context));
  workcell::InitialSupportContact outer;outer.initialize(node,"");
  workcell::StraightCartesianPath inner;inner.initialize(node,"");
  planning_interface::MotionPlanResponse response;std::vector<std::size_t> indexes;
  EXPECT_FALSE(outer.adaptAndPlan([&](const auto& scene,const auto& request,auto& out) {
    return inner.adaptAndPlan([&](const auto&,const auto&,auto& planned) {
      planned.trajectory_=ordinaryQuintic(f,1.,.4);planned.error_code_.val=1;
      context->shutdown("owned nested planning context stopped");return true;
    },scene,request,out,indexes);
  },f.scene,f.request,response,indexes));
  EXPECT_FALSE(response.trajectory_);
  EXPECT_EQ(response.error_code_.val,moveit_msgs::msg::MoveItErrorCodes::FAILURE);
}
TEST(ControllerCertificate, InterruptionDuringIntervalTraversalCannotCertify) {
  SupportFixture f(0.);prepareOrdinaryRequest(f,2.);
  workcell::ControllerAuditOptions options;unsigned checks=0;
  options.checkpoint=[&] {
    if(++checks==10) throw std::runtime_error("OWNED_AUDIT_INTERRUPTED");
  };
  const auto report=workcell::controller_certificate::certify(*ordinaryQuintic(f,1.,.4),*f.scene,options);
  EXPECT_EQ(checks,10U);EXPECT_GT(report.inspected,0U);
  EXPECT_EQ(report.result,workcell::ControllerCertificate::UNCERTIFIED);
  EXPECT_EQ(report.reason,"OWNED_AUDIT_INTERRUPTED");
}

TEST(ControllerCertificate, PrecisionLimitCannotBecomeSuccess) {
  SupportFixture f(0.);prepareOrdinaryRequest(f,.0275000000005);
  workcell::ControllerAuditOptions options;options.max_depth=0;
  const auto report=workcell::controller_certificate::certify(*ordinaryQuintic(f,.001,0.),*f.scene,options);
  EXPECT_EQ(report.result,workcell::ControllerCertificate::UNCERTIFIED);
  EXPECT_EQ(report.reason,"PRECISION_OR_DEPTH_LIMIT");
  EXPECT_EQ(report.certified,0U);EXPECT_EQ(report.failure_begin_ns,0);
}
TEST(ControllerCertificate, MovingWorldWithoutBoundRejects) {
  SupportFixture f(0.);prepareOrdinaryRequest(f,2.);
  workcell::ControllerAuditOptions options;options.stationary_world=false;
  EXPECT_EQ(workcell::controller_certificate::certify(*ordinaryQuintic(f),*f.scene,options).result,
            workcell::ControllerCertificate::UNCERTIFIED);
}

namespace {
void prepareYawedPileCertificate(SupportFixture& fixture) {
  fixture.scene->getWorldNonConst()->removeObject("fixture");
  Eigen::Isometry3d floor=Eigen::Isometry3d::Identity();
  floor.linear()=Eigen::AngleAxisd(.23,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  floor.translation().z()=-.05;
  fixture.scene->getWorldNonConst()->addToObject(
    "fixture",shapes::ShapeConstPtr(new shapes::Box(1.,1.,.1)),floor);
  collision_detection::DecideContactFn predicate=workcell::PileContact{"part","fixture"};
  fixture.scene->getAllowedCollisionMatrixNonConst().setEntry("part","fixture",predicate);
}
robot_trajectory::RobotTrajectory pileCertificatePath(
    const SupportFixture& fixture,const std::vector<double>& heights) {
  robot_trajectory::RobotTrajectory trajectory(fixture.scene->getRobotModel(),"arm");
  for(double height:heights) {
    auto state=fixture.scene->getCurrentState();state.setVariablePosition("lift",height);
    state.setVariableVelocity("lift",0.);state.setVariableAcceleration("lift",0.);state.update();
    trajectory.addSuffixWayPoint(state,trajectory.getWayPointCount()?.1:0.);
  }
  return trajectory;
}
}

TEST(ControllerCertificate, InitialPileYawedBoxZeroSpeedLiftKeepsConditionalPolicy) {
  SupportFixture fixture(.00005);prepareYawedPileCertificate(fixture);
  ASSERT_FALSE(fixture.scene->isStateColliding());
  const auto trajectory=pileCertificatePath(fixture,{0.,.005});
  const auto report=workcell::controller_certificate::certify(trajectory,*fixture.scene);
  EXPECT_EQ(report.result,workcell::ControllerCertificate::CERTIFIED_CLEAR)<<report.reason;
  collision_detection::DecideContactFn predicate;
  ASSERT_TRUE(fixture.scene->getAllowedCollisionMatrix().getAllowedCollision("part","fixture",predicate));
  const auto* policy=predicate.target<workcell::PileContact>();
  ASSERT_NE(policy,nullptr);EXPECT_EQ(policy->object,"part");EXPECT_EQ(policy->neighbor,"fixture");
}

TEST(ControllerCertificate, InitialPileSeparationAndRecontactWithinOneSplineRejects) {
  SupportFixture fixture(.00005);prepareYawedPileCertificate(fixture);
  auto trajectory=ordinaryQuintic(fixture,.1,.1);
  auto strict=fixture.scene->diff();strict->decoupleParent();
  strict->getAllowedCollisionMatrixNonConst().setEntry("part","fixture",false);
  ASSERT_TRUE(strict->isStateColliding(trajectory->getFirstWayPoint(),""));
  ASSERT_TRUE(strict->isStateColliding(trajectory->getLastWayPoint(),""));
  moveit_msgs::msg::RobotTrajectory message;trajectory->getRobotTrajectoryMsg(message);
  trajectory_msgs::msg::JointTrajectoryPoint sample;
  joint_trajectory_controller::Trajectory jtc;
  jtc.interpolate_between_points(rclcpp::Time(0),message.joint_trajectory.points[0],
    rclcpp::Time(100000000),message.joint_trajectory.points[1],rclcpp::Time(50000000),sample);
  auto middle=trajectory->getFirstWayPoint();middle.setVariablePosition("lift",sample.positions.at(0));middle.update();
  ASSERT_FALSE(strict->isStateColliding(middle,""));
  const auto report=workcell::controller_certificate::certify(*trajectory,*fixture.scene);
  EXPECT_NE(report.result,workcell::ControllerCertificate::CERTIFIED_CLEAR);
}

TEST(ControllerCertificate, InitialPileExpiryPersistsAcrossExactWaypointReversal) {
  SupportFixture fixture(.00005);prepareYawedPileCertificate(fixture);
  const auto prefix=pileCertificatePath(fixture,{0.,.005});
  const auto prefix_report=workcell::controller_certificate::certify(prefix,*fixture.scene);
  ASSERT_EQ(prefix_report.result,workcell::ControllerCertificate::CERTIFIED_CLEAR)<<prefix_report.reason;
  const auto reversed=pileCertificatePath(fixture,{0.,.005,0.});
  const auto report=workcell::controller_certificate::certify(reversed,*fixture.scene);
  EXPECT_NE(report.result,workcell::ControllerCertificate::CERTIFIED_CLEAR);
  EXPECT_GE(report.failure_begin_ns,100000000);
  EXPECT_GT(report.certified,0U);
}

TEST(ControllerCertificate, InitialPileDepthAboveExistingToleranceRejects) {
  SupportFixture fixture(.00010001);prepareYawedPileCertificate(fixture);
  ASSERT_TRUE(fixture.scene->isStateColliding());
  const auto report=workcell::controller_certificate::certify(
    pileCertificatePath(fixture,{0.,.005}),*fixture.scene);
  EXPECT_NE(report.result,workcell::ControllerCertificate::CERTIFIED_CLEAR);
  EXPECT_EQ(report.certified,0U);
}

TEST(ControllerCertificate, InitialPilePermissionCannotHideUnrelatedCollision) {
  SupportFixture fixture(.00005);prepareYawedPileCertificate(fixture);
  Eigen::Isometry3d obstacle=Eigen::Isometry3d::Identity();obstacle.translation().z()=.03;
  fixture.scene->getWorldNonConst()->addToObject(
    "unrelated",shapes::ShapeConstPtr(new shapes::Box(.005,.005,.005)),obstacle);
  const auto trajectory=pileCertificatePath(fixture,{0.,.005});
  ASSERT_FALSE(fixture.scene->isStateColliding(trajectory.getFirstWayPoint(),""));
  ASSERT_TRUE(fixture.scene->isStateColliding(trajectory.getLastWayPoint(),""));
  const auto report=workcell::controller_certificate::certify(trajectory,*fixture.scene);
  EXPECT_NE(report.result,workcell::ControllerCertificate::CERTIFIED_CLEAR);
}

TEST(ControllerCertificate, InitialPileZeroTimestampBudgetFailsClosed) {
  SupportFixture fixture(.00005);prepareYawedPileCertificate(fixture);
  workcell::ControllerAuditOptions options;options.max_contact_timestamps=0;
  const auto report=workcell::controller_certificate::certify(
    pileCertificatePath(fixture,{0.,.005}),*fixture.scene,options);
  EXPECT_EQ(report.result,workcell::ControllerCertificate::UNCERTIFIED);
  EXPECT_EQ(report.reason,"CONTACT_TIMESTAMP_LIMIT");
  EXPECT_EQ(report.contact_timestamps,1U);
  EXPECT_EQ(report.initial_contacts_expired,0U);
}

TEST(ControllerCertificate, InitialPileBoxInteriorRequiresFullMotionMargin) {
  using namespace workcell::controller_certificate;
  SupportFixture fixture(.00005);prepareYawedPileCertificate(fixture);
  auto bodies=geometryBodies(*fixture.scene,fixture.scene->getCurrentState());
  auto& moving=bodies.at("part");const auto& fixed=bodies.at("fixture");
  // The overlap is 50 micrometres. Its common-point witness must retain
  // positive clearance to every face after the entire motion enclosure.
  EXPECT_TRUE(boxContactInterior(moving,fixed,.00001));
  EXPECT_FALSE(boxContactInterior(moving,fixed,.00003));
  moving.poses[0].translation().z()+=.000051;
  EXPECT_FALSE(boxContactInterior(moving,fixed,0.));
}

TEST(ControllerCertificate, InitialPileBoxGapDoesNotUseContainedIntersectionWidth) {
  using namespace workcell::controller_certificate;
  SupportFixture fixture(.00005);prepareYawedPileCertificate(fixture);
  auto bodies=geometryBodies(*fixture.scene,fixture.scene->getCurrentState());
  auto& moving=bodies.at("part");const auto& fixed=bodies.at("fixture");
  moving.shapes[0]=shapes::ShapeConstPtr(new shapes::Box(.00004,.00004,.00004));
  moving.poses[0]=fixed.poses[0];
  // A tiny fully contained box intersects only 40 micrometres of each axis,
  // but needs more than 50 millimetres of translation to leave the solid.
  EXPECT_LT(boxGap(moving,fixed),-.05);
  EXPECT_TRUE(boxContactInterior(moving,fixed,.00001));
  EXPECT_FALSE(boxContactInterior(moving,fixed,.00003));
}

TEST(ControllerCertificate, IntervalContainsActualInstalledJtcInteriorExtrema) {
  using namespace workcell::controller_certificate;
  trajectory_msgs::msg::JointTrajectoryPoint a,b;
  a.positions={.01};b.positions={.01000000001};
  a.velocities={.2};b.velocities={-.2};a.accelerations={.4};b.accelerations={-.4};
  auto c=polynomial(a,b,0,1000000000);
  EXPECT_GT(hull(c).upper(),.05);
  joint_trajectory_controller::Trajectory jtc;
  for(int64_t t=0;t<=1000000000;t+=100000) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    jtc.interpolate_between_points(rclcpp::Time(0),a,rclcpp::Time(1000000000),b,rclcpp::Time(t),point);
    EXPECT_GE(point.positions[0],hull(c).lower()-1e-14);
    EXPECT_LE(point.positions[0],hull(c).upper()+1e-14);
  }
}
TEST(ControllerCertificate, CollisionNearEitherEndCannotHideBetweenSamples) {
  for(bool reverse:{false,true}) {
    SupportFixture f(0.);prepareOrdinaryRequest(f,.027500010);
    // q(u)=.002*u*(.02-u)*(1-u)^3. The positive bump is
    // within the first 2% of the segment; its time reversal is near the end.
    prepareOrdinaryRequest(f,.027500075);
    auto trajectory=ordinaryQuintic(f,.001,.001);
    auto& first=*trajectory->getWayPointPtr(0);auto& last=*trajectory->getWayPointPtr(1);
    first.setVariableVelocity("lift",reverse?0.:.04);
    last.setVariableVelocity("lift",reverse?-.04:0.);
    first.setVariableAcceleration("lift",reverse?0.:-4240.);
    last.setVariableAcceleration("lift",reverse?-4240.:0.);
    // Endpoint/midpoint oracle must actually be clear; the interior bump collides.
    moveit_msgs::msg::RobotTrajectory msg;trajectory->getRobotTrajectoryMsg(msg);
    joint_trajectory_controller::Trajectory jtc;bool collision=false;
    for(int64_t t=0;t<=1000000;t+=1000) {
      trajectory_msgs::msg::JointTrajectoryPoint point;
      jtc.interpolate_between_points(rclcpp::Time(0),msg.joint_trajectory.points[0],rclcpp::Time(1000000),
        msg.joint_trajectory.points[1],rclcpp::Time(t),point);
      auto state=first;state.setVariablePosition("lift",point.positions[0]);state.update();
      const bool hit=f.scene->isStateColliding(state,"");collision=collision||hit;
      if(t==0||t==500000||t==1000000) { EXPECT_FALSE(hit); }
    }
    ASSERT_TRUE(collision);
    const auto report=workcell::controller_certificate::certify(*trajectory,*f.scene);
    EXPECT_NE(report.result,workcell::ControllerCertificate::CERTIFIED_CLEAR);
    EXPECT_GT(report.subdivided,0U);
  }
}
TEST(ControllerCertificate, RevoluteRadiusAndBothSelfCollisionBodies) {
  const std::string urdf=R"(<robot name="two"><link name="base"/>
    <link name="left"><collision><origin xyz="2 0 0"/><geometry><sphere radius="0.05"/></geometry></collision></link>
    <link name="right"><collision><origin xyz="2 0 0"/><geometry><sphere radius="0.05"/></geometry></collision></link>
    <joint name="l" type="revolute"><parent link="base"/><child link="left"/><axis xyz="0 0 1"/>
      <limit lower="-2" upper="2" effort="1" velocity="1"/></joint>
    <joint name="r" type="revolute"><parent link="base"/><child link="right"/><axis xyz="0 0 1"/>
      <limit lower="-2" upper="2" effort="1" velocity="1"/></joint></robot>)";
  auto u=urdf::parseURDF(urdf);auto semantic=std::make_shared<srdf::Model>();
  semantic->initString(*u,"<robot name='two'><group name='arm'><joint name='l'/><joint name='r'/></group></robot>");
  auto model=std::make_shared<moveit::core::RobotModel>(u,semantic);planning_scene::PlanningScene scene(model);
  auto a=scene.getCurrentState();a.setVariablePosition("l",-.1);a.setVariablePosition("r",.1);a.update();
  auto b=a;b.setVariablePosition("l",.1);b.setVariablePosition("r",-.1);b.update();
  robot_trajectory::RobotTrajectory trajectory(model,"arm");trajectory.addSuffixWayPoint(a,0.);trajectory.addSuffixWayPoint(b,1.);
  using namespace workcell::controller_certificate;
  Polynomials p{{"l",{Interval(-.1),Interval(.1)}},{"r",{Interval(.1),Interval(-.1)}}};
  const auto movement=bodyDisplacements(scene,a,p);
  EXPECT_GE(movement.at("left"),.2*2.05);EXPECT_GE(movement.at("right"),.2*2.05);
  EXPECT_GE(movement.at("left")+movement.at("right"),.82);
  ASSERT_FALSE(scene.isStateColliding(a,""));ASSERT_FALSE(scene.isStateColliding(b,""));
  EXPECT_EQ(certify(trajectory,scene).result,workcell::ControllerCertificate::COLLISION);
}

TEST(ControllerCertificate, UnsupportedOrNonfiniteWorldFailsClosedBeforeFcl) {
  SupportFixture f(0.);prepareOrdinaryRequest(f,2.);
  f.scene->getWorldNonConst()->addToObject("unbounded",shapes::ShapeConstPtr(new shapes::Plane(0,0,1,0)),Eigen::Isometry3d::Identity());
  EXPECT_EQ(workcell::controller_certificate::certify(*ordinaryQuintic(f),*f.scene).result,workcell::ControllerCertificate::UNCERTIFIED);
}
TEST(ControllerCertificate, FclDistanceOverestimateCannotProvideClearance) {
  using namespace workcell::controller_certificate;
  GeometryBody a{"a",collision_detection::BodyTypes::ROBOT_LINK,{shapes::ShapeConstPtr(new shapes::Box(1,1,1))},{Eigen::Isometry3d::Identity()}, {}};
  auto b=a;b.name="b";b.poses[0].translation().x()=.5;
  collision_detection::DistanceResultsData distance;distance.distance=1e100;
  distance.nearest_points[0]=Eigen::Vector3d::Zero();distance.nearest_points[1]=Eigen::Vector3d::UnitX();
  EXPECT_EQ(clearanceLowerBound(a,b,distance),0.);
  b.poses[0].translation().x()=2.;
  EXPECT_GT(clearanceLowerBound(a,b,distance),.999999999);
  EXPECT_LE(clearanceLowerBound(a,b,distance),1.);
}
TEST(ControllerCertificate, OddNanosecondSubdivisionEnclosesPolynomial) {
  using namespace workcell::controller_certificate;
  trajectory_msgs::msg::JointTrajectoryPoint a,b;a.positions={0.};b.positions={0.};
  a.velocities={.04};b.velocities={0.};a.accelerations={-4240.};b.accelerations={0.};
  double error=0.;auto original=polynomial(a,b,0,1000001,&error);
  auto children=split(original,Interval(500000.)/Interval(1000001.));
  joint_trajectory_controller::Trajectory jtc;
  for(int64_t t:{0,10000,499999,500000,500001,990000,1000001}) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    jtc.interpolate_between_points(rclcpp::Time(0),a,rclcpp::Time(1000001),b,rclcpp::Time(t),point);
    auto range=hull(t<=500000?children.first:children.second)+Interval(-error,error);
    EXPECT_GE(point.positions[0],range.lower());EXPECT_LE(point.positions[0],range.upper());
  }
}
TEST(ControllerCertificate, ProjectionEnclosesBoxAndRestoresCallerRoundingOnException) {
  using namespace workcell::controller_certificate;
  const int original_mode=std::fegetround();
  ASSERT_NE(original_mode,-1);
  struct RestoreRounding {
    int mode;
    ~RestoreRounding() { std::fesetround(mode); }
  } restore{original_mode};
  auto pose=Eigen::Isometry3d::Identity();
  pose.linear()<<0.,-1.,0., 1.,0.,0., 0.,0.,1.;
  pose.translation()=Eigen::Vector3d(.25,-.5,1.);
  GeometryBody box{"box",collision_detection::BodyTypes::WORLD_OBJECT,
    {shapes::ShapeConstPtr(new shapes::Box(2.,4.,6.))},{pose},{}};
  auto invalid=box;
  auto malformed=std::make_shared<shapes::Box>(2.,4.,6.);
  malformed->size[1]=-4.;invalid.shapes={malformed};
  const Eigen::Vector3d axis(1.,2.,-1.);
  for(int mode:{FE_TONEAREST,FE_DOWNWARD,FE_UPWARD,FE_TOWARDZERO}) {
    SCOPED_TRACE(mode);
    ASSERT_EQ(std::fesetround(mode),0);
    const auto range=projection(box,axis);
    EXPECT_EQ(std::fegetround(),mode);
    // Exact binary inputs: projected center -1.75, half-width 7.
    EXPECT_EQ(range.lower(),-8.75);
    EXPECT_EQ(range.upper(),5.25);
    ASSERT_EQ(std::fesetround(mode),0);
    EXPECT_THROW(projection(invalid,axis),std::runtime_error);
    EXPECT_EQ(std::fegetround(),mode);
  }
}
TEST(ControllerCertificate, ShapeCachePreservesExactRadiusAndRejectsMalformedGeometry) {
  using namespace workcell::controller_certificate;
  ShapeValidationCache cache;
  const shapes::ShapeConstPtr sphere(new shapes::Sphere(2.));
  EXPECT_EQ(cache.radius(sphere),std::nextafter(2.,INFINITY));
  EXPECT_EQ(cache.radius(sphere),radius(*sphere));
  auto mesh=std::make_shared<shapes::Mesh>(3,1);
  const double vertices[]={0.,0.,0.,3.,0.,0.,0.,4.,0.};
  std::copy(std::begin(vertices),std::end(vertices),mesh->vertices);
  mesh->triangles[0]=0;mesh->triangles[1]=1;mesh->triangles[2]=2;
  EXPECT_EQ(cache.radius(mesh),std::nextafter(4.,INFINITY));
  EXPECT_EQ(cache.radius(mesh),radius(*mesh));
  auto invalid_box=std::make_shared<shapes::Box>(1.,1.,1.);
  invalid_box->size[0]=-1.;
  EXPECT_THROW(cache.radius(invalid_box),std::runtime_error);
  auto invalid_mesh=std::make_shared<shapes::Mesh>(3,1);
  std::copy(std::begin(vertices),std::end(vertices),invalid_mesh->vertices);
  invalid_mesh->triangles[0]=0;invalid_mesh->triangles[1]=1;invalid_mesh->triangles[2]=3;
  EXPECT_THROW(cache.radius(invalid_mesh),std::runtime_error);
  EXPECT_THROW(cache.radius(invalid_mesh),std::runtime_error);
}
TEST(ControllerCertificate, ShapeCacheDoesNotSurviveItsCertificate) {
  using namespace workcell::controller_certificate;
  auto sphere=std::make_shared<shapes::Sphere>(2.);
  {ShapeValidationCache cache;EXPECT_EQ(cache.radius(sphere),std::nextafter(2.,INFINITY));}
  // Changes between frozen scenes must never inherit a prior certificate's bound.
  sphere->radius=4.;
  ShapeValidationCache next;
  EXPECT_EQ(next.radius(sphere),std::nextafter(4.,INFINITY));
}
TEST(ControllerCertificate, ShapeCacheKeepsCurrentPosesAndDisplacementBounds) {
  using namespace workcell::controller_certificate;
  SupportFixture f(0.);ShapeValidationCache cache;
  auto state=f.scene->getCurrentState();state.update();
  const auto first=geometryBodies(*f.scene,state,&cache);
  state.setVariablePosition("lift",.1);state.update();
  const auto cached=geometryBodies(*f.scene,state,&cache),uncached=geometryBodies(*f.scene,state);
  ASSERT_EQ(cached.size(),uncached.size());
  bool moved=false;
  for(const auto& body:cached) for(std::size_t i=0;i<body.second.poses.size();++i) {
    EXPECT_EQ(body.second.poses[i].matrix(),uncached.at(body.first).poses[i].matrix());
    moved=moved||body.second.poses[i].matrix()!=first.at(body.first).poses[i].matrix();
  }
  EXPECT_TRUE(moved);
  Polynomials p;std::map<std::string,double> errors;
  for(const auto& name:f.scene->getRobotModel()->getVariableNames()) {
    p[name]={Interval(state.getVariablePosition(name))};errors[name]=0.;
  }
  EXPECT_EQ(bodyDisplacements(*f.scene,state,p,errors,&cache),bodyDisplacements(*f.scene,state,p,errors));
  state.setVariablePosition("lift",std::numeric_limits<double>::quiet_NaN());state.update();
  EXPECT_THROW(geometryBodies(*f.scene,state,&cache),std::runtime_error);
}
namespace {
std::string readControllerFixture(const std::filesystem::path& directory,const std::string& name) {
  std::ifstream file(directory/name,std::ios::binary);
  if(!file) throw std::runtime_error("Missing real Stage A evidence: "+name);
  return std::string(std::istreambuf_iterator<char>(file),std::istreambuf_iterator<char>());
}
template<class Message>
void deserializeControllerFixture(const std::filesystem::path& directory,const std::string& name,Message& message) {
  auto bytes=readControllerFixture(directory,name);rclcpp::SerializedMessage serialized(bytes.size());
  auto& raw=serialized.get_rcl_serialized_message();std::memcpy(raw.buffer,bytes.data(),bytes.size());raw.buffer_length=bytes.size();
  rclcpp::Serialization<Message>().deserialize_message(&serialized,&message);
}
moveit::core::RobotModelPtr controllerFixtureModel(const std::filesystem::path& directory) {
  auto u=urdf::parseURDF(readControllerFixture(directory,"robot.urdf"));
  auto semantic=std::make_shared<srdf::Model>();
  if(!u||!semantic->initString(*u,readControllerFixture(directory,"robot.srdf")))
    throw std::runtime_error("Invalid real Stage A robot descriptions");
  return std::make_shared<moveit::core::RobotModel>(u,semantic);
}
}
TEST(ControllerCertificate, KnownClearRealStageAApproach) {
  const char* directory=std::getenv("WORKCELL_STAGE_A_CERTIFICATE_FIXTURE");
  if(!directory) GTEST_SKIP()<<"Set the historical scene/trajectory fixture directory for the real Stage A acceptance gate";
  auto model=controllerFixtureModel(directory);
  moveit_msgs::msg::PlanningScene message;deserializeControllerFixture(directory,"scene.cdr",message);
  planning_scene::PlanningScene scene(model);scene.setPlanningSceneMsg(message);
  moveit_msgs::msg::RobotTrajectory emitted;deserializeControllerFixture(directory,"trajectory.cdr",emitted);
  ASSERT_EQ(emitted.joint_trajectory.points.size(),101U); // original telemetry-stage approach
  robot_trajectory::RobotTrajectory trajectory(model,"manipulator");
  trajectory.setRobotTrajectoryMsg(scene.getCurrentState(),emitted);
  const auto report=workcell::controller_certificate::certify(trajectory,scene);
  std::cout<<"REAL_STAGE_A result="<<int(report.result)<<" reason="<<report.reason<<" inspected="<<report.inspected
    <<" certified="<<report.certified<<" subdivided="<<report.subdivided<<" depth="<<report.deepest
    <<" seconds="<<report.wall_seconds<<" failure=["<<report.failure_begin_ns<<","<<report.failure_end_ns<<"]\n";
  EXPECT_EQ(report.result,workcell::ControllerCertificate::CERTIFIED_CLEAR);
}

TEST(ControllerCertificate, RealStageAInitialLiftWithYawedBoxesAndRevoluteArm) {
  const auto directory=std::filesystem::path(__FILE__).parent_path()/"evidence/stage_a1_initial_lift_20260930";
  auto model=controllerFixtureModel(directory);
  moveit_msgs::msg::PlanningScene frozen;deserializeControllerFixture(directory,"scene.cdr",frozen);
  moveit_msgs::msg::RobotTrajectory emitted;deserializeControllerFixture(directory,"trajectory.cdr",emitted);
  moveit_msgs::msg::MotionPlanRequest request;deserializeControllerFixture(directory,"request.cdr",request);
  ASSERT_FALSE(frozen.is_diff);
  ASSERT_EQ(emitted.joint_trajectory.points.size(),42U);
  ASSERT_EQ(emitted.joint_trajectory.joint_names.size(),6U);
  for(const auto& name:emitted.joint_trajectory.joint_names)
    ASSERT_EQ(model->getJointOfVariable(name)->getType(),moveit::core::JointModel::REVOLUTE);
  for(double speed:emitted.joint_trajectory.points.front().velocities) ASSERT_DOUBLE_EQ(speed,0.);
  YAML::Node metadata;unsigned metadata_count=0;
  for(const auto& constraint:request.trajectory_constraints.constraints) {
    const std::string prefix="workcell_cartesian_path:";
    if(constraint.name.rfind(prefix,0)==0) {metadata=YAML::Load(constraint.name.substr(prefix.size()));++metadata_count;}
  }
  ASSERT_EQ(metadata_count,1U);
  ASSERT_EQ(metadata["schema"].as<std::string>(),"workcell_cartesian_path/v1");
  ASSERT_EQ(metadata["stage"].as<std::string>(),"PREPLAN_LIFT");
  ASSERT_TRUE(metadata["allow_initial_attached_world_separation"].as<bool>());
  std::set<std::string> requested;
  for(const auto& neighbor:metadata["initial_separation_object_ids"]) requested.insert(neighbor.as<std::string>());
  ASSERT_EQ(requested.size(),2U);
  planning_scene::PlanningScene scene(model);scene.setPlanningSceneMsg(frozen);
  robot_trajectory::RobotTrajectory trajectory(model,request.group_name);
  trajectory.setRobotTrajectoryMsg(scene.getCurrentState(),emitted);
  auto first=trajectory.getFirstWayPoint();first.update();
  std::vector<const moveit::core::AttachedBody*> attached;first.getAttachedBodies(attached);
  ASSERT_EQ(attached.size(),1U);
  const auto object=attached.front()->getName();
  ASSERT_GT((attached.front()->getGlobalCollisionBodyTransforms().front().linear()-
             Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff(),.1);
  // ROS CDR cannot store DecideContactFn. Restore only pairs named by the
  // actual request AND established by complete initial typed contact evidence.
  for(const auto& neighbor:requested) scene.getAllowedCollisionMatrixNonConst().setEntry(object,neighbor,false);
  collision_detection::CollisionRequest cr;cr.contacts=true;cr.max_contacts=256;cr.max_contacts_per_pair=64;
  cr.group_name=request.group_name;
  collision_detection::CollisionResult contacts;scene.checkCollision(cr,contacts,first);
  ASSERT_TRUE(contacts.collision);ASSERT_FALSE(contacts.contacts.empty());
  ASSERT_LT(contacts.contact_count,cr.max_contacts);
  std::set<std::string> actual;std::size_t count=0;
  for(const auto& pair:contacts.contacts) {
    ASSERT_FALSE(pair.second.empty());ASSERT_LT(pair.second.size(),cr.max_contacts_per_pair);
    count+=pair.second.size();
    for(const auto& contact:pair.second) {
      const auto neighbor=contact.body_name_1==object?contact.body_name_2:contact.body_name_1;
      ASSERT_EQ(requested.count(neighbor),1U);
      ASSERT_TRUE((workcell::PileContact{object,neighbor}(contact)));
      actual.insert(neighbor);
    }
  }
  ASSERT_EQ(count,contacts.contact_count);ASSERT_EQ(actual,requested);
  for(const auto& neighbor:actual) {
    collision_detection::DecideContactFn fn=workcell::PileContact{object,neighbor};
    scene.getAllowedCollisionMatrixNonConst().setEntry(object,neighbor,fn);
  }
  ASSERT_FALSE(scene.isStateColliding(first,""));
  const auto report=workcell::controller_certificate::certify(trajectory,scene);
  std::cout<<"REAL_STAGE_A_INITIAL_LIFT result="<<int(report.result)<<" reason="<<report.reason
    <<" inspected="<<report.inspected<<" certified="<<report.certified<<" subdivided="<<report.subdivided
    <<" depth="<<report.deepest<<" seconds="<<report.wall_seconds
    <<" contact_timestamps="<<report.contact_timestamps<<" expired="<<report.initial_contacts_expired
    <<" failure=["<<report.failure_begin_ns<<","<<report.failure_end_ns<<"]\n";
  EXPECT_EQ(report.result,workcell::ControllerCertificate::CERTIFIED_CLEAR);
  EXPECT_GT(report.contact_timestamps,0U);
  EXPECT_EQ(report.initial_contacts_expired,actual.size());
}

#include <geometric_shapes/mesh_operations.h>
struct DetachedFixture {
  moveit::core::RobotModelPtr model;
  planning_scene::PlanningScenePtr scene;
  explicit DetachedFixture(double overlap=.00005,bool self_risk=false,bool disjoint_lobe=false) {
    auto robot=urdf::parseURDF(R"(<robot name="detached"><link name="base"/>
      <link name="tip"><collision><geometry><box size=".02 .02 .02"/></geometry></collision></link>
      <link name="other"><collision><geometry><box size=".02 .02 .02"/></geometry></collision></link>
      <joint name="slide" type="prismatic"><parent link="base"/><child link="tip"/><axis xyz="0 0 1"/>
        <limit lower="-.2" upper=".2" effort="10" velocity="1"/></joint>
      <joint name="second" type="prismatic"><origin xyz=".2 0 0"/><parent link="base"/><child link="other"/><axis xyz="0 0 1"/>
        <limit lower="-.2" upper=".2" effort="10" velocity="1"/></joint></robot>)");
    auto srdf=std::make_shared<srdf::Model>();
    srdf->initString(*robot,"<robot name='detached'><group name='hand'><joint name='slide'/><joint name='second'/></group></robot>");
    model=std::make_shared<moveit::core::RobotModel>(robot,srdf);
    for(const auto* name:{"tip","other"})
      const_cast<moveit::core::LinkModel*>(model->getLinkModel(name))->setGeometry(
        {shapes::ShapeConstPtr(shapes::createMeshFromShape(shapes::Box(.02,.02,.02)))},{Eigen::Isometry3d::Identity()});
    if(self_risk) {
      auto pose=Eigen::Isometry3d::Identity();pose.translation().z()=.025;
      const_cast<moveit::core::LinkModel*>(model->getLinkModel("other"))->setJointOriginTransform(pose);
    }
    if(disjoint_lobe) {
      const std::unique_ptr<shapes::Mesh> cube(shapes::createMeshFromShape(shapes::Box(.02,.02,.02)));
      auto* combined=new shapes::Mesh(2*cube->vertex_count,2*cube->triangle_count);
      for(unsigned lobe=0;lobe<2;++lobe) {
        for(unsigned v=0;v<cube->vertex_count;++v) for(unsigned axis=0;axis<3;++axis)
          combined->vertices[3*(v+lobe*cube->vertex_count)+axis]=cube->vertices[3*v+axis]+(lobe?(axis==0?.1:axis==2?-.02:0.):0.);
        for(unsigned t=0;t<3*cube->triangle_count;++t)
          combined->triangles[t+3*lobe*cube->triangle_count]=cube->triangles[t]+lobe*cube->vertex_count;
      }
      combined->computeTriangleNormals();combined->computeVertexNormals();
      const_cast<moveit::core::LinkModel*>(model->getLinkModel("tip"))->setGeometry({shapes::ShapeConstPtr(combined)},{Eigen::Isometry3d::Identity()});
    }
    scene=std::make_shared<planning_scene::PlanningScene>(model);
    scene->getCurrentStateNonConst().setToDefaultValues();scene->getCurrentStateNonConst().update();
    box("work",0.,-.02+overlap);
  }
  void box(const std::string& name,double x,double z) {
    auto pose=Eigen::Isometry3d::Identity();pose.translation()=Eigen::Vector3d(x,0,z);
    scene->getWorldNonConst()->addToObject(name,shapes::ShapeConstPtr(new shapes::Box(.02,.02,.02)),pose);
  }
  robot_trajectory::RobotTrajectory path(std::vector<std::array<double,2>> positions={{0.,0.},{.01,0.}}) {
    robot_trajectory::RobotTrajectory result(model,"hand");
    for(const auto& q:positions) {
      auto state=scene->getCurrentState();state.setVariablePosition("slide",q[0]);state.setVariablePosition("second",q[1]);state.update();
      result.addSuffixWayPoint(state,result.getWayPointCount()?1.:0.);
    }
    return result;
  }
};
TEST(DetachedContact, SeparatingMeshMustQualify) {
  DetachedFixture f;
  ASSERT_TRUE(f.scene->isStateColliding());
  auto trajectory=f.path();ASSERT_FALSE(f.scene->isStateColliding(trajectory.getLastWayPoint(),""));
  const auto report=workcell::controller_certificate::certifyDetached(trajectory,*f.scene,"epoch-1").audit;
  EXPECT_EQ(report.result,workcell::ControllerCertificate::CERTIFIED_CLEAR)<<report.reason;
}
namespace dc=workcell::controller_certificate;
TEST(DetachedContact, ExactInitialPairStartsActiveWithGeometryAndEpoch) {
  DetachedFixture f;const auto pairs=dc::enumerateDetached(*f.scene,f.scene->getCurrentState(),"measured-epoch");
  ASSERT_EQ(pairs.size(),1U);const auto& p=pairs[0];
  EXPECT_EQ(p.robot_link,"tip");EXPECT_EQ(p.world_object,"work");EXPECT_EQ(p.epoch,"measured-epoch");
  EXPECT_EQ(p.state,dc::DetachedPairState::ACTIVE_INITIAL_CONTACT);
  EXPECT_FALSE(p.geometry_identity.empty());EXPECT_FALSE(p.initial_evidence.empty());
  EXPECT_EQ(p.robot_poses.size(),1U);EXPECT_EQ(p.world_poses.size(),1U);
  EXPECT_NEAR(p.axis.z(),1.,1e-15);EXPECT_GE(p.initial_gap,-.0001);
}
TEST(DetachedContact, DifferentPairNeverInheritsPermission) {
  DetachedFixture f;const auto pairs=dc::enumerateDetached(*f.scene,f.scene->getCurrentState(),"e");
  EXPECT_NE(dc::activePair(pairs,dc::pairKey("tip","work")),nullptr);
  EXPECT_EQ(dc::activePair(pairs,dc::pairKey("other","work")),nullptr);
  EXPECT_EQ(dc::activePair(pairs,dc::pairKey("tip","work_extra")),nullptr);
}
TEST(DetachedContact, RawDepthFluctuationsAreNotAMonotonicityOracle) {
  DetachedFixture f;auto pair=dc::enumerateDetached(*f.scene,f.scene->getCurrentState(),"e")[0];
  dc::Polynomials p{{"slide",{dc::Interval(0.),dc::Interval(.01)}},{"second",{dc::Interval(0.)}}};
  const auto speed=dc::derivativeControls(p,1000000000);
  // Deliberately perturb only reported witness depths. Every run still uses
  // actual full mesh geometry and interval kinematics, never this ordering.
  for(double raw_depth:{58.07e-9,57.72e-9,64.56e-9,59.70e-9}) {
    for(auto& c:pair.initial_evidence) c.depth=raw_depth;
    EXPECT_TRUE(dc::separatingInterval(pair,*f.scene,f.scene->getCurrentState(),p,speed,{}));
  }
}
TEST(DetachedContact, GeometricApproachRejectsWithoutExpiry) {
  DetachedFixture f;const auto r=dc::certifyDetached(f.path({{0.,0.},{-.01,0.}}),*f.scene,"e");
  EXPECT_EQ(r.audit.result,workcell::ControllerCertificate::UNCERTIFIED);
  EXPECT_EQ(r.audit.reason,"DETACHED_MONOTONICITY_UNCERTIFIED");
  ASSERT_EQ(r.pairs.size(),1U);EXPECT_TRUE(r.pairs[0].transitions.empty());
}
TEST(DetachedContact, PositiveFullGeometryClearanceExpiresWithoutChangingAcm) {
  DetachedFixture f;moveit_msgs::msg::AllowedCollisionMatrix before,after;
  f.scene->getAllowedCollisionMatrix().getMessage(before);
  const auto r=dc::certifyDetached(f.path(),*f.scene,"e");
  ASSERT_EQ(r.audit.result,workcell::ControllerCertificate::CERTIFIED_CLEAR)<<r.audit.reason;
  ASSERT_EQ(r.pairs.size(),1U);const auto& p=r.pairs[0];
  EXPECT_EQ(p.state,dc::DetachedPairState::EXPIRED);ASSERT_EQ(p.transitions.size(),2U);
  EXPECT_EQ(p.transitions[0].state,dc::DetachedPairState::SEPARATED);
  EXPECT_EQ(p.transitions[1].state,dc::DetachedPairState::EXPIRED);
  EXPECT_GT(p.transitions[1].clearance_lower,0.);EXPECT_EQ(p.transitions[1].end_ns,1000000000);
  f.scene->getAllowedCollisionMatrix().getMessage(after);EXPECT_EQ(before,after);
  EXPECT_TRUE(f.scene->isStateColliding()); // no durable policy mutation
}
TEST(DetachedContact, ExpiredPairRecontactIsHardFailure) {
  DetachedFixture f;const auto r=dc::certifyDetached(f.path({{0.,0.},{.01,0.},{0.,0.}}),*f.scene,"e");
  EXPECT_EQ(r.audit.result,workcell::ControllerCertificate::COLLISION);
  EXPECT_EQ(r.audit.reason,"DETACHED_NEW_OR_EXPIRED_CONTACT");
  ASSERT_EQ(r.pairs.size(),1U);EXPECT_EQ(r.pairs[0].state,dc::DetachedPairState::EXPIRED);
}
TEST(DetachedContact, IndependentPairsExpireSeparately) {
  DetachedFixture f;f.box("second_work",.2,-.01995);
  const auto r=dc::certifyDetached(f.path(),*f.scene,"e");
  EXPECT_EQ(r.audit.result,workcell::ControllerCertificate::UNCERTIFIED);
  EXPECT_EQ(r.audit.reason,"DETACHED_CONTACTS_UNEXPIRED");ASSERT_EQ(r.pairs.size(),2U);
  for(const auto& p:r.pairs) EXPECT_EQ(p.state,p.robot_link=="tip"?dc::DetachedPairState::EXPIRED:dc::DetachedPairState::ACTIVE_INITIAL_CONTACT);
  const auto complete=dc::certifyDetached(f.path({{0.,0.},{.01,0.},{.01,.01}}),*f.scene,"e");
  EXPECT_EQ(complete.audit.result,workcell::ControllerCertificate::CERTIFIED_CLEAR)<<complete.audit.reason;
  for(const auto& p:complete.pairs) {
    EXPECT_EQ(p.state,dc::DetachedPairState::EXPIRED);
    EXPECT_EQ(p.transitions.back().end_ns,p.robot_link=="tip"?1000000000LL:2000000000LL);
  }
}
TEST(DetachedContact, NewThirdPairIsHardFailure) {
  DetachedFixture f;f.box("second_work",.2,-.01995);f.box("third",0.,.025);
  const auto r=dc::certifyDetached(f.path({{0.,0.},{.01,.01}}),*f.scene,"e");
  ASSERT_EQ(r.pairs.size(),2U);EXPECT_EQ(r.audit.result,workcell::ControllerCertificate::COLLISION);
  EXPECT_EQ(r.audit.reason,"DETACHED_NEW_OR_EXPIRED_CONTACT");
}
TEST(DetachedContact, SelfCollisionIsHardFailure) {
  DetachedFixture f(.00005,true);
  const auto r=dc::certifyDetached(f.path(),*f.scene,"e");
  ASSERT_EQ(r.pairs.size(),1U);EXPECT_EQ(r.audit.result,workcell::ControllerCertificate::COLLISION);
}
TEST(DetachedContact, FullMeshWithoutStablePlaneFailsClosed) {
  DetachedFixture f(.00005,false,true);
  const auto r=dc::certifyDetached(f.path(),*f.scene,"e");
  EXPECT_EQ(r.audit.result,workcell::ControllerCertificate::UNCERTIFIED);
  EXPECT_EQ(r.audit.reason,"DETACHED_STABLE_PLANE_UNAVAILABLE");
}
TEST(DetachedContact, TruncatedEnumerationFailsClosed) {
  DetachedFixture f;
  for(std::size_t cap:{0U,1U}) {
    const auto r=dc::certifyDetached(f.path(),*f.scene,"e",{},cap);
    EXPECT_FALSE(r.enumeration_complete);EXPECT_EQ(r.audit.result,workcell::ControllerCertificate::UNCERTIFIED);
    EXPECT_EQ(r.audit.reason,"CONTACT_ENUMERATION_INCOMPLETE");
  }
}
TEST(DetachedContact, MidpointCollisionOutsideInitialPairStillRejects) {
  DetachedFixture f;f.box("interior_obstacle",.2,.08);auto trajectory=f.path();
  auto& first=*trajectory.getWayPointPtr(0);auto& last=*trajectory.getWayPointPtr(1);
  for(auto* state:{&first,&last}) {state->zeroVelocities();state->zeroAccelerations();state->setVariableVelocity("slide",.01);}
  first.setVariableVelocity("second",.2);last.setVariableVelocity("second",-.2);
  ASSERT_FALSE(f.scene->isStateColliding(last,""));
  const auto r=dc::certifyDetached(trajectory,*f.scene,"e");
  EXPECT_EQ(r.audit.result,workcell::ControllerCertificate::COLLISION);
  EXPECT_EQ(r.audit.reason,"DETACHED_NEW_OR_EXPIRED_CONTACT");
}
TEST(DetachedContact, OrdinaryClearTrajectoryNeedsNoContactAllowance) {
  DetachedFixture f(-.01);const auto r=dc::certifyDetached(f.path(),*f.scene,"e");
  EXPECT_EQ(r.audit.result,workcell::ControllerCertificate::CERTIFIED_CLEAR);EXPECT_TRUE(r.pairs.empty());
}
TEST(DetachedContact, StaleStartAndMissingEpochReject) {
  DetachedFixture f;
  EXPECT_EQ(dc::certifyDetached(f.path(),*f.scene,"").audit.reason,"RECOVERY_EPOCH_REQUIRED");
  EXPECT_EQ(dc::certifyDetached(f.path({{.001,0.},{.01,0.}}),*f.scene,"e").audit.reason,"DETACHED_START_STATE_MISMATCH");
}
TEST(DetachedContact, ZeroStartVelocityCannotBeProvedByAnErrorTube) {
  DetachedFixture f;auto trajectory=f.path();
  for(std::size_t i=0;i<trajectory.getWayPointCount();++i) {
    trajectory.getWayPointPtr(i)->zeroVelocities();trajectory.getWayPointPtr(i)->zeroAccelerations();
  }
  const auto r=dc::certifyDetached(trajectory,*f.scene,"e");
  EXPECT_EQ(r.audit.result,workcell::ControllerCertificate::UNCERTIFIED);
  EXPECT_EQ(r.audit.reason,"DETACHED_MONOTONICITY_UNCERTIFIED");
  EXPECT_EQ(r.audit.failure_begin_ns,0);EXPECT_LE(r.audit.failure_end_ns,1);
  ASSERT_EQ(r.pairs.size(),1U);EXPECT_TRUE(r.pairs[0].transitions.empty());
}
TEST(DetachedContact, InitialBoundCannotBeExpanded) {
  DetachedFixture f(.000100001);const auto r=dc::certifyDetached(f.path(),*f.scene,"e");
  EXPECT_EQ(r.audit.result,workcell::ControllerCertificate::UNCERTIFIED);
  EXPECT_EQ(r.audit.reason,"DETACHED_INITIAL_CONTACT_INVALID");
}
TEST(DetachedContact, OrdinaryEntryPointNeverEnablesDetachedPermission) {
  DetachedFixture f;
  EXPECT_EQ(dc::certify(f.path(),*f.scene).result,workcell::ControllerCertificate::COLLISION);
}
struct MimicDetachedFixture {
  moveit::core::RobotModelPtr model;
  planning_scene::PlanningScenePtr scene;
  MimicDetachedFixture() {
    auto u=urdf::parseURDF(R"(<robot name="mimic"><link name="base"/><link name="arm"/>
      <link name="tip"><collision><geometry><box size=".02 .02 .02"/></geometry></collision></link>
      <joint name="leader" type="revolute"><parent link="base"/><child link="arm"/><axis xyz="0 1 0"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
      <joint name="follower" type="revolute"><origin xyz=".1 0 0"/><parent link="arm"/><child link="tip"/><axis xyz="0 1 0"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/><mimic joint="leader" multiplier="-1" offset="0"/></joint></robot>)");
    auto srdf=std::make_shared<srdf::Model>();srdf->initString(*u,"<robot name='mimic'><group name='hand'><chain base_link='base' tip_link='tip'/></group></robot>");
    model=std::make_shared<moveit::core::RobotModel>(u,srdf);
    const_cast<moveit::core::LinkModel*>(model->getLinkModel("tip"))->setGeometry(
      {shapes::ShapeConstPtr(shapes::createMeshFromShape(shapes::Box(.02,.02,.02)))},{Eigen::Isometry3d::Identity()});
    scene=std::make_shared<planning_scene::PlanningScene>(model);auto& state=scene->getCurrentStateNonConst();
    state.setToDefaultValues();state.update();auto pose=Eigen::Isometry3d::Identity();pose.translation()=Eigen::Vector3d(.1,0.,-.01995);
    scene->getWorldNonConst()->addToObject("box",shapes::ShapeConstPtr(new shapes::Box(.02,.02,.02)),pose);
  }
  robot_trajectory::RobotTrajectory path() {
    auto a=scene->getCurrentState(),b=a;b.setVariablePosition("leader",-.1);b.update();
    robot_trajectory::RobotTrajectory t(model,"hand");t.addSuffixWayPoint(a,0.);t.addSuffixWayPoint(b,1.);return t;
  }
};
TEST(DetachedContact, RevoluteMimicFullMeshAndFixedOriginsCertify) {
  MimicDetachedFixture f;const auto r=dc::certifyDetached(f.path(),*f.scene,"mimic-epoch");
  EXPECT_EQ(r.audit.result,workcell::ControllerCertificate::CERTIFIED_CLEAR)<<r.audit.reason;
  ASSERT_EQ(r.pairs.size(),1U);EXPECT_EQ(r.pairs[0].state,dc::DetachedPairState::EXPIRED);
}
TEST(DetachedContact, MimicStartDiscontinuityCannotBeSilentlyNormalized) {
  MimicDetachedFixture f;f.scene->getCurrentStateNonConst().setVariablePosition("follower",1e-6);f.scene->getCurrentStateNonConst().update();
  const auto r=dc::certifyDetached(f.path(),*f.scene,"e");
  EXPECT_EQ(r.audit.result,workcell::ControllerCertificate::UNCERTIFIED);
  EXPECT_EQ(r.audit.reason,"DETACHED_CONTROLLER_START_DISCONTINUITY");
}
TEST(DetachedContact, UniformErrorIncludesFullPrismaticReach) {
  auto u=urdf::parseURDF(R"(<robot name="reach"><link name="base"/><link name="arm"/>
    <link name="tip"><collision><geometry><box size=".02 .02 .02"/></geometry></collision></link>
    <joint name="turn" type="revolute"><parent link="base"/><child link="arm"/><axis xyz="0 1 0"/>
      <limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
    <joint name="extend" type="prismatic"><parent link="arm"/><child link="tip"/><axis xyz="1 0 0"/>
      <limit lower="-2" upper="2" effort="1" velocity="1"/></joint></robot>)");
  auto srdf=std::make_shared<srdf::Model>();srdf->initString(*u,"<robot name='reach'/>");
  auto model=std::make_shared<moveit::core::RobotModel>(u,srdf);
  const_cast<moveit::core::LinkModel*>(model->getLinkModel("tip"))->setGeometry(
    {shapes::ShapeConstPtr(shapes::createMeshFromShape(shapes::Box(.02,.02,.02)))},{Eigen::Isometry3d::Identity()});
  planning_scene::PlanningScene scene(model);auto state=scene.getCurrentState();state.setToDefaultValues();state.update();
  dc::DetachedPair pair;pair.robot_link="tip";pair.axis=Eigen::Vector3d::UnitX();
  dc::Polynomials controls{{"turn",{dc::Interval(0.)}},{"extend",{dc::Interval(-1.),dc::Interval(1.)}}};
  const auto speed=dc::derivativeControls(controls,100000000000LL);
  // The supplied uniform angular error is amplified by the full +/-1m reach,
  // not merely the 17mm mesh radius at the midpoint (extension zero).
  EXPECT_FALSE(dc::separatingInterval(pair,scene,state,controls,speed,{{"turn",1e-10},{"extend",0.}}));
}
TEST(DetachedContact, FailedParentDoesNotCommitPrematureExpiry) {
  DetachedFixture f;f.box("side_obstacle",.221,.005);
  const auto r=dc::certifyDetached(f.path({{0.,0.},{.01,.01}}),*f.scene,"e");
  EXPECT_EQ(r.audit.result,workcell::ControllerCertificate::CERTIFIED_CLEAR)<<r.audit.reason;
  EXPECT_GT(r.audit.subdivided,0U);ASSERT_EQ(r.pairs.size(),1U);ASSERT_EQ(r.pairs[0].transitions.size(),2U);
  EXPECT_LT(r.pairs[0].transitions[0].end_ns,1000000000);
}
TEST(DetachedContact, BackwardsStoredWaypointSwitchRejects) {
  DetachedFixture f;const auto pair=dc::enumerateDetached(*f.scene,f.scene->getCurrentState(),"e")[0];
  auto prior=f.scene->getCurrentState(),terminal=prior;
  prior.setVariablePosition("slide",.01);terminal.setVariablePosition("slide",.009);prior.update();terminal.update();
  EXPECT_FALSE(dc::detachedBoundaryMonotone(pair,*f.scene,prior,terminal,{}));
}
TEST(DetachedContact, ClearanceIsMetricForNonUnitPlaneCoefficients) {
  DetachedFixture f;auto pair=dc::enumerateDetached(*f.scene,f.scene->getCurrentState(),"e")[0];
  auto terminal=f.path().getLastWayPoint();const double original=dc::detachedGap(pair,*f.scene,terminal,{});
  pair.axis*=2.;pair.world_support*=2.;
  const double scaled=dc::detachedGap(pair,*f.scene,terminal,{});
  EXPECT_NEAR(scaled,original,1e-11);
}

TEST(ControllerCertificate, SavedTransferMeshesFindIndependentSeparatingAxis) {
  using namespace workcell::controller_certificate;
  struct Case {const char* file;Eigen::Vector3d first,second;double minimum;};
  const Case cases[]={
    {"vertices.csv",{.063428225205688499,.023367000452644934,.87858459855173776},
      {.07126162350026001,.034379453169062033,.88237108010685583},.01},
    {"later-vertices.csv",{.35763513011852005,.35224646056199305,.61805315429562258},
      {.35979622525632204,.3416548842380055,.60865818353257017},.003}};
  for(const auto& example:cases) {
  SCOPED_TRACE(example.file);
  std::ifstream input(std::filesystem::path(__FILE__).parent_path()/"evidence/stage_a1_transfer_axes_20260930"/example.file);
  ASSERT_TRUE(input.good());std::map<std::string,std::vector<Eigen::Vector3d>> vertices;
  std::string line;
  while(std::getline(input,line)) {
    std::replace(line.begin(),line.end(),',',' ');std::istringstream row(line);
    std::string name;double x,y,z;ASSERT_TRUE(bool(row>>name>>x>>y>>z));
    vertices[name].emplace_back(x,y,z);
  }
  ASSERT_EQ(vertices["forearm_link"].size(),537u);ASSERT_EQ(vertices["wrist_2_link"].size(),355u);
  auto body=[&](const std::string& name) {
    const auto& points=vertices.at(name);auto mesh=new shapes::Mesh(points.size(),0);
    for(std::size_t i=0;i<points.size();++i) for(unsigned j=0;j<3;++j) mesh->vertices[3*i+j]=points[i][j];
    return GeometryBody{name,collision_detection::BodyTypes::ROBOT_LINK,
      {shapes::ShapeConstPtr(mesh)},{Eigen::Isometry3d::Identity()}, {}};
  };
  const auto a=body("forearm_link"),b=body("wrist_2_link");
  collision_detection::DistanceResultsData distance;distance.distance=.014034731427425626;
  distance.nearest_points[0]=example.first;distance.nearest_points[1]=example.second;
  EXPECT_GT(clearanceLowerBound(a,b,distance,1.7677344936e-9),example.minimum);
  // An optimistic witness cannot certify identical overlapping geometry.
  auto overlap=a;overlap.name="overlap";distance.distance=1e100;
  EXPECT_EQ(clearanceLowerBound(a,overlap,distance,1e-12),0.);
  }
}

TEST(ControllerCertificate, SavedTransferBodiesClearEveryConcaveBinTriangle) {
  using namespace workcell::controller_certificate;
  for(const char* file:{"bin.fixture.txt","finger-bin.fixture.txt"}) {
  SCOPED_TRACE(file);
  std::ifstream input(std::filesystem::path(__FILE__).parent_path()/"evidence/stage_a1_transfer_axes_20260930"/file);
  ASSERT_TRUE(input.good());std::array<std::vector<Eigen::Vector3d>,2> vertices;
  std::array<std::vector<std::array<unsigned,3>>,2> triangles;
  Eigen::Vector3d size=Eigen::Vector3d::Zero();Eigen::Isometry3d body_pose,mesh_pose;
  collision_detection::DistanceResultsData distance;std::string line;
  while(std::getline(input,line)) {
    std::istringstream row(line);std::string kind;row>>kind;
    if(kind=="box_size") row>>size[0]>>size[1]>>size[2];
    else if(kind=="box_pose"||kind=="body_pose"||kind=="mesh_pose") {
      auto& pose=kind=="mesh_pose"?mesh_pose:body_pose;
      for(unsigned i=0;i<4;++i) for(unsigned j=0;j<4;++j) row>>pose.matrix()(i,j);
    } else if(kind=="vertex"||kind=="mesh_vertex"||kind=="body_vertex") {
      Eigen::Vector3d v;row>>v[0]>>v[1]>>v[2];vertices[kind=="body_vertex"?0:1].push_back(v);
    } else if(kind=="triangle"||kind=="mesh_triangle"||kind=="body_triangle") {
      std::array<unsigned,3> t;row>>t[0]>>t[1]>>t[2];triangles[kind=="body_triangle"?0:1].push_back(t);
    } else if(kind=="witness0"||kind=="witness1") {auto& p=distance.nearest_points[kind=="witness0"?0:1];row>>p[0]>>p[1]>>p[2];}
    else if(kind=="fcl_distance") row>>distance.distance;
    else FAIL()<<"unknown fixture row: "<<kind;
    ASSERT_TRUE(bool(row));
  }
  ASSERT_EQ(vertices[1].size(),160u);ASSERT_EQ(triangles[1].size(),316u);
  auto make_mesh=[&](unsigned side) {
    auto mesh=std::make_shared<shapes::Mesh>(vertices[side].size(),triangles[side].size());
    for(std::size_t i=0;i<vertices[side].size();++i) for(unsigned j=0;j<3;++j) mesh->vertices[3*i+j]=vertices[side][i][j];
    for(std::size_t i=0;i<triangles[side].size();++i) for(unsigned j=0;j<3;++j) mesh->triangles[3*i+j]=triangles[side][i][j];
    return mesh;
  };
  const bool primitive=vertices[0].empty();
  shapes::ShapeConstPtr shape=primitive?shapes::ShapeConstPtr(std::make_shared<shapes::Box>(size[0],size[1],size[2])):make_mesh(0);
  GeometryBody body{"moving",collision_detection::BodyTypes::ROBOT_ATTACHED,{shape},{body_pose},{}};
  GeometryBody bin{"bin",collision_detection::BodyTypes::WORLD_OBJECT,{make_mesh(1)},{mesh_pose},{}};
  EXPECT_GT(clearanceLowerBound(body,bin,distance,.05),.05);
  EXPECT_GT(clearanceLowerBound(bin,body,distance,.05),.05);
  // Translate the box interior or a full mesh-face centroid onto each bin
  // triangle. Every original face must remain an obstacle, including the floor.
  Eigen::Vector3d local=Eigen::Vector3d::Zero();
  if(!primitive) {
    ASSERT_FALSE(triangles[0].empty());const auto& t=triangles[0][0];
    local=(vertices[0][t[0]]+vertices[0][t[1]]+vertices[0][t[2]])/3.;
  }
  for(std::size_t i=0;i<triangles[1].size();++i) {
    SCOPED_TRACE(i);const auto& t=triangles[1][i];
    body.poses[0].translation()=mesh_pose*((vertices[1][t[0]]+vertices[1][t[1]]+vertices[1][t[2]])/3.)-body_pose.linear()*local;
    EXPECT_LE(clearanceLowerBound(body,bin,distance,1e-12),1e-12);
  }
  }
}

// Revolute support lift: a lever rotates about a horizontal axis, so the
// carried BOX both translates and rotates. upwardBoxContact (identity
// rotation, prismatic only) cannot apply; eligible single-BOX support pairs
// must use the convex-BOX initial-contact proof with the SupportContact policy.
namespace {
struct RevoluteSupportFixture {
  planning_scene::PlanningScenePtr scene;
  explicit RevoluteSupportFixture(double penetration,double yaw=.1,bool box_support=true,
                                  const std::string& policy_support="fixture") {
    const std::string urdf=R"(<robot name="lever"><link name="base"/><link name="arm"/><link name="tool"/>
      <joint name="pivot" type="revolute"><parent link="base"/><child link="arm"/><axis xyz="0 1 0"/>
      <origin xyz="0 0 .0125"/><limit lower="-1" upper="1" effort="10" velocity="1"/></joint>
      <joint name="mount" type="fixed"><parent link="arm"/><child link="tool"/><origin xyz=".3 0 0"/></joint></robot>)";
    auto robot=urdf::parseURDF(urdf);auto srdf=std::make_shared<srdf::Model>();
    srdf->initString(*robot,"<robot name='lever'><group name='arm'><joint name='pivot'/></group></robot>");
    auto model=std::make_shared<moveit::core::RobotModel>(robot,srdf);
    scene=std::make_shared<planning_scene::PlanningScene>(model);
    Eigen::Isometry3d table=Eigen::Isometry3d::Identity();table.translation()=Eigen::Vector3d(.3,0,-.05);
    scene->getWorldNonConst()->addToObject("fixture",box_support?
      shapes::ShapeConstPtr(new shapes::Box(1,1,.1)):shapes::ShapeConstPtr(new shapes::Cylinder(.5,.1)),table);
    auto& state=scene->getCurrentStateNonConst();state.setToDefaultValues();
    Eigen::Isometry3d part=Eigen::Isometry3d::Identity();
    part.linear()=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    part.translation().z()=-penetration;
    state.attachBody("part",Eigen::Isometry3d::Identity(),{shapes::ShapeConstPtr(new shapes::Box(.025,.025,.025))},
                     EigenSTL::vector_Isometry3d{part},std::set<std::string>{},"tool");
    state.update();
    collision_detection::DecideContactFn predicate=workcell::SupportContact{"part",policy_support,0.};
    scene->getAllowedCollisionMatrixNonConst().setEntry("part","fixture",predicate);
  }
  robot_trajectory::RobotTrajectory path(const std::vector<double>& angles) const {
    robot_trajectory::RobotTrajectory trajectory(scene->getRobotModel(),"arm");
    for(double angle:angles) {
      auto state=scene->getCurrentState();state.setVariablePosition("pivot",angle);
      state.setVariableVelocity("pivot",0.);state.setVariableAcceleration("pivot",0.);state.update();
      trajectory.addSuffixWayPoint(state,trajectory.getWayPointCount()?.1:0.);
    }
    return trajectory;
  }
};
}

TEST(ControllerCertificate, RevoluteYawedSupportLiftCertifiesAndExpiresContact) {
  RevoluteSupportFixture fixture(1.2e-7);
  ASSERT_FALSE(fixture.scene->isStateColliding());   // the conditional support policy admits the start
  auto strict=fixture.scene->diff();strict->decoupleParent();
  strict->getAllowedCollisionMatrixNonConst().setEntry("part","fixture",false);
  ASSERT_TRUE(strict->isStateColliding(fixture.scene->getCurrentState(),""));  // genuinely in contact at t=0
  // Negative pivot raises the tool: 0.02 rad lifts the part about 6 mm.
  const auto report=workcell::controller_certificate::certify(fixture.path({0.,-.02}),*fixture.scene);
  EXPECT_EQ(report.result,workcell::ControllerCertificate::CERTIFIED_CLEAR)<<report.reason;
  EXPECT_EQ(report.reason,"ALL_INTERVALS_CERTIFIED");
  EXPECT_EQ(report.initial_contacts_expired,1U);
}

TEST(ControllerCertificate, RevoluteSupportMotionIntoTableRejects) {
  RevoluteSupportFixture fixture(1.2e-7);
  const auto report=workcell::controller_certificate::certify(fixture.path({0.,.02}),*fixture.scene);
  EXPECT_NE(report.result,workcell::ControllerCertificate::CERTIFIED_CLEAR);
  EXPECT_EQ(report.certified,0U);
}

TEST(ControllerCertificate, RevoluteSupportLiftThenRecontactRejects) {
  RevoluteSupportFixture fixture(1.2e-7);
  const auto report=workcell::controller_certificate::certify(fixture.path({0.,-.02,0.}),*fixture.scene);
  EXPECT_NE(report.result,workcell::ControllerCertificate::CERTIFIED_CLEAR);
  EXPECT_GT(report.certified,0U);
  EXPECT_GE(report.failure_begin_ns,100000000);
}

TEST(ControllerCertificate, RevoluteSupportDepthAboveToleranceRejects) {
  RevoluteSupportFixture fixture(.00010001);
  const auto report=workcell::controller_certificate::certify(fixture.path({0.,-.02}),*fixture.scene);
  EXPECT_NE(report.result,workcell::ControllerCertificate::CERTIFIED_CLEAR);
  EXPECT_EQ(report.certified,0U);
}

TEST(ControllerCertificate, RevoluteSupportPolicyForDifferentSupportStaysUnproven) {
  RevoluteSupportFixture fixture(1.2e-7,.1,true,"other_support");
  const auto report=workcell::controller_certificate::certify(fixture.path({0.,-.02}),*fixture.scene);
  EXPECT_NE(report.result,workcell::ControllerCertificate::CERTIFIED_CLEAR);
}

TEST(ControllerCertificate, RevoluteNonBoxSupportStaysUnproven) {
  RevoluteSupportFixture fixture(1.2e-7,.1,false);
  const auto report=workcell::controller_certificate::certify(fixture.path({0.,-.02}),*fixture.scene);
  EXPECT_NE(report.result,workcell::ControllerCertificate::CERTIFIED_CLEAR);
}

TEST(ControllerCertificate, RevoluteSupportPermissionCannotHideUnrelatedCollision) {
  RevoluteSupportFixture fixture(1.2e-7);
  Eigen::Isometry3d obstacle=Eigen::Isometry3d::Identity();obstacle.translation()=Eigen::Vector3d(.3,0,.03);
  fixture.scene->getWorldNonConst()->addToObject(
    "unrelated",shapes::ShapeConstPtr(new shapes::Box(.005,.005,.005)),obstacle);
  const auto trajectory=fixture.path({0.,-.02});
  ASSERT_TRUE(fixture.scene->isStateColliding(trajectory.getLastWayPoint(),""));
  const auto report=workcell::controller_certificate::certify(trajectory,*fixture.scene);
  EXPECT_NE(report.result,workcell::ControllerCertificate::CERTIFIED_CLEAR);
}

TEST(ControllerCertificate, RevoluteSupportZeroTimestampBudgetFailsClosed) {
  RevoluteSupportFixture fixture(1.2e-7);
  workcell::ControllerAuditOptions options;options.max_contact_timestamps=0;
  const auto report=workcell::controller_certificate::certify(fixture.path({0.,-.02}),*fixture.scene,options);
  EXPECT_NE(report.result,workcell::ControllerCertificate::CERTIFIED_CLEAR);
  EXPECT_EQ(report.initial_contacts_expired,0U);
}

// Independent-review regressions (F1): motion that approaches or drags along the
// support while still in initial contact must not certify, even within the
// depth tolerance. Rig: prismatic lift, revolute swing (z), revolute pivot (y).
namespace {
struct RigSupportFixture {
  planning_scene::PlanningScenePtr scene;
  explicit RigSupportFixture(double penetration) {
    const std::string urdf=R"(<robot name="rig"><link name="base"/><link name="l1"/><link name="l2"/><link name="arm"/><link name="tool"/>
      <joint name="lift" type="prismatic"><parent link="base"/><child link="l1"/><axis xyz="0 0 1"/>
      <limit lower="-.05" upper=".05" effort="10" velocity="1"/></joint>
      <joint name="swing" type="revolute"><parent link="l1"/><child link="l2"/><axis xyz="0 0 1"/>
      <limit lower="-1" upper="1" effort="10" velocity="1"/></joint>
      <joint name="pivot" type="revolute"><parent link="l2"/><child link="arm"/><axis xyz="0 1 0"/>
      <origin xyz="0 0 .0125"/><limit lower="-1" upper="1" effort="10" velocity="1"/></joint>
      <joint name="mount" type="fixed"><parent link="arm"/><child link="tool"/><origin xyz=".3 0 0"/></joint></robot>)";
    auto robot=urdf::parseURDF(urdf);auto srdf=std::make_shared<srdf::Model>();
    srdf->initString(*robot,"<robot name='rig'><group name='arm'><joint name='lift'/><joint name='swing'/><joint name='pivot'/></group></robot>");
    auto model=std::make_shared<moveit::core::RobotModel>(robot,srdf);
    scene=std::make_shared<planning_scene::PlanningScene>(model);
    Eigen::Isometry3d table=Eigen::Isometry3d::Identity();table.translation()=Eigen::Vector3d(.3,0,-.05);
    scene->getWorldNonConst()->addToObject("fixture",shapes::ShapeConstPtr(new shapes::Box(1,1,.1)),table);
    auto& state=scene->getCurrentStateNonConst();state.setToDefaultValues();
    Eigen::Isometry3d part=Eigen::Isometry3d::Identity();
    part.linear()=Eigen::AngleAxisd(.1,Eigen::Vector3d::UnitZ()).toRotationMatrix();part.translation().z()=-penetration;
    state.attachBody("part",Eigen::Isometry3d::Identity(),{shapes::ShapeConstPtr(new shapes::Box(.025,.025,.025))},
                     EigenSTL::vector_Isometry3d{part},std::set<std::string>{},"tool");
    state.update();
    collision_detection::DecideContactFn predicate=workcell::SupportContact{"part","fixture",0.};
    scene->getAllowedCollisionMatrixNonConst().setEntry("part","fixture",predicate);
  }
  robot_trajectory::RobotTrajectory path(const std::vector<std::array<double,3>>& points) const {
    robot_trajectory::RobotTrajectory trajectory(scene->getRobotModel(),"arm");
    for(const auto& q:points) {
      auto state=scene->getCurrentState();
      state.setVariablePosition("lift",q[0]);state.setVariablePosition("swing",q[1]);state.setVariablePosition("pivot",q[2]);
      for(const char* n:{"lift","swing","pivot"}) {state.setVariableVelocity(n,0.);state.setVariableAcceleration(n,0.);}
      state.update();trajectory.addSuffixWayPoint(state,trajectory.getWayPointCount()?.1:0.);
    }
    return trajectory;
  }
};
}

TEST(ControllerCertificate, RigRevoluteSupportLiftCertifies) {
  RigSupportFixture f(1.2e-7);
  const auto r=workcell::controller_certificate::certify(f.path({{0,0,0},{0,0,-.02}}),*f.scene);
  EXPECT_EQ(r.result,workcell::ControllerCertificate::CERTIFIED_CLEAR)<<r.reason;
  EXPECT_EQ(r.reason,"ALL_INTERVALS_CERTIFIED");EXPECT_EQ(r.initial_contacts_expired,1U);
}

TEST(ControllerCertificate, SupportPressWithinToleranceThenLiftRejects) {
  RigSupportFixture f(1.2e-7);   // pivot +1e-4 rad presses the part ~30 um into the table
  const auto r=workcell::controller_certificate::certify(f.path({{0,0,0},{0,0,1e-4},{0,0,-.02}}),*f.scene);
  EXPECT_NE(r.result,workcell::ControllerCertificate::CERTIFIED_CLEAR)<<r.reason;
  EXPECT_LT(r.failure_begin_ns,100000000);   // rejected within the press segment
}

TEST(ControllerCertificate, SupportLateralDragInContactThenLiftRejects) {
  RigSupportFixture f(1.2e-7);   // swing 5e-4 rad drags the part ~0.15 mm along the table
  const auto r=workcell::controller_certificate::certify(f.path({{0,0,0},{0,.0005,0},{0,.0005,-.02}}),*f.scene);
  EXPECT_NE(r.result,workcell::ControllerCertificate::CERTIFIED_CLEAR)<<r.reason;
  EXPECT_LT(r.failure_end_ns,100000001);
}

TEST(ControllerCertificate, SupportLongLateralDragRejects) {
  RigSupportFixture f(1.2e-7);
  const auto r=workcell::controller_certificate::certify(f.path({{0,0,0},{0,.05,0}}),*f.scene);
  EXPECT_NE(r.result,workcell::ControllerCertificate::CERTIFIED_CLEAR)<<r.reason;
}

TEST(ControllerCertificate, SupportDragBeyondToleranceRejectedByDragBoundNotBudget) {
  RigSupportFixture f(1.2e-7);   // swing 5e-4 rad: ~0.15 mm drag, then lift
  workcell::ControllerAuditOptions options;options.max_intervals=3000000;
  const auto r=workcell::controller_certificate::certify(
    f.path({{0,0,0},{0,.0005,0},{0,.0005,-.02}}),*f.scene,options);
  EXPECT_NE(r.result,workcell::ControllerCertificate::CERTIFIED_CLEAR)<<r.reason;
  EXPECT_EQ(r.reason,"PRECISION_OR_DEPTH_LIMIT");   // the drag bound, not INTERVAL_LIMIT
}

TEST(ControllerCertificate, SupportDragBelowToleranceThenLiftCertifies) {
  RigSupportFixture f(1.2e-7);   // swing 1e-4 rad: ~30 um drag, inside the 0.1 mm allowance
  const auto r=workcell::controller_certificate::certify(
    f.path({{0,0,0},{0,.0001,0},{0,.0001,-.02}}),*f.scene);
  EXPECT_EQ(r.result,workcell::ControllerCertificate::CERTIFIED_CLEAR)<<r.reason;
  EXPECT_EQ(r.initial_contacts_expired,1U);
}
