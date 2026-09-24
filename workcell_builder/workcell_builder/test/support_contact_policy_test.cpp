#include <gtest/gtest.h>
#include <array>
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
TEST(ControllerCertificate, KnownClearRealStageAApproach) {
  const char* directory=std::getenv("WORKCELL_STAGE_A_CERTIFICATE_FIXTURE");
  if(!directory) GTEST_SKIP()<<"Set the historical scene/trajectory fixture directory for the real Stage A acceptance gate";
  auto read=[&](const std::string& name) {
    std::ifstream file(std::string(directory)+"/"+name,std::ios::binary);
    if(!file) throw std::runtime_error("Missing real Stage A evidence: "+name);
    return std::string(std::istreambuf_iterator<char>(file),std::istreambuf_iterator<char>());
  };
  auto u=urdf::parseURDF(read("robot.urdf"));ASSERT_TRUE(u);
  auto semantic=std::make_shared<srdf::Model>();ASSERT_TRUE(semantic->initString(*u,read("robot.srdf")));
  auto model=std::make_shared<moveit::core::RobotModel>(u,semantic);
  auto deserialize=[&](const std::string& name,auto& message) {
    auto bytes=read(name);rclcpp::SerializedMessage serialized(bytes.size());
    auto& raw=serialized.get_rcl_serialized_message();std::memcpy(raw.buffer,bytes.data(),bytes.size());raw.buffer_length=bytes.size();
    rclcpp::Serialization<std::decay_t<decltype(message)>> serializer;serializer.deserialize_message(&serialized,&message);
  };
  moveit_msgs::msg::PlanningScene message;deserialize("scene.cdr",message);
  planning_scene::PlanningScene scene(model);scene.setPlanningSceneMsg(message);
  moveit_msgs::msg::RobotTrajectory emitted;deserialize("trajectory.cdr",emitted);
  ASSERT_EQ(emitted.joint_trajectory.points.size(),101U); // original telemetry-stage approach
  robot_trajectory::RobotTrajectory trajectory(model,"manipulator");
  trajectory.setRobotTrajectoryMsg(scene.getCurrentState(),emitted);
  const auto report=workcell::controller_certificate::certify(trajectory,scene);
  std::cout<<"REAL_STAGE_A result="<<int(report.result)<<" reason="<<report.reason<<" inspected="<<report.inspected
    <<" certified="<<report.certified<<" subdivided="<<report.subdivided<<" depth="<<report.deepest
    <<" seconds="<<report.wall_seconds<<" failure=["<<report.failure_begin_ns<<","<<report.failure_end_ns<<"]\n";
  EXPECT_EQ(report.result,workcell::ControllerCertificate::CERTIFIED_CLEAR);
}
