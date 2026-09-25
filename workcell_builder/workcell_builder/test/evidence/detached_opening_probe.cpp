// Offline complete-scene diagnosis only. This creates no recovery authority.
#include "../support_contact_policy_test.cpp"
TEST(WithdrawalContinuousGate, CompleteDetachedStoppedSceneAndOpeningCandidate) {
  const char* fixture=std::getenv("WORKCELL_DETACHED_FIXTURE");
  ASSERT_NE(fixture,nullptr);
  const std::string directory=std::string(fixture)+"/";
  auto read=[&](const std::string& name) {
    std::ifstream file(directory+name,std::ios::binary);if(!file) throw std::runtime_error(name);
    return std::string(std::istreambuf_iterator<char>(file),std::istreambuf_iterator<char>());
  };
  auto u=urdf::parseURDF(read("robot.urdf"));ASSERT_TRUE(u);
  auto semantic=std::make_shared<srdf::Model>();ASSERT_TRUE(semantic->initString(*u,read("robot.srdf")));
  auto model=std::make_shared<moveit::core::RobotModel>(u,semantic);
  auto bytes=read("scene.cdr");rclcpp::SerializedMessage serialized(bytes.size());
  auto& raw=serialized.get_rcl_serialized_message();std::memcpy(raw.buffer,bytes.data(),bytes.size());raw.buffer_length=bytes.size();
  moveit_msgs::msg::PlanningScene message;rclcpp::Serialization<moveit_msgs::msg::PlanningScene> serializer;
  serializer.deserialize_message(&serialized,&message);
  planning_scene::PlanningScene scene(model);scene.setPlanningSceneMsg(message);
  auto first=scene.getCurrentState();first.update();
  std::vector<const moveit::core::AttachedBody*> bodies;first.getAttachedBodies(bodies);ASSERT_TRUE(bodies.empty());
  moveit_msgs::msg::AllowedCollisionMatrix baseline;scene.getAllowedCollisionMatrix().getMessage(baseline);
  collision_detection::CollisionRequest request;request.contacts=true;request.max_contacts=1000000;request.max_contacts_per_pair=1000000;
  collision_detection::CollisionResult start;scene.checkCollision(request,start,first);
  std::cout<<std::setprecision(17)<<"START collision="<<start.collision<<" contacts="<<start.contact_count<<" pairs="<<start.contacts.size()<<"\n";
  ASSERT_LT(start.contact_count,request.max_contacts);
  for(const auto& pair:start.contacts) {
    ASSERT_LT(pair.second.size(),request.max_contacts_per_pair);
    double maximum=0.;for(const auto& c:pair.second) maximum=std::max(maximum,c.depth);
    const auto& c=pair.second.front();
    std::cout<<"CONTACT first="<<pair.first.first<<" second="<<pair.first.second<<" count="<<pair.second.size()
      <<" max_depth="<<maximum<<" type1="<<int(c.body_type_1)<<" type2="<<int(c.body_type_2)<<"\n";
  }
  ASSERT_TRUE(start.collision);
  const std::string leader=read("leader.txt");const moveit::core::JointModelGroup* group=nullptr;
  for(const auto* candidate:model->getJointModelGroups())
    if(candidate->getActiveJointModels().size()==1&&candidate->getActiveJointModels()[0]->getName()==leader) {group=candidate;break;}
  ASSERT_NE(group,nullptr);
  auto last=first;last.setVariablePosition(leader,first.getVariablePosition(leader)-.01);last.update();
  first.zeroVelocities();first.zeroAccelerations();last.zeroVelocities();last.zeroAccelerations();
  robot_trajectory::RobotTrajectory trajectory(model,group->getName());trajectory.addSuffixWayPoint(first,0.);trajectory.addSuffixWayPoint(last,1.);
  std::size_t moving=0;double maximum_translation=0.;
  for(const auto* link:model->getLinkModels()) {
    bool affected=false;
    for(auto* cursor=link;cursor;cursor=cursor->getParentLinkModel()) {
      auto* joint=cursor->getParentJointModel();
      while(joint->getMimic()) joint=joint->getMimic();
      affected=affected||joint->getName()==leader;
    }
    if(!affected) continue;
    const auto& a=first.getGlobalLinkTransform(link);const auto& b=last.getGlobalLinkTransform(link);
    const double translation=(a.translation()-b.translation()).norm();const double angle=Eigen::AngleAxisd(a.linear()*b.linear().transpose()).angle();
    if(translation>0.||angle>0.) {++moving;maximum_translation=std::max(maximum_translation,translation);std::cout<<"MOVING link="<<link->getName()<<" translation="<<translation<<" angle="<<angle<<"\n";}
  }
  moveit_msgs::msg::RobotTrajectory emitted;trajectory.getRobotTrajectoryMsg(emitted);
  joint_trajectory_controller::Trajectory jtc;
  double previous_depth=0.;bool saw_start=false;
  for(int64_t stamp:{0LL,1000LL,10000LL,100000LL,1000000LL,3000000LL,4000000LL,5000000LL,10000000LL}) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    jtc.interpolate_between_points(rclcpp::Time(0),emitted.joint_trajectory.points[0],rclcpp::Time(1000000000LL),
      emitted.joint_trajectory.points[1],rclcpp::Time(stamp),point);
    auto state=first;state.setVariablePosition(leader,point.positions[0]);state.update();
    collision_detection::CollisionResult sampled;scene.checkCollision(request,sampled,state);
    ASSERT_LT(sampled.contact_count,request.max_contacts);
    double depth=0.;
    for(const auto& pair:sampled.contacts) {
      ASSERT_LT(pair.second.size(),request.max_contacts_per_pair);
      for(const auto& contact:pair.second) depth=std::max(depth,contact.depth);
    }
    std::cout<<"EARLY_JTC ns="<<stamp<<" delta="<<point.positions[0]-first.getVariablePosition(leader)
      <<" pairs="<<sampled.contacts.size()<<" max_depth="<<depth<<" depth_increased="<<(saw_start&&depth>previous_depth)<<"\n";
    previous_depth=depth;saw_start=true;
  }
  collision_detection::CollisionResult endpoint;scene.checkCollision(request,endpoint,last);
  const auto qualified=workcell::controller_certificate::certifyDetached(trajectory,scene,"historical-stopped-104485-diagnostic");
  const auto& report=qualified.audit;
  std::cout<<"ENUMERATION complete="<<qualified.enumeration_complete<<" pairs="<<qualified.pairs.size()<<"\n";
  for(const auto& pair:qualified.pairs) {
    double raw=0.;for(const auto& c:pair.initial_evidence) raw=std::max(raw,c.depth);
    std::cout<<"PAIR link="<<pair.robot_link<<" world="<<pair.world_object<<" epoch="<<pair.epoch
      <<" certificate="<<pair.certificate_type<<" initial_depth="<<raw<<" plane_axis="<<pair.axis.transpose()
      <<" world_support="<<pair.world_support<<" initial_projection_gap="<<pair.initial_gap
      <<" state="<<int(pair.state)<<" transitions="<<pair.transitions.size()<<"\n";
    for(const auto& event:pair.transitions) std::cout<<"TRANSITION state="<<int(event.state)
      <<" begin_ns="<<event.begin_ns<<" end_ns="<<event.end_ns<<" clearance_lower="<<event.clearance_lower<<"\n";
    if(const char* records=std::getenv("WORKCELL_DETACHED_RECORDS")) {
      std::ofstream identity(std::string(records)+"/"+pair.robot_link+".geometry.txt");
      identity<<pair.geometry_identity<<"\n";
    }
  }
  // The source fixture remains untouched. Record the measured/controller start
  // delta if nominal mimic evaluation cannot reproduce the saved start exactly.
  auto normalized=first;normalized.setVariablePosition(leader,first.getVariablePosition(leader));normalized.update();
  for(const auto& name:model->getVariableNames()) if(normalized.getVariablePosition(name)!=first.getVariablePosition(name))
    std::cout<<"START_DISCONTINUITY variable="<<name<<" measured="<<first.getVariablePosition(name)
      <<" controller="<<normalized.getVariablePosition(name)<<" delta="<<normalized.getVariablePosition(name)-first.getVariablePosition(name)<<"\n";
  moveit_msgs::msg::AllowedCollisionMatrix after;scene.getAllowedCollisionMatrix().getMessage(after);EXPECT_EQ(baseline,after);
  std::cout<<"CANDIDATE leader="<<leader<<" group="<<group->getName()<<" delta=-0.01 duration=1 moving_links="<<moving
    <<" max_link_origin_translation="<<maximum_translation<<" endpoint_collision="<<endpoint.collision<<" endpoint_contacts="<<endpoint.contact_count<<"\n";
  std::cout<<"AUDIT result="<<int(report.result)<<" reason="<<report.reason<<" inspected="<<report.inspected<<" certified="<<report.certified
    <<" subdivided="<<report.subdivided<<" depth="<<report.deepest<<" failure_begin="<<report.failure_begin_ns<<" failure_end="<<report.failure_end_ns<<" seconds="<<report.wall_seconds<<"\n";
  EXPECT_TRUE(qualified.enumeration_complete);
  EXPECT_EQ(moving,8U);
  // This is a diagnostic, not an assertion that endpoint clearance authorizes motion.
  std::cout<<"QUALIFIED="<<(report.result==workcell::ControllerCertificate::CERTIFIED_CLEAR)<<"\n";
  std::cout<<"AUTHORITY_R=NOT_ISSUED EXECUTION=NOT_RUN BASELINE_ACM=UNCHANGED ATTACHMENT=NONE\n";
}
