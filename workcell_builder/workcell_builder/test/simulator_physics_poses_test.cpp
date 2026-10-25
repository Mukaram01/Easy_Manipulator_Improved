#include <gtest/gtest.h>
#include "src_simulator_physics_poses.hpp"
using namespace workcell;
namespace {
struct Fixture : testing::Test {
 sim::EntityComponentManager ecm;
 sim::UpdateInfo info;
 sim::Entity model, link;
 PhysicsPoseQueries queries;
 ignition::math::Pose3d offset{.1,.2,.3,.2,.3,.4};
 void SetUp() override {
   model=ecm.CreateEntity(); link=ecm.CreateEntity();
   ecm.CreateComponent(model,c::Model());
   ecm.CreateComponent(model,c::Pose(ignition::math::Pose3d(2,3,4,.4,.2,.1)));
   ecm.CreateComponent(model,c::ModelCanonicalLink(link));
   ecm.CreateComponent(link,c::Link()); ecm.CreateComponent(link,c::CanonicalLink());
   ecm.CreateComponent(link,c::Pose(offset)); ecm.CreateComponent(link,c::ParentEntity(model));
   ecm.SetParentEntity(link,model); info.iterations=7; info.simTime=std::chrono::milliseconds(7);
 }
 void Write(const ignition::math::Pose3d& p) {
   *ecm.Component<c::WorldPose>(queries.QueryEntity(link))=c::WorldPose(p);
 }
};
TEST_F(Fixture, DirectSubMicrometreMotionAndCanonicalFrame) {
 const auto cached=sim::worldPose(link,ecm);
 const ignition::math::Pose3d world(2,3,4,.4,.2,.1);
 for(int i=0;i<5;++i) {
   queries.Prepare(info,ecm);
   auto expected=world; expected.Pos().Z()+=i*1e-8; Write(expected*offset);
   EXPECT_LT((queries.ModelPose(model,info,ecm).Pos()-expected.Pos()).Length(),1e-14);
   EXPECT_LT((queries.LinkPose(link,info,ecm).Pos()-(expected*offset).Pos()).Length(),1e-14);
   EXPECT_NEAR(queries.ModelPose(model,info,ecm).Rot().W(),expected.Rot().W(),1e-14);
   EXPECT_NEAR(queries.ModelPose(model,info,ecm).Rot().X(),expected.Rot().X(),1e-14);
   EXPECT_NEAR(queries.ModelPose(model,info,ecm).Rot().Y(),expected.Rot().Y(),1e-14);
   EXPECT_NEAR(queries.ModelPose(model,info,ecm).Rot().Z(),expected.Rot().Z(),1e-14);
   EXPECT_EQ(sim::worldPose(link,ecm),cached);
   const std::unordered_set<sim::ComponentTypeId> observation_only{
     c::Pose::typeId,c::WorldPose::typeId,c::Name::typeId,c::ParentEntity::typeId};
   EXPECT_EQ(ecm.ComponentTypes(queries.QueryEntity(link)),observation_only);
   EXPECT_EQ(ecm.ParentEntity(queries.QueryEntity(link)),link);
   EXPECT_FALSE(ecm.Component<c::Link>(queries.QueryEntity(link)));
   EXPECT_FALSE(ecm.Component<c::Collision>(queries.QueryEntity(link)));
   EXPECT_FALSE(ecm.Component<c::Joint>(queries.QueryEntity(link)));
   ++info.iterations; info.simTime+=std::chrono::milliseconds(1);
 }
}
TEST_F(Fixture, MissingOrStaleBackendWriteFailsClosed) {
 queries.Prepare(info,ecm);
 EXPECT_THROW(queries.LinkPose(link,info,ecm),std::runtime_error);
 Write(ignition::math::Pose3d()); EXPECT_NO_THROW(queries.LinkPose(link,info,ecm));
 ++info.iterations; info.simTime+=std::chrono::milliseconds(1);
 EXPECT_THROW(queries.LinkPose(link,info,ecm),std::runtime_error);
 queries.Prepare(info,ecm);
 EXPECT_THROW(queries.LinkPose(link,info,ecm),std::runtime_error);
}
TEST_F(Fixture, SimTimeAndNonfiniteQueryAreRejected) {
 queries.Prepare(info,ecm); Write(ignition::math::Pose3d());
 auto other_tick=info; other_tick.simTime+=std::chrono::nanoseconds(1);
 EXPECT_THROW(queries.LinkPose(link,other_tick,ecm),std::runtime_error);
 auto invalid=ignition::math::Pose3d(); invalid.Rot().W()=std::numeric_limits<double>::infinity();
 Write(invalid); EXPECT_THROW(queries.LinkPose(link,info,ecm),std::runtime_error);
 ecm.RemoveComponent<c::WorldPose>(queries.QueryEntity(link));
 EXPECT_THROW(queries.LinkPose(link,info,ecm),std::runtime_error);
}
TEST_F(Fixture, MissingEntityOrIdentityChangeFailsClosed) {
 queries.Prepare(info,ecm); Write(ignition::math::Pose3d());
 EXPECT_THROW(queries.LinkPose(999999,info,ecm),std::runtime_error);
 EXPECT_THROW(queries.ModelPose(999999,info,ecm),std::runtime_error);
 ecm.RemoveComponent<c::Link>(link);
 EXPECT_THROW(queries.LinkPose(link,info,ecm),std::runtime_error);
}
TEST_F(Fixture, MissingCanonicalOrNestedCanonicalFailsClosed) {
 queries.Prepare(info,ecm); Write(ignition::math::Pose3d());
 ecm.RemoveComponent<c::ModelCanonicalLink>(model);
 EXPECT_THROW(queries.ModelPose(model,info,ecm),std::runtime_error);
 ecm.CreateComponent(model,c::ModelCanonicalLink(link));
 auto other=ecm.CreateEntity(); *ecm.Component<c::ParentEntity>(link)=c::ParentEntity(other);
 ecm.SetParentEntity(link,other);
 EXPECT_THROW(queries.ModelPose(model,info,ecm),std::runtime_error);
}
TEST_F(Fixture, QueryIdentityAndFixedCanonicalOffsetCannotDrift) {
 queries.Prepare(info,ecm); Write(ignition::math::Pose3d());
 *ecm.Component<c::Pose>(link)=c::Pose();
 EXPECT_THROW(queries.ModelPose(model,info,ecm),std::runtime_error);
 *ecm.Component<c::Pose>(queries.QueryEntity(link))=c::Pose(offset);
 EXPECT_THROW(queries.LinkPose(link,info,ecm),std::runtime_error);
}
}
