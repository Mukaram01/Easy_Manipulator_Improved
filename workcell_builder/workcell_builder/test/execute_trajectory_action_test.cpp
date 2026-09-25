#include "commission_execute_server.hpp"
#include <gtest/gtest.h>
#include <future>
using namespace std::chrono_literals;
using workcell::ExecutionBackend;
using workcell::ExecutionOutcome;
using workcell::CommissionExecuteServer;
using Action=moveit_msgs::action::ExecuteTrajectory;
class ControlledBackend : public ExecutionBackend {
public:
 std::mutex mutex; std::condition_variable cv;
 bool preparing=false,hold_prepare=false,hold_result=false,prepared_ok=true,running=false,done=false;
 bool hold_start=false,starting=false,throw_wait=false;
 int starts=0,stops=0; ExecutionOutcome result=ExecutionOutcome::Failed;
 bool prepare(const Action::Goal&) override {
  std::unique_lock<std::mutex> lock(mutex);preparing=true;cv.notify_all();
  cv.wait(lock,[&]{return !hold_prepare;});return prepared_ok;
 }
 void start() override {std::unique_lock<std::mutex> lock(mutex);starting=true;cv.notify_all();cv.wait(lock,[&]{return !hold_start;});starts++;running=true;cv.notify_all();}
 ExecutionOutcome wait() override {
  std::unique_lock<std::mutex> lock(mutex);cv.wait(lock,[&]{return done&&!hold_result;});if(throw_wait)throw std::runtime_error("injected wait failure");return result;
 }
 void stop() override {
  std::lock_guard<std::mutex> lock(mutex);stops++;
  if(!done){result=ExecutionOutcome::Interrupted;done=true;running=false;}cv.notify_all();
 }
 void discard() override {}
 void complete(ExecutionOutcome status){std::lock_guard<std::mutex> lock(mutex);result=status;done=true;running=false;cv.notify_all();}
 void release(){std::lock_guard<std::mutex> lock(mutex);hold_prepare=false;hold_result=false;hold_start=false;cv.notify_all();}
 void reached(bool before){std::unique_lock<std::mutex> lock(mutex);ASSERT_TRUE(cv.wait_for(lock,3s,[&]{return before?preparing:running;}));}
};
class ActionTest:public ::testing::Test {
protected:
 static void SetUpTestSuite(){if(!rclcpp::ok())rclcpp::init(0,nullptr);}
 rclcpp::Node::SharedPtr node;std::shared_ptr<ControlledBackend> backend;
 std::unique_ptr<CommissionExecuteServer> server;
 rclcpp_action::Client<Action>::SharedPtr client;
 rclcpp::executors::SingleThreadedExecutor executor;
 std::thread spin;
 void SetUp() override {
  if(!rclcpp::ok())rclcpp::init(0,nullptr);
  node=std::make_shared<rclcpp::Node>("commission_action_test");backend=std::make_shared<ControlledBackend>();
  server=std::make_unique<CommissionExecuteServer>(node,backend);
  client=rclcpp_action::create_client<Action>(node,"execute_trajectory");executor.add_node(node);spin=std::thread([&]{executor.spin();});
  ASSERT_TRUE(client->wait_for_action_server(3s));
 }
 void TearDown() override {backend->release();server->shutdown();server.reset();executor.cancel();spin.join();executor.remove_node(node);client.reset();node.reset();}
 template<class T> auto get(T future){if(future.wait_for(4s)!=std::future_status::ready)throw std::runtime_error("action test response timeout");return future.get();}
 auto goal(){Action::Goal g;return get(client->async_send_goal(g));}
 auto cancel(const rclcpp_action::ClientGoalHandle<Action>::SharedPtr& h){return get(client->async_cancel_goal(h));}
};
TEST_F(ActionTest, CancelBeforeStart){
 backend->hold_prepare=true;auto h=goal();ASSERT_TRUE(h);backend->reached(true);
 auto c=cancel(h);ASSERT_EQ(c->goals_canceling.size(),1u);EXPECT_EQ(c->goals_canceling[0].goal_id.uuid,h->get_goal_id());
 backend->release();auto r=get(client->async_get_result(h));EXPECT_EQ(r.code,rclcpp_action::ResultCode::CANCELED);EXPECT_EQ(backend->starts,0);
}
TEST_F(ActionTest, CancelDuringExecutionPropagatesStop){
 auto h=goal();backend->reached(false);ASSERT_TRUE(h);auto c=cancel(h);EXPECT_EQ(c->return_code,0);
 auto r=get(client->async_get_result(h));EXPECT_EQ(r.code,rclcpp_action::ResultCode::CANCELED);EXPECT_EQ(backend->stops,1);EXPECT_FALSE(backend->running);
}
TEST_F(ActionTest, NaturalCompletionWinsCancelRace){
 backend->hold_result=true;auto h=goal();backend->reached(false);backend->complete(ExecutionOutcome::Succeeded);
 auto c=cancel(h);EXPECT_EQ(c->return_code,0);backend->release();
 EXPECT_EQ(get(client->async_get_result(h)).code,rclcpp_action::ResultCode::SUCCEEDED);
 auto service=node->create_client<action_msgs::srv::CancelGoal>("/execute_trajectory/_action/cancel_goal");
 ASSERT_TRUE(service->wait_for_service(2s));
 auto late=std::make_shared<action_msgs::srv::CancelGoal::Request>();late->goal_info.goal_id.uuid=h->get_goal_id();
 EXPECT_TRUE(get(service->async_send_request(late))->goals_canceling.empty());
}
TEST_F(ActionTest, DuplicateWrongGoalAndOverlap){
 backend->hold_result=true;auto h=goal();backend->reached(false);EXPECT_FALSE(goal());
 auto service=node->create_client<action_msgs::srv::CancelGoal>("/execute_trajectory/_action/cancel_goal");ASSERT_TRUE(service->wait_for_service(2s));
 auto req=std::make_shared<action_msgs::srv::CancelGoal::Request>();req->goal_info.goal_id.uuid.fill(99);
 EXPECT_TRUE(get(service->async_send_request(req))->goals_canceling.empty());EXPECT_EQ(backend->stops,0);
 EXPECT_EQ(cancel(h)->return_code,0);cancel(h);backend->release();
 EXPECT_EQ(get(client->async_get_result(h)).code,rclcpp_action::ResultCode::CANCELED);EXPECT_EQ(backend->stops,1);
}
TEST_F(ActionTest, BackendFailure){auto h=goal();backend->reached(false);backend->complete(ExecutionOutcome::Failed);EXPECT_EQ(get(client->async_get_result(h)).code,rclcpp_action::ResultCode::ABORTED);}
TEST_F(ActionTest, ShutdownStopsAndJoins){auto h=goal();backend->reached(false);server->shutdown();EXPECT_EQ(backend->stops,1);EXPECT_FALSE(backend->running);EXPECT_EQ(get(client->async_get_result(h)).code,rclcpp_action::ResultCode::ABORTED);}
TEST_F(ActionTest, PreparationFailure){backend->prepared_ok=false;auto h=goal();EXPECT_EQ(get(client->async_get_result(h)).code,rclcpp_action::ResultCode::ABORTED);EXPECT_EQ(backend->starts,0);}

TEST_F(ActionTest, CancelResponsiveWhileBackendStarts){
 backend->hold_start=true;auto h=goal();
 {std::unique_lock<std::mutex> lock(backend->mutex);ASSERT_TRUE(backend->cv.wait_for(lock,3s,[&]{return backend->starting;}));}
 auto pending=client->async_cancel_goal(h);
 const bool prompt=pending.wait_for(200ms)==std::future_status::ready;
 backend->release();EXPECT_TRUE(prompt);EXPECT_EQ(get(pending)->return_code,0);
 EXPECT_EQ(get(client->async_get_result(h)).code,rclcpp_action::ResultCode::CANCELED);EXPECT_EQ(backend->stops,1);
}
TEST_F(ActionTest, WaitExceptionDuringCancelStopsOnce){
 backend->throw_wait=true;auto h=goal();backend->reached(false);EXPECT_EQ(cancel(h)->return_code,0);
 EXPECT_EQ(get(client->async_get_result(h)).code,rclcpp_action::ResultCode::ABORTED);EXPECT_EQ(backend->stops,1);EXPECT_FALSE(backend->running);
}

#include "controller_terminal_audit.hpp"
TEST_F(ActionTest, ControllerTerminalAuditReadsActualNaturalSuccessAndCancellation){
 using FJT=control_msgs::action::FollowJointTrajectory;
 using GoalHandle=rclcpp_action::ServerGoalHandle<FJT>;
 std::mutex control_mutex;std::condition_variable control_cv;std::shared_ptr<GoalHandle> owned;
 auto controller=rclcpp_action::create_server<FJT>(node,"/test_controller/follow_joint_trajectory",
  [](const auto&,const auto&){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
  [](const auto&){return rclcpp_action::CancelResponse::ACCEPT;},
  [&](auto h){std::lock_guard<std::mutex> lock(control_mutex);owned=h;control_cv.notify_all();});
 workcell::ControllerTerminalAudit audit(node,"/test_controller/follow_joint_trajectory");
 auto controller_client=rclcpp_action::create_client<FJT>(node,"/test_controller/follow_joint_trajectory");
 ASSERT_TRUE(controller_client->wait_for_action_server(3s));ASSERT_TRUE(audit.arm());
 auto first=get(controller_client->async_send_goal(FJT::Goal()));ASSERT_TRUE(first);
 {std::unique_lock<std::mutex> lock(control_mutex);ASSERT_TRUE(control_cv.wait_for(lock,2s,[&]{return bool(owned);}));owned->succeed(std::make_shared<FJT::Result>());owned.reset();}
 // This immutable result is SUCCEEDED even if TEM/handle caches say PREEMPTED.
 EXPECT_EQ(audit.terminal(),4);ASSERT_TRUE(audit.arm());
 auto second=get(controller_client->async_send_goal(FJT::Goal()));ASSERT_TRUE(second);
 EXPECT_EQ(get(controller_client->async_cancel_goal(second))->return_code,0);
 {std::unique_lock<std::mutex> lock(control_mutex);ASSERT_TRUE(control_cv.wait_for(lock,2s,[&]{return bool(owned);}));owned->canceled(std::make_shared<FJT::Result>());}
 EXPECT_EQ(audit.terminal(),5);
}
