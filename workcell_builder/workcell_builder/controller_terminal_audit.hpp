#pragma once
// Read-only observation of the existing controller action; never sends a goal
// or cancellation. TEM remains the sole owner of controller execution.
#include <rclcpp/rclcpp.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <condition_variable>
#include <map>
#include <set>
#include <mutex>
namespace workcell {
class ControllerTerminalAudit {
 using ResultService=control_msgs::action::FollowJointTrajectory::Impl::GetResultService;
 using UUID=std::array<uint8_t,16>;
 rclcpp::Node::SharedPtr node_;
 struct State {
  std::mutex mutex;std::condition_variable cv;
  std::map<UUID,int8_t> states;std::set<UUID> baseline;
 };
 std::shared_ptr<State> state_=std::make_shared<State>();
 rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr subscription_;
 rclcpp::Client<ResultService>::SharedPtr client_;
public:
 ControllerTerminalAudit(rclcpp::Node::SharedPtr node,const std::string& action):node_(std::move(node)) {
  subscription_=node_->create_subscription<action_msgs::msg::GoalStatusArray>(action+"/_action/status",
   rclcpp::QoS(10).reliable().transient_local(),[state=state_](const action_msgs::msg::GoalStatusArray& msg){
    std::lock_guard<std::mutex> lock(state->mutex);
    for(const auto& item:msg.status_list){
     auto& status=state->states[item.goal_info.goal_id.uuid];
     // A result-service reply may precede the queued terminal status message.
     // Once terminal, an older status cannot make that goal active again.
     if(status<4)status=item.status;
    }
    state->cv.notify_all();
   });
  client_=node_->create_client<ResultService>(action+"/_action/get_result");
 }
 bool arm() {
  std::lock_guard<std::mutex> lock(state_->mutex);state_->baseline.clear();
  for(const auto& [uuid,status]:state_->states){if(status>=1&&status<=3)return false;state_->baseline.insert(uuid);}
  return true;
 }
 int8_t terminal() {
  std::unique_lock<std::mutex> lock(state_->mutex);
  auto new_goals=[&]{std::vector<UUID> ids;for(const auto& item:state_->states)if(!state_->baseline.count(item.first))ids.push_back(item.first);return ids;};
  if(!state_->cv.wait_for(lock,std::chrono::seconds(2),[&]{return !new_goals().empty();}))throw std::runtime_error("controller terminal goal identity missing");
  const auto ids=new_goals();if(ids.size()!=1)throw std::runtime_error("controller terminal goal identity ambiguous");
  auto request=std::make_shared<ResultService::Request>();request->goal_id.uuid=ids.front();lock.unlock();
  auto future=client_->async_send_request(request);
  if(future.wait_for(std::chrono::seconds(2))!=std::future_status::ready)throw std::runtime_error("controller terminal result missing");
  const auto result=future.get();
  lock.lock();if(new_goals()!=ids)throw std::runtime_error("controller ownership changed during result audit");
  if(result->status<4||result->status>6)throw std::runtime_error("controller result is not terminal");
  state_->states[ids.front()]=result->status;
  return result->status;
 }
};
}
