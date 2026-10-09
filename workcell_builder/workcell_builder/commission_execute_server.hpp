#pragma once
// Action ownership correction for reviewed MoveIt 2.5.9/2.5.10. The backend remains its TEM.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <sstream>
#include <iomanip>
namespace workcell {
using ExecuteAction=moveit_msgs::action::ExecuteTrajectory;
enum class ExecutionOutcome { Succeeded, Interrupted, Failed };
struct ExecutionBackend {
 virtual ~ExecutionBackend()=default;
 virtual bool prepare(const ExecuteAction::Goal&)=0; // bounded, no motion
 virtual void start()=0; // nonblocking start, serialized against cancel
 virtual ExecutionOutcome wait()=0; // actual backend/controller terminal
 virtual void stop()=0; // returns only when backend execution has stopped
 virtual void discard()=0;
};
class CommissionExecuteServer {
 using Handle=rclcpp_action::ServerGoalHandle<ExecuteAction>;
 struct Owned {
  rclcpp_action::GoalUUID uuid;std::shared_ptr<Handle> handle;
  bool cancel=false,started=false,completed=false,stop_inflight=false,stop_done=false,stop_failed=false;
 };
 rclcpp::Node::SharedPtr node_;std::shared_ptr<ExecutionBackend> backend_;
 std::mutex mutex_;std::condition_variable cv_;std::shared_ptr<Owned> owned_;bool shutdown_=false;
 rclcpp::CallbackGroup::SharedPtr group_;
 rclcpp::executors::SingleThreadedExecutor callbacks_;
 rclcpp_action::Server<ExecuteAction>::SharedPtr server_;
 std::thread callback_thread_,worker_,stopper_;
 void event(const char* what,const Owned& goal) {
  std::ostringstream id;for(auto b:goal.uuid)id<<std::hex<<std::setw(2)<<std::setfill('0')<<int(b);
  const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  RCLCPP_INFO(node_->get_logger(),"commission_execute event=%s uuid=%s wall_ns=%ld",what,id.str().c_str(),ns);
 }
 void stopLoop() {
  std::unique_lock<std::mutex> lock(mutex_);
  for(;;) {
   cv_.wait(lock,[&]{return (shutdown_&&!owned_) || (owned_&&owned_->started&&!owned_->completed&&!owned_->stop_done&&!owned_->stop_inflight&&(owned_->cancel||shutdown_));});
   if(shutdown_&&!owned_)return;
   auto goal=owned_;goal->stop_inflight=true;event("stop_begin",*goal);lock.unlock();
   bool failed=false;try{backend_->stop();}catch(const std::exception& e){RCLCPP_ERROR(node_->get_logger(),"stop failed: %s",e.what());failed=true;}
   lock.lock();goal->stop_inflight=false;goal->stop_done=true;goal->stop_failed=failed;event("stop_return",*goal);cv_.notify_all();
  }
 }
 void workLoop() {
  std::unique_lock<std::mutex> lock(mutex_);
  for(;;) {
   cv_.wait(lock,[&]{return shutdown_ || (owned_&&owned_->handle);});
   if(shutdown_&&!owned_)return;
   if(!owned_->handle){cv_.wait_for(lock,std::chrono::milliseconds(1));continue;}
   auto goal=owned_;auto handle=goal->handle;
   ExecutionOutcome outcome=ExecutionOutcome::Failed;bool prestart_cancel=false,start_attempted=false;
   lock.unlock();
   try {
    const bool prepared=backend_->prepare(*handle->get_goal());
    lock.lock();
    prestart_cancel=goal->cancel;
    if(prepared&&!goal->cancel&&!shutdown_) {
     // Linearize the decision to start, but do not hold the callback mutex
     // while TEM validates/starts. A concurrent cancel is accepted promptly;
     // the stopper runs as soon as start returns, never before it can stop TEM.
     start_attempted=true;lock.unlock();backend_->start();lock.lock();
     goal->started=true;event("started",*goal);cv_.notify_all();
     lock.unlock();outcome=backend_->wait();lock.lock();
    }
    goal->completed=true;cv_.notify_all();
    cv_.wait(lock,[&]{return !goal->stop_inflight;});
    lock.unlock();backend_->discard();lock.lock();
   } catch(const std::exception& e) {
    if(!lock.owns_lock())lock.lock();
    RCLCPP_ERROR(node_->get_logger(),"execution failed: %s",e.what());
    // Backend errors must also stop any started execution before releasing its ownership.
    goal->completed=true; // exclude the stopper before taking error cleanup
    cv_.wait(lock,[&]{return !goal->stop_inflight;});
    if(start_attempted&&!goal->stop_done) {
     goal->stop_inflight=true;lock.unlock();bool failed=false;
     try{backend_->stop();}catch(...){failed=true;}
     lock.lock();goal->stop_inflight=false;goal->stop_failed=failed;goal->stop_done=true;
    }
    outcome=ExecutionOutcome::Failed;cv_.notify_all();
   }
   const bool canceled=!goal->stop_failed && goal->cancel &&
       (prestart_cancel || (goal->started&&goal->stop_done&&outcome==ExecutionOutcome::Interrupted));
   // rclcpp marks CANCELING after the cancel callback returns. Do not issue a
   // terminal transition until that action-server transition is visible.
   while(canceled&&!handle->is_canceling()&&rclcpp::ok())cv_.wait_for(lock,std::chrono::milliseconds(1));
   auto result=std::make_shared<ExecuteAction::Result>();
   try {
    if(outcome==ExecutionOutcome::Succeeded&&!goal->stop_failed) {
     result->error_code.val=moveit_msgs::msg::MoveItErrorCodes::SUCCESS;handle->succeed(result);event("succeeded",*goal);
    } else if(canceled&&handle->is_canceling()) {
     result->error_code.val=moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;handle->canceled(result);event("canceled",*goal);
    } else {
     result->error_code.val=moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;handle->abort(result);event("aborted",*goal);
    }
   } catch(const std::exception& e){RCLCPP_ERROR(node_->get_logger(),"terminal publication failed: %s",e.what());}
   owned_.reset();cv_.notify_all();
  }
 }
public:
 CommissionExecuteServer(rclcpp::Node::SharedPtr node,std::shared_ptr<ExecutionBackend> backend):node_(std::move(node)),backend_(std::move(backend)) {
  group_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
  callbacks_.add_callback_group(group_,node_->get_node_base_interface());
  server_=rclcpp_action::create_server<ExecuteAction>(node_->get_node_base_interface(),node_->get_node_clock_interface(),node_->get_node_logging_interface(),node_->get_node_waitables_interface(),"execute_trajectory",
   [this](const rclcpp_action::GoalUUID& uuid,const std::shared_ptr<const ExecuteAction::Goal>&){
    std::lock_guard<std::mutex> lock(mutex_);if(shutdown_||owned_)return rclcpp_action::GoalResponse::REJECT;
    owned_=std::make_shared<Owned>();owned_->uuid=uuid;event("accepted",*owned_);return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
   },
   [this](const std::shared_ptr<Handle> handle){
    std::lock_guard<std::mutex> lock(mutex_);
    if(shutdown_||!owned_||owned_->uuid!=handle->get_goal_id()||owned_->completed)return rclcpp_action::CancelResponse::REJECT;
    owned_->cancel=true;event("cancel_request",*owned_);cv_.notify_all();return rclcpp_action::CancelResponse::ACCEPT;
   },
   [this](const std::shared_ptr<Handle> handle){std::lock_guard<std::mutex> lock(mutex_);owned_->handle=handle;cv_.notify_all();},
   rcl_action_server_get_default_options(),group_);
  worker_=std::thread([this]{workLoop();});stopper_=std::thread([this]{stopLoop();});callback_thread_=std::thread([this]{callbacks_.spin();});
 }
 void shutdown(){
  {std::lock_guard<std::mutex> lock(mutex_);shutdown_=true;cv_.notify_all();}
  if(worker_.joinable())worker_.join();
  if(stopper_.joinable())stopper_.join();
 }
 ~CommissionExecuteServer(){
  shutdown();
  callbacks_.cancel();
  if(callback_thread_.joinable())callback_thread_.join();
 }
};
}
