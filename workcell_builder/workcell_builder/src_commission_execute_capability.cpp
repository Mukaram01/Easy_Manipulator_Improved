// Pinned overlay for MoveIt 2.5.9 ExecuteTrajectoryAction; provenance and build
// instructions: docs/manuals/EXECUTE_TRAJECTORY_COMMISSIONING.md.
#include "commission_execute_server.hpp"
#include "controller_terminal_audit.hpp"
#include <moveit/move_group/move_group_capability.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <pluginlib/class_list_macros.hpp>
#include <set>
namespace workcell {
class TemBackend final : public ExecutionBackend {
 trajectory_execution_manager::TrajectoryExecutionManagerPtr manager_;
 std::vector<moveit_controller_manager::MoveItControllerHandlePtr> handles_;
 std::map<std::string,std::unique_ptr<ControllerTerminalAudit>> audits_;
 std::vector<ControllerTerminalAudit*> selected_audits_;
 std::mutex mutex_;std::condition_variable cv_;bool finished_=false;
 ExecutionOutcome outcome_=ExecutionOutcome::Failed;
public:
 explicit TemBackend(rclcpp::Node::SharedPtr node,trajectory_execution_manager::TrajectoryExecutionManagerPtr manager):manager_(std::move(manager)){
  std::vector<std::string> names;manager_->getControllerManager()->getControllersList(names);
  for(const auto& name:names){
   // These are the action names from the existing MoveIt controller config.
   std::string type,ns;
   node->get_parameter("moveit_simple_controller_manager."+name+".type",type);
   node->get_parameter("moveit_simple_controller_manager."+name+".action_ns",ns);
   if(type=="FollowJointTrajectory"&&!ns.empty())audits_[name]=std::make_unique<ControllerTerminalAudit>(node,"/"+name+"/"+ns);
  }
 }
 bool prepare(const ExecuteAction::Goal& goal) override {
  if(!manager_)return false;
  std::vector<std::string> active,controllers;manager_->getControllerManager()->getActiveControllers(active);
  handles_.clear();selected_audits_.clear();std::set<std::string> covered;
  const std::set<std::string> requested(goal.trajectory.joint_trajectory.joint_names.begin(),goal.trajectory.joint_trajectory.joint_names.end());
  for(const auto& name:active) {
   std::vector<std::string> joints;manager_->getControllerManager()->getControllerJoints(name,joints);
   bool used=false;for(const auto& joint:joints)if(requested.count(joint)){used=true;if(!covered.insert(joint).second)return false;}
   if(!used)continue;
   auto handle=manager_->getControllerManager()->getControllerHandle(name);if(!handle)return false;
   if(!audits_.count(name)||!audits_[name]->arm())return false;
   selected_audits_.push_back(audits_[name].get());controllers.push_back(name);handles_.push_back(handle);
  }
  if(requested.empty()||covered!=requested)return false;
  {std::lock_guard<std::mutex> lock(mutex_);finished_=false;outcome_=ExecutionOutcome::Failed;}
  return manager_->push(goal.trajectory,controllers);
 }
 void start() override {
  manager_->execute([this](const auto& status){
   using Status=moveit_controller_manager::ExecutionStatus;
   std::lock_guard<std::mutex> lock(mutex_);
   outcome_=status==Status::SUCCEEDED ? ExecutionOutcome::Succeeded :
       status==Status::PREEMPTED ? ExecutionOutcome::Interrupted : ExecutionOutcome::Failed;
   finished_=true;cv_.notify_all();
  });
 }
 ExecutionOutcome wait() override {
  std::unique_lock<std::mutex> lock(mutex_);cv_.wait(lock,[&]{return finished_;});auto outcome=outcome_;lock.unlock();
  // Join TEM after its completion callback. cancelExecution() in 2.5.9 can
  // overwrite its handle's status even when the controller already succeeded.
  // Query each exact controller goal's immutable action result instead.
  manager_->waitForExecution();
  if(outcome==ExecutionOutcome::Failed)return outcome;
  bool success=true,interrupted=false,valid=true;
  for(auto* audit:selected_audits_){auto status=audit->terminal();success=success&&status==4;interrupted=interrupted||status==5;valid=valid&&(status==4||status==5);}
  return success ? ExecutionOutcome::Succeeded : valid&&interrupted&&outcome==ExecutionOutcome::Interrupted ? ExecutionOutcome::Interrupted : ExecutionOutcome::Failed;
 }
 void stop() override {manager_->stopExecution(true);}
 void discard() override {manager_->stopExecution(true);}
};
class CommissionExecuteTrajectory final: public move_group::MoveGroupCapability {
 std::unique_ptr<CommissionExecuteServer> server_;
public:
 CommissionExecuteTrajectory():MoveGroupCapability("CommissionExecuteTrajectory"){}
 void initialize() override {
  auto node=context_->moveit_cpp_->getNode();
  if(!node->get_parameter("simulator_commissioning").as_bool() ||
     node->get_parameter("execution_backend").as_string()!="simulator" ||
     node->get_parameter("use_fake_hardware").as_bool() ||
     !node->get_parameter("use_sim_time").as_bool())throw std::runtime_error("commissioning capability requires explicit simulator identity");
  server_=std::make_unique<CommissionExecuteServer>(node,std::make_shared<TemBackend>(node,context_->trajectory_execution_manager_));
  RCLCPP_INFO(node->get_logger(),"workcell ExecuteTrajectory correction: upstream MoveIt 2.5.9, simulator commissioning only");
 }
};
}
PLUGINLIB_EXPORT_CLASS(workcell::CommissionExecuteTrajectory,move_group::MoveGroupCapability)
