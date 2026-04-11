// Copyright 2020 ROS Industrial Consortium Asia Pacific
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <chrono>
#include <deque>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "emd/grasp_execution/core/scheduler.hpp"

namespace grasp_execution
{
namespace core
{

struct Worker
{
  std::shared_ptr<std::thread> execution_thread;
  std::shared_future<void> execution_future;
};

class WorkflowImplT
{
public:
  WorkflowImplT() = default;

  explicit WorkflowImplT(Scheduler::WorkflowT _workflow)
  : workflow(std::move(_workflow)), sig(), future(sig.get_future())
  {
  }

  WorkflowImplT & operator=(const WorkflowImplT &) = delete;
  WorkflowImplT(const WorkflowImplT &) = delete;

  WorkflowImplT & operator=(WorkflowImplT && rhs) noexcept
  {
    sig = std::move(rhs.sig);
    future = std::move(rhs.future);
    workflow = std::move(rhs.workflow);
    return *this;
  }

  WorkflowImplT(WorkflowImplT && rhs) noexcept
  : workflow(std::move(rhs.workflow)),
    sig(std::move(rhs.sig)),
    future(std::move(rhs.future))
  {
  }

  Scheduler::WorkflowT workflow;
  std::promise<result_t> sig;
  std::shared_future<result_t> future;
};

class Scheduler::Impl
{
public:
  enum class LifecycleState : uint8_t
  {
    QUEUED = 1,
    RUNNABLE = 2,
    ONGOING = 3,
    FINISHED = 4,
    FAILED = 5,
    CANCELLED = 6
  };

  explicit Impl(size_t _concurrency)
  : concurrency(_concurrency)
  {
    workers.resize(_concurrency);
  }

  ~Impl()
  {
    for (auto & worker : workers) {
      if (worker.execution_thread) {
        worker.execution_thread->join();
      }
    }
  }

  bool would_create_cycle(
    const std::string & workflow_id,
    const std::vector<std::string> & prerequisites) const;

  void stop_finished_worker();

  void start_worker(size_t worker_id, const std::string & workflow_id);

  int get_available_worker() const;

  void try_schedule_runnable();

  void execution_ending_cb(
    const std::string & workflow_id,
    std::promise<result_t> & workflow_sig,
    std::promise<void> & worker_sig,
    result_t result);

  Workflow::Status terminal_status_from_id(const std::string & workflow_id) const;

  void cancel_workflow_and_dependents(const std::string & workflow_id, bool set_result);

  size_t concurrency;

  std::mutex metadata_mutex;

  std::deque<std::string> runnable_queue;
  std::unordered_map<std::string, std::shared_ptr<WorkflowImplT>> waiting_workflow_map;
  std::unordered_map<std::string, std::shared_future<result_t>> queued_task_map;
  std::unordered_map<std::string, std::shared_future<result_t>> ongoing_task_map;
  std::unordered_map<std::string, result_t> finished_task_map;

  std::unordered_map<std::string, size_t> prerequisite_count;
  std::unordered_map<std::string, std::vector<std::string>> reverse_dependency_map;
  std::unordered_map<std::string, LifecycleState> lifecycle_state_map;
  std::unordered_map<std::string, bool> allow_failed_prerequisite;

  std::vector<Worker> workers;
};

bool Scheduler::Impl::would_create_cycle(
  const std::string & workflow_id,
  const std::vector<std::string> & prerequisites) const
{
  std::unordered_set<std::string> nodes;
  for (const auto & kv : lifecycle_state_map) {
    nodes.insert(kv.first);
  }
  nodes.insert(workflow_id);
  for (const auto & prereq : prerequisites) {
    nodes.insert(prereq);
  }

  std::unordered_map<std::string, std::vector<std::string>> adjacency = reverse_dependency_map;
  adjacency[workflow_id];
  for (const auto & prereq : prerequisites) {
    adjacency[prereq].push_back(workflow_id);
  }

  std::unordered_map<std::string, size_t> indegree;
  for (const auto & node : nodes) {
    indegree[node] = 0;
  }
  for (const auto & kv : adjacency) {
    for (const auto & to : kv.second) {
      if (nodes.count(to) != 0U) {
        indegree[to]++;
      }
    }
  }

  std::deque<std::string> zero_indegree;
  for (const auto & kv : indegree) {
    if (kv.second == 0) {
      zero_indegree.push_back(kv.first);
    }
  }

  size_t visited = 0;
  while (!zero_indegree.empty()) {
    auto node = std::move(zero_indegree.front());
    zero_indegree.pop_front();
    visited++;
    for (const auto & dep : adjacency[node]) {
      if (--indegree[dep] == 0) {
        zero_indegree.push_back(dep);
      }
    }
  }

  return visited != nodes.size();
}

Workflow::Status Scheduler::Impl::terminal_status_from_id(const std::string & workflow_id) const
{
  auto state_it = lifecycle_state_map.find(workflow_id);
  if (state_it == lifecycle_state_map.end()) {
    return Workflow::Status::INVALID;
  }

  switch (state_it->second) {
    case LifecycleState::FINISHED:
      return Workflow::Status::COMPLETED;
    case LifecycleState::FAILED:
      return Workflow::Status::FAILED;
    case LifecycleState::CANCELLED:
      return Workflow::Status::CANCELLED;
    default:
      return Workflow::Status::INVALID;
  }
}

void Scheduler::Impl::stop_finished_worker()
{
  for (auto & worker : workers) {
    if (!worker.execution_thread) {
      continue;
    }

    auto status = worker.execution_future.wait_for(std::chrono::nanoseconds(0));
    if (status != std::future_status::ready) {
      continue;
    }

    worker.execution_thread->join();
    worker.execution_thread.reset();
  }
}

int Scheduler::Impl::get_available_worker() const
{
  for (size_t i = 0; i < concurrency; i++) {
    if (!workers[i].execution_thread) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

void Scheduler::Impl::try_schedule_runnable()
{
  stop_finished_worker();
  while (!runnable_queue.empty()) {
    int worker_id = get_available_worker();
    if (worker_id == -1) {
      return;
    }
    const auto runnable_id = std::move(runnable_queue.front());
    runnable_queue.pop_front();
    start_worker(static_cast<size_t>(worker_id), runnable_id);
  }
}

void Scheduler::Impl::start_worker(size_t worker_id, const std::string & workflow_id)
{
  auto wf_it = waiting_workflow_map.find(workflow_id);
  if (wf_it == waiting_workflow_map.end()) {
    return;
  }

  auto workflow_impl = wf_it->second;
  waiting_workflow_map.erase(wf_it);

  auto sig = std::promise<void>();
  workers[worker_id].execution_future = sig.get_future().share();

  queued_task_map.erase(workflow_id);
  ongoing_task_map[workflow_id] = workflow_impl->future;
  lifecycle_state_map[workflow_id] = LifecycleState::ONGOING;

  workers[worker_id].execution_thread = std::make_shared<std::thread>(
    [this, wf = workflow_impl](std::promise<void> && worker_sig, std::string id) mutable {
      result_t current_result = wf->workflow(id);
      execution_ending_cb(id, wf->sig, worker_sig, current_result);
    }, std::move(sig), workflow_id);
}

void Scheduler::Impl::cancel_workflow_and_dependents(const std::string & workflow_id, bool set_result)
{
  if (lifecycle_state_map[workflow_id] == LifecycleState::CANCELLED) {
    return;
  }

  lifecycle_state_map[workflow_id] = LifecycleState::CANCELLED;
  finished_task_map[workflow_id] = false;
  prerequisite_count.erase(workflow_id);

  auto queued_it = queued_task_map.find(workflow_id);
  if (queued_it != queued_task_map.end()) {
    auto waiting_it = waiting_workflow_map.find(workflow_id);
    if (set_result && waiting_it != waiting_workflow_map.end()) {
      waiting_it->second->sig.set_value(false);
    }
    queued_task_map.erase(queued_it);
    waiting_workflow_map.erase(workflow_id);
  }

  runnable_queue.erase(
    std::remove(runnable_queue.begin(), runnable_queue.end(), workflow_id),
    runnable_queue.end());

  for (const auto & dependent : reverse_dependency_map[workflow_id]) {
    if (allow_failed_prerequisite[dependent]) {
      continue;
    }
    cancel_workflow_and_dependents(dependent, set_result);
  }
}

void Scheduler::Impl::execution_ending_cb(
  const std::string & workflow_id,
  std::promise<result_t> & workflow_sig,
  std::promise<void> & worker_sig,
  result_t result)
{
  std::string current_workflow_id = workflow_id;
  std::promise<result_t> * current_workflow_sig = &workflow_sig;
  result_t current_result = result;

  while (true) {
    std::string next_workflow_id;
    std::shared_ptr<WorkflowImplT> next_workflow;

    {
      std::lock_guard<std::mutex> guard(metadata_mutex);

      current_workflow_sig->set_value(current_result);
      finished_task_map[current_workflow_id] = current_result;
      lifecycle_state_map[current_workflow_id] =
        current_result ? LifecycleState::FINISHED : LifecycleState::FAILED;
      ongoing_task_map.erase(current_workflow_id);

      for (const auto & dependent : reverse_dependency_map[current_workflow_id]) {
        if (lifecycle_state_map[dependent] == LifecycleState::CANCELLED) {
          continue;
        }

        if (!current_result && !allow_failed_prerequisite[dependent]) {
          cancel_workflow_and_dependents(dependent, true);
          continue;
        }

        if (prerequisite_count[dependent] > 0) {
          prerequisite_count[dependent]--;
          if (prerequisite_count[dependent] == 0) {
            lifecycle_state_map[dependent] = LifecycleState::RUNNABLE;
            runnable_queue.push_back(dependent);
          }
        }
      }

      if (!runnable_queue.empty()) {
        next_workflow_id = std::move(runnable_queue.front());
        runnable_queue.pop_front();
        auto it = waiting_workflow_map.find(next_workflow_id);
        if (it != waiting_workflow_map.end()) {
          next_workflow = it->second;
          waiting_workflow_map.erase(it);
          queued_task_map.erase(next_workflow_id);
          ongoing_task_map[next_workflow_id] = next_workflow->future;
          lifecycle_state_map[next_workflow_id] = LifecycleState::ONGOING;
        }
      }

      if (!next_workflow) {
        worker_sig.set_value();
        return;
      }
    }

    current_result = next_workflow->workflow(next_workflow_id);
    current_workflow_id = std::move(next_workflow_id);
    current_workflow_sig = &next_workflow->sig;
  }
}

Scheduler::Scheduler(size_t concurrency)
: impl_(std::make_unique<Impl>(concurrency))
{
}

Scheduler::~Scheduler()
{
  std::lock_guard<std::mutex> guard(impl_->metadata_mutex);
  std::vector<std::string> to_cancel;
  to_cancel.reserve(impl_->queued_task_map.size());
  for (const auto & kv : impl_->queued_task_map) {
    to_cancel.push_back(kv.first);
  }

  for (const auto & id : to_cancel) {
    impl_->cancel_workflow_and_dependents(id, true);
  }
}

Workflow::Status Scheduler::add_workflow(
  const std::string & workflow_id,
  WorkflowT workflow)
{
  return add_workflow(workflow_id, std::move(workflow), {}, false);
}

Workflow::Status Scheduler::add_workflow(
  const std::string & workflow_id,
  WorkflowT workflow,
  const std::vector<std::string> & prerequisites,
  bool allow_if_prerequisite_failed)
{
  std::lock_guard<std::mutex> guard(impl_->metadata_mutex);

  if (impl_->lifecycle_state_map.find(workflow_id) != impl_->lifecycle_state_map.end()) {
    return Workflow::Status::INVALID;
  }

  for (const auto & prereq : prerequisites) {
    if (impl_->lifecycle_state_map.find(prereq) == impl_->lifecycle_state_map.end()) {
      std::cerr << "[Scheduler] Rejecting workflow '" << workflow_id
                << "' due to unknown prerequisite '" << prereq << "'" << std::endl;
      return Workflow::Status::INVALID;
    }
  }

  if (impl_->would_create_cycle(workflow_id, prerequisites)) {
    std::cerr << "[Scheduler] Rejecting cyclic workflow DAG involving '" << workflow_id << "'"
              << std::endl;
    return Workflow::Status::INVALID;
  }

  auto workflow_impl = std::make_shared<WorkflowImplT>(std::move(workflow));
  impl_->waiting_workflow_map[workflow_id] = workflow_impl;
  impl_->queued_task_map[workflow_id] = workflow_impl->future;
  impl_->allow_failed_prerequisite[workflow_id] = allow_if_prerequisite_failed;
  impl_->prerequisite_count[workflow_id] = prerequisites.size();
  impl_->lifecycle_state_map[workflow_id] =
    prerequisites.empty() ? Impl::LifecycleState::RUNNABLE : Impl::LifecycleState::QUEUED;

  for (const auto & prereq : prerequisites) {
    impl_->reverse_dependency_map[prereq].push_back(workflow_id);
  }

  if (prerequisites.empty()) {
    impl_->runnable_queue.push_back(workflow_id);
  }

  impl_->try_schedule_runnable();

  if (impl_->lifecycle_state_map[workflow_id] == Impl::LifecycleState::ONGOING) {
    return Workflow::Status::ONGOING;
  }
  if (impl_->lifecycle_state_map[workflow_id] == Impl::LifecycleState::RUNNABLE) {
    return Workflow::Status::RUNNABLE;
  }
  return Workflow::Status::QUEUED;
}

Workflow::Status Scheduler::add_workflows(
  const std::unordered_map<std::string, WorkflowT> & workflows,
  const std::unordered_map<std::string, std::vector<std::string>> & prerequisites,
  bool allow_if_prerequisite_failed)
{
  std::lock_guard<std::mutex> guard(impl_->metadata_mutex);

  for (const auto & kv : workflows) {
    if (impl_->lifecycle_state_map.find(kv.first) != impl_->lifecycle_state_map.end()) {
      return Workflow::Status::INVALID;
    }
  }

  std::unordered_map<std::string, std::vector<std::string>> local_adj;
  std::unordered_map<std::string, size_t> local_indegree;
  for (const auto & kv : workflows) {
    local_indegree[kv.first] = 0;
  }

  for (const auto & kv : workflows) {
    const auto it = prerequisites.find(kv.first);
    if (it == prerequisites.end()) {
      continue;
    }
    for (const auto & prereq : it->second) {
      if (workflows.find(prereq) == workflows.end() &&
        impl_->lifecycle_state_map.find(prereq) == impl_->lifecycle_state_map.end())
      {
        std::cerr << "[Scheduler] Rejecting workflow batch due to unknown prerequisite '" << prereq
                  << "'" << std::endl;
        return Workflow::Status::INVALID;
      }
      if (workflows.find(prereq) != workflows.end()) {
        local_adj[prereq].push_back(kv.first);
        local_indegree[kv.first]++;
      }
    }
  }

  std::deque<std::string> zero;
  for (const auto & kv : local_indegree) {
    if (kv.second == 0) {
      zero.push_back(kv.first);
    }
  }

  size_t visited = 0;
  while (!zero.empty()) {
    auto n = std::move(zero.front());
    zero.pop_front();
    visited++;
    for (const auto & dep : local_adj[n]) {
      if (--local_indegree[dep] == 0) {
        zero.push_back(dep);
      }
    }
  }

  if (visited != workflows.size()) {
    std::cerr << "[Scheduler] Rejecting cyclic workflow DAG submission" << std::endl;
    return Workflow::Status::INVALID;
  }

  Workflow::Status last_status = Workflow::Status::INVALID;
  for (const auto & kv : workflows) {
    auto it = prerequisites.find(kv.first);
    const std::vector<std::string> deps = (it == prerequisites.end()) ? std::vector<std::string>{} : it->second;
    if (impl_->would_create_cycle(kv.first, deps)) {
      std::cerr << "[Scheduler] Rejecting cyclic workflow DAG involving '" << kv.first << "'"
                << std::endl;
      return Workflow::Status::INVALID;
    }

    auto workflow_impl = std::make_shared<WorkflowImplT>(kv.second);
    impl_->waiting_workflow_map[kv.first] = workflow_impl;
    impl_->queued_task_map[kv.first] = workflow_impl->future;
    impl_->allow_failed_prerequisite[kv.first] = allow_if_prerequisite_failed;
    impl_->prerequisite_count[kv.first] = deps.size();
    impl_->lifecycle_state_map[kv.first] =
      deps.empty() ? Impl::LifecycleState::RUNNABLE : Impl::LifecycleState::QUEUED;

    for (const auto & prereq : deps) {
      impl_->reverse_dependency_map[prereq].push_back(kv.first);
    }

    if (deps.empty()) {
      impl_->runnable_queue.push_back(kv.first);
    }

    last_status = deps.empty() ? Workflow::Status::RUNNABLE : Workflow::Status::QUEUED;
  }

  impl_->try_schedule_runnable();
  return last_status;
}

Workflow::Status Scheduler::cancel_workflow(const std::string & workflow_id)
{
  std::lock_guard<std::mutex> guard(impl_->metadata_mutex);
  if (impl_->ongoing_task_map.find(workflow_id) != impl_->ongoing_task_map.end()) {
    return Workflow::Status::INVALID;
  }

  if (impl_->queued_task_map.find(workflow_id) == impl_->queued_task_map.end()) {
    return Workflow::Status::INVALID;
  }

  impl_->cancel_workflow_and_dependents(workflow_id, true);
  return Workflow::Status::CANCELLED;
}

void Scheduler::wait_till_all_complete() const
{
  std::vector<std::shared_future<void>> worker_futures;
  {
    std::lock_guard<std::mutex> guard(impl_->metadata_mutex);
    worker_futures.reserve(impl_->workers.size());
    for (const auto & worker : impl_->workers) {
      if (worker.execution_future.valid()) {
        worker_futures.emplace_back(worker.execution_future);
      }
    }
  }

  for (auto & worker_future : worker_futures) {
    worker_future.wait();
  }
}

Workflow::Status Scheduler::wait_till_complete(
  const std::string & workflow_id,
  result_t & result) const
{
  std::shared_future<result_t> future;
  {
    std::lock_guard<std::mutex> guard(impl_->metadata_mutex);

    auto finished_it = impl_->finished_task_map.find(workflow_id);
    if (finished_it != impl_->finished_task_map.end()) {
      result = finished_it->second;
      return impl_->terminal_status_from_id(workflow_id);
    }

    auto ongoing_it = impl_->ongoing_task_map.find(workflow_id);
    if (ongoing_it != impl_->ongoing_task_map.end()) {
      future = ongoing_it->second;
    } else {
      auto queued_it = impl_->queued_task_map.find(workflow_id);
      if (queued_it == impl_->queued_task_map.end()) {
        return Workflow::Status::INVALID;
      }
      future = queued_it->second;
    }
  }

  future.wait();

  std::lock_guard<std::mutex> guard(impl_->metadata_mutex);
  result = impl_->finished_task_map.at(workflow_id);
  return impl_->terminal_status_from_id(workflow_id);
}

Workflow::Status Scheduler::get_status(const std::string & workflow_id) const
{
  std::lock_guard<std::mutex> guard(impl_->metadata_mutex);

  const auto it = impl_->lifecycle_state_map.find(workflow_id);
  if (it == impl_->lifecycle_state_map.end()) {
    return Workflow::Status::INVALID;
  }

  switch (it->second) {
    case Impl::LifecycleState::QUEUED:
      return Workflow::Status::QUEUED;
    case Impl::LifecycleState::RUNNABLE:
      return Workflow::Status::RUNNABLE;
    case Impl::LifecycleState::ONGOING:
      return Workflow::Status::ONGOING;
    case Impl::LifecycleState::FINISHED:
      return Workflow::Status::COMPLETED;
    case Impl::LifecycleState::FAILED:
      return Workflow::Status::FAILED;
    case Impl::LifecycleState::CANCELLED:
      return Workflow::Status::CANCELLED;
    default:
      return Workflow::Status::INVALID;
  }
}

result_t Scheduler::get_result(const std::string & workflow_id) const
{
  std::lock_guard<std::mutex> guard(impl_->metadata_mutex);
  return impl_->finished_task_map.at(workflow_id);
}

}  // namespace core
}  // namespace grasp_execution
