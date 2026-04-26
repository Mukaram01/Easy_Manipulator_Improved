#ifndef RUN_GRASP_EXECUTION__HOME_RETURN_UTILS_HPP_
#define RUN_GRASP_EXECUTION__HOME_RETURN_UTILS_HPP_

#include <string>
#include <vector>

namespace run_grasp_execution
{

struct HomeReturnStepResult
{
  bool success{false};
  bool executed{false};
  std::string failure_reason;
};

inline std::string default_home_return_failure_reason(
  const std::string & step_name,
  int attempt,
  int max_attempts)
{
  return step_name + " planning failed on attempt " + std::to_string(attempt) +
         "/" + std::to_string(max_attempts) +
         " (no valid plan returned by MoveIt).";
}

inline bool safe_intermediate_enabled(
  bool use_safe_intermediate,
  const std::vector<double> & safe_joint_state)
{
  return use_safe_intermediate && !safe_joint_state.empty();
}

}  // namespace run_grasp_execution

#endif  // RUN_GRASP_EXECUTION__HOME_RETURN_UTILS_HPP_
