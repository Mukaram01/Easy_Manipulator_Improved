#ifndef WORKCELL_BUILDER__PREVIEW_PROCESS_STATE_HPP_
#define WORKCELL_BUILDER__PREVIEW_PROCESS_STATE_HPP_

namespace workcell_builder {

enum class PreviewProcessOutcome { ExpectedStop, BuildFailure, LaunchFailure, UnexpectedExit };

inline bool preview_process_error_is_expected(bool user_stop_requested, bool preview_stopping)
{
  return user_stop_requested || preview_stopping;
}

inline PreviewProcessOutcome classify_preview_process_failure(
  bool user_stop_requested, bool preview_stopping, bool build_stage, bool launch_stage)
{
  if (preview_process_error_is_expected(user_stop_requested, preview_stopping)) {
    return PreviewProcessOutcome::ExpectedStop;
  }
  if (build_stage) return PreviewProcessOutcome::BuildFailure;
  if (launch_stage) return PreviewProcessOutcome::LaunchFailure;
  return PreviewProcessOutcome::UnexpectedExit;
}

}  // namespace workcell_builder

#endif
