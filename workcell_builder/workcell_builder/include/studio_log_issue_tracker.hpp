#ifndef WORKCELL_BUILDER__STUDIO_LOG_ISSUE_TRACKER_HPP_
#define WORKCELL_BUILDER__STUDIO_LOG_ISSUE_TRACKER_HPP_

#include <string>
#include <unordered_set>

namespace workcell_builder
{

enum class StudioLogSeverity { Info, Warning, Error, Blocker };

class StudioLogIssueTracker
{
public:
  void set_scope(const std::string & scope)
  {
    if (scope == scope_) return;
    scope_ = scope;
    issue_keys_.clear();
    warning_count_ = 0;
    error_count_ = 0;
  }

  bool report(StudioLogSeverity severity, const std::string & issue_key)
  {
    if (severity == StudioLogSeverity::Info) return false;
    const std::string scoped_key = scope_ + "|" + issue_key;
    if (!issue_keys_.insert(scoped_key).second) return false;
    if (severity == StudioLogSeverity::Warning) ++warning_count_;
    else ++error_count_;
    return true;
  }

  int warning_count() const { return warning_count_; }
  int error_count() const { return error_count_; }

private:
  std::string scope_;
  std::unordered_set<std::string> issue_keys_;
  int warning_count_{0};
  int error_count_{0};
};

}  // namespace workcell_builder

#endif  // WORKCELL_BUILDER__STUDIO_LOG_ISSUE_TRACKER_HPP_
