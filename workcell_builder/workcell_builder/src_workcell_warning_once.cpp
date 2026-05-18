#include "workcell_warning_once.hpp"

#include <boost/system/error_code.hpp>

#include <iostream>
#include <mutex>
#include <set>

namespace workcell_builder {

bool log_warning_once_per_context_path_reason(
  const std::string & context,
  const boost::filesystem::path & canonical_path,
  const std::string & reason)
{
  static std::mutex mutex;
  static std::set<std::string> seen;

  boost::system::error_code ec;
  const boost::filesystem::path normalized_path =
    boost::filesystem::weakly_canonical(canonical_path, ec);
  const std::string path_key = (ec ? canonical_path.lexically_normal() : normalized_path).string();
  const std::string key = context + "\n" + path_key + "\n" + reason;

  std::lock_guard<std::mutex> lock(mutex);
  if (!seen.insert(key).second) {
    return false;
  }

  std::cerr << "[workcell_builder] warning context=" << context
            << " path=" << path_key
            << " reason=" << reason << std::endl;
  return true;
}

}  // namespace workcell_builder
