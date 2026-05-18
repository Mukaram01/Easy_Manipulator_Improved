#pragma once

#include <boost/filesystem.hpp>
#include <string>

namespace workcell_builder {

bool log_warning_once_per_context_path_reason(
  const std::string & context,
  const boost::filesystem::path & canonical_path,
  const std::string & reason);

}  // namespace workcell_builder
