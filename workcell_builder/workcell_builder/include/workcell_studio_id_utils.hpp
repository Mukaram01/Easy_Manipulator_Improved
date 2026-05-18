#pragma once

#include <boost/filesystem.hpp>
#include <set>
#include <string>

namespace workcell_builder {

std::string workcell_studio_id_prefix_for_type(const std::string & type_or_category);
bool workcell_studio_is_valid_id(const std::string & id);
std::set<std::string> workcell_studio_collect_layout_ids(const boost::filesystem::path & layout_path);
std::string workcell_studio_next_id(const std::string & type_or_category, const std::set<std::string> & reserved_ids);

}  // namespace workcell_builder
