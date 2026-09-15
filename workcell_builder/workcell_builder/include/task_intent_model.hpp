#pragma once

#include <string>

namespace workcell_builder
{

// Canonical semantic JSON is UTF-8, recursively key-sorted, arrays preserved,
// and all finite numbers rendered as decimal strings (12 places max). The
// SHA-256 of those bytes is the v2 intent hash shared with Python.
std::string canonical_task_intent_json(const std::string & yaml_text);
std::string canonical_task_intent_sha256(const std::string & yaml_text);

}  // namespace workcell_builder
