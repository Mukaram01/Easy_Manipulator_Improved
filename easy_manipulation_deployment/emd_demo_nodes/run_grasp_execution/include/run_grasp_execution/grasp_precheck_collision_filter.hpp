// Copyright 2026 Mukaram
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

#ifndef RUN_GRASP_EXECUTION__GRASP_PRECHECK_COLLISION_FILTER_HPP_
#define RUN_GRASP_EXECUTION__GRASP_PRECHECK_COLLISION_FILTER_HPP_

#include <algorithm>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace run_grasp_execution
{

struct CollisionFilterResult
{
  std::vector<std::string> allowed_pairs;
  std::vector<std::string> invalid_pairs;
};

inline std::string pair_to_string(const std::string & first, const std::string & second)
{
  return first + "<->" + second;
}

inline bool pair_contains(
  const std::string & first,
  const std::string & second,
  const std::set<std::string> & ids)
{
  return ids.find(first) != ids.end() || ids.find(second) != ids.end();
}

inline CollisionFilterResult filter_grasp_precheck_collision_pairs(
  const std::vector<std::pair<std::string, std::string>> & collision_pairs,
  const std::set<std::string> & allowed_touch_links,
  const std::set<std::string> & allowed_collision_ids)
{
  CollisionFilterResult result;
  for (const auto & pair : collision_pairs) {
    const bool first_is_touch_link =
      allowed_touch_links.find(pair.first) != allowed_touch_links.end();
    const bool second_is_touch_link =
      allowed_touch_links.find(pair.second) != allowed_touch_links.end();
    const bool first_is_allowed_id =
      allowed_collision_ids.find(pair.first) != allowed_collision_ids.end();
    const bool second_is_allowed_id =
      allowed_collision_ids.find(pair.second) != allowed_collision_ids.end();

    const bool is_allowed_pair =
      (first_is_touch_link && second_is_allowed_id) ||
      (second_is_touch_link && first_is_allowed_id);

    if (is_allowed_pair) {
      result.allowed_pairs.push_back(pair_to_string(pair.first, pair.second));
    } else {
      result.invalid_pairs.push_back(pair_to_string(pair.first, pair.second));
    }
  }

  return result;
}

}  // namespace run_grasp_execution

#endif  // RUN_GRASP_EXECUTION__GRASP_PRECHECK_COLLISION_FILTER_HPP_
