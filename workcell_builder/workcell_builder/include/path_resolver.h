// Copyright 2024 Advanced Remanufacturing and Technology Centre
// Copyright 2024 ROS-Industrial Consortium Asia Pacific Team
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

#ifndef EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__INCLUDE__PATH_RESOLVER_H_
#define EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__INCLUDE__PATH_RESOLVER_H_

#include <boost/filesystem.hpp>

namespace workcell_builder
{
class PathResolver
{
public:
  static PathResolver & instance()
  {
    static PathResolver resolver;
    return resolver;
  }

  void set_workcell_root(const boost::filesystem::path & root)
  {
    workcell_root_ = boost::filesystem::absolute(root);
    scenes_root_ = workcell_root_ / "scenes";
  }

  void set_scenes_root(const boost::filesystem::path & root)
  {
    scenes_root_ = boost::filesystem::absolute(root);
    workcell_root_ = scenes_root_.parent_path();
  }

  const boost::filesystem::path & workcell_root() const
  {
    return workcell_root_;
  }

  const boost::filesystem::path & scenes_root() const
  {
    return scenes_root_;
  }

private:
  PathResolver() = default;
  boost::filesystem::path workcell_root_;
  boost::filesystem::path scenes_root_;
};
}  // namespace workcell_builder

#endif  // EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__INCLUDE__PATH_RESOLVER_H_
