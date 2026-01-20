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

#ifndef WORKCELL_BUILDER__PATH_RESOLVER_H_
#define WORKCELL_BUILDER__PATH_RESOLVER_H_

#include <boost/filesystem.hpp>

class PathResolver
{
public:
  static boost::filesystem::path assets_root();
  static boost::filesystem::path templates_root();
  static boost::filesystem::path workspace_root();
  static boost::filesystem::path scenes_root();

  static void set_workspace_root_override(const boost::filesystem::path & workspace_root);
  static void set_scenes_root_override(const boost::filesystem::path & scenes_root);
};

#endif  // WORKCELL_BUILDER__PATH_RESOLVER_H_
