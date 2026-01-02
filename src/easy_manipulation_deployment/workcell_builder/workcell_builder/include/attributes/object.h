// Copyright 2020 Advanced Remanufacturing and Technology Centre
// Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
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


#ifndef ATTRIBUTES__OBJECT_H_
#define ATTRIBUTES__OBJECT_H_

#include <algorithm>
#include <string>
#include <vector>

#include "attributes/external_joint.h"
#include "attributes/joint.h"
#include "attributes/link.h"


class Object
{
public:
  std::string name;
  std::vector < Link > link_vector;
  std::vector < Joint > joint_vector;
  ExternalJoint ext_joint;

  /// Remove the link at `link_index` and any joints that reference it.
  ///
  /// @param link_index Index of the link to remove from `link_vector`.
  /// @return Indices of the joints removed from `joint_vector` in ascending order.
  std::vector<size_t> remove_link_and_joints(size_t link_index)
  {
    std::vector<size_t> removed_indices;
    if (link_index >= link_vector.size()) {
      return removed_indices;
    }

    std::string link_name = link_vector[link_index].name;
    link_vector.erase(link_vector.begin() + link_index);

    for (size_t i = joint_vector.size(); i-- > 0;) {
      if (joint_vector[i].parent_link.name == link_name ||
        joint_vector[i].child_link.name == link_name)
      {
        removed_indices.push_back(i);
        joint_vector.erase(joint_vector.begin() + static_cast<long>(i));
      }
    }
    std::reverse(removed_indices.begin(), removed_indices.end());

    // Adjust external joint child link reference if necessary
    if (ext_joint.child_link_pos == static_cast<int>(link_index)) {
      ext_joint.child_link_pos = 0;
    } else if (ext_joint.child_link_pos > static_cast<int>(link_index)) {
      ext_joint.child_link_pos--;
    } else if (ext_joint.child_link_pos >= static_cast<int>(link_vector.size())) {
      ext_joint.child_link_pos = 0;
    }

    return removed_indices;
  }
};

#endif  // ATTRIBUTES__OBJECT_H_
