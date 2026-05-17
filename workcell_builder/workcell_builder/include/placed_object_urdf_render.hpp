#pragma once

#include <string>

#include "object_placement_model.hpp"

namespace workcell_builder
{

std::string render_placed_object_urdf_snippet(const PlacedObject & object, const std::string & link_name);

}  // namespace workcell_builder
