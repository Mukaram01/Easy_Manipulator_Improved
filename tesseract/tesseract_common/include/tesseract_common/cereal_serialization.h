#ifndef TESSERACT_COMMON_CEREAL_SERIALIZATION_H
#define TESSERACT_COMMON_CEREAL_SERIALIZATION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <cereal/types/memory.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/variant.hpp>
#include <cereal/types/vector.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/allowed_collision_matrix.h>
#include <tesseract_common/any_poly.h>
#include <tesseract_common/joint_state.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/profile_dictionary.h>
#include <tesseract_common/resource_locator.h>

namespace tesseract_common
{
/**
 * @brief Placeholder aggregation header for cereal serialization support
 *
 * Upstream distributions expose a richer set of cereal helpers. The code in
 * this repository only relies on the header being available, so providing this
 * light wrapper keeps dependent packages building without changing behavior.
 */
struct CerealSerializationPlaceholder
{
};

}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_CEREAL_SERIALIZATION_H
