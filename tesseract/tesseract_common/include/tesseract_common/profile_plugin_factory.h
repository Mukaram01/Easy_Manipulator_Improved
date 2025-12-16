#ifndef TESSERACT_COMMON_PROFILE_PLUGIN_FACTORY_H
#define TESSERACT_COMMON_PROFILE_PLUGIN_FACTORY_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_common
{
/**
 * @brief Minimal placeholder for loading profile plugins from configuration
 *
 * The upstream tesseract_common package provides a richer implementation for
 * discovering profile plugins. The code in this workspace only requires the
 * interface to exist, so this lightweight version allows callers to compile
 * without altering their behavior.
 */
class ProfilePluginFactory
{
public:
  ProfilePluginFactory() = default;

  template <typename ProfileType>
  std::shared_ptr<ProfileType> create(const std::string& /*name*/) const
  {
    return nullptr;
  }

  template <typename ProfileType>
  bool hasPlugin(const std::string& /*name*/) const { return false; }
};

}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_PROFILE_PLUGIN_FACTORY_H
