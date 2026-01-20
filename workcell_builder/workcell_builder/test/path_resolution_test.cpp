#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
#include <cstdlib>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "file_functions.h"

namespace fs = boost::filesystem;

namespace {

class ScopedEnvVar
{
public:
  ScopedEnvVar(const std::string & name, const std::string & value, bool set)
  : name_(name)
  {
    const char * existing = std::getenv(name_.c_str());
    if (existing != nullptr) {
      had_previous_ = true;
      previous_value_ = existing;
    }
    if (set) {
      setenv(name_.c_str(), value.c_str(), 1);
    } else {
      unsetenv(name_.c_str());
    }
  }

  ~ScopedEnvVar()
  {
    if (had_previous_) {
      setenv(name_.c_str(), previous_value_.c_str(), 1);
    } else {
      unsetenv(name_.c_str());
    }
  }

private:
  std::string name_;
  std::string previous_value_;
  bool had_previous_{false};
};

}  // namespace

TEST(WorkcellBuilderPathResolution, DefaultsToPackageShare)
{
  ScopedEnvVar guard("WORKCELL_BUILDER_ROOT", "", false);
  const fs::path expected =
    ament_index_cpp::get_package_share_directory("workcell_builder");
  const fs::path resolved = resolve_workcell_builder_root();
  EXPECT_EQ(resolved, expected);
}

TEST(WorkcellBuilderPathResolution, UsesEnvironmentOverride)
{
  const fs::path temp_root = fs::temp_directory_path() / fs::unique_path("wcb-env-%%%%-%%%%");
  ScopedEnvVar guard("WORKCELL_BUILDER_ROOT", temp_root.string(), true);
  const fs::path resolved = resolve_workcell_builder_root();
  EXPECT_EQ(resolved, temp_root);
}

TEST(WorkcellBuilderPathResolution, UsesCliOverride)
{
  const fs::path env_root = fs::temp_directory_path() / fs::unique_path("wcb-env-%%%%-%%%%");
  const fs::path cli_root = fs::temp_directory_path() / fs::unique_path("wcb-cli-%%%%-%%%%");
  ScopedEnvVar guard("WORKCELL_BUILDER_ROOT", env_root.string(), true);

  const std::vector<std::string> args = {"workcell_builder",
    "--workcell-builder-root", cli_root.string()};
  const std::string parsed = parse_workcell_builder_root_flag(args);
  const fs::path resolved = resolve_workcell_builder_root(parsed);
  EXPECT_EQ(resolved, cli_root);
}
