#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <fstream>

#include "workcell_studio_layout_editor.hpp"

namespace fs = boost::filesystem;

TEST(WorkcellStudioLayoutEditor, WritesRequiredMetadataFieldsForBackwardCompatibility)
{
  const fs::path root = fs::temp_directory_path() / "wc_layout_editor_defaults";
  fs::remove_all(root);

  const auto r = workcell_builder::persist_workcell_studio_layout(root, "items: []\n");
  EXPECT_FALSE(r.warnings.empty());

  const fs::path out = root / "layout" / "workcell_studio_layout.yaml";
  ASSERT_TRUE(fs::exists(out));

  std::ifstream in(out.string());
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  EXPECT_NE(text.find("schema_version: workcell_studio_layout/v1"), std::string::npos);
  EXPECT_NE(text.find("saved_at_utc: 1970-01-01T00:00:00Z"), std::string::npos);
  EXPECT_NE(text.find("layout_changed_since_acceptance: true"), std::string::npos);
  EXPECT_NE(text.find("layout_stale_label: Layout changed since last acceptance"), std::string::npos);
  EXPECT_NE(text.find("layout_stale_action: Run acceptance again"), std::string::npos);
}

TEST(WorkcellStudioLayoutEditor, PreservesProvidedSchemaVersionAndSavedAt)
{
  const fs::path root = fs::temp_directory_path() / "wc_layout_editor_preserve";
  fs::remove_all(root);

  const std::string yaml = "schema_version: workcell_studio_layout/v9\nsaved_at_utc: 2026-05-16T00:00:00Z\nitems: []\n";
  workcell_builder::persist_workcell_studio_layout(root, yaml);

  const fs::path out = root / "layout" / "workcell_studio_layout.yaml";
  std::ifstream in(out.string());
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  EXPECT_NE(text.find("schema_version: workcell_studio_layout/v9"), std::string::npos);
  EXPECT_NE(text.find("saved_at_utc: 2026-05-16T00:00:00Z"), std::string::npos);
}
