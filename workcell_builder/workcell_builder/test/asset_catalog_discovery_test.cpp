#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
#include <algorithm>
#include <fstream>

#include "gui/asset_catalog_discovery.h"
#include "include/asset_catalog_model.h"

namespace fs = boost::filesystem;

namespace {

fs::path make_tmp_dir(const std::string & name)
{
  const fs::path base = fs::temp_directory_path() / ("workcell_asset_catalog_test_" + name);
  boost::system::error_code ec;
  fs::remove_all(base, ec);
  fs::create_directories(base);
  return base;
}

void write_text(const fs::path & path, const std::string & text)
{
  fs::create_directories(path.parent_path());
  std::ofstream out(path.string());
  out << text;
}

}  // namespace

TEST(AssetCatalogDiscovery, HandlesManifestScalarMissingAndMalformedWithoutThrow)
{
  const fs::path repo = make_tmp_dir("repo");
  const fs::path ws = make_tmp_dir("ws");

  const fs::path env_objects = repo / "assets" / "environment_objects" / "demo_asset";
  fs::create_directories(env_objects);
  write_text(env_objects / "mesh.stl", "solid mesh\nendsolid\n");

  write_text(env_objects / "asset_manifest.yaml", "assets:\n  - scalar-entry\n  - id: ok_entry\n    path: .\n");

  write_text(repo / "catalog" / "workcell_studio_demos.yaml", "templates: [oops\n");

  EXPECT_NO_THROW({
    const auto entries = workcell_builder::discover_asset_catalog_entries(repo, ws);
    ASSERT_FALSE(entries.empty());
    bool found_ok = false;
    for (const auto & e : entries) {
      if (e.asset_id == "ok_entry") {
        found_ok = true;
        EXPECT_EQ(e.availability, "ready");
      }
    }
    EXPECT_TRUE(found_ok);
  });
}

TEST(AssetCatalogDiscovery, ValidTemplatePathDoesNotFallbackToPrimitive)
{
  const fs::path repo = make_tmp_dir("repo_templates");
  const fs::path ws = make_tmp_dir("ws_templates");

  write_text(repo / "assets" / "environment_objects" / "bin_a" / "mesh.obj", "# mesh\n");
  write_text(repo / "catalog" / "workcell_studio_demos.yaml",
    "templates:\n"
    "  - name: demo\n"
    "    asset_references:\n"
    "      - id: bin_ref\n"
    "        path: assets/environment_objects/bin_a/mesh.obj\n");

  const auto entries = workcell_builder::discover_asset_catalog_entries(repo, ws);
  bool found = false;
  for (const auto & e : entries) {
    if (e.asset_id == "bin_ref") {
      found = true;
      EXPECT_EQ(e.source_kind, "scene_template");
      EXPECT_EQ(e.availability, "ready");
      EXPECT_TRUE(e.reason.empty());
    }
  }
  EXPECT_TRUE(found);
}

TEST(AssetCatalogDiscovery, SceneImportedManifestUsesStableIdResolvedPathAndScale)
{
  const fs::path repo = make_tmp_dir("scene_repo");
  const fs::path ws = make_tmp_dir("scene_ws");
  const fs::path imported = repo / "scenes" / "ur5_2f_test" / "assets" / "imported";
  write_text(imported / "part.stl", "solid part\nendsolid\n");
  write_text(imported / "asset_manifest.yaml",
    "assets:\n"
    "  - id: imported_test_asset\n"
    "    display_name: Imported Part\n"
    "    path: part.stl\n"
    "    category: Imported\n"
    "    scale: 0.001\n");

  const auto entries = workcell_builder::discover_asset_catalog_entries(repo, ws, imported);
  const auto match = std::find_if(entries.begin(), entries.end(), [](const auto & entry) {
    return entry.asset_id == "imported_test_asset";
  });
  ASSERT_NE(match, entries.end());
  EXPECT_EQ(match->source_path, fs::canonical(imported / "part.stl").string());
  EXPECT_EQ(match->category, "Imported");
  EXPECT_DOUBLE_EQ(match->scale, 0.001);

  const AssetCatalogModel model = discover_asset_catalog(ws.string(), repo.string(), imported.string());
  const auto model_match = std::find_if(model.assets.begin(), model.assets.end(), [](const auto & entry) {
    return entry.id == "imported_test_asset";
  });
  ASSERT_NE(model_match, model.assets.end());
  EXPECT_EQ(model_match->path, fs::canonical(imported / "part.stl").string());
  EXPECT_DOUBLE_EQ(model_match->scale, 0.001);
  EXPECT_TRUE(model_match->can_add_to_scene);
}
