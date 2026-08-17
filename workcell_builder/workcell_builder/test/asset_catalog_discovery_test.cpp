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

TEST(AssetLibraryModel, SearchCategoriesProvenanceAndCompositionUseCatalogMetadata)
{
  AssetCatalogEntry robot;
  robot.id = "ur_description";
  robot.display_name = "Universal Robot UR5";
  robot.category = "robot";
  robot.path = "/opt/ros/share/ur_description/urdf/ur.urdf.xacro";
  robot.source = "canonical_asset";
  robot.role_hints = {"robot_base"};
  robot.can_add_to_scene = true;

  AssetCatalogEntry camera;
  camera.id = "d435_visual";
  camera.display_name = "RealSense D435i";
  camera.category = "camera_sensor";
  camera.path = "/catalog/realsense_description/meshes/d435.dae";
  camera.source = "canonical_asset";
  camera.role_hints = {"camera"};

  AssetCatalogEntry imported;
  imported.id = "imported_2068_001_24";
  imported.display_name = "2068_001_24";
  imported.category = "Imported";
  imported.path = "/scene/assets/imported/2068_001_24.stl";
  imported.source = "scene_imported_asset";
  imported.role_hints = {"object"};

  const std::vector<AssetCatalogEntry> catalog{robot, camera, imported};
  EXPECT_EQ(normalize_asset_category(robot), "robot");
  EXPECT_EQ(normalize_asset_category(camera), "camera");
  EXPECT_EQ(normalize_asset_category(imported), "object");
  EXPECT_EQ(asset_provenance(robot), "built_in");
  EXPECT_EQ(asset_provenance(imported), "imported");

  EXPECT_EQ(filter_asset_catalog(catalog, "UNIVERSAL", "all"), (std::vector<size_t>{0}));
  EXPECT_EQ(filter_asset_catalog(catalog, "ur_description", "robot"), (std::vector<size_t>{0}));
  EXPECT_EQ(filter_asset_catalog(catalog, "real", "camera"), (std::vector<size_t>{1}));
  EXPECT_EQ(filter_asset_catalog(catalog, "d435.dae", "camera"), (std::vector<size_t>{1}));
  EXPECT_TRUE(filter_asset_catalog(catalog, "real", "robot").empty());
  EXPECT_EQ(filter_asset_catalog(catalog, "", "imported"), (std::vector<size_t>{2}));
}

TEST(AssetLibraryModel, MissingOptionalMetadataAndRefreshRemainStable)
{
  AssetCatalogEntry minimal;
  minimal.id = "minimal";
  minimal.path = "part.stl";
  minimal.source = "scene_imported_asset";
  EXPECT_NO_THROW({
    EXPECT_EQ(normalize_asset_category(minimal), "object");
    EXPECT_EQ(asset_package_hint(minimal), "part.stl");
    EXPECT_TRUE(asset_library_matches(minimal, "PART.STL", "imported"));
  });

  const std::vector<AssetCatalogEntry> catalog{minimal};
  EXPECT_EQ(filter_asset_catalog(catalog, "", "all").size(), 1u);
  EXPECT_EQ(filter_asset_catalog(catalog, "", "all").size(), 1u);
}

TEST(AssetLibraryModel, DiscoveryRefreshDoesNotDuplicateImportedManifestEntry)
{
  const fs::path repo = make_tmp_dir("refresh_repo");
  const fs::path ws = make_tmp_dir("refresh_ws");
  const fs::path imported = repo / "scene" / "assets" / "imported";
  write_text(imported / "part.stl", "solid part\nendsolid\n");
  write_text(imported / "asset_manifest.yaml",
    "assets:\n"
    "  - id: imported_part\n    display_name: Imported Part\n    path: part.stl\n    category: Imported\n"
    "  - id: imported_part\n    display_name: Imported Part\n    path: part.stl\n    category: Imported\n");

  const auto first = discover_asset_catalog(ws.string(), repo.string(), imported.string());
  const auto second = discover_asset_catalog(ws.string(), repo.string(), imported.string());
  const auto count_id = [](const AssetCatalogModel & model) {
    return std::count_if(model.assets.begin(), model.assets.end(), [](const auto & entry) {
      return entry.id == "imported_part";
    });
  };
  EXPECT_EQ(count_id(first), 1);
  EXPECT_EQ(count_id(second), 1);
}
