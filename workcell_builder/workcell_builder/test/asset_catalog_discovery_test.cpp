#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
#include <algorithm>
#include <fstream>
#include <utility>

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

TEST(AssetLibraryPresentation, NamesCategoriesFormatsAndIdentityRemainSeparate)
{
  AssetCatalogEntry explicit_name;
  explicit_name.id = "robotiq_2f_85_description/internal";
  explicit_name.display_name = "Robotiq 2F-85";
  explicit_name.category = "end_effector";
  explicit_name.path = "/repo/robotiq_2f_85_description/mesh.dae";
  EXPECT_EQ(asset_display_name(explicit_name), "Robotiq 2F-85");
  EXPECT_EQ(asset_category_label(explicit_name), "Gripper");
  EXPECT_EQ(asset_format_label(explicit_name), "COLLADA");
  EXPECT_EQ(explicit_name.id, "robotiq_2f_85_description/internal");
  EXPECT_TRUE(asset_library_matches(explicit_name, "2f_85", "all"));
  EXPECT_TRUE(asset_library_matches(explicit_name, "Robotiq", "all"));

  AssetCatalogEntry package_fallback;
  package_fallback.id = "sorting_bin_description";
  EXPECT_EQ(asset_display_name(package_fallback), "Sorting Bin");

  AssetCatalogEntry filename_fallback;
  filename_fallback.id = "canonical-import-id";
  filename_fallback.path = "/scene/imported/2068_001_24.stl";
  EXPECT_EQ(asset_display_name(filename_fallback), "2068 001 24");
  EXPECT_EQ(asset_format_label(filename_fallback), "STL");
  EXPECT_EQ(filename_fallback.id, "canonical-import-id");

  for (const auto & expected : std::vector<std::pair<std::string, std::string>>{
      {"part.obj", "OBJ"}, {"part.glb", "GLB"}, {"part.gltf", "glTF"}})
  {
    filename_fallback.path = expected.first;
    EXPECT_EQ(asset_format_label(filename_fallback), expected.second);
  }

  AssetCatalogEntry missing;
  EXPECT_EQ(asset_display_name(missing), "—");
  EXPECT_EQ(asset_category_label(missing), "Other");
  EXPECT_TRUE(asset_format_label(missing).empty());
  EXPECT_TRUE(asset_source_hint(missing).empty());
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

TEST(AssetLibraryModel, RecentIdsAreDeduplicatedOrderedAndBoundedAfterCommit)
{
  std::vector<std::string> recent;
  recent = record_recent_asset_id(recent, "asset_a", 3);
  recent = record_recent_asset_id(recent, "asset_b", 3);
  recent = record_recent_asset_id(recent, "asset_a", 3);
  EXPECT_EQ(recent, (std::vector<std::string>{"asset_a", "asset_b"}));
  recent = record_recent_asset_id(recent, "asset_c", 3);
  recent = record_recent_asset_id(recent, "asset_d", 3);
  EXPECT_EQ(recent, (std::vector<std::string>{"asset_d", "asset_c", "asset_a"}));
}

TEST(AssetLibraryModel, RecentFilterPreservesUseOrderComposesWithSearchAndSkipsStaleIds)
{
  AssetCatalogEntry robot;
  robot.id = "robot";
  robot.display_name = "Robotiq 2F-85";
  AssetCatalogEntry camera;
  camera.id = "camera";
  camera.display_name = "RealSense Camera";
  AssetCatalogEntry table;
  table.id = "table";
  table.display_name = "Workbench";
  const std::vector<AssetCatalogEntry> catalog{robot, camera, table};
  const std::vector<std::string> recent{"camera", "stale_removed_asset", "robot"};

  EXPECT_EQ(filter_recent_asset_catalog(catalog, recent, ""), (std::vector<size_t>{1, 0}));
  EXPECT_EQ(filter_recent_asset_catalog(catalog, recent, "rob"), (std::vector<size_t>{0}));
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

TEST(AssetLibraryModel, CameraStandsAreFixturesNotSecondCameras)
{
  const fs::path repo = make_tmp_dir("repo_camera_mounts");
  const fs::path ws = make_tmp_dir("ws_camera_mounts");
  write_text(repo / "assets" / "environment" / "camera_stand_description" / "meshes" / "camera_stand.stl", "solid s\nendsolid s\n");
  write_text(repo / "assets" / "environment" / "realsense2_description" / "meshes" / "d435.dae", "<COLLADA/>\n");
  const AssetCatalogModel model = discover_asset_catalog(ws.string(), repo.string(), std::string());
  bool stand = false, camera = false;
  for (const auto & entry : model.assets) {
    if (entry.path.find("camera_stand.stl") != std::string::npos) {
      stand = true;
      EXPECT_EQ(entry.category, "environment_object");
      ASSERT_FALSE(entry.role_hints.empty());
      EXPECT_EQ(entry.role_hints.front(), "fixture");
    }
    if (entry.path.find("d435.dae") != std::string::npos) {
      camera = true;
      EXPECT_EQ(entry.category, "camera_sensor");
    }
  }
  EXPECT_TRUE(stand);
  EXPECT_TRUE(camera);
}

TEST(AssetLibraryModel, PackageMarkedReplaceIsWithheldWithItsReason)
{
  const fs::path repo = make_tmp_dir("repo_withheld");
  const fs::path ws = make_tmp_dir("ws_withheld");
  const fs::path stand = repo / "assets" / "environment" / "pipe_camera_stand_description";
  write_text(stand / "package.xml",
    "<package format=\"2\"><name>pipe_camera_stand_description</name><export>"
    "<workcell_asset_library status=\"replace\" reason=\"generic box, not this product\"/></export></package>\n");
  write_text(stand / "meshes" / "visual" / "pipe_camera_stand.stl", "solid s\nendsolid s\n");
  const fs::path bin = repo / "assets" / "environment" / "sorting_bin_description";
  write_text(bin / "package.xml", "<package format=\"2\"><name>sorting_bin_description</name></package>\n");
  write_text(bin / "meshes" / "sorting_bin.stl", "solid s\nendsolid s\n");

  const AssetCatalogModel model = discover_asset_catalog(ws.string(), repo.string(), std::string());
  bool bin_listed = false;
  for (const auto & entry : model.assets) {
    EXPECT_EQ(entry.path.find("pipe_camera_stand"), std::string::npos) << entry.path;
    bin_listed = bin_listed || entry.id == "sorting_bin_description";
  }
  EXPECT_TRUE(bin_listed);
  ASSERT_EQ(model.withheld_packages.size(), 1u);
  EXPECT_EQ(model.withheld_packages.front(), "pipe_camera_stand_description: generic box, not this product");
}

TEST(AssetLibraryModel, UnpackagedDescriptionMeshAndXacroAreOneEntry)
{
  const fs::path repo = make_tmp_dir("repo_unpackaged");
  const fs::path ws = make_tmp_dir("ws_unpackaged");
  const fs::path tray = repo / "workcell_builder" / "workcell_builder" / "assets" / "environment" / "tray_description";
  write_text(tray / "meshes" / "tray.stl", "solid s\nendsolid s\n");
  write_text(tray / "urdf" / "tray.urdf.xacro", "<robot name=\"tray\"/>\n");

  const AssetCatalogModel model = discover_asset_catalog(ws.string(), repo.string(), std::string());
  int tray_entries = 0;
  for (const auto & entry : model.assets) {
    if (entry.path.find("tray_description") != std::string::npos) {
      ++tray_entries;
      EXPECT_EQ(entry.id, "tray");  // stable identity used by placed items is unchanged
    }
  }
  EXPECT_EQ(tray_entries, 1);
}

TEST(AssetCatalogDiscovery, PackagesAreNamedAsProductsAndMultiPartToolsAreNotLooseObjects)
{
  const fs::path repo = make_tmp_dir("product_names");
  const fs::path ws = make_tmp_dir("product_names_ws");
  const fs::path three_finger = repo / "assets" / "end_effectors" / "robotiq_3f_gripper_description";
  write_text(three_finger / "package.xml", "<package><name>robotiq_3f_gripper_description</name></package>");
  write_text(three_finger / "meshes" / "visual" / "link_0.stl", "solid a\nendsolid\n");
  write_text(three_finger / "meshes" / "visual" / "finger.stl", "solid b\nendsolid\n");
  const fs::path cup = repo / "assets" / "end_effectors" / "airpick_description";
  write_text(cup / "package.xml", "<package><name>airpick_description</name></package>");
  write_text(cup / "meshes" / "airpick.stl", "solid c\nendsolid\n");

  const auto model = discover_asset_catalog(ws.string(), repo.string());
  const auto find = [&model](const std::string & id) {
    return std::find_if(model.assets.begin(), model.assets.end(), [&id](const auto & e) { return e.id == id; });
  };
  const auto gripper = find("robotiq_3f_gripper_description");
  ASSERT_NE(gripper, model.assets.end());
  EXPECT_EQ(gripper->display_name, "Robotiq 3f Gripper");
  EXPECT_EQ(gripper->display_name.find("Link"), std::string::npos);
  EXPECT_EQ(gripper->display_name.find("Finger"), std::string::npos);
  EXPECT_FALSE(gripper->can_add_to_scene);
  EXPECT_EQ(gripper->readiness, "ASSIGN_AS_END_EFFECTOR");
  ASSERT_FALSE(gripper->blockers.empty());
  EXPECT_NE(gripper->blockers.front().find("End Effector"), std::string::npos);

  const auto single = find("airpick_description");
  ASSERT_NE(single, model.assets.end());
  EXPECT_TRUE(single->can_add_to_scene);
}

TEST(AssetCatalogDiscovery, IdenticalGeometryFromTwoPackagesIsListedOnce)
{
  const fs::path repo = make_tmp_dir("same_geometry");
  const fs::path ws = make_tmp_dir("same_geometry_ws");
  const std::string table_mesh = "solid table\nfacet normal 0 0 1\nendsolid table\n";
  for (const std::string name : {"table_description", "workbench_description"}) {
    const fs::path package = repo / "assets" / "environment" / name;
    write_text(package / "package.xml", "<package><name>" + name + "</name></package>");
    write_text(package / "meshes" / "visual" / "table.stl", table_mesh);
  }
  // A description that only references the same mesh is that product again.
  write_text(repo / "workcell_builder" / "workcell_builder" / "assets" / "environment" / "table_description" /
    "urdf" / "table.urdf.xacro", "<robot><mesh filename=\"package://table_description/meshes/visual/table.stl\"/></robot>\n");
  // A different product with its own geometry stays listed.
  const fs::path block = repo / "workcell_builder" / "workcell_builder" / "assets" / "environment" / "workbench_description";
  write_text(block / "meshes" / "workbench.stl", "solid block\nendsolid block\n");

  const auto model = discover_asset_catalog(ws.string(), repo.string());
  int tables = 0;
  bool block_listed = false;
  for (const auto & entry : model.assets) {
    tables += entry.path.find("table.stl") != std::string::npos || entry.path.find("table.urdf") != std::string::npos;
    block_listed = block_listed || entry.path.find("workbench.stl") != std::string::npos;
  }
  EXPECT_EQ(tables, 1);
  EXPECT_TRUE(block_listed);
}

TEST(AssetLibraryModel, LocalLibraryImportsAreListedAsImportedInEveryCell)
{
  const fs::path repo = make_tmp_dir("library_repo");
  const fs::path ws = make_tmp_dir("library_ws");
  const fs::path mesh = repo / "workcell_builder/workcell_builder/assets/imported/customer_part.stl";
  write_text(mesh, "solid part\nendsolid part\n");
  AssetCatalogEntry imported;
  imported.id = "imported_customer_part";
  imported.display_name = "customer_part";
  imported.path = mesh.string();
  imported.scale = 0.001;
  imported.can_add_to_scene = true;
  AssetCatalogEntry missing;
  missing.id = "imported_gone";
  missing.path = "workcell_builder/workcell_builder/assets/imported/gone.stl";
  missing.blockers = {"the imported mesh file is missing"};

  // No cell is open: the library is not scene-owned.
  const AssetCatalogModel model = discover_asset_catalog(ws.string(), repo.string(), std::string(), {imported, missing});
  const auto listed = std::find_if(model.assets.begin(), model.assets.end(), [](const auto & e) {
    return e.id == "imported_customer_part";
  });
  ASSERT_NE(listed, model.assets.end());
  EXPECT_EQ(listed->path, mesh.string());
  EXPECT_DOUBLE_EQ(listed->scale, 0.001);
  EXPECT_TRUE(listed->can_add_to_scene);
  EXPECT_EQ(asset_provenance(*listed), "imported");
  EXPECT_EQ(listed->category, "Imported");
  const auto gone = std::find_if(model.assets.begin(), model.assets.end(), [](const auto & e) {
    return e.id == "imported_gone";
  });
  ASSERT_NE(gone, model.assets.end());
  EXPECT_FALSE(gone->can_add_to_scene);
  EXPECT_EQ(gone->blockers.front(), "the imported mesh file is missing");
}
