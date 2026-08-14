#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>

namespace {
std::string load_file(const std::string & path)
{
  std::ifstream in(path);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}
}

TEST(LayoutSerializationContractTest, SaveLayoutWritesCanonicalFields)
{
  const std::string src = load_file("gui/mainwindow.cpp");
  EXPECT_NE(src.find("item[\"id\"]"), std::string::npos);
  EXPECT_NE(src.find("item[\"display_name\"]"), std::string::npos);
  EXPECT_NE(src.find("item[\"category\"]"), std::string::npos);
  EXPECT_NE(src.find("item[\"type\"]"), std::string::npos);
  EXPECT_NE(src.find("pose[\"xyz\"]"), std::string::npos);
  EXPECT_NE(src.find("pose[\"rpy\"]"), std::string::npos);
  EXPECT_NE(src.find("item[\"dimensions\"]"), std::string::npos);
  EXPECT_NE(src.find("item[\"editable\"]"), std::string::npos);
  EXPECT_NE(src.find("item[\"locked\"]"), std::string::npos);
  EXPECT_NE(src.find("ensure_map_node(item, \"mesh\")"), std::string::npos);
}

TEST(LayoutSerializationContractTest, EmbeddedStructuralSaveUsesNativeSessionAuthority)
{
  const std::string src = load_file("gui/mainwindow.cpp");
  const std::string controller = load_file("gui/embedded_web_edit_save_controller.hpp");
  EXPECT_NE(src.find("save_native_layout_changes(const QJsonObject & web_patch"), std::string::npos);
  EXPECT_NE(src.find("if (save_layout_button_) save_layout_button_->setEnabled(true)"), std::string::npos);
  EXPECT_NE(controller.find("host_dirty_ && host_dirty_()"), std::string::npos);
  EXPECT_NE(controller.find("native structural changes"), std::string::npos);
  EXPECT_NE(controller.find("native_label"), std::string::npos);
}

TEST(LayoutSerializationContractTest, SessionAddedTransformsBypassStaleDiskPatchValidation)
{
  const std::string src = load_file("gui/mainwindow.cpp");
  const std::string controller = load_file("gui/embedded_web_edit_save_controller.hpp");
  EXPECT_NE(src.find("command.kind == QStringLiteral(\"add\")"), std::string::npos);
  EXPECT_NE(src.find("command.kind == QStringLiteral(\"duplicate\")"), std::string::npos);
  EXPECT_NE(src.find("apply_web_transforms_to_editable_layout_session"), std::string::npos);
  EXPECT_NE(src.find("canvas->setData(RolePoseZ"), std::string::npos);
  EXPECT_NE(controller.find("session-added Web3D transforms serialized without stale web_scene validation"), std::string::npos);
}

TEST(LayoutSerializationContractTest, InstanceIdentityAndMeshScaleRemainIndependent)
{
  const std::string src = load_file("gui/mainwindow.cpp");
  EXPECT_NE(src.find("item[\"catalog_asset_id\"]"), std::string::npos);
  EXPECT_NE(src.find("RoleCatalogAssetId"), std::string::npos);
  EXPECT_NE(src.find("RoleMeshScaleX"), std::string::npos);
  EXPECT_NE(src.find("scale.push_back(mesh_scale_x)"), std::string::npos);
  EXPECT_NE(src.find("saved_ids.contains(id)"), std::string::npos);
  EXPECT_NE(src.find("editable_by_id.insert"), std::string::npos);
}

TEST(LayoutSerializationContractTest, MixedUnsafeValidationRejectsBeforeAnyWrite)
{
  const std::string controller = load_file("gui/embedded_web_edit_save_controller.hpp");
  const auto reject = controller.find("Save Layout cannot safely compose native structural edits");
  const auto stage = controller.find("writePatchAtomically(patch");
  ASSERT_NE(reject, std::string::npos);
  ASSERT_NE(stage, std::string::npos);
  EXPECT_LT(reject, stage);
  EXPECT_NE(controller.find("both dirty states were retained"), std::string::npos);
  EXPECT_NE(controller.find("Existing editable items must keep using guarded patch validation"), std::string::npos);
}

TEST(LayoutSerializationContractTest, SaveLayoutPreservesUnknownFieldsViaCloneMerge)
{
  const std::string src = load_file("gui/mainwindow.cpp");
  EXPECT_NE(src.find("YAML::Clone(existing)"), std::string::npos);
  EXPECT_NE(src.find("existing_by_id"), std::string::npos);
}

TEST(LayoutSerializationContractTest, SaveLayoutKeepsLockedCanonicalItemsOnRoundTrip)
{
  const std::string src = load_file("gui/mainwindow.cpp");
  EXPECT_NE(src.find("YAML::Node existing_items = root[\"items\"]"), std::string::npos);
  EXPECT_NE(src.find("editable_by_id.contains(id)"), std::string::npos);
  EXPECT_NE(src.find("serialized_editable_canvas_item(editable_by_id.value(id), existing_item)"), std::string::npos);
  EXPECT_NE(src.find("updated_placed.push_back(YAML::Clone(existing_item))"), std::string::npos);
  EXPECT_NE(src.find("if (saved_ids.contains(id)) continue;"), std::string::npos);
}

TEST(LayoutSerializationContractTest, MalformedLayoutBackupBehaviorStillPresent)
{
  const std::string src = load_file("gui/mainwindow.cpp");
  EXPECT_NE(src.find(".malformed_backup_"), std::string::npos);
  EXPECT_NE(src.find("Malformed layout YAML detected"), std::string::npos);
}

TEST(LayoutSerializationContractTest, InspectorEditsResolveStableEditableLayoutTarget)
{
  const std::string src = load_file("gui/mainwindow.cpp");
  EXPECT_NE(src.find("resolve_selected_editable_layout_target"), std::string::npos);
  EXPECT_NE(src.find("current_selected_scene_item_id_"), std::string::npos);
  EXPECT_NE(src.find("Use or create an editable layout record first"), std::string::npos);
  EXPECT_NE(src.find("fallback_view_refreshed"), std::string::npos);
}

TEST(LayoutSerializationContractTest, SelectionTransformEditorSupportsStateWithoutCanvasItem)
{
  const std::string src = load_file("gui/mainwindow.cpp");
  EXPECT_NE(src.find("refresh_selection_transform_editor_from_state"), std::string::npos);
  EXPECT_NE(src.find("refresh_selection_transform_editor_from_state(selected_item_state_)"), std::string::npos);
  EXPECT_NE(src.find("state.pose_available"), std::string::npos);
}


TEST(LayoutSerializationContractTest, Scene3DGizmoEditsOnlyEditableLayoutItems)
{
  const std::string src = load_file("gui/mainwindow.cpp");
  EXPECT_NE(src.find("scene3d_viewport->transform_changed_cb"), std::string::npos);
  EXPECT_NE(src.find("const bool locked_item = item->data(RoleLocked).toBool()"), std::string::npos);
  EXPECT_NE(src.find("item->data(RoleSourceLayer).toString().trimmed()"), std::string::npos);
  EXPECT_NE(src.find("source_layer.compare(QStringLiteral(\"editable_layout\"), Qt::CaseInsensitive) == 0"), std::string::npos);
  EXPECT_NE(src.find("if (locked_item || !editable_source_layer)"), std::string::npos);
  EXPECT_NE(src.find("Scene3D transform edit blocked: locked item cannot be edited"), std::string::npos);
  EXPECT_NE(src.find("mark_layout_dirty(\"Scene3D Gizmo Transform\")"), std::string::npos);
}

TEST(LayoutSerializationContractTest, SaveLayoutForSceneWithOnlyLegacyEnvironmentLayoutWritesCanonical)
{
  const std::string src = load_file("gui/mainwindow.cpp");
  EXPECT_NE(src.find("selected_scene_canonical_layout_save_path"), std::string::npos);
  EXPECT_NE(src.find("return scene_dir / \"layout\" / \"workcell_studio_layout.yaml\""), std::string::npos);
  EXPECT_NE(src.find("selected_scene_layout_import_candidates"), std::string::npos);
  EXPECT_NE(src.find("append_unique(canonical_layout);"), std::string::npos);
  EXPECT_NE(src.find("append_unique(legacy_environment_layout);"), std::string::npos);
  EXPECT_NE(src.find("append_unique(manifest_layout);"), std::string::npos);
  EXPECT_EQ(src.find("static fs::path selected_scene_environment_layout_path"), std::string::npos);
}

TEST(LayoutSerializationContractTest, SaveLayoutForSceneWithNonCanonicalManifestLayoutWritesCanonicalOnly)
{
  const std::string src = load_file("gui/mainwindow.cpp");
  EXPECT_NE(src.find("const fs::path layout_path = selected_scene_canonical_layout_save_path(scene_browser_result_, selected_scene_index_);"), std::string::npos);
  EXPECT_NE(src.find("for (const auto & candidate : selected_scene_layout_import_candidates(scene_dir))"), std::string::npos);
  EXPECT_NE(src.find("root = YAML::LoadFile(candidate.string())"), std::string::npos);
  EXPECT_NE(src.find("QSaveFile out(QString::fromStdString(effective_layout_path.string()))"), std::string::npos);
}

TEST(LayoutSerializationContractTest, DeleteSelectedUsesEditableSelectionAndUndoRedo)
{
  const std::string src = load_file("gui/mainwindow.cpp");
  const std::string hdr = load_file("gui/mainwindow.h");
  EXPECT_NE(src.find("register_scene_action(\"layout.remove\", \"Delete Selected\""), std::string::npos);
  EXPECT_NE(src.find("delete_action->setShortcut(QKeySequence(Qt::Key_Delete))"), std::string::npos);
  EXPECT_NE(src.find("resolve_selected_editable_layout_target()"), std::string::npos);
  EXPECT_NE(src.find("Selected item cannot be deleted"), std::string::npos);
  EXPECT_NE(src.find("deleted_layout_item_ids_.insert(id)"), std::string::npos);
  EXPECT_NE(src.find("undo_stack_.push_back(command); redo_stack_.clear();"), std::string::npos);
  EXPECT_NE(src.find("Restored %1"), std::string::npos);
  EXPECT_NE(src.find("Deleted %1"), std::string::npos);
  EXPECT_NE(hdr.find("QSet<QString> deleted_layout_item_ids_"), std::string::npos);
}

TEST(LayoutSerializationContractTest, SaveLayoutPersistsDeletedAuthoredItemAbsence)
{
  const std::string src = load_file("gui/mainwindow.cpp");
  EXPECT_NE(src.find("deleted_layout_item_ids_.contains(id)"), std::string::npos);
  EXPECT_NE(src.find("deleted_layout_item_ids_.clear();"), std::string::npos);
  EXPECT_NE(src.find("for (int i = all_scene_preview_items_.size() - 1; i >= 0; --i) if (all_scene_preview_items_[i].id == id) all_scene_preview_items_.removeAt(i);"), std::string::npos);
}

TEST(LayoutSerializationContractTest, DuplicateSelectedUsesSharedEditableSelectionActionAndUndo)
{
  const std::string src = load_file("gui/mainwindow.cpp");
  const std::string hdr = load_file("gui/mainwindow.h");
  EXPECT_NE(src.find("register_scene_action(\"layout.duplicate\", \"Duplicate Selected\""), std::string::npos);
  EXPECT_NE(src.find("duplicate_action->setShortcut(QKeySequence(QStringLiteral(\"Ctrl+D\")))"), std::string::npos);
  EXPECT_NE(src.find("canvas_more_menu->addAction(scene_builder_action(\"layout.duplicate\"))"), std::string::npos);
  EXPECT_NE(src.find("selected_item_can_be_duplicated()"), std::string::npos);
  EXPECT_NE(src.find("resolve_selected_editable_layout_target()"), std::string::npos);
  EXPECT_NE(src.find("Selected item cannot be duplicated"), std::string::npos);
  EXPECT_NE(src.find("true, false, {copy}}"), std::string::npos);
  EXPECT_NE(src.find("undo_stack_.push_back(command); redo_stack_.clear();"), std::string::npos);
  EXPECT_NE(src.find("Removed duplicate %1"), std::string::npos);
  EXPECT_NE(hdr.find("refresh_duplicate_selected_action"), std::string::npos);
}

TEST(LayoutSerializationContractTest, CanvasEditCommandsInitializePreviewItemsExplicitly)
{
  const std::string src = load_file("gui/mainwindow.cpp");
  const std::string hdr = load_file("gui/mainwindow.h");
  EXPECT_NE(hdr.find("CanvasEditCommand("), std::string::npos);
  EXPECT_NE(hdr.find("preview_items(preview_items)"), std::string::npos);
  EXPECT_NE(src.find("true, false, {copy}}"), std::string::npos);
  EXPECT_NE(src.find("false, true, deleted_preview_items}"), std::string::npos);
  EXPECT_NE(src.find("false, false, {}}"), std::string::npos);
}

TEST(LayoutSerializationContractTest, DuplicateSelectedCopiesAuthoredRecordWithStableOffsetIdentity)
{
  const std::string src = load_file("gui/mainwindow.cpp");
  EXPECT_NE(src.find("copy = p; found_preview = true"), std::string::npos);
  EXPECT_NE(src.find("reserved_ids.insert(c.item_id.trimmed().toStdString())"), std::string::npos);
  EXPECT_NE(src.find("const QString copy_base = base_id + QStringLiteral(\"_copy\")"), std::string::npos);
  EXPECT_NE(src.find("base_name.replace(copy_suffix, QString())"), std::string::npos);
  EXPECT_NE(src.find("new_name = QStringLiteral(\"%1 copy %2\")"), std::string::npos);
  EXPECT_NE(src.find("copy.x += 0.10"), std::string::npos);
  EXPECT_NE(src.find("copy.y += 0.10"), std::string::npos);
  EXPECT_NE(src.find("item->setData(RoleSource, copy.source_path)"), std::string::npos);
  EXPECT_NE(src.find("item->setData(RoleWidth, copy.sx)"), std::string::npos);
  EXPECT_NE(src.find("apply_scene_selection(new_id, copy.role, false, false)"), std::string::npos);
  EXPECT_NE(src.find("Duplicated %1 as %2"), std::string::npos);
}
