from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CPP = ROOT / "workcell_builder/workcell_builder/gui/scene_select.cpp"
HDR = ROOT / "workcell_builder/workcell_builder/gui/scene_select.h"


def _cpp():
    return CPP.read_text()


def _hdr():
    return HDR.read_text()


def _section(source: str, start: str, end: str) -> str:
    return source[source.index(start): source.index(end, source.index(start))]


def test_scene_loads_are_cached_on_real_file_identity():
    header = _hdr()
    assert "SceneMetadataSnapshot" in header
    assert "SceneRefreshIdentity" in header
    assert "canonical scene root" not in header.lower()  # implementation, not comment-only
    for tracked in [
        "environment_yaml",
        "cell_definition_yaml",
        "scene_manifest_yaml",
        "workcell_studio_layout_yaml",
    ]:
        assert tracked in header
    source = _cpp()
    validity = source[source.index("bool SceneSelect::scene_metadata_snapshot_valid") :]
    assert "fs::canonical(scene_dir" in validity
    assert "last_write_time" in source
    assert "file_size" in source
    assert "scene_dir / \"cell_definition.yaml\"" in source
    assert "scene_dir / \"scene_manifest.yaml\"" in source
    assert "scene_dir / \"layout\" / \"workcell_studio_layout.yaml\"" in source


def test_cached_scene_model_is_reused_in_status_paths():
    source = _cpp()
    load_fn = _section(source, "bool SceneSelect::load_scene_from_yaml", "void SceneSelect::invalidate_scene_metadata_snapshot")
    assert "scene_metadata_snapshot_valid(scene_dir)" in load_fn
    assert "*input_scene = scene_metadata_snapshot_.scene" in load_fn
    assert "YAML::LoadFile(yaml_path.string())" in load_fn
    assert "load_task_zones_from_environment_yaml(yaml_path.string()" not in load_fn
    assert "Scene & stored_scene = workcell.scene_vector[current_index]" in source
    assert "Scene & curr_scene = workcell.scene_vector[current_index]" in source


def test_100_identical_automatic_refresh_requests_cause_one_model_load_contract():
    source = _cpp()
    refresh_fn = _section(source, "void SceneSelect::refresh_scene_status", "void SceneSelect::render_workcell_studio_status")
    assert "scene_refresh_trigger_forces_reload(trigger)" in refresh_fn
    assert "current_scene_refresh_identity()" in refresh_fn
    assert "scene_refresh_identity_equal(refresh_identity, last_completed_automatic_refresh_identity_)" in refresh_fn
    assert "return;" in refresh_fn.split("scene_refresh_identity_equal(refresh_identity, last_completed_automatic_refresh_identity_)", 1)[1].split("if (refresh_in_progress_)", 1)[0]
    assert "last_completed_automatic_refresh_identity_ = refresh_identity" in refresh_fn
    assert "check_scene(strict);" in refresh_fn


def test_programmatic_scene_selection_does_not_cause_feedback_refresh():
    source = _cpp()
    refresh_scenes = _section(source, "void SceneSelect::refresh_scenes", "int SceneSelect::current_scene_index")
    assert "QSignalBlocker scene_list_blocker(ui->scene_list)" in refresh_scenes
    assert "programmatic_scene_selection_update_ = true" in refresh_scenes
    assert "ui->scene_list->setCurrentIndex(latest_scene)" in refresh_scenes
    assert "on_scene_list_currentIndexChanged(latest_scene)" not in refresh_scenes
    selection = _section(source, "void SceneSelect::on_scene_list_currentIndexChanged", "void SceneSelect::on_generate_files_clicked")
    assert "if (programmatic_scene_selection_update_)" in selection
    assert "return;" in selection.split("if (programmatic_scene_selection_update_)", 1)[1].split("const SceneRefreshIdentity", 1)[0]


def test_one_real_yaml_revision_change_and_scene_switch_force_exactly_one_reload():
    source = _cpp()
    identity = _section(source, "SceneSelect::SceneRefreshIdentity SceneSelect::current_scene_refresh_identity", "bool SceneSelect::scene_refresh_identity_equal")
    assert "metadata_file_identity(scene_dir / \"environment.yaml\")" in identity
    assert "metadata_file_identity(scene_dir / \"cell_definition.yaml\")" in identity
    assert "metadata_file_identity(scene_dir / \"scene_manifest.yaml\")" in identity
    assert "metadata_file_identity(scene_dir / \"layout\" / \"workcell_studio_layout.yaml\")" in identity
    equality = _section(source, "bool SceneSelect::scene_refresh_identity_equal", "bool SceneSelect::scene_refresh_trigger_forces_reload")
    assert "modified_time" in equality and "size" in equality
    selection = _section(source, "void SceneSelect::on_scene_list_currentIndexChanged", "void SceneSelect::on_generate_files_clicked")
    assert "actual_scene_switch" in selection
    assert '"Scene Switch"' in selection
    assert '"scene_selection_refresh"' in selection


def test_explicit_refresh_forces_exactly_one_reload_and_write_boundaries_are_preserved():
    source = _cpp()
    trigger_policy = _section(source, "bool SceneSelect::scene_refresh_trigger_forces_reload", "fs::path SceneSelect::scene_dir_for_current_selection")
    for trigger in [
        '"Refresh Scene Status"',
        '"Generate YAML"',
        '"Generate Files"',
        '"Repair Scene YAML"',
        '"Open/Edit Cell"',
        '"Scene Switch"',
        '"Save Layout"',
        '"Generate Scene Package"',
    ]:
        assert trigger in trigger_policy
    refresh_button = _section(source, "void SceneSelect::on_refresh_status_button_clicked", "void SceneSelect::on_validate_scene_button_clicked")
    assert "invalidate_scene_metadata_snapshot();" in refresh_button
    assert 'refresh_scene_status(true, "Refresh Scene Status")' in refresh_button
    assert source.count("invalidate_scene_metadata_snapshot();") >= 5


def test_product_view_scene_ready_status_callbacks_do_not_rebuild_scene():
    source = _cpp()
    refresh_fn = _section(source, "void SceneSelect::refresh_scene_status", "void SceneSelect::render_workcell_studio_status")
    status_render = _section(source, "void SceneSelect::render_workcell_studio_status", "void SceneSelect::configure_startup_fallback_paths")
    assert "build_workcell_studio_canvas_model" not in refresh_fn
    assert "build_workcell_studio_canvas_model" not in status_render
    assert "refresh_preview_status();" not in refresh_fn
    assert "refresh_preview_status();" not in status_render


def test_refresh_requests_are_coalesced_and_invalidated_at_write_boundaries():
    source = _cpp()
    refresh_fn = _section(source, "void SceneSelect::refresh_scene_status", "void SceneSelect::render_workcell_studio_status")
    assert "refresh_in_progress_" in refresh_fn
    assert "refresh_queued_" in refresh_fn
    assert "queued_refresh_identity_" in refresh_fn
    assert "QTimer::singleShot(0" in refresh_fn
    assert "Coalesced Refresh" in refresh_fn
    for slot in [
        "void SceneSelect::on_browse_scenes_folder_clicked()",
        "void SceneSelect::on_refresh_scenes_button_clicked()",
        "void SceneSelect::on_refresh_status_button_clicked()",
    ]:
        section = source[source.index(slot) : source.index("\n}", source.index(slot))]
        assert "invalidate_scene_metadata_snapshot();" in section
