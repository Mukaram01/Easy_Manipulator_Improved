from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CPP = ROOT / "workcell_builder/workcell_builder/gui/scene_select.cpp"
HDR = ROOT / "workcell_builder/workcell_builder/gui/scene_select.h"


def _cpp():
    return CPP.read_text()


def _hdr():
    return HDR.read_text()


def test_scene_loads_are_cached_on_real_file_identity():
    header = _hdr()
    assert "SceneMetadataSnapshot" in header
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
    load_fn = source[source.index("bool SceneSelect::load_scene_from_yaml") : source.index("void SceneSelect::invalidate_scene_metadata_snapshot")]
    assert "scene_metadata_snapshot_valid(scene_dir)" in load_fn
    assert "*input_scene = scene_metadata_snapshot_.scene" in load_fn
    assert "YAML::LoadFile(yaml_path.string())" in load_fn
    assert "load_task_zones_from_environment_yaml(yaml_path.string()" not in load_fn
    assert "Scene & stored_scene = workcell.scene_vector[current_index]" in source
    assert "Scene & curr_scene = workcell.scene_vector[current_index]" in source


def test_refresh_requests_are_coalesced_and_invalidated_at_write_boundaries():
    source = _cpp()
    refresh_fn = source[source.index("void SceneSelect::refresh_scene_status") : source.index("void SceneSelect::render_workcell_studio_status")]
    assert "refresh_in_progress" in refresh_fn
    assert "refresh_queued" in refresh_fn
    assert "QTimer::singleShot(0" in refresh_fn
    for slot in [
        "void SceneSelect::on_browse_scenes_folder_clicked()",
        "void SceneSelect::on_refresh_scenes_button_clicked()",
        "void SceneSelect::on_refresh_status_button_clicked()",
    ]:
        section = source[source.index(slot) : source.index("\n}", source.index(slot))]
        assert "invalidate_scene_metadata_snapshot();" in section
    assert source.count("invalidate_scene_metadata_snapshot();") >= 5
