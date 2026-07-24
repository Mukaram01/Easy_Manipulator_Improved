from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / "workcell_builder/workcell_builder/src_workcell_studio_canvas_model.cpp").read_text(encoding="utf-8")
HPP = (ROOT / "workcell_builder/workcell_builder/include/workcell_studio_canvas_model.hpp").read_text(encoding="utf-8")
MAIN = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")


def test_snapshot_tracks_authoritative_scene_metadata_files_and_content_identity():
    for token in [
        "environment.yaml",
        "cell_definition.yaml",
        "scene_manifest.yaml",
        "environment_layout.yaml",
        "layout/workcell_studio_layout.yaml",
        "config/workcell_builder_task_intent.yaml",
        "config/task_recipe.yaml",
    ]:
        assert token in CPP
    assert "file_size(path" in CPP
    assert "last_write_time(path" in CPP
    assert "content_hash" in CPP
    assert "std::hash<std::string>{}(buffer.str())" in CPP


def test_repeated_consumers_share_one_same_scene_snapshot_parse_without_cache_hit_log_noise():
    assert "load_scene_metadata_snapshot(scene_dir, scene_name" in CPP
    assert "snapshot_yaml(snapshot, \"environment.yaml\"" in CPP
    assert "snapshot_yaml(snapshot, \"scene_manifest.yaml\"" in CPP
    assert "snapshot_yaml(snapshot, \"environment_layout.yaml\"" in CPP
    assert "snapshot_yaml(snapshot, \"layout/workcell_studio_layout.yaml\"" in CPP
    assert "++cached.cache_hits;" in CPP
    assert "files_parsed=0 cache_hits=" not in CPP
    assert CPP.count("Workcell Studio scene metadata snapshot: scene_id=") == 1


def test_revision_change_and_scene_switch_cannot_reuse_old_metadata():
    assert "cached.scene_dir == key && cached.revision == revision" in CPP
    assert "file_revision_change" in CPP
    assert "scene_switch" in CPP
    assert "canonical_scene_cache_key" in CPP
    assert "weakly_canonical" in CPP


def test_parse_failures_are_not_permanently_cached_after_file_changes():
    assert "snapshot.statuses[rel] = status" in CPP
    assert "if (status.loaded)" in CPP
    assert "snapshot.documents[rel] = doc" in CPP
    assert "content_hash" in CPP
    assert "cached.revision == revision" in CPP


def test_explicit_successful_mutations_invalidate_snapshot_once():
    assert "invalidate_workcell_studio_scene_metadata_snapshot" in HPP
    assert 'invalidate_workcell_studio_scene_metadata_snapshot(scene_dir, "save_layout")' in MAIN
    assert 'invalidate_workcell_studio_scene_metadata_snapshot(sc.scene_dir, "generation")' in MAIN
    assert "if (!state.cached.scene_dir.empty() && state.cached.scene_dir == key) state.cached = SceneMetadataSnapshot{};" in CPP


def test_snapshot_logging_replaces_per_file_success_noise_with_one_reload_summary():
    assert "Workcell Studio scene metadata snapshot: scene_id=" in CPP
    assert "revision=" in CPP
    assert "files_parsed=" in CPP
    assert "cache_hits=0" in CPP
    assert "invalidation_reason=" in CPP


def test_fifty_unchanged_cache_reads_do_not_emit_repeated_summary_logs():
    cache_hit_block = "if (!explicitly_invalidated && !cached.scene_dir.empty() && cached.scene_dir == key && cached.revision == revision)"
    assert cache_hit_block in CPP
    after_cache_hit = CPP.split(cache_hit_block, 1)[1].split("SceneMetadataSnapshot snapshot", 1)[0]
    assert "++cached.cache_hits;" in after_cache_hit
    assert "std::cerr" not in after_cache_hit


def test_real_file_revision_change_emits_one_new_summary_log():
    assert "cached.scene_dir == key ? \"file_revision_change\" : \"scene_switch\"" in CPP
    assert "cached = snapshot;" in CPP
    assert CPP.count("Workcell Studio scene metadata snapshot: scene_id=") == 1
