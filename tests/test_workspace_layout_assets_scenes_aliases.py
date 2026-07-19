from pathlib import Path


def test_fix_workspace_layout_uses_assets_and_scenes_aliases():
    text = Path('scripts/fix_workspace_layout.sh').read_text(encoding='utf-8')
    assert 'ensure_workspace_alias "assets"' in text
    assert 'ensure_workspace_alias "scenes"' in text
    assert 'Removing legacy asset package symlink:' in text
    assert 'Duplicate package discovery detected' in text
    assert 'Workspace layout summary' in text
    assert 'src/assets ->' in text
    assert 'src/scenes ->' in text
    assert 'Exposed ${#EXPOSED_PACKAGES[@]}' not in text


def test_fix_workspace_layout_no_per_package_asset_symlink_creation():
    text = Path('scripts/fix_workspace_layout.sh').read_text(encoding='utf-8')
    assert 'find "$REPO_DIR/assets" -name package.xml' in text  # still used for discovery checks
    assert 'Linking repository package' not in text


def test_verify_workspace_discovery_accepts_alias_layout():
    text = Path('scripts/verify_workspace_discovery.sh').read_text(encoding='utf-8')
    assert 'CANONICAL_REPO="$SRC_DIR/easy_manipulation_deployment"' in text
    assert 'canonical repository layout at $CANONICAL_REPO' in text
    assert 'legacy workspace aliases at $SRC_DIR/assets and $SRC_DIR/scenes' in text
    assert 'Missing workspace layout: expected canonical repo at $CANONICAL_REPO' in text
    assert 'Duplicate package discovered' in text
    assert 'required=(easy_manipulation_deployment workcell_builder)' in text


def test_curated_asset_tokens_present():
    txt = Path('workcell_builder/workcell_builder/config/asset_profiles/environment_assets.json').read_text(encoding='utf-8')
    assert "table_small" in txt and "pick_box" in txt


def test_portable_bundle_markers_present():
    blob = Path('workcell_builder/workcell_builder/gui/scene_select.cpp').read_text(encoding='utf-8')
    for m in ['Export Scene Bundle','Import Scene Bundle','Portable Scene Bundle','Bundle Validation Status','Imported Scene Ready','Exported Scene Archive']:
        assert m in blob
