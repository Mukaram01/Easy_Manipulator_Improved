from pathlib import Path

STARTUP = Path(
    "workcell_builder/workcell_builder/gui/startup_dialog.cpp"
).read_text(encoding="utf-8")
MAINWINDOW = Path(
    "workcell_builder/workcell_builder/gui/mainwindow.cpp"
).read_text(encoding="utf-8")
CATALOG = Path(
    "workcell_builder/workcell_builder/gui/asset_catalog_model.cpp"
).read_text(encoding="utf-8")


def test_interactive_startup_exports_validated_workspace_before_mainwindow():
    assert 'qputenv("WORKCELL_WORKSPACE_ROOT", workspace_root.toUtf8())' in STARTUP
    assert 'is_valid_workcell_workspace(path)' in STARTUP
    assert STARTUP.index('is_valid_workcell_workspace(path)') < STARTUP.index(
        'qputenv("WORKCELL_WORKSPACE_ROOT", workspace_root.toUtf8())'
    )
    assert STARTUP.index('qputenv("WORKCELL_WORKSPACE_ROOT", workspace_root.toUtf8())') < STARTUP.index(
        'accept();'
    )


def test_mainwindow_catalog_consumes_the_exported_workspace_root():
    assert 'qgetenv("WORKCELL_WORKSPACE_ROOT")' in MAINWINDOW
    assert 'discover_asset_catalog(' in MAINWINDOW


def test_catalog_scans_both_canonical_and_workspace_symlink_asset_roots():
    assert 'workspace_root + "/src/easy_manipulation_deployment/assets"' in CATALOG
    assert 'workspace_root + "/src/assets"' in CATALOG


def test_pathless_seed_assets_are_only_a_last_resort():
    # These names match the four startup warnings seen when workspace discovery
    # was accidentally empty. A valid exported workspace must make discovery
    # populate real mesh-backed entries before this fallback is reached.
    assert 'if (model.assets.empty())' in CATALOG
    for name in ["UR5", "Robotiq 2F", "simple_conveyor", "RealSense D435i"]:
        assert f'make_seed("{name}"' in CATALOG
