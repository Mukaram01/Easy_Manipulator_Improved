from pathlib import Path


ROOT = Path(__file__).parents[1]
CPP = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text(encoding="utf-8")
HEADER = (ROOT / "workcell_builder/workcell_builder/gui/mainwindow.h").read_text(encoding="utf-8")


def test_selected_item_card_exposes_intentional_controlled_semantic_roles():
    assert "QComboBox * inspector_semantic_role_" in HEADER
    assert 'setObjectName("sceneBuilderSemanticRoleCombo")' in CPP
    for label, value in (
        ("Generic asset", "asset"),
        ("Pick object", "pick_object"),
        ("Target bin", "target_bin"),
        ("Fixture", "fixture"),
        ("Support surface", "support_surface"),
        ("Camera", "camera"),
        ("Machine", "machine"),
        ("Conveyor", "conveyor"),
    ):
        assert f'{{"{label}", "{value}"}}' in CPP
    assert 'metadata_form->addRow("Semantic role", inspector_semantic_role_)' in CPP


def test_role_change_updates_instance_state_and_roundtrips_to_layout():
    assert "updated_semantic_role = inspector_semantic_role_->currentData().toString().trimmed()" not in CPP
    assert "const QString chosen_role = inspector_semantic_role_->currentData().toString().trimmed()" in CPP
    assert "i->setData(RoleRole, updated_semantic_role)" in CPP
    assert "refreshed_state.role = updated_semantic_role" in CPP
    assert "p.role = refreshed_state.role" in CPP
    assert "state.role = gi->data(RoleRole).toString().toStdString()" in CPP


def test_imported_mesh_defaults_to_generic_asset_without_filename_role_inference():
    assert 'item->setData(RoleRole, "asset")' in CPP
    assert "Filenames never infer pick/task semantics" in CPP
    import_section = CPP[CPP.index("void MainWindow::place_selected_asset_from_dialog"):]
    import_section = import_section[: import_section.index("void MainWindow::")]
    assert "pick_object" not in import_section
    assert "target_bin" not in import_section
