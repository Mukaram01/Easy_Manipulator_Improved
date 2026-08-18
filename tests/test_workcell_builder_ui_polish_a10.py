from pathlib import Path


CPP = (
    Path(__file__).parents[1] / "workcell_builder/workcell_builder/gui/mainwindow.cpp"
).read_text(encoding="utf-8")
SIMPLE_UI_JS = (
    Path(__file__).parents[1] / "workcell_studio_web/viewer/simple_product_ui.js"
).read_text(encoding="utf-8")
SIMPLE_UI_CSS = (
    Path(__file__).parents[1] / "workcell_studio_web/viewer/simple_product_ui.css"
).read_text(encoding="utf-8")
SERIALIZER = (
    Path(__file__).parents[1] / "workcell_builder/workcell_builder/gui/layout_item_serializer.hpp"
).read_text(encoding="utf-8")


def test_workflow_guidance_lives_in_workflow_tab_instead_of_crowding_every_tab():
    assert "workflow_tab_layout->addWidget(workflow_card)" in CPP
    assert "right_layout->addWidget(workflow_card)" not in CPP


def test_narrow_inspector_uses_scrollable_tabs_and_progressive_disclosure():
    assert "inspector_scroll->setMinimumWidth(320)" in CPP
    assert "scene_builder_inspector_tabs_->setUsesScrollButtons(true)" in CPP
    assert "scene_builder_inspector_tabs_->setDocumentMode(true)" in CPP
    assert 'new QPushButton("Bind selected as..."' in CPP
    assert 'addTab(actions_tab, "Actions")' not in CPP
    assert 'new QGroupBox("Setup checklist"' in CPP
    assert "auto * transform_actions = new QGridLayout()" in CPP
    assert 'transform_actions->addWidget(inspector_apply_button_, 0, 0)' in CPP
    assert 'transform_actions->addWidget(inspector_paste_transform_button_, 1, 1)' in CPP


def test_product_view_keeps_only_primary_actions_in_the_top_toolbar():
    assert "function consolidateToolbar()" in SIMPLE_UI_JS
    assert "document.getElementById('undo-edit')" in SIMPLE_UI_JS
    assert "document.getElementById('transform-space')?.closest('label')" in SIMPLE_UI_JS
    assert '.file-picker input[type="file"]' in SIMPLE_UI_CSS


def test_scene_relative_assets_never_keep_package_prefix_metadata():
    assert "is_scene_relative_asset_path" in SERIALIZER
    assert 'mesh.remove("source_package")' in SERIALIZER
