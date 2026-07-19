from pathlib import Path

MAIN = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
HDR = Path('workcell_builder/workcell_builder/gui/mainwindow.h').read_text(encoding='utf-8')


def test_scene_builder_header_uses_compact_relative_context_not_absolute_home_path():
    assert 'compact_scene_path_context' in HDR
    assert 'return QStringLiteral("%1  ·  %2").arg(visible_name, relative_path);' in MAIN
    assert 'if (it->string() == "scenes")' in MAIN
    assert 'metrics.elidedText(compact_text, Qt::ElideMiddle, available_width)' in MAIN
    old_absolute_header = 'scene_builder_path_label_->setText(QString("Path: %1").arg(short_path));'
    assert old_absolute_header not in MAIN


def test_scene_builder_header_tooltip_and_copy_keep_full_path():
    assert 'scene_builder_path_label_->setToolTip(cleaned_path);' in MAIN
    assert 'scene_builder_copy_path_button_->setProperty("fullScenePath", cleaned_path);' in MAIN
    assert 'QApplication::clipboard()->setText(scene_builder_copy_path_button_->property("fullScenePath").toString())' in MAIN
    assert 'QApplication::clipboard()->setText(selected_scene_path())' not in MAIN


def test_scene_builder_empty_path_disables_copy_and_clears_stale_copy_data():
    empty_block_start = MAIN.index('if (cleaned_path.isEmpty())')
    empty_block = MAIN[empty_block_start:MAIN.index('return;\n  }', empty_block_start)]
    assert 'scene_builder_path_label_->setText(QStringLiteral("No scene selected"));' in empty_block
    assert 'scene_builder_copy_path_button_->setProperty("fullScenePath", QString());' in empty_block
    assert 'scene_builder_copy_path_button_->setEnabled(false);' in empty_block


def test_scene_changes_refresh_visible_tooltip_and_copy_data_together():
    assert 'update_scene_builder_path_header(selected_scene_state_.name, selected_scene_path());' in MAIN
    assert 'update_scene_builder_path_header(QString(), QString());' in MAIN
    assert 'watched == scene_builder_path_label_' in MAIN and 'event->type() == QEvent::Resize' in MAIN
