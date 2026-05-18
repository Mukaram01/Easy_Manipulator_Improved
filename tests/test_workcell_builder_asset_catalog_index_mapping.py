from pathlib import Path


def test_asset_catalog_uses_indexed_roles_and_entries_for_actions():
    text = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text()

    assert 'enum AssetCatalogRoles' in text
    assert 'CatalogRoleIndex = Qt::UserRole' in text
    assert 'CatalogRolePlaceable = Qt::UserRole + 10' in text
    assert 'CatalogRoleSourcePath = Qt::UserRole + 11' in text

    assert 'item->setData(0, CatalogRoleIndex, idx);' in text
    assert 'item->setData(0, CatalogRolePlaceable, e.disabled_reason.trimmed().isEmpty());' in text
    assert 'item->setData(0, CatalogRoleSourcePath, e.source_path);' in text

    assert 'const int idx = it->data(0, CatalogRoleIndex).toInt();' in text
    assert 'const auto & e = asset_catalog_entries_[idx];' in text
    assert 'can_add = item->data(0, CatalogRolePlaceable).toBool();' in text
    assert 'return asset_catalog_tree_->currentItem()->data(0, CatalogRoleSourcePath).toString();' in text

    assert 'const QString category = has_valid_index ? asset_catalog_entries_[idx].category : item->text(1);' in text
    assert 'if (auto * current_item = add_asset_dialog_table_->currentItem()) {' in text
    assert 'entry_index = current_item->data(CatalogRoleIndex).toInt();' in text
    assert 'const auto & e = asset_catalog_entries_[entry_index];' in text


def test_discovered_entries_are_mapped_into_ui_entry_fields():
    text = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text()
    required = [
        'ui_entry.asset_type = QString::fromStdString(entry.source_kind);',
        'ui_entry.display_name = QString::fromStdString(entry.display_name);',
        'ui_entry.role = QString::fromStdString(entry.role_hint);',
        'ui_entry.source_path = QString::fromStdString(entry.source_path);',
        'ui_entry.availability_status = QString::fromStdString(entry.availability);',
        'ui_entry.disabled_reason = QString::fromStdString(entry.reason);',
        'ui_entry.category = QString::fromStdString(entry.category);',
        'asset_catalog_entries_.push_back(ui_entry);',
    ]
    for snippet in required:
        assert snippet in text
