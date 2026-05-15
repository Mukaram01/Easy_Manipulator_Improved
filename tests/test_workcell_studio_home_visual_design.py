from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')
QSS = Path('workcell_builder/workcell_builder/gui/resources/workcell_studio_dark.qss').read_text(encoding='utf-8')


def test_studio_home_object_names_exist():
    for token in [
        'setObjectName("studioHomeHeroCard")',
        'setObjectName("studioHomeSummaryCard")',
        'setObjectName("studioHomeSceneTable")',
        'setObjectName("studioHomeSearchBox")',
        'setObjectName("studioHomeStatusFilter")',
        'setObjectName("studioHomeDetailsCard")',
        'setObjectName("studioHomeLog")',
        'setObjectName("studioHomeSafetyPill")',
        'setObjectName("studioHomePrimaryButton")',
        'setObjectName("studioHomeSecondaryButton")',
        'setObjectName("studioHomeDangerButton")',
    ]:
        assert token in CPP


def test_qss_styles_studio_home_objects_and_dark_table_rules():
    for token in [
        'QFrame#studioHomeHeroCard',
        'QLabel#studioHomeSummaryCard',
        'QTableWidget#studioHomeSceneTable',
        'QLineEdit#studioHomeSearchBox, QComboBox#studioHomeStatusFilter',
        'QFrame#studioHomeDetailsCard',
        'QTextEdit#studioHomeLog',
        'QLabel#studioHomeSafetyPill',
        'QHeaderView::section',
        'alternate-background-color',
        '::item:selected',
    ]:
        assert token in QSS


def test_no_obvious_dark_on_dark_text_for_studio_home_styles():
    lowered = [line.strip().lower() for line in QSS.splitlines() if line.strip().lower().startswith('color:')]
    for banned in ['color: #111827', 'color: #0f172a', 'color: black', 'color: #000']:
        assert banned not in lowered


def test_behavior_and_safety_tokens_remain_present_and_banned_tokens_absent():
    for token in [
        'Open Selected Scene',
        'Dashboard Open in Scene Builder',
        'Delete Scene',
        '.workcell_studio_trash',
        'is_safe_scene_path_for_trash_move',
        'refresh_scene_browser_ui();',
    ]:
        assert token in CPP
    for banned in ['.scenes_root', 'QPolygonF{', 'select_preview_item(item->']:
        assert banned not in CPP
