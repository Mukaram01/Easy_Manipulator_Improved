from pathlib import Path

CPP = Path('workcell_builder/workcell_builder/gui/mainwindow.cpp').read_text(encoding='utf-8')


def test_more_actions_and_menu_components_exist():
    for token in ['More Actions', 'QToolButton', 'QMenu', 'setPopupMode(QToolButton::InstantPopup)']:
        assert token in CPP


def test_primary_actions_visible():
    for token in [
        'Add to Canvas',
        'Save Layout',
        'Generate/Update Task Intent',
        'Run Fake-Hardware Simulation',
        'Copy Launch Command',
        'Run Offline Validation',
        'Generate Readiness Pack',
        'Export Scene Bundle',
        'Import Scene Bundle',
    ]:
        assert token in CPP


def test_advanced_actions_present_in_menus_and_not_wired_not_used():
    for token in [
        'Remove Selected Layout Item',
        'Open Validation Report',
        'Copy Validation Summary',
        'Open Readiness Dashboard',
        'Open Scene Folder',
        'Open Preview Report',
        'Open Dashboard',
        'Copy Source Command',
        'Copy Build Command',
        'Open Export Folder',
        'Copy Bundle Summary',
        'Open Bundle Docs',
    ]:
        assert token in CPP
    assert 'show_not_wired_message("Run Offline Validation")' not in CPP


def test_mode_and_safety_chips_present():
    for token in [
        'Design',
        'Plan',
        'Plan',
        'Simulate',
        'fake-hardware',
        'Real robot locked',
    ]:
        assert token in CPP


def test_secondary_controls_moved_to_menus_and_theme_tokens_present():
    for token in [
        'Camera / View',
        'Canvas More',
        'Snap/Grid settings',
        'More Actions',
        'QLabel#studioHomeSafetyPill',
        'QToolButton',
        'QMenu',
    ]:
        assert token in CPP or token in Path('workcell_builder/workcell_builder/gui/resources/workcell_studio_dark.qss').read_text(encoding='utf-8')


def test_redundant_workspace_change_controls_removed_from_studio_pages():
    assert 'Change Workspace' not in CPP
    assert 'Workcell Studio change workspace' not in CPP


def test_legacy_startup_header_rows_removed_from_studio_shell():
    banned = [
        'Mode: Design | Runtime: Disabled | Hardware: Fake by default | Safety: Guarded',
        'Workcell loaded (using selected path/src as workcell root)',
        'studio_workspace_path_ = new QLineEdit',
        'ROS 2 humble',
        'Workspace:',
    ]
    for token in banned:
        assert token not in CPP
    assert 'workspaceChip' not in CPP


def test_dashboard_has_named_widgets_and_readable_qss_palette():
    for token in [
        'setObjectName("workcellStudioDashboardPage")',
        'setObjectName("dashboardTitleLabel")',
        'setObjectName("dashboardSummaryLabel")',
        'setObjectName("studioHomeSceneTable")',
        'Studio Home',
    ]:
        assert token in CPP

    qss = Path('workcell_builder/workcell_builder/gui/resources/workcell_studio_dark.qss').read_text(encoding='utf-8')
    for color in ['#f8fafc', '#cbd5e1', '#94a3b8']:
        assert color in qss
    lowered_color_lines = [line.strip().lower() for line in qss.splitlines() if line.strip().lower().startswith('color:')]
    for banned in ['color: black', 'color: #000', 'color: #0b1118', 'color: #111827', 'color: #0f172a']:
        assert banned not in lowered_color_lines


def test_dark_qss_has_readable_explicit_foreground_rules_and_no_banned_dark_text_colors():
    qss = Path('workcell_builder/workcell_builder/gui/resources/workcell_studio_dark.qss').read_text(encoding='utf-8')
    for token in [
        'QLabel { color: #f8fafc;',
        'QPushButton',
        'QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox',
        'QTreeWidget, QTableWidget',
        'QTextEdit, QPlainTextEdit',
        ':disabled',
    ]:
        assert token in qss
    lowered_color_lines = [line.strip().lower() for line in qss.splitlines() if line.strip().lower().startswith('color:')]
    for banned in ['color: #111827', 'color: #0f172a', 'color: black', 'color: #000']:
        assert banned not in lowered_color_lines


def test_studio_home_scene_management_and_safe_delete_tokens_present():
    assert ('Manage scenes, review readiness, and open the selected workcell.' in CPP or
            'Design, validate, and preview robotic workcells.' in CPP)
    for token in [
        'Open in Scene Builder',
        'Validate',
        'Plan & Simulate',
        'Export',
        'Delete Scene',
        '.workcell_studio_trash',
        'Selected path is outside scenes root.',
        'Move scene',
        'refresh_scene_browser_ui();',
    ]:
        assert token in CPP
    assert 'rm -rf' not in CPP
    assert 'show_not_wired_message("Delete Scene")' not in CPP


def test_studio_home_landing_page_tokens_present():
    for token in [
        'Workcell Studio',
        'Design, validate, and preview robotic workcells.',
        'Fake hardware default / Real robot locked',
        'Total Scenes',
        'Ready / Validated',
        'Warnings / Blocked',
        'Search scenes...',
        'No scenes found',
        'Create a new cell to get started.',
        'Selected Scene',
        'Open Selected Scene',
    ]:
        assert token in CPP
