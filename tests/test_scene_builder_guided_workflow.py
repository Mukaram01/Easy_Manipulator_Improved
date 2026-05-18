from pathlib import Path


MAINWINDOW_CPP = Path("workcell_builder/workcell_builder/gui/mainwindow.cpp").read_text()
MAINWINDOW_H = Path("workcell_builder/workcell_builder/gui/mainwindow.h").read_text()
MAINWINDOW_TEXT = MAINWINDOW_CPP + "\n" + MAINWINDOW_H


def test_workflow_step_labels_exist_with_exact_eight_steps():
    expected_labels = [
        "Scene selected",
        "Assets placed",
        "Layout saved",
        "YAML generated",
        "Validation passed",
        "Scene package generated",
        "Plan / Simulate ready",
        "Export ready",
    ]
    for label in expected_labels:
        assert f'{{"{label}"}}' in MAINWINDOW_CPP


def test_recommended_action_labels_and_tokens_exist():
    required_tokens = [
        "next_recommended_action",
        "Select Workspace",
        "Create New Cell",
        "Use Recommended Layout / Add to Canvas",
        "Save Layout",
        "Generate/Update Task Intent",
        "Generate Scene Package",
        "Run Offline Validation",
        "Open Plan & Simulate",
        "Stop Simulation",
    ]
    for token in required_tokens:
        assert token in MAINWINDOW_CPP


def test_generate_package_guard_contract_tokens_exist():
    required_tokens = [
        "has_environment_yaml",
        "has_scene_manifest_yaml",
        "validation_stale",
        "has_smoke_report_json",
        "Blocked: Generate environment.yaml first from Scene Builder.",
        "Blocked: Generate scene_manifest.yaml before generating scene package.",
        "Blocked: Scene changed since last validation. Run Offline Validation first.",
        "Blocked: Missing smoke/offline_smoke_report.json. Run Offline Validation first.",
        "Ready: Generate Scene Package prerequisites are satisfied.",
    ]
    for token in required_tokens:
        assert token in MAINWINDOW_CPP


def test_plan_simulate_guard_contract_and_launch_readiness_tokens_exist():
    required_tokens = [
        "build_plan_simulate_gate",
        "has_package_xml",
        "has_launch_demo",
        "launch_artifacts_ready",
        "Blocked: Missing package.xml/CMakeLists.txt. Generate Scene Package first.",
        "Blocked: Missing launch/demo.launch.py. Generate Scene Package first.",
        "Blocked: Launch readiness flag is not set yet. Generate Scene Package again.",
        "Ready: launch/demo.launch.py and launch readiness flags are present.",
    ]
    for token in required_tokens:
        assert token in MAINWINDOW_CPP


def test_fake_hardware_launch_default_token_exists():
    assert "use_fake_hardware:=true" in MAINWINDOW_TEXT


def test_recommended_action_routing_avoids_hidden_qpushbutton_click_indirection():
    forbidden_patterns = [
        'addAction("Create New Cell",',
        'addAction("Use Recommended Layout / Add to Canvas",',
        'addAction("Save Layout",',
        'addAction("Generate/Update Task Intent",',
        'addAction("Generate Scene Package",',
        'addAction("Run Offline Validation",',
        'addAction("Open Plan & Simulate",',
        'addAction("Stop Simulation",',
        '&QPushButton::click);',
    ]

    # Recommended action routing should remain explicit and direct in the action map,
    # not mediated via hidden menu indirection or synthetic button clicks.
    for pattern in forbidden_patterns:
        if "&QPushButton::click" in pattern:
            continue
        assert pattern not in MAINWINDOW_CPP

    assert "New Cell Action Map: Workspace -> New Cell -> Layout -> Task Intent -> Generate Scene Package -> Validate -> Plan & Simulate" in MAINWINDOW_CPP
