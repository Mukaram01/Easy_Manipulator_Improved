"""R1.1 durable freshness, corrective action, panel ownership and HTTP regressions."""
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import urllib.error
import urllib.request

import pytest

ROOT = Path(__file__).resolve().parents[1]
CPP = ROOT / 'workcell_builder/workcell_builder/gui/mainwindow.cpp'
spec = importlib.util.spec_from_file_location('acceptance', ROOT / 'scripts/validate_workcell_studio_generated_scene.py')
validator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(validator)


@pytest.fixture
def canonical(tmp_path):
    # Committed scene only: excludes local user edits and every ignored cache.
    archive = subprocess.check_output(['git', 'archive', 'HEAD', 'scenes/ur5_2f_test'], cwd=ROOT)
    with tarfile.open(fileobj=io.BytesIO(archive)) as files:
        files.extractall(tmp_path)
    scene = tmp_path / 'scenes/ur5_2f_test'
    # Model a valid accepted snapshot of the canonical authored files. Main's
    # historical acceptance predates later real scene changes (tested separately).
    receipt = scene / 'acceptance/generated_scene_acceptance.json'
    report = json.loads(receipt.read_text())
    report['authored_input_fingerprint'] = validator.authored_input_fingerprint(scene)
    receipt.write_text(json.dumps(report))
    return scene


def test_committed_scene_without_merge_cache_is_current(canonical):
    assert not (canonical / 'generated/workcell_studio_layout_merge_report.json').exists()
    report = validator.validate(canonical)
    assert report['status'] == 'PASS', report['blockers']
    assert not report['layout_stale']


def test_same_content_newer_mtime_is_current(canonical):
    layout = canonical / 'layout/workcell_studio_layout.yaml'
    os.utime(layout, (2000000000, 2000000000))
    assert validator.validate(canonical)['status'] == 'PASS'


def test_authored_change_requires_generation_then_validation(canonical):
    layout = canonical / 'layout/workcell_studio_layout.yaml'
    layout.write_text(layout.read_text().replace('Industrial Workbench', 'Edited Workbench'))
    first = validator.validate(canonical)
    assert first['status'] == 'BLOCKED'
    assert first['layout_stale']
    # Repeated Validate cannot bless content never regenerated.
    assert validator.validate(canonical)['status'] == 'BLOCKED'
    receipt = canonical / 'acceptance/generation_fingerprint.json'
    receipt.write_text(json.dumps({'authored_input_fingerprint': validator.authored_input_fingerprint(canonical)}))
    assert validator.validate(canonical)['status'] == 'PASS'


def body(name, following):
    return CPP.read_text().split(name, 1)[1].split(following, 1)[0]


def test_shared_gate_and_enabled_primary():
    steps = body('MainWindow::scene_workflow_steps() const', 'MainWindow::scene_workflow_status')
    assert 'const auto readiness = selected_scene_readiness();' in steps
    assert 'const bool fake_hardware_ready = readiness.ready;' in steps
    assert 'const bool validation_gate_ready = readiness.validation_current;' in steps
    preview = body('bool MainWindow::selected_scene_preview_ready(', 'bool MainWindow::preview_command_is_safe')
    assert 'selected_scene_readiness()' in preview
    assert 'last_write_time' not in preview
    rail = body('void MainWindow::refresh_scene_workflow_rail()', 'void MainWindow::refresh_run_next_menu')
    assert 'return action.enabled;' in rail
    assert 'recommendations.front()' not in rail
    assert 'Plan / Simulate blocked:' in rail
    assert 'readiness.blockers.join' in rail
    actions = body('MainWindow::resolve_recommended_workflow_actions() const', 'void MainWindow::trigger_recommended')
    assert actions.index('!readiness.generation_current') < actions.index('!readiness.validation_current')
    assert '"generate_scene_package", "Generate scene package", true' in actions
    assert '"validate_scene", "Validate scene", true' in actions


def test_task_checks_ownership_and_geometry():
    text = CPP.read_text()
    assert 'readiness_tab_layout->insertWidget(0, workflow_card)' in text
    assert 'workflow_tab_layout->addWidget(workflow_card)' not in text
    assert 'workflow_tab_layout->addWidget(ar_card)' in text
    assert 'setup_checklist_contents_layout->addWidget(readiness_label_)' in text
    assert 'current localization adapter does not provide confidence' in text
    assert 'sb->setReadOnly(locked || !editable_dimensions)' in text
    task = body('void MainWindow::refresh_task_intent_panel()', 'void MainWindow::generate_or_update_task_intent')
    assert 'Source task file:' not in task
    assert 'Target class:' in task and 'Release:' in task


def test_optional_http_404_and_required_mesh_failure(tmp_path):
    process = subprocess.Popen([sys.executable, str(ROOT / 'scripts/workcell_product_view_server.py'),
        '--directory', str(tmp_path), '--session', 'r11'], stdin=subprocess.PIPE,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        handshake = json.loads(process.stdout.readline())
        for path, expected in [('/favicon.ico', 'optional browser resource'),
                               ('/.well-known/appspecific/com.chrome.devtools.json', 'optional browser resource'),
                               ('/meshes/required.stl', 'ERROR required resource'),
                               ('/scene.json', 'ERROR required resource')]:
            with pytest.raises(urllib.error.HTTPError) as error:
                urllib.request.urlopen(f"http://127.0.0.1:{handshake['port']}{path}", timeout=5)
            assert error.value.code == 404
            diagnostic = process.stderr.readline()
            assert expected in diagnostic and path in diagnostic and '404' in diagnostic
            if 'optional' in expected:
                assert 'ERROR' not in diagnostic and 'failed' not in diagnostic
    finally:
        process.stdin.close()
        process.wait(timeout=5)


def test_production_qt_readiness_actions_and_panels(canonical, tmp_path):
    """Exercise compiled production members, with access checks disabled only for this probe."""
    import shlex
    build = Path(os.environ.get('WORKCELL_BUILD_DIR', ROOT.parents[1] / 'build/workcell_builder'))
    target = build / 'CMakeFiles/workcell_builder.dir'
    if not (target / 'link.txt').exists():
        pytest.skip('Build workcell_builder with colcon first to run the production Qt probe')
    flags = {}
    for line in (target / 'flags.make').read_text().splitlines():
        if ' = ' in line:
            key, value = line.split(' = ', 1)
            flags[key] = shlex.split(value)
    obj = tmp_path / 'probe.o'
    binary = tmp_path / 'probe'
    subprocess.run(['c++', *flags['CXX_DEFINES'], *flags['CXX_INCLUDES'], *flags['CXX_FLAGS'],
        '-fno-access-control', '-c', str(ROOT / 'tests/cpp/r11_readiness_probe.cpp'), '-o', str(obj)], check=True)
    link = shlex.split((target / 'link.txt').read_text())
    link = [str(obj) if arg.endswith('/gui/main.cpp.o') else arg for arg in link]
    link[link.index('-o') + 1] = str(binary)
    subprocess.run(link, cwd=build, check=True, capture_output=True)
    env = dict(os.environ, QT_QPA_PLATFORM='offscreen', QTWEBENGINE_DISABLE_SANDBOX='1')
    run = subprocess.run([str(binary), str(canonical), str(ROOT.parents[1])],
        env=env, capture_output=True, text=True, timeout=60)
    assert run.returncode == 0, run.stdout + run.stderr
    assert 'R1.1 Qt readiness, actions, ownership, HTTP: PASS' in run.stdout


def test_real_generator_writes_durable_receipt(canonical):
    layout = canonical / 'layout/workcell_studio_layout.yaml'
    layout.write_text(layout.read_text() + '\n# generation regression\n')
    run = subprocess.run([sys.executable, str(ROOT / 'scripts/generate_workcell_from_cell_definition.py'),
        str(canonical / 'cell_definition.yaml'), '--output-dir', str(canonical.parent),
        '--package-name', canonical.name, '--existing-package-dir', str(canonical),
        '--workspace-root', str(ROOT.parents[1]), '--force'], capture_output=True, text=True, timeout=120)
    assert run.returncode == 0, run.stdout + run.stderr
    receipt = json.loads((canonical / 'acceptance/generation_fingerprint.json').read_text())
    assert receipt['authored_input_fingerprint'] == validator.authored_input_fingerprint(canonical)
    assert validator.validate(canonical)['status'] == 'PASS'
