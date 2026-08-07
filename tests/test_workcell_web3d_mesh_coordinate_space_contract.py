import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
VIEWER = ROOT / "workcell_studio_web" / "viewer"
SOURCE = (VIEWER / "viewer.js").read_text(encoding="utf-8")
FIXTURE = ROOT / "tests" / "fixtures" / "web3d_mesh_coordinate_space_cases.json"


def test_mesh_bounds_coordinate_space_contract_is_documented_and_used():
    assert "const MESH_BOUNDS_COORDINATE_SPACE_CONTRACT" in SOURCE
    assert "mesh_local_before_visual_origin_or_world_transform" in SOURCE
    assert "authored_visual_local_after_unit_correction" in SOURCE
    assert "scene_world_after_visual_origin_and_item_pose" in SOURCE
    assert "function measureLoadedMeshBoundsInAuthoredLocalSpace" in SOURCE
    assert "const rawAuthoredLocalBounds = measureLoadedMeshBoundsInAuthoredLocalSpace(visualRoot);" in SOURCE
    assert "maybeApplyMeshUnitAutoscale(item, meshObject, visualRoot, rawAuthoredLocalBounds, uri);" in SOURCE
    assert "const validationLocalBounds = measureLoadedMeshBoundsInAuthoredLocalSpace(visualRoot);" in SOURCE
    assert "diagnoseLoadedMeshBounds(item, visualRoot, rendered, validationLocalBounds)" in SOURCE
    assert "const nativeBounds = new THREE.Box3().setFromObject(visualRoot);" not in SOURCE
    assert "loaded_mesh_local_bounds" in SOURCE
    assert "loaded_mesh_world_bounds" in SOURCE
    assert "MESH_OVERSIZED_RATIO_THRESHOLD = 3" in SOURCE


def test_coordinate_space_regression_fixture_covers_canonical_rotation_units_and_failure(tmp_path):
    payload = json.loads(FIXTURE.read_text(encoding="utf-8"))
    assert payload["schema_version"] == "workcell_web3d_mesh_coordinate_space_fixture/v1"
    case_ids = {case["id"] for case in payload["cases"]}
    assert {
        "ur5_2f_test_realsense_overhead_rotated_visual",
        "rotated_authored_object",
        "millimetre_authored_object",
        "genuine_oversized_object",
    } <= case_ids

    script = tmp_path / "mesh_coordinate_space_contract.mjs"
    script.write_text(r'''
import fs from 'node:fs';
import vm from 'node:vm';
import assert from 'node:assert/strict';
import { pathToFileURL } from 'node:url';

const viewerPath = process.argv[2];
const fixturePath = process.argv[3];
const threePath = process.argv[4];
const injectedThree = await import(pathToFileURL(threePath).href);
let source = fs.readFileSync(viewerPath, 'utf8').replace(/boot\(\);\s*$/, '');
const fixture = JSON.parse(fs.readFileSync(fixturePath, 'utf8'));
const events = [];
const warnings = [];
const element = () => ({
  hidden: false, checked: false, disabled: false, textContent: '', className: '',
  innerHTML: '', value: '', style: {}, dataset: {},
  classList: { toggle() {}, add() {}, remove() {} },
  querySelector() { return null; }, querySelectorAll() { return []; },
  appendChild() {}, addEventListener() {}, setAttribute() {},
  getBoundingClientRect() { return { left: 0, top: 0, width: 800, height: 600 }; },
});
const context = {
  console,
  assert,
  injectedThree,
  fixture,
  events,
  warnings,
  window: {
    location: { search: '' },
    dispatchEvent(event) { events.push(event?.detail || null); },
    parent: { postMessage() {} },
    addEventListener() {},
  },
  document: {
    getElementById() { return element(); },
    createElement() { return element(); },
    querySelectorAll() { return []; },
    addEventListener() {},
    body: element(),
  },
  URL,
  URLSearchParams,
  CustomEvent: function CustomEvent(type, init) { return { type, detail: init?.detail || {} }; },
  requestAnimationFrame() { return 0; },
  cancelAnimationFrame() {},
  setTimeout() { return 1; },
  clearTimeout() {},
  performance: { now() { return 0; } },
};
vm.createContext(context);
vm.runInContext(source + `
THREE = injectedThree;
appendViewerDiagnosticWarning = (...args) => warnings.push(args);
appendRuntimeWarning = (...args) => warnings.push(args);
updateViewerStatus = () => ({ readiness_failure: state.web3dReadiness?.failure || null });

const close = (actual, expected, tolerance = 1e-8) => Math.abs(actual - expected) <= tolerance;
const d435Comparison = meshDimensionComparison({ x: 0.08, y: 0.08, z: 0.06 }, { x: 0.08991429954767227, y: 0.025054700672626502, z: 0.025000000372529037 });
assert(d435Comparison.uniformRatio > MESH_OVERSIZED_RATIO_THRESHOLD);
assert(d435Comparison.maxRatio < MESH_OVERSIZED_RATIO_THRESHOLD);
assert.strictEqual(d435Comparison.oversized, false);
const dimensions = box => {
  const size = new THREE.Vector3();
  box.getSize(size);
  return [size.x, size.y, size.z];
};
const makeCase = testCase => {
  const item = JSON.parse(JSON.stringify(testCase.item));
  const root = new THREE.Group();
  const rpy = testCase.visual_rotation_rpy || [0, 0, 0];
  root.rotation.set(rpy[0], rpy[1], rpy[2], 'XYZ');
  const geometry = new THREE.BoxGeometry(...testCase.geometry_dimensions);
  const mesh = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial());
  root.add(mesh);
  const object3d = new THREE.Group();
  object3d.add(root);
  return { item, root, mesh, rendered: { item, object3d } };
};

for (const testCase of fixture.cases.filter(value => value.expected_outcome === 'valid' && !value.expected_unit_scale)) {
  const { item, root, rendered } = makeCase(testCase);
  const local = measureLoadedMeshBoundsInAuthoredLocalSpace(root);
  const world = measureLoadedMeshBoundsInWorldSpace(root);
  const localDims = dimensions(local);
  for (let index = 0; index < 3; index += 1) assert(close(localDims[index], testCase.item.expected_dimensions_m[index]), testCase.id + ': local dimension mismatch');
  if (testCase.visual_rotation_rpy.some(value => Math.abs(value) > 1e-8)) {
    const worldDims = dimensions(world);
    assert(worldDims.some((value, index) => !close(value, localDims[index], 1e-6)), testCase.id + ': fixture must demonstrate world-axis rotation effects');
  }
  assert.strictEqual(diagnoseLoadedMeshBounds(item, root, rendered, local), true, testCase.id);
  assert.notStrictEqual(item.visual_bounds_status, 'oversized', testCase.id);
  assert.deepStrictEqual(item.loaded_mesh_bounds_coordinate_space, MESH_BOUNDS_COORDINATE_SPACE_CONTRACT);
}

const millimetreCase = fixture.cases.find(value => value.id === 'millimetre_authored_object');
const millimetre = makeCase(millimetreCase);
const millimetreRaw = measureLoadedMeshBoundsInAuthoredLocalSpace(millimetre.root);
assert.strictEqual(maybeApplyMeshUnitAutoscale(millimetre.item, millimetre.mesh, millimetre.root, millimetreRaw, 'millimetre_fixture.stl'), true);
assert(close(millimetre.mesh.scale.x, millimetreCase.expected_unit_scale));
const corrected = measureLoadedMeshBoundsInAuthoredLocalSpace(millimetre.root);
for (const [index, value] of dimensions(corrected).entries()) assert(close(value, millimetreCase.item.expected_dimensions_m[index], 1e-7));
const scaleAfterFirstCorrection = millimetre.mesh.scale.x;
assert.strictEqual(maybeApplyMeshUnitAutoscale(millimetre.item, millimetre.mesh, millimetre.root, corrected, 'millimetre_fixture.stl'), false, 'unit correction is applied only once');
assert(close(millimetre.mesh.scale.x, scaleAfterFirstCorrection));
assert.strictEqual(diagnoseLoadedMeshBounds(millimetre.item, millimetre.root, millimetre.rendered, corrected), true);
assert.strictEqual(millimetre.item.mesh_unit_correction.applied_once_at, 'loaded_mesh_object.scale');

const oversizedCase = fixture.cases.find(value => value.id === 'genuine_oversized_object');
const oversized = makeCase(oversizedCase);
const oversizedLocal = measureLoadedMeshBoundsInAuthoredLocalSpace(oversized.root);
assert.strictEqual(diagnoseLoadedMeshBounds(oversized.item, oversized.root, oversized.rendered, oversizedLocal), false);
assert.strictEqual(oversized.item.visual_bounds_status, 'oversized');
assert(oversized.item.loaded_mesh_maximum_ratio > MESH_OVERSIZED_RATIO_THRESHOLD);

state.sceneJson = { scene: { id: 'mesh_coordinate_space_fixture' }, objects: [oversized.item] };
state.sourceWebSceneFile = 'mesh_coordinate_space_fixture.web_scene.json';
state.builderRevision = 'fixture-v1';
state.objects = [oversized.rendered];
state.web3dReadiness = {
  state: 'scene_loading', terminal: false, terminalState: '', terminalNavigationKey: '',
  terminalEmissionCount: 0, statusSequence: 0,
  required: { robot_arm: true, attached_tool_gripper: true, workbench_support_surface: true, configured_camera: true },
  pending: new Set(), failed: false, failure: null,
};
physicalLoadToken += 1;
const operation = registerReadinessOperation(['authored_physical_mesh:oversized_fixture']);
const attempt = physicalMeshAttempt(oversized.item, operation);
assert.strictEqual(failPhysicalMeshAttempt(
  attempt,
  oversized.item,
  '/assets/oversized_fixture.stl',
  'loaded mesh bounds validation failed (oversized)',
  { mesh_status: 'loaded', ...physicalMeshBoundsFailurePayload(oversized.item, '/assets/oversized_fixture.stl', 'STLLoader') },
), true);
assert.strictEqual(state.web3dReadiness.state, 'scene_failed');
assert.strictEqual(state.web3dReadiness.failure.item_id, 'oversized_fixture');
assert(state.web3dReadiness.failure.maximum_ratio > 3);
assert.strictEqual(state.web3dReadiness.failure.loaded_local_bounds_coordinate_space, 'authored_visual_local_after_unit_correction');
assert(events.some(event => event?.state === 'scene_failed'));
`, context);
''', encoding="utf-8")

    subprocess.run(
        [
            "node",
            str(script),
            str(VIEWER / "viewer.js"),
            str(FIXTURE),
            str(VIEWER / "node_modules" / "three" / "build" / "three.module.js"),
        ],
        cwd=ROOT,
        check=True,
        text=True,
    )
