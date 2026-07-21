from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
VIEWER = ROOT / "workcell_studio_web/viewer"
INDEX = VIEWER / "index.html"
BASELINE = VIEWER / "rviz_light_baseline.js"


def test_rviz_light_baseline_loads_after_the_pinned_viewer_bundle():
    index = INDEX.read_text(encoding="utf-8")
    bundle_script = 'src="./dist/viewer.bundle.js"'
    baseline_script = 'src="./rviz_light_baseline.js"'
    assert bundle_script in index
    assert baseline_script in index
    assert index.index(bundle_script) < index.index(baseline_script)


def test_rviz_light_baseline_is_light_clear_and_shadow_enabled():
    source = BASELINE.read_text(encoding="utf-8")
    for token in [
        "background: 0xf2f4f6",
        "renderer.outputColorSpace = THREE.SRGBColorSpace",
        "renderer.toneMapping = THREE.ACESFilmicToneMapping",
        "renderer.shadowMap.enabled = true",
        "renderer.shadowMap.type = THREE.PCFSoftShadowMap",
        "rviz_light_hemisphere",
        "rviz_light_key",
        "gridMajor: 0x8996a3",
        "gridMinor: 0xc8d0d8",
        "node.castShadow = shadows",
        "node.receiveShadow = shadows",
        "workcell-studio.rviz-light-baseline.v1",
        "__WORKCELL_RVIZ_LIGHT_BASELINE__",
    ]:
        assert token in source


def test_rviz_light_baseline_reuses_existing_scene_geometry():
    source = BASELINE.read_text(encoding="utf-8")
    for unwanted in [
        "new THREE.PlaneGeometry",
        "new THREE.MeshStandardMaterial",
        "new THREE.DirectionalLight",
        "new THREE.HemisphereLight",
    ]:
        assert unwanted not in source
