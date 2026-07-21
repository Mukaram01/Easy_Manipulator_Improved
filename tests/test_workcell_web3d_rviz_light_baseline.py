from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ENTRY = ROOT / "workcell_studio_web/viewer/src/viewer_entry.js"
BASELINE = ROOT / "workcell_studio_web/viewer/src/rviz_light_baseline.js"
BUNDLE = ROOT / "workcell_studio_web/viewer/dist/viewer.bundle.js"


def test_rviz_light_baseline_loads_before_viewer_startup():
    entry = ENTRY.read_text(encoding="utf-8")
    assert "import './rviz_light_baseline.js';" in entry
    assert entry.index("import './rviz_light_baseline.js';") < entry.index(
        "import '../viewer.js';"
    )


def test_rviz_light_baseline_reuses_existing_scene_helpers_and_lights():
    source = BASELINE.read_text(encoding="utf-8")
    for token in [
        "background: 0xf2f4f6",
        "renderer.outputColorSpace = THREE.SRGBColorSpace",
        "renderer.toneMapping = THREE.ACESFilmicToneMapping",
        "renderer.shadowMap.enabled = true",
        "renderer.shadowMap.type = THREE.PCFSoftShadowMap",
        "rviz_light_hemisphere",
        "rviz_light_key",
        "child?.isGridHelper",
        "child.position.z = 0.002",
        "material.opacity = 0.72",
        "node.castShadow = shadows",
        "node.receiveShadow = shadows",
        "installRvizLightBaseline();",
    ]:
        assert token in source

    for unwanted in ["new THREE.PlaneGeometry", "new THREE.DirectionalLight"]:
        assert unwanted not in source


def test_generated_bundle_contains_the_rviz_light_baseline():
    bundle = BUNDLE.read_text(encoding="utf-8")
    for token in [
        "__WORKCELL_RVIZ_LIGHT_BASELINE__",
        "rviz_light_hemisphere",
        "rviz_light_key",
        "workcell-studio.rviz-light-baseline.v1",
    ]:
        assert token in bundle
