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
    assert "import '../urdf_robot_renderer.js';" in entry


def test_rviz_light_baseline_uses_light_background_grid_lighting_and_shadows():
    source = BASELINE.read_text(encoding="utf-8")

    for token in [
        "background: 0xf2f4f6",
        "ground: 0xf7f8fa",
        "renderer.outputColorSpace = THREE.SRGBColorSpace",
        "renderer.toneMapping = THREE.ACESFilmicToneMapping",
        "renderer.shadowMap.enabled = true",
        "renderer.shadowMap.type = THREE.PCFSoftShadowMap",
        "rviz_light_ground",
        "rviz_light_key",
        "rviz_light_fill",
        "node.castShadow = !translucent",
        "node.receiveShadow = !translucent",
        "node.position.z = 0.002",
        "material.opacity = 0.72",
        "installRvizLightBaseline();",
    ]:
        assert token in source


def test_generated_bundle_contains_the_rviz_light_baseline():
    bundle = BUNDLE.read_text(encoding="utf-8")

    for token in [
        "__WORKCELL_RVIZ_LIGHT_BASELINE__",
        "rviz_light_ground",
        "rviz_light_key",
        "rviz_light_fill",
        "workcell-studio.rviz-light-baseline.v1",
    ]:
        assert token in bundle
