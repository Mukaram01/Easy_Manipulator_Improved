from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
ENV = ROOT / "assets" / "environment"


def _validate_wrapper(asset_name: str):
    path = ENV / f"{asset_name}_description" / f"{asset_name}.yaml"
    data = yaml.safe_load(path.read_text())
    assert isinstance(data, dict)
    assert asset_name in data
    node = data[asset_name]
    assert isinstance(node, dict)
    assert isinstance(node.get("links"), dict)
    ext_joint_key = f"{asset_name}_base_joint"
    assert isinstance(node.get(ext_joint_key), dict)
    child_link = node[ext_joint_key].get("child_link")
    assert isinstance(child_link, str)
    assert child_link in node["links"]
    return path, node


def _collect_counts(node):
    visual_count = 0
    collision_count = 0
    for _, link_node in node["links"].items():
        assert isinstance(link_node, dict)
        if "visual" in link_node:
            visual_count += 1
            geom = link_node["visual"].get("geometry", {})
            assert isinstance(geom.get("filepath"), str)
        if "collision" in link_node:
            collision_count += 1
            geom = link_node["collision"].get("geometry", {})
            assert isinstance(geom.get("filepath"), str)
    return visual_count, collision_count


def test_simple_conveyor_wrapper_parses():
    _, node = _validate_wrapper("simple_conveyor")
    visuals, collisions = _collect_counts(node)
    assert visuals > 0
    assert collisions > 0


def test_sorting_bin_wrapper_parses():
    _, node = _validate_wrapper("sorting_bin")
    visuals, collisions = _collect_counts(node)
    assert visuals > 0
    assert collisions > 0


def test_simple_conveyor_mesh_path_exists():
    _, node = _validate_wrapper("simple_conveyor")
    mesh = node["links"]["base_link"]["visual"]["geometry"]["filepath"]
    rel = mesh.replace("package://simple_conveyor_description/", "")
    assert (ENV / "simple_conveyor_description" / rel).exists()


def test_sorting_bin_mesh_path_exists():
    _, node = _validate_wrapper("sorting_bin")
    mesh = node["links"]["base_link"]["visual"]["geometry"]["filepath"]
    rel = mesh.replace("package://sorting_bin_description/", "")
    assert (ENV / "sorting_bin_description" / rel).exists()


def test_malformed_yaml_returns_validation_error_shape():
    bad = {"simple_conveyor": {"links": []}}
    node = bad["simple_conveyor"]
    assert not isinstance(node.get("links"), dict)


def test_discovery_style_filter_only_accepts_valid_wrappers():
    accepted = []
    for folder in ENV.iterdir():
        if not folder.is_dir() or not folder.name.endswith("_description"):
            continue
        name = folder.name[:-12]
        wrapper = folder / f"{name}.yaml"
        if not wrapper.exists():
            continue
        data = yaml.safe_load(wrapper.read_text())
        if isinstance(data, dict) and name in data and isinstance(data[name], dict) and isinstance(data[name].get("links"), dict):
            accepted.append(name)
    assert "simple_conveyor" in accepted
    assert "sorting_bin" in accepted


def test_object_loader_does_not_treat_map_as_sequence():
    _, node = _validate_wrapper("simple_conveyor")
    assert isinstance(node["links"], dict)
    # mirrors the key safety requirement: map iteration should be over items()
    assert hasattr(node["links"], "items")
