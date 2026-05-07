from pathlib import Path

from workcell_builder.workcell_builder.workcell_asset_catalog import load_catalog, validate_catalog


def test_catalog_loads_without_ros() -> None:
    payload = load_catalog(Path.cwd())
    assert payload.get("schema_version")


def test_catalog_minimum_counts_and_required_fields() -> None:
    payload = load_catalog(Path.cwd())
    items = payload["items"]
    assert len([i for i in items if i["category"] == "robots"]) >= 20
    assert len([i for i in items if i["category"] in {"end_effectors", "tools"}]) >= 20
    assert len([i for i in items if i["category"] in {"environment", "fixtures", "conveyors", "machines", "safety", "sensors"}]) >= 30
    assert len([i for i in items if i["category"] == "objects"]) >= 15
    for item in items:
        for req in ("id", "display_name", "category", "support_status", "runtime_status"):
            assert item.get(req)


def test_required_named_assets_and_placeholders() -> None:
    payload = load_catalog(Path.cwd())
    by_id = {i["id"]: i for i in payload["items"]}
    assert by_id["ur5"]["support_status"] == "supported"
    assert by_id["robotiq_2f_85"]["id"] == "robotiq_2f_85"
    assert by_id["onrobot_airpick"]["id"] == "onrobot_airpick"
    assert by_id["onrobot_airpick4"]["id"] == "onrobot_airpick4"
    assert by_id["generic_delta_robot"]["support_status"] in {"preview_only", "placeholder"}
    assert by_id["generic_cartesian_gantry"]["support_status"] in {"preview_only", "placeholder"}
    errors = validate_catalog(payload, Path.cwd())
    assert not errors
