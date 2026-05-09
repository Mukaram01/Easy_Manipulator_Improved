from workcell_builder.workcell_builder.workcell_studio_catalog import load_workcell_studio_catalog


def test_catalog_loader_has_initial_entries() -> None:
    payload = load_workcell_studio_catalog()
    assert any(x["id"] == "ur5" for x in payload["robots"])
    assert any(x["id"] == "generic_delta_placeholder" for x in payload["robots"])
    assert any(x["id"] == "generic_cartesian_placeholder" for x in payload["robots"])
    assert any(x["id"] == "robotiq_2f_85" for x in payload["end_effectors"])
    assert any(x["id"] == "suction_basic" for x in payload["end_effectors"])
    assert any(x["id"] == "vacuum_array_placeholder" for x in payload["end_effectors"])
    assert any(x["id"] == "realsense_d435i" or x["id"] == "intel_realsense_d435i" for x in payload["sensors"])


def test_catalog_required_fields_present() -> None:
    payload = load_workcell_studio_catalog()
    for group in ("robots", "end_effectors", "sensors", "environment_assets", "tasks"):
        for item in payload[group]:
            assert item.get("id")
            assert item.get("label")
            assert item.get("family")
