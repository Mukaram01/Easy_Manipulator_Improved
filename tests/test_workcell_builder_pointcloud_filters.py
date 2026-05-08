from tools.workcell_studio_streamlit import backend


def test_pointcloud_filter_fields_exist_and_export(tmp_path):
    state = backend.default_manual_authoring_state("filters")
    state["pick_sources"][0]["filters"]["voxel_leaf_size"] = 0.01
    backend.export_manual_authoring_bundle(state, tmp_path)
    payload = backend.load_environment_layout(tmp_path / "cell_definition.yaml")
    filters = payload["pick_sources"][0]["filters"]
    assert filters["remove_table_plane"] is True
    assert filters["voxel_leaf_size"] == 0.01
