from pathlib import Path

FIELDS = ['x_min','x_max','y_min','y_max','z_min','z_max','remove_table_plane','remove_floor','voxel_leaf_size','min_cluster_size','max_cluster_size','grasp_strategy','approach_distance','retreat_distance','approach_axis','orientation_mode','target_id','target_type']

def test_editor_field_expectations_are_declared():
    src = Path('workcell_builder/workcell_builder/gui/main.cpp').read_text(encoding='utf-8').lower()
    for f in FIELDS:
        assert f.replace('_',' ') in src or f in src
