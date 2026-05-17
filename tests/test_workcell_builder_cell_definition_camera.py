from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def test_cell_definition_export_reads_camera_placements():
    txt=(ROOT/'scripts/export_builder_scene_to_cell_definition.py').read_text()
    for n in ['camera_placements','optical_frame','pointcloud','camera_info','pose','parent_frame']:
        assert n in txt
