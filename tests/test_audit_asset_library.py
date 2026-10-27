import json
import struct
from pathlib import Path

from scripts import audit_asset_library as audit

CUBE = [  # outward (counter-clockwise from outside) unit cube
    ((0, 0, 0), (0, 1, 0), (1, 1, 0)), ((0, 0, 0), (1, 1, 0), (1, 0, 0)),
    ((0, 0, 1), (1, 0, 1), (1, 1, 1)), ((0, 0, 1), (1, 1, 1), (0, 1, 1)),
    ((0, 0, 0), (1, 0, 0), (1, 0, 1)), ((0, 0, 0), (1, 0, 1), (0, 0, 1)),
    ((0, 1, 0), (0, 1, 1), (1, 1, 1)), ((0, 1, 0), (1, 1, 1), (1, 1, 0)),
    ((0, 0, 0), (0, 0, 1), (0, 1, 1)), ((0, 0, 0), (0, 1, 1), (0, 1, 0)),
    ((1, 0, 0), (1, 1, 0), (1, 1, 1)), ((1, 0, 0), (1, 1, 1), (1, 0, 1)),
]


def write_binary_stl(path: Path, triangles):
    path.parent.mkdir(parents=True, exist_ok=True)
    data = bytearray(b"\0" * 80) + struct.pack("<I", len(triangles))
    for a, b, c in triangles:
        data += struct.pack("<3f", 0, 0, 0) + struct.pack("<9f", *a, *b, *c) + b"\0\0"
    path.write_bytes(bytes(data))


def test_geometry_checks_flag_stub_and_inside_out_but_not_a_clean_cube(tmp_path):
    write_binary_stl(tmp_path / "stub.stl", [((0, 0, 0), (1, 0, 0), (0, 1, 0))])
    write_binary_stl(tmp_path / "inside_out.stl", [(a, c, b) for a, b, c in CUBE])
    write_binary_stl(tmp_path / "cube.stl", CUBE)

    stub = audit.inspect_mesh(tmp_path / "stub.stl")
    assert stub["triangles"] == 1 and stub["closed"] is False
    assert any("flat mesh" in p for p in stub["problems"])

    inside_out = audit.inspect_mesh(tmp_path / "inside_out.stl")
    assert any("inside-out" in p for p in inside_out["problems"])

    cube = audit.inspect_mesh(tmp_path / "cube.stl")
    assert cube["problems"] == [] and cube["closed"] is True and cube["signed_volume"] > 0


def test_library_port_withholds_marked_packages_and_merges_unpackaged_mesh_and_xacro(tmp_path):
    repo = tmp_path
    stand = repo / "assets/environment/pipe_camera_stand_description"
    (stand).mkdir(parents=True)
    (stand / "package.xml").write_text(
        '<package><name>pipe_camera_stand_description</name><export>'
        '<workcell_asset_library status="replace" reason="generic box"/></export></package>')
    write_binary_stl(stand / "meshes/visual/pipe_camera_stand.stl", CUBE)
    tray = repo / "workcell_builder/workcell_builder/assets/environment/tray_description"
    write_binary_stl(tray / "meshes/tray.stl", CUBE)
    (tray / "urdf").mkdir(parents=True)
    (tray / "urdf/tray.urdf.xacro").write_text('<robot name="tray"/>')

    withheld = {}
    entries = audit.gui_catalog(repo, withheld)
    assert withheld == {"pipe_camera_stand_description": "generic box"}
    assert [e["id"] for e in entries] == ["tray"]


def test_inventory_cli_writes_report(tmp_path):
    repo = tmp_path / "repo"
    write_binary_stl(repo / "assets/objects/cube_description/meshes/cube.stl", CUBE)
    out = tmp_path / "report.json"
    assert audit.main(["--repo-root", str(repo), "--json", str(out)]) == 0
    report = json.loads(out.read_text())
    assert report["mesh_files"][0]["path"] == "assets/objects/cube_description/meshes/cube.stl"
    assert report["mesh_files"][0]["problems"] == []
