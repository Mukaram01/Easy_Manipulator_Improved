from pathlib import Path
BAD=["/root/workcell_ws","~/workcell_ws","home/user/workcell_ws","cd ~/workcell_ws","cd /root/workcell_ws"]
TARGET=["run_workcell_builder_scene3d_gui_smoke.py","run_workcell_studio_scene_readiness_gate.py","validate_scene3d_runtime_acceptance.py","check_scene3d_canvas_contract.py","run_workcell_studio_local_validation.py","workcell_studio_path_resolver.py"]
def test_no_hardcoded_workspace_paths_in_runtime_scripts():
 repo=Path(__file__).resolve().parents[1]
 for name in TARGET:
  text=(repo/"scripts"/name).read_text(encoding="utf-8")
  for b in BAD: assert b not in text
