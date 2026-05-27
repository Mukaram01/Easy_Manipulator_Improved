#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    import yaml
except Exception:  # pragma: no cover
    yaml = None

REQUIRED_FILES = [
    "package.xml",
    "CMakeLists.txt",
    "environment.yaml",
    "cell_definition.yaml",
    "layout/workcell_studio_layout.yaml",
    "launch/demo.launch.py",
    "urdf/scene.urdf.xacro",
    "generated/scene_package_readiness.json",
]


def load_yaml(path: Path) -> dict:
    if yaml is None:
        raise RuntimeError("PyYAML is not installed; cannot parse YAML metadata.")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    return data if isinstance(data, dict) else {}


def run_shell(cmd: str, timeout_sec: int | None = None) -> tuple[int, str, str, bool]:
    p = subprocess.Popen(["bash", "-lc", cmd], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    timed_out = False
    try:
        out, err = p.communicate(timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        timed_out = True
        p.kill()
        out, err = p.communicate()
    return p.returncode, out, err, timed_out


def resolve_scene(scene_arg: str, repo_root: Path) -> Path:
    candidate = Path(scene_arg)
    if candidate.is_absolute() and candidate.exists():
        return candidate
    if candidate.exists():
        return candidate.resolve()
    return (repo_root / "scenes" / scene_arg).resolve()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", required=True)
    ap.add_argument("--workspace-root", type=Path, required=True)
    ap.add_argument("--repo-root", type=Path, required=True)
    ap.add_argument("--json", action="store_true", dest="json_out")
    ap.add_argument("--skip-build", action="store_true")
    ap.add_argument("--skip-launch-smoke", action="store_true")
    ap.add_argument("--timeout-sec", type=int, default=30)
    args = ap.parse_args()

    scene_dir = resolve_scene(args.scene, args.repo_root)
    report = {
        "scene": str(scene_dir),
        "package_name": "",
        "required_files": {},
        "package_xml_ok": False,
        "cmake_ok": False,
        "xacro_ok": False,
        "launch_contract_ok": False,
        "build": {"status": "SKIPPED", "returncode": None, "log_path": "", "timed_out": False},
        "launch_smoke": {"status": "SKIPPED", "returncode": None, "timed_out": False},
        "status": "PASS",
        "blockers": [],
        "warnings": [],
        "commands_run": [],
        "artifact_paths": {},
    }

    if not scene_dir.exists():
        report["blockers"].append(f"Scene path does not exist: {scene_dir}")
        report["status"] = "FAIL"
        print(json.dumps(report, indent=2) if args.json_out else report)
        return 2

    for rel in REQUIRED_FILES:
        p = scene_dir / rel
        ok = p.is_file()
        report["required_files"][rel] = {"exists": ok, "path": str(p)}
        if not ok:
            report["blockers"].append(f"Missing required file: {rel}")

    readiness = {}
    readiness_path = scene_dir / "generated/scene_package_readiness.json"
    if readiness_path.is_file():
        try:
            readiness = json.loads(readiness_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as e:
            report["warnings"].append(f"Could not parse readiness metadata JSON: {e}")

    folder_name = scene_dir.name
    package_name_from_meta = readiness.get("package_name") if isinstance(readiness, dict) else None

    pkg_xml = scene_dir / "package.xml"
    package_name = ""
    if pkg_xml.is_file():
        try:
            root = ET.fromstring(pkg_xml.read_text(encoding="utf-8"))
            name_node = root.find("name")
            package_name = (name_node.text or "").strip() if name_node is not None else ""
            report["package_name"] = package_name
            if not package_name:
                report["blockers"].append("package.xml missing <name> value")
            else:
                allowed = {folder_name}
                if package_name_from_meta:
                    allowed.add(str(package_name_from_meta))
                if package_name not in allowed:
                    report["blockers"].append(
                        f"package.xml <name> '{package_name}' does not match scene folder '{folder_name}'"
                        + (f" or readiness package '{package_name_from_meta}'" if package_name_from_meta else "")
                    )
                else:
                    report["package_xml_ok"] = True
        except ET.ParseError as e:
            report["blockers"].append(f"Malformed package.xml: {e}")

    cmake = scene_dir / "CMakeLists.txt"
    if cmake.is_file() and package_name:
        txt = cmake.read_text(encoding="utf-8", errors="ignore")
        report["cmake_ok"] = package_name in txt and "ament_package" in txt
        if not report["cmake_ok"]:
            report["warnings"].append("CMakeLists.txt missing obvious package wiring (name/ament_package).")

    launch_file = scene_dir / "launch/demo.launch.py"
    if launch_file.is_file():
        lt = launch_file.read_text(encoding="utf-8", errors="ignore")
        checks = {
            "use_fake_hardware": "use_fake_hardware" in lt,
            "launch_rviz": "launch_rviz" in lt,
            "robot_state_publisher": "robot_state_publisher" in lt,
            "rviz": "rviz" in lt.lower(),
            "xacro": "xacro" in lt.lower(),
        }
        missing = [k for k, ok in checks.items() if not ok]
        if missing:
            report["blockers"].append(f"Launch contract missing: {', '.join(missing)}")
        else:
            report["launch_contract_ok"] = True

    xacro = scene_dir / "urdf/scene.urdf.xacro"
    if xacro.is_file():
        xt = xacro.read_text(encoding="utf-8", errors="ignore")
        try:
            ET.fromstring(xt)
        except ET.ParseError as e:
            report["blockers"].append(f"Malformed xacro XML: {e}")

        bad_tokens = [r"\{\{", r"\}\}", r"\$\{resolver", r"TODO_RESOLVE", r"UNRESOLVED", r"REPLACE_ME"]
        hits = [pat for pat in bad_tokens if re.search(pat, xt)]
        if hits:
            report["blockers"].append(f"Unresolved placeholders detected in xacro: {', '.join(hits)}")

        for m in re.finditer(r"package://([^/\s]*)", xt):
            if not m.group(1).strip():
                report["blockers"].append("Broken package:// reference with empty package name in xacro.")

        for m in re.finditer(r'mesh\s*=\s*["\']([^"\']+)["\']', xt):
            mesh_path = m.group(1)
            if mesh_path.startswith(("package://", "http://", "https://")):
                continue
            if not (scene_dir / mesh_path).exists() and not (scene_dir / "urdf" / mesh_path).exists():
                report["warnings"].append(f"Local mesh path not found: {mesh_path}")

        cell = scene_dir / "cell_definition.yaml"
        if cell.is_file() and yaml is not None:
            try:
                cd = load_yaml(cell)
                for sec in ("robot", "end_effector", "environment"):
                    if sec in cd and sec not in xt:
                        report["warnings"].append(f"cell_definition declares '{sec}' but xacro text has no obvious '{sec}' token")
            except Exception as e:
                report["warnings"].append(f"Failed to parse cell_definition.yaml: {e}")

        if not any("Malformed xacro" in b or "Unresolved placeholders" in b for b in report["blockers"]):
            report["xacro_ok"] = True

    build_log = scene_dir / "generated/build_readiness/colcon_build.log"
    report["artifact_paths"]["build_log"] = str(build_log)
    if not args.skip_build:
        if not package_name:
            report["build"].update({"status": "FAIL", "returncode": -1})
            report["blockers"].append("Cannot run build without resolved package_name")
        else:
            build_log.parent.mkdir(parents=True, exist_ok=True)
            cmd = (
                f"cd {args.workspace_root} && "
                "source /opt/ros/humble/setup.bash && "
                f"colcon build --symlink-install --packages-select {package_name}"
            )
            report["commands_run"].append(cmd)
            rc, out, err, to = run_shell(cmd, timeout_sec=max(args.timeout_sec * 6, 120))
            build_log.write_text((out + "\n" + err).strip() + "\n", encoding="utf-8")
            report["build"].update({"status": "PASS" if rc == 0 else "FAIL", "returncode": rc, "log_path": str(build_log), "timed_out": to})
            if rc != 0:
                report["blockers"].append("colcon build failed")
    if not args.skip_launch_smoke:
        if not package_name:
            report["launch_smoke"].update({"status": "FAIL", "returncode": -1})
            report["blockers"].append("Cannot run launch smoke without resolved package_name")
        else:
            cmd = (
                f"cd {args.workspace_root} && "
                "source /opt/ros/humble/setup.bash && "
                "source install/setup.bash && "
                f"ros2 launch {package_name} demo.launch.py use_fake_hardware:=true launch_rviz:=false"
            )
            report["commands_run"].append(cmd)
            rc, out, err, to = run_shell(cmd, timeout_sec=args.timeout_sec)
            combined = (out + "\n" + err).lower()
            if to and ("robot_state_publisher" in combined or "started" in combined or "launch" in combined):
                report["launch_smoke"].update({"status": "PASS_WITH_TIMEOUT", "returncode": rc, "timed_out": True})
                report["warnings"].append("Launch smoke timed out after apparent successful startup.")
            elif rc == 0:
                report["launch_smoke"].update({"status": "PASS", "returncode": rc, "timed_out": False})
            else:
                report["launch_smoke"].update({"status": "FAIL", "returncode": rc, "timed_out": to})
                report["blockers"].append("Launch smoke failed")

    if any("source /opt/ros/humble/setup.bash" in c for c in report["commands_run"]):
        pass

    if report["blockers"]:
        if any("PyYAML is not installed" in b or "Cannot run" in b for b in report["blockers"]):
            report["status"] = "BLOCKED"
        else:
            report["status"] = "FAIL"
    elif report["warnings"] or report["launch_smoke"]["status"] == "PASS_WITH_TIMEOUT":
        report["status"] = "PASS_WITH_WARNINGS"
    else:
        report["status"] = "PASS"

    output = json.dumps(report, indent=2)
    if args.json_out:
        print(output)
    else:
        print(output)

    return 0 if report["status"] in {"PASS", "PASS_WITH_WARNINGS"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
