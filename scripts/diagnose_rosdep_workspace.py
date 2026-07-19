#!/usr/bin/env python3
"""Print rosdep/package diagnostics for a workspace dependency failure."""

from __future__ import annotations

import argparse
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

DEPENDENCY_TAGS = {
    "build_depend",
    "build_export_depend",
    "buildtool_depend",
    "depend",
    "exec_depend",
    "test_depend",
}


def package_name(package_xml: Path) -> str:
    try:
        return (ET.parse(package_xml).getroot().findtext("name") or "").strip()
    except ET.ParseError:
        return ""


def collect_packages(src: Path) -> dict[str, Path]:
    packages: dict[str, Path] = {}
    for package_xml in sorted(src.rglob("package.xml")):
        if any((parent / "COLCON_IGNORE").exists() or (parent / "AMENT_IGNORE").exists() for parent in package_xml.parents):
            continue
        name = package_name(package_xml)
        if name:
            packages[name] = package_xml.parent
    return packages


def collect_declared_dependencies(src: Path) -> dict[str, set[Path]]:
    declared: dict[str, set[Path]] = {}
    for package_xml in sorted(src.rglob("package.xml")):
        if any((parent / "COLCON_IGNORE").exists() or (parent / "AMENT_IGNORE").exists() for parent in package_xml.parents):
            continue
        try:
            root = ET.parse(package_xml).getroot()
        except ET.ParseError:
            continue
        for element in root:
            if element.tag not in DEPENDENCY_TAGS or not element.text:
                continue
            key = element.text.strip()
            if key:
                declared.setdefault(key, set()).add(package_xml)
    return declared


def rosdep_resolve(key: str, rosdistro: str, os_name: str, os_codename: str) -> tuple[int, str]:
    cmd = [
        "rosdep",
        "resolve",
        key,
        "--rosdistro",
        rosdistro,
        "--os",
        f"{os_name}:{os_codename}",
    ]
    proc = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False)
    return proc.returncode, proc.stdout.strip()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--src", default="src", type=Path)
    parser.add_argument("--rosdistro", required=True)
    parser.add_argument("--os-name", default="ubuntu")
    parser.add_argument("--os-codename", required=True)
    parser.add_argument("--skip-keys", default="")
    args = parser.parse_args()

    skip_keys = {key for key in args.skip_keys.replace(",", " ").split() if key}
    packages = collect_packages(args.src)
    declared = collect_declared_dependencies(args.src)

    print("rosdep failure diagnostics")
    print(f"  ROS distribution: {args.rosdistro}")
    print(f"  OS codename: {args.os_name}:{args.os_codename}")
    print(f"  workspace source root: {args.src}")
    print(f"  discovered source packages: {len(packages)}")
    if skip_keys:
        print(f"  skipped rosdep keys: {', '.join(sorted(skip_keys))}")
    if not packages:
        print(f"No source packages discovered under {args.src}; check source import and workspace layout before rosdep.", file=sys.stderr)
        return 1

    unresolved = []
    for key in sorted(declared):
        providers = sorted(str(path) for path in declared[key])
        if key in packages:
            print(f"SOURCE {key}: provided by {packages[key]}; declared by {', '.join(providers)}")
            continue
        if key in skip_keys:
            print(f"SKIP {key}: declared by {', '.join(providers)}")
            continue
        rc, output = rosdep_resolve(key, args.rosdistro, args.os_name, args.os_codename)
        first_line = output.splitlines()[0] if output else "<no rosdep output>"
        if rc == 0:
            print(f"ROSDEP {key}: {first_line}; declared by {', '.join(providers)}")
        else:
            unresolved.append((key, providers, output))
            print(f"UNRESOLVED {key}: declared by {', '.join(providers)}")
            print(output)

    if unresolved:
        print("Unresolved rosdep keys:", ", ".join(key for key, _, _ in unresolved), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
