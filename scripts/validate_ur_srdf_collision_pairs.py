#!/usr/bin/env python3
"""Validate UR scene SRDF collision disable entries for base_link_inertia compatibility."""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


REQUIRED_PAIRS = {tuple(sorted(("base_link", "base_link_inertia"))), tuple(sorted(("base_link_inertia", "shoulder_link")))}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf", type=Path, help="Path to resolved URDF XML file")
    parser.add_argument("srdf", type=Path, help="Path to resolved SRDF XML file")
    return parser.parse_args()


def load_xml(path: Path):
    try:
        return ET.fromstring(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise RuntimeError(f"Failed parsing XML '{path}': {exc}") from exc


def extract_pairs(root):
    pairs = []
    for element in root.findall("disable_collisions"):
        link1 = element.attrib.get("link1")
        link2 = element.attrib.get("link2")
        if link1 and link2:
            pairs.append(tuple(sorted((link1, link2))))
    return pairs


def main():
    args = parse_args()
    urdf_root = load_xml(args.urdf)
    srdf_root = load_xml(args.srdf)

    urdf_links = {link.attrib.get("name") for link in urdf_root.findall("link") if link.attrib.get("name")}
    srdf_pairs = extract_pairs(srdf_root)
    srdf_pair_set = set(srdf_pairs)

    if {"base_link_inertia", "shoulder_link"}.issubset(urdf_links):
        missing = REQUIRED_PAIRS - srdf_pair_set
        duplicates = [pair for pair in REQUIRED_PAIRS if srdf_pairs.count(pair) > 1]
        if missing:
            missing_text = ", ".join([f"{a}<->{b}" for a, b in sorted(missing)])
            raise RuntimeError(f"Missing required disable_collisions pairs: {missing_text}")
        if duplicates:
            duplicate_text = ", ".join([f"{a}<->{b}" for a, b in sorted(duplicates)])
            raise RuntimeError(f"Duplicate required disable_collisions pairs detected: {duplicate_text}")
        print("PASS: Required base_link_inertia collision pairs are present exactly once.")
        return 0

    print("SKIP: URDF does not include both base_link_inertia and shoulder_link.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
