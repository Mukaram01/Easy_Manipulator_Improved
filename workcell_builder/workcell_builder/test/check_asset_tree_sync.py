#!/usr/bin/env python3
import argparse
import hashlib
from pathlib import Path
import subprocess
import sys


def tracked_files(repo_root: Path, root: Path):
    rel = root.relative_to(repo_root).as_posix()
    cmd = ["git", "-C", str(repo_root), "ls-files", f"{rel}/**"]
    out = subprocess.check_output(cmd, text=True)
    return [Path(line.strip()) for line in out.splitlines() if line.strip()]


def digest(path: Path):
    h = hashlib.sha256()
    h.update(path.read_bytes())
    return h.hexdigest()


def relative_map(repo_root: Path, base: Path):
    files = tracked_files(repo_root, base)
    result = {}
    for rel in files:
        rel_from_base = rel.relative_to(base.relative_to(repo_root)).as_posix()
        result[rel_from_base] = digest(repo_root / rel)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", required=True)
    parser.add_argument("--canonical", required=True)
    parser.add_argument("--compare", action="append", default=[])
    args = parser.parse_args()

    repo_root = Path(args.repo_root).resolve()
    canonical = (repo_root / args.canonical).resolve()
    if not canonical.exists():
        print(f"Canonical asset tree missing: {canonical}")
        return 1

    canonical_map = relative_map(repo_root, canonical)
    if not canonical_map:
        print(f"No tracked files found in canonical tree: {canonical}")
        return 1

    ok = True
    for cmp_rel in args.compare:
        compare_root = (repo_root / cmp_rel).resolve()
        if not compare_root.exists():
            continue
        compare_map = relative_map(repo_root, compare_root)
        if not compare_map:
            continue

        if set(compare_map) != set(canonical_map):
            ok = False
            only_canonical = sorted(set(canonical_map) - set(compare_map))
            only_compare = sorted(set(compare_map) - set(canonical_map))
            print(f"Asset path mismatch for {cmp_rel}")
            if only_canonical:
                print(f"  Missing in {cmp_rel}: {only_canonical[:10]}")
            if only_compare:
                print(f"  Extra in {cmp_rel}: {only_compare[:10]}")

        for rel_path, canonical_hash in canonical_map.items():
            compare_hash = compare_map.get(rel_path)
            if compare_hash is not None and compare_hash != canonical_hash:
                ok = False
                print(f"Asset checksum mismatch for {cmp_rel}/{rel_path}")

    if not ok:
        print("Tracked duplicate asset trees diverged from canonical assets.")
        return 1

    print("Tracked duplicate asset trees match canonical assets.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
