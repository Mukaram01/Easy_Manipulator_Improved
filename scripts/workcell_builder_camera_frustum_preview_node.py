#!/usr/bin/env python3
"""Visual-only camera frustum preview marker publisher.
Does not start runtime perception/motion/hardware.
"""
import argparse, yaml

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--preview-dir', default=''); ap.add_argument('--environment-yaml', default='')
    args=ap.parse_args()
    path=args.environment_yaml
    if not path and args.preview_dir:
        path=f"{args.preview_dir}/camera_frustum_preview.yaml"
    try:
        data=yaml.safe_load(open(path, 'r', encoding='utf-8')) if path else {}
    except Exception:
        data={}
    cams=data.get('camera_placements', []) if isinstance(data, dict) else []
    print(f"camera_frustum_markers={len(cams)}")
    return 0

if __name__=='__main__':
    raise SystemExit(main())
