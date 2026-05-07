#!/usr/bin/env python3
from __future__ import annotations
import argparse, json
from pathlib import Path
from typing import Any
try:
    import yaml
except Exception:
    yaml = None

def _load(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"schema_version":"environment_layout/v1","layout_id":"authored_layout","metadata":{"generated_defaults":False},"assets":[],"zones":[],"targets":[]}
    if yaml is None:
        raise RuntimeError("PyYAML is required")
    data=yaml.safe_load(path.read_text(encoding="utf-8"))
    return data if isinstance(data,dict) else {}

def _zone_from_args(a: argparse.Namespace) -> dict[str, Any]:
    x,y,z=a.xyz; sx,sy,sz=a.size
    return {"id":a.target_id,"type":a.target_type,"label":a.label,"frame":a.frame,"pose":{"frame":a.frame,"xyz":[x,y,z],"rpy":list(a.rpy)},"size":list(a.size),"bounds_xyz":{"min":[x-sx/2,y-sy/2,z-sz/2],"max":[x+sx/2,y+sy/2,z+sz/2]}}

def main()->int:
    ap=argparse.ArgumentParser()
    ap.add_argument("--environment-layout", type=Path, required=True)
    ap.add_argument("--target-id", required=True); ap.add_argument("--target-type", choices=["pick_zone","place_target","bin"], required=True)
    ap.add_argument("--label", default=""); ap.add_argument("--frame", default="world")
    ap.add_argument("--xyz", nargs=3, type=float, required=True); ap.add_argument("--rpy", nargs=3, type=float, required=True); ap.add_argument("--size", nargs=3, type=float, required=True)
    ap.add_argument("--output", type=Path, required=True); ap.add_argument("--json", action="store_true")
    a=ap.parse_args()
    payload=_load(a.environment_layout)
    payload.setdefault("schema_version","environment_layout/v1"); payload.setdefault("zones",[]); payload.setdefault("assets",[]); payload.setdefault("targets",[])
    zone=_zone_from_args(a); updated=False
    zones=payload["zones"] if isinstance(payload.get("zones"),list) else []
    for i,z in enumerate(zones):
        if isinstance(z,dict) and z.get("id")==a.target_id:
            zones[i]=zone; updated=True; break
    else:
        zones.append(zone)
    payload["zones"]=zones
    t={"id":a.target_id,"type":a.target_type,"label":a.label,"frame":a.frame,"xyz":list(a.xyz),"rpy":list(a.rpy),"size":list(a.size)}
    tlist=payload["targets"] if isinstance(payload.get("targets"),list) else []
    for i,x in enumerate(tlist):
        if isinstance(x,dict) and x.get("id")==a.target_id:
            tlist[i]=t; break
    else: tlist.append(t)
    payload["targets"]=tlist
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    out={"result":"PASS","output_path":a.output.as_posix(),"target_id":a.target_id,"target_type":a.target_type,"frame":a.frame,"xyz":list(a.xyz),"rpy":list(a.rpy),"size":list(a.size),"updated_existing":updated,"warnings":[],"errors":[]}
    print(json.dumps(out,indent=2) if a.json else out); return 0

if __name__=="__main__":
    raise SystemExit(main())
