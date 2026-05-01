#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, sys
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))
from run_scenario_pack import run_scenario
from validate_detected_objects import _load_yaml_or_json


def _load(path: Path):
    d,_,_ = _load_yaml_or_json(path)
    return d if isinstance(d, dict) else {}

def main(argv=None)->int:
    ap=argparse.ArgumentParser()
    ap.add_argument('--scenario-dir', type=Path, required=True)
    ap.add_argument('--output-root', type=Path, required=True)
    ap.add_argument('--json', action='store_true')
    args=ap.parse_args(argv)
    files = sorted([*args.scenario_dir.glob('*.yaml'), *args.scenario_dir.glob('*.yml')])
    reports=[]
    coverage={"robots":set(),"end_effectors":set(),"task_types":set(),"cell_definitions":set()}
    counts={"passed":0,"warned":0,"failed":0,"skipped":0}
    for f in files:
        s=_load(f)
        if s.get('schema_version')!='scenario_pack/v1':
            continue
        e=s.get('expected') or {}
        coverage['robots'].add(e.get('robot'))
        coverage['end_effectors'].add(e.get('end_effector'))
        coverage['task_types'].add(e.get('task_type'))
        coverage['cell_definitions'].add(s.get('cell_definition'))
        rep=run_scenario(f,args.output_root,args.json)
        reports.append({"scenario":rep['scenario'],"status":rep['status'],"report_path":str(args.output_root/rep['scenario']/ 'scenario_run_report.json')})
        if rep['status']=='PASS':counts['passed']+=1
        elif rep['status']=='WARN':counts['warned']+=1
        elif rep['status']=='SKIPPED':counts['skipped']+=1
        else:counts['failed']+=1
    payload={"schema_version":"scenario_matrix_report/v1","total_scenarios":len(reports),**counts,"scenarios":reports,
             "coverage_summary":{k:sorted([x for x in v if x]) for k,v in coverage.items()}}
    args.output_root.mkdir(parents=True, exist_ok=True)
    path=args.output_root/'scenario_matrix_report.json'
    path.write_text(json.dumps(payload,indent=2,sort_keys=True)+'\n',encoding='utf-8')
    if args.json: print(json.dumps(payload,indent=2,sort_keys=True))
    else: print(path)
    return 0 if counts['failed']==0 else 1

if __name__=='__main__':
    raise SystemExit(main())
