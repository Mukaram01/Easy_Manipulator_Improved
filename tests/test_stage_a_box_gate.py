"""Offline diagnostics only; no inference or simulator execution."""
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import stage_a_box_gate as gate


def test_exact_grid_and_centre_scaling_control():
    box=np.array([10.,20.,110.,220.],dtype=np.float32)
    grid=list(gate.candidates(box))
    assert len(grid)==1225
    assert len({tuple(r['parameters']) for r in grid})==1225
    control=[r for r in grid if r['control']]
    assert len(control)==1 and control[0]['box_xyxy']==box.tolist()
    row=next(r for r in grid if r['parameters']==[2,-4,.9,1.1])
    assert row['box_xyxy']==[17.,6.,107.,226.]
    assert sorted({r['parameters'][0] for r in grid})==list(range(-6,7,2))
    assert sorted({r['parameters'][2] for r in grid})==[.9,.95,1.,1.05,1.1]


def test_pareto_keeps_equal_metrics_and_excludes_dominated_tradeoff():
    rows=[dict(background=b,missing_object=m) for b,m in [(0,4),(1,2),(1,2),(2,3),(4,0)]]
    assert gate.pareto_indices(rows)==[0,1,2,4]


def test_metrics_literal_displaced_square():
    ids=np.zeros((512,512),dtype=np.uint32);ids[20:30,20:30]=65517
    mask=np.zeros((512,512),dtype=bool);mask[20:30,21:31]=True
    m=gate.measure(mask,ids,65517)
    assert (m['background'],m['missing_object'],m['intersection'],m['union'])==(10,10,90,110)
    assert m['iou']==90/110
    assert m['boundary_error_max']==1.


def test_frozen_baseline_and_hash_rejection(tmp_path):
    data=gate.load_inputs()
    assert [(r['raw_native']['background'],r['roi_native']['background'],r['raw_native']['missing_object']) for r in data['baseline']['detections'][:2]]==[(456,303,75),(635,564,0)]
    assert data['baseline']['strict_accepted_associations']==0
    (tmp_path/'sha256.json').write_text('{}')
    with pytest.raises(ValueError,match='manifest'):
        gate.load_inputs(tmp_path)


def test_control_reconstructs_genuine_binary_without_head_mutation():
    data=gate.load_inputs();heads=data['heads'].copy()
    for i in range(3):
        result,_=gate.analysis.reconstruct(data['heads'][i,0],data['boxes'][i])
        assert np.array_equal(result>.5,data['raw'][i,0]>.5)
        cropped=gate.measure(gate.cropped_mask(result>.5,data['boxes'][i]),data['ids'],(65517,65523,65517)[i])
        prior=data['baseline']['detections'][i]['roi_native']
        assert (cropped['background'],cropped['missing_object'])==(prior['background'],prior['missing_object'])
    assert np.array_equal(heads,data['heads'])


def test_original_threshold_is_strict_and_zero_padding_preserved():
    head=np.full((28,28),.5,dtype=np.float32)
    result,_=gate.analysis.reconstruct(head,np.array([100.,100.,128.,128.],np.float32))
    assert not np.any(result>.5)
    assert result[114,114]==.5 and result[0,0]==0.


def test_frozen_grid_report_exact_regeneration():
    import gzip
    import json
    folder=gate.graph.PAIR.parent/'box_geometry_gate'
    raw=gzip.decompress((folder/'results.json.gz').read_bytes())
    assert gate.graph.sha(raw)==json.loads((folder/'sha256.json').read_text())['results.json']
    frozen=json.loads(raw)
    assert gate.run()==frozen
    for d in frozen['detections']:
        assert len(d['candidates'])==1225
        assert all(r['metrics']['cross_object']==0 for r in d['candidates'])
        assert d['exact_candidates']==d['cropped_exact_candidates']==[]
        assert d['cropped_minimum_background_at_full_coverage']>0
        assert d['minimum_background_at_full_coverage']>0
        assert d['pareto_candidate_indices']==gate.pareto_indices([r['metrics'] for r in d['candidates']])


def test_diagonal_pixel_distance_has_exact_integer_squared_length():
    ids=np.zeros((512,512),np.uint32);ids[100:110,100:110]=65517
    mask=ids==65517;mask[111,110]=True
    m=gate.measure(mask,ids,65517)
    assert m['spill_distance_max']==np.sqrt(5)
    assert m['spill_distance_mean']==np.sqrt(5)


def test_pareto_csv_hash_coordinates_and_complete_frontiers():
    import csv
    import gzip
    import json
    folder=gate.graph.PAIR.parent/'box_geometry_gate'
    raw=(folder/'pareto.csv').read_bytes()
    assert gate.graph.sha(raw)==json.loads((folder/'sha256.json').read_text())['pareto.csv']
    rows=list(csv.DictReader(raw.decode().splitlines()))
    report=json.loads(gzip.decompress((folder/'results.json.gz').read_bytes()))
    expected=set()
    for d in report['detections']:
        for path,key in [('raw','pareto_candidate_indices'),('cropped','cropped_pareto_candidate_indices')]:
            expected.update((path,d['identity'],n) for n in d[key])
    assert {(r['path'],int(r['identity']),int(r['candidate_index'])) for r in rows}==expected
    assert len(rows)==len(expected)
    for r in rows:
        d=next(d for d in report['detections'] if d['identity']==int(r['identity']))
        c=d['candidates'][int(r['candidate_index'])]
        assert [float(r[k]) for k in ('x0','y0','x1','y1')]==c['box_xyxy']
