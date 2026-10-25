"""Rejection-only closure screening against the existing native BOX predicate."""
import copy
import json
from pathlib import Path
import pytest
from closure_clearance import screen_closure_neighbors, expanded_closure_scene
from pile_extraction import ExtractionFailure


def box(name, x):
    return dict(id=name,shape='BOX',frame_id='world',dimensions=[.025]*3,
                pose=[x,0.,.02,0.,0.,0.,1.])


def test_recorded_new_contact_is_rejected_before_motion_without_object_specific_rules():
    data=json.loads((Path(__file__).parent/'fixtures/closure_new_neighbor_contact.json').read_text())
    target,neighbor=data['target'],data['neighbor']
    target['id']='arbitrary-selected';neighbor['id']='arbitrary-neighbor'
    with pytest.raises(ExtractionFailure) as exc:
        screen_closure_neighbors(target,[neighbor],{})
    assert exc.value.details['extraction_reason_code']=='CLOSURE_NEIGHBOR_CLEARANCE'
    assert 0<exc.value.details['gap_m']<.00001
    assert exc.value.details['required_clearance_m']>.001


def test_well_separated_neighbor_passes_and_preserves_all_input_geometry():
    target,neighbor=box('target',0.),box('neighbor',.04)
    before=copy.deepcopy([target,neighbor])
    result=screen_closure_neighbors(target,[neighbor],{})
    assert result['accepted'] is True
    assert result['checks'][0]['gap_m']==pytest.approx(.015)
    assert result['neighbor_inflation_m']['neighbor']>.001
    assert [target,neighbor]==before


def test_contact_name_alone_cannot_waive_a_separated_neighbor():
    with pytest.raises(ExtractionFailure):
        screen_closure_neighbors(box('target',0.),[box('neighbor',.02501)],
                                 {'neighbor':[[999.,999.,999.]]})


def test_measured_existing_contact_needs_native_geometry_and_retains_neighbor_for_sweep():
    target,neighbor=box('target',0.),box('support',.025)
    result=screen_closure_neighbors(target,[neighbor],{'support':[[.0125,0.,.02]]})
    assert result['accepted']
    assert result['checks'][0]['measured_initial_contact'] is True
    assert 'support' in result['neighbor_inflation_m']


def test_private_expansion_covers_every_axis_without_changing_nominal_scene_or_acm():
    from moveit_msgs.msg import PlanningScene,CollisionObject
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose
    scene=PlanningScene()
    p=Pose();p.orientation.w=1.
    scene.world.collision_objects=[CollisionObject(id=n,primitives=[SolidPrimitive(type=1,dimensions=[.025]*3)],primitive_poses=[p]) for n in ('target','neighbor')]
    before=copy.deepcopy(scene)
    padded=expanded_closure_scene(scene,{'neighbor':.003})
    assert list(padded.world.collision_objects[1].primitives[0].dimensions)==pytest.approx([.031]*3)
    assert padded.world.collision_objects[0]==scene.world.collision_objects[0]
    assert padded.allowed_collision_matrix==scene.allowed_collision_matrix
    assert scene==before
    with pytest.raises(ValueError):expanded_closure_scene(scene,{'missing':.003})


def test_admitted_pose_shift_uses_measured_geometry_for_existing_contact_points():
    target,neighbor=box('target',0.),box('support',.025)
    poses={'target':[.00015,0.,.02,0.,0.,0.,1.],
           'support':[.02515,0.,.02,0.,0.,0.,1.]}
    result=screen_closure_neighbors(target,[neighbor],{'support':[[.01265,0.,.02]]},
                                    measured_poses=poses)
    assert result['accepted']
    poses['target'][0]=.001
    with pytest.raises(ValueError,match='observation'):
        screen_closure_neighbors(target,[neighbor],{'support':[[.01265,0.,.02]]},
                                 measured_poses=poses)


def test_pose_bound_includes_neighbor_orientation_at_motion_boundary():
    import math
    from closure_clearance import check_closure_pose_bounds
    target,neighbor=box('target',0.),box('neighbor',.04)
    poses={item['id']:list(item['pose']) for item in (target,neighbor)}
    poses['neighbor'][3:]=[0.,0.,math.sin(.001/2),math.cos(.001/2)]
    with pytest.raises(ValueError,match='neighbor'):
        check_closure_pose_bounds([target,neighbor],poses)


def test_compound_box_plus_plane_is_not_silently_reduced_to_box():
    from moveit_msgs.msg import PlanningScene,CollisionObject
    from shape_msgs.msg import SolidPrimitive,Plane
    from geometry_msgs.msg import Pose
    p=Pose();p.orientation.w=1.
    obj=CollisionObject(id='neighbor',primitives=[SolidPrimitive(type=1,dimensions=[.025]*3)],
                        primitive_poses=[p],planes=[Plane(coef=[0.,0.,1.,0.])],plane_poses=[p])
    scene=PlanningScene();scene.world.collision_objects=[obj]
    with pytest.raises(ValueError):expanded_closure_scene(scene,{'neighbor':.003})
