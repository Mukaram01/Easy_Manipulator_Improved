"""Projection-only arithmetic is deliberately insufficient for contact authority."""
import copy
from fractions import Fraction as F
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import stage_a_ogre2_registration as r


def fixture():
    p=[[2.,0.,0.,0.],[0.,2.,0.,0.],[0.,0.,-1.,-.04],[0.,0.,-1.,0.]]
    v=[[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]]
    m=dict(attached_camera_count=1,projection=p,view=v)
    report=dict(width=512,height=512,depth_width=512,depth_height=512,segmentation_width=512,segmentation_height=512,
        depth_channels=1,depth_format='FLOAT32',segmentation_channels=3,segmentation_format='R8G8B8',
        depth_frames=3,segmentation_frames=3,shared_render_batches=3,native_camera_matrices=[copy.deepcopy(m) for _ in range(3)])
    rgb=np.zeros((512,512,3),dtype=np.uint8);rgb[100:120,100:120]=255
    depth=np.ones((512,512),dtype=np.float32)
    labels=np.zeros_like(rgb);labels[100:120,100:120]=11;labels[100:120,140:160]=22
    return report,rgb,depth,labels


def test_projection_difference_uses_exact_outward_bound_and_no_authority():
    a,rgb,depth,labels=fixture();a['native_camera_matrices'][1]['projection'][0][0]=1.9999998807907104
    result=r.analyse(a,rgb,depth,labels)
    expected=256*abs(F(2)-F(1.9999998807907104))/F(1.9999998807907104)
    assert F(result['projection_only_registration_upper_px'])>=expected
    assert not result['contact_authority'] and result['total_registration_bound_px'] is None
    assert result['decision']=='BLOCKED'


@pytest.mark.parametrize('case',['blank','infinite','missing_labels','wrong_resolution','wrong_view','nonfinite','wrong_batch','wrong_format'])
def test_invalid_or_uncertifiable_images_rejected(case):
    a,rgb,depth,labels=fixture()
    if case=='blank':rgb[:]=0
    if case=='infinite':depth[:]=np.inf
    if case=='missing_labels':labels[:]=0
    if case=='wrong_resolution':a['depth_width']=511
    if case=='wrong_view':a['native_camera_matrices'][1]['view'][0][3]=.2
    if case=='nonfinite':a['native_camera_matrices'][1]['projection'][0][0]=float('nan')
    if case=='wrong_batch':a['segmentation_frames']=2
    if case=='wrong_format':a['segmentation_format']='RGB_FLOAT32'
    with pytest.raises(ValueError):r.analyse(a,rgb,depth,labels)
