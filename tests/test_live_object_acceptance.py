import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]/'scripts'))
from run_live_object_acceptance import MOTION_STAGES, verify_execution_result, perception_ready, epd_capture_command


def test_capture_requires_matching_reliable_publisher_and_live_health():
    kind = 'epd_msgs/msg/EPDObjectTracking'
    endpoints = dict(actual_topic_type=[kind], publisher_count=1, publisher_qos=[dict(reliability='RELIABLE')])
    health = dict(inference_completed='2', geometry_valid_total='1')
    assert perception_ready(endpoints, health, kind)
    assert not perception_ready(endpoints, {}, kind)
    assert not perception_ready(dict(endpoints, publisher_count=0), health, kind)
    assert not perception_ready(endpoints, health, 'epd_msgs/msg/EPDObjectLocalization')
    assert not perception_ready(dict(endpoints, publisher_qos=[dict(reliability='BEST_EFFORT')]), health, kind)


@pytest.mark.parametrize('mode,number', [('tracking','4'),('localization','3')])
def test_owned_epd_mode_and_reliable_output_are_explicit(mode, number):
    command = epd_capture_command(Path('/epd/bin'), mode)
    assert 'usecase_mode_override:='+number in command
    for topic in ('tracking','localize'):
        assert 'qos_overrides./easy_perception_deployment/epd_'+topic+'_output.publisher.reliability:=reliable' in command


def complete():
    return dict(result='PASS', full_cycle_execution_success=True, attach_verified=True,
        detach_verified=True, home_verified=True, destination_verified=True,
        stages=['EXECUTE_'+stage for stage in MOTION_STAGES])


def test_state_verified_cycle_required():
    verify_execution_result(complete())
    with pytest.raises(RuntimeError):
        verify_execution_result(dict(result='PASS',execution_results=[dict(code=1)]*9))


@pytest.mark.parametrize('key', ['attach_verified','detach_verified','home_verified','destination_verified'])
def test_missing_lifecycle_evidence_rejected(key):
    result=complete()
    result[key]=False
    with pytest.raises(RuntimeError, match=key):
        verify_execution_result(result)


def test_missing_motion_rejected():
    result=complete()
    result['stages'].remove('EXECUTE_RETREAT')
    with pytest.raises(RuntimeError, match='EXECUTE_RETREAT'):
        verify_execution_result(result)
