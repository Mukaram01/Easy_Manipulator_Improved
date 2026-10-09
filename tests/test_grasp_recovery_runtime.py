"""Owned-action recovery tests with fake ROS transport and no physical motion."""
from types import SimpleNamespace as N

import pytest

import simulator_execution as guards
from test_grasp_recovery_boundary import executor, nested


class Future:
    def __init__(self, value=None, *, done=True):
        self.value = value
        self.ready = done

    def done(self):
        return self.ready

    def result(self):
        return self.value


def action_fixture(*, accepted_cancel=True, terminal=True, stopped=True):
    """Build the real nested action around deterministic action/telemetry fakes."""
    from moveit_msgs.msg import AllowedCollisionMatrix, PlanningScene

    events = []
    summary = {'current_stage': 'EXECUTE_LIFT', 'full_cycle_prevalidated': True,
               'selected_object_id': 'runtime::target', 'resolution_sha256': 'old'}
    cycle = {'full_cycle_prevalidated': True,
             'steps': [{'trajectory': 'old-trajectory'}], 'candidate': object()}
    uuid = bytes(range(16))
    sample = {'run_id': 'run', 'pid': 99, 'iteration': 40, 'sim_ns': 40_000_000, 'wall_ns': 40_000_000}
    result_future = Future(done=False)
    cancel_response = N(return_code=0, goals_canceling=[N(goal_id=N(uuid=uuid))])

    def cancel_goal_async():
        events.append('cancel')
        if terminal:
            result_future.value = N(status=5, result=N(error_code=N(val=-4)))
            result_future.ready = True
        return Future(cancel_response if accepted_cancel else N(return_code=1, goals_canceling=[]))

    handle = N(accepted=True, goal_id=N(uuid=uuid), get_result_async=lambda: result_future,
               cancel_goal_async=cancel_goal_async)

    def send_goal_async(goal):
        events.append('send')
        return Future(handle)

    client = N(wait_for_server=lambda **kw: True, send_goal_async=send_goal_async)
    failure = guards.GraspRetentionLoss({'left'}, 0., 0.)
    failure.sample = dict(sample)
    recovery = []

    def begin_recovery(exc, *, during_action):
        events.append('invalidate')
        assert exc is failure
        item = guards.RetentionRecovery(cycle, summary, exc, during_action=during_action)
        recovery.append(item)
        return item

    def monitor():
        events.append('guard')
        raise failure

    def wait_stopped():
        events.append('stop')
        if stopped:
            summary['stopped_measurement'] = sample
        return stopped

    def reconcile():
        events.append('reconcile')
        assert summary['cancellation_confirmed'] is True
        assert summary['motion_stop_verified'] is True
        if recovery:
            assert summary['grasp_recovery'] is recovery[0].evidence

    def apply(_diff):
        events.append('revoke')

    context = dict(summary=summary, cycle=cycle, execute_client=client,
                   contact_guard=None, controlled_cancel=False, controller_audit=None,
                   measurements=N(fresh=lambda: sample), execution_monitor=monitor,
                   rclpy=N(spin_until_future_complete=lambda *a, **kw: None,
                           spin_once=lambda *a, **kw: None), node=object(),
                   begin_recovery=begin_recovery, wait_stopped=wait_stopped,
                   measured_reconcile=reconcile, reconcile_retention_loss=reconcile,
                   apply=apply, baseline=AllowedCollisionMatrix(),
                   PlanningScene=PlanningScene, stage=lambda _name: None)
    return nested('action', context), client, summary, cycle, recovery, events


def test_owned_action_invalidates_before_cancel_then_stops_and_reconciles():
    from moveit_msgs.action import ExecuteTrajectory

    action, client, summary, cycle, recovery, events = action_fixture()
    with pytest.raises(guards.GraspRetentionLoss):
        action(client, ExecuteTrajectory.Goal(), 1)
    assert events.index('invalidate') < events.index('cancel') < events.index('stop')
    assert events.index('stop') < events.index('reconcile')
    assert summary['cancellation_confirmed'] is True
    assert summary['motion_stop_verified'] is True
    assert 'cancellation_failure' not in summary
    assert cycle == {'full_cycle_prevalidated': False, 'invalidated_by': 'GRASP_RETENTION_LOSS'}
    assert summary['grasp_recovery']['requires_owned_cancel'] is True
    assert len(recovery) == 1
    with pytest.raises(RuntimeError, match='RECOVERY_EXECUTION_INVALIDATED'):
        action(client, ExecuteTrajectory.Goal(), 1)
    assert events.count('send') == 1


@pytest.mark.parametrize('fault', ['cancel_rejected', 'terminal_missing', 'stop_missing'])
def test_owned_cancellation_failure_revokes_but_cannot_reconcile(fault):
    from moveit_msgs.action import ExecuteTrajectory

    action, client, summary, cycle, recovery, events = action_fixture(
        accepted_cancel=fault != 'cancel_rejected', terminal=fault != 'terminal_missing',
        stopped=fault != 'stop_missing')
    with pytest.raises(guards.GraspRetentionLoss):
        action(client, ExecuteTrajectory.Goal(), 1)
    assert events.index('invalidate') < events.index('cancel')
    assert 'revoke' in events
    assert 'reconcile' not in events
    assert summary['cancellation_confirmed'] is False
    assert 'CANCELLATION_UNCONFIRMED' in summary['cancellation_failure']
    assert summary['grasp_recovery']['state'] == 'CANCEL_AND_STOP'
    assert cycle['full_cycle_prevalidated'] is False
    assert len(recovery) == 1


def test_non_retention_monitor_failure_does_not_enter_retention_recovery():
    from moveit_msgs.action import ExecuteTrajectory

    action, client, summary, cycle, recovery, events = action_fixture()
    action.__globals__['execution_monitor'] = lambda: (_ for _ in ()).throw(RuntimeError('telemetry missing'))
    with pytest.raises(RuntimeError, match='telemetry missing'):
        action(client, ExecuteTrajectory.Goal(), 1)
    assert not recovery
    assert 'grasp_recovery' not in summary
    assert cycle['full_cycle_prevalidated'] is True


def test_fake_transport_runs_complete_boundary_and_refuses_retry():
    """Real action/cancel/reconcile/state boundary; transport/physics are simulated."""
    import copy
    from moveit_msgs.action import ExecuteTrajectory
    from test_grasp_recovery_boundary import runtime_reconcile_fixture

    action, client, summary, cycle, recoveries, events = action_fixture()
    reconcile, _, _, sample, guard, queries, updates = runtime_reconcile_fixture()
    sample.update(iteration=42,sim_ns=42_000_000,wall_ns=42_000_000)
    stop=copy.deepcopy(sample)
    stop.update(iteration=41,sim_ns=41_000_000,wall_ns=41_000_000)
    def stopped():
        events.append('stop')
        summary['stopped_measurement']=stop
        return True
    def reconcile_real():
        events.append('reconcile')
        reconcile.__globals__.update(recovery=recoveries[0],summary=summary)
        reconcile()
    action.__globals__.update(wait_stopped=stopped,measured_reconcile=reconcile_real)
    with pytest.raises(guards.GraspRetentionLoss):action(client,ExecuteTrajectory.Goal(),1)
    assert 'cancellation_failure' not in summary
    assert summary['cancellation_confirmed'] and summary['motion_stop_verified']
    assert summary['measured_reconciliation']['attached_ids']==[]
    assert summary['measured_reconciliation']['acm_restored']
    assert recoveries[0].evidence['transitions']==[
        'CANCEL_AND_STOP','FAILURE_RECONCILE','QUALIFY_RETREAT','RECOVERY_BLOCKED']
    assert recoveries[0].evidence['retry_count']==0
    assert guard.held is None and not guard.planning_attached
    assert len(queries)==1 and list(queries[0].robot_state.joint_state.position)==[.9,.3]
    assert updates[-1].world.collision_objects[1].pose.position.x==1.
    with pytest.raises(RuntimeError,match='RECOVERY_EXECUTION_INVALIDATED'):
        action(client,ExecuteTrajectory.Goal(),1)
    assert events.count('send')==1
