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


def action_fixture(*, accepted_cancel=True, terminal=True, stopped=True, cancel_done=True):
    """Build the real nested action around deterministic action/telemetry fakes."""
    from moveit_msgs.msg import AllowedCollisionMatrix, PlanningScene

    events = []
    summary = {'current_stage': 'EXECUTE_LIFT', 'full_cycle_prevalidated': True,
               'selected_object_id': 'runtime::target', 'resolution_sha256': 'old'}
    cycle = {'full_cycle_prevalidated': True,
             'steps': [{'trajectory': 'old-trajectory'}], 'candidate': object()}
    uuid = bytes(range(16))
    sample = {'run_id': 'run', 'pid': 99, 'iteration': 40, 'sim_ns': 40_000_000, 'wall_ns': 40_000_000, 'monotonic_raw_ns': 40_000_000}
    result_future = Future(done=False)
    cancel_response = N(return_code=0, goals_canceling=[N(goal_id=N(uuid=uuid))])

    def cancel_goal_async():
        events.append('cancel')
        if terminal and accepted_cancel and cancel_done:   # a rejected/unanswered cancel leaves the goal running
            result_future.value = N(status=5, result=N(error_code=N(val=-4)))
            result_future.ready = True
        return Future(cancel_response if accepted_cancel else N(return_code=1, goals_canceling=[]), done=cancel_done)

    handle = N(accepted=True, goal_id=N(uuid=uuid), get_result_async=lambda: result_future,
               cancel_goal_async=cancel_goal_async)

    def send_goal_async(goal):
        events.append('send')
        result_future.value, result_future.ready = None, False   # each owned goal has its own result
        return Future(handle)

    client = N(wait_for_server=lambda **kw: True, send_goal_async=send_goal_async, result_future=result_future)
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

    scene = object()
    context = dict(summary=summary, cycle=cycle, execute_client=client,
                   apply_client=scene, exchanged_clients={scene}, plan_client=object(),
                   cross_check_gate=executor.PlanningCrossCheckGate(summary),
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
    sample.update(iteration=42,sim_ns=42_000_000,wall_ns=42_000_000,monotonic_raw_ns=42_000_000)
    stop=copy.deepcopy(sample)
    stop.update(iteration=41,sim_ns=41_000_000,wall_ns=41_000_000,monotonic_raw_ns=41_000_000)
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


# Interrupted-motion stop certification (Trial 33): only a stop measured after the
# interrupted owned motion may set motion_stop_verified.
OLD_STOP = {'run_id': 'run', 'pid': 99, 'iteration': 10, 'sim_ns': 10_000_000, 'wall_ns': 10_000_000, 'monotonic_raw_ns': 10_000_000}
STALE = 'SIMULATOR_TELEMETRY_INVALID: stale measurement'


def stale_interruption(stop='fresh', **faults):
    """Real action; the monitor fails on stale telemetry after an earlier stop was certified."""
    action, client, summary, cycle, recovery, events = action_fixture(**faults)
    summary.update(motion_stop_verified=True, stopped_measurement=dict(OLD_STOP),
                   stopped_window={'samples': [dict(OLD_STOP)], 'duration_sim_ns': 300_000_000})
    def monitor():
        events.append('guard')
        raise guards.StaleMeasurement(STALE)
    measured = []
    def wait_stopped():
        events.append('stop')
        if stop == 'stale':
            raise guards.StaleMeasurement(STALE)       # what the real wait_stopped's fresh() raises
        sample = dict(OLD_STOP, iteration=50 + len(measured), sim_ns=50_000_000, wall_ns=50_000_000,monotonic_raw_ns=50_000_000)
        measured.append(sample)
        summary['stopped_measurement'] = sample
        return True
    action.__globals__.update(execution_monitor=monitor, wait_stopped=wait_stopped)
    return action, client, summary, events, measured


def interrupt(action, client):
    from moveit_msgs.action import ExecuteTrajectory
    with pytest.raises(guards.StaleMeasurement):
        action(client, ExecuteTrajectory.Goal(), 1)


def assert_uncertified(summary):
    assert summary['motion_stop_verified'] is False
    assert 'stopped_measurement' not in summary and 'stopped_window' not in summary


def test_case_a_only_the_post_cancel_stop_certifies_an_interrupted_motion():
    action, client, summary, events, measured = stale_interruption()
    interrupt(action, client)
    assert events.index('cancel') < events.index('stop') < events.index('reconcile')
    assert summary['cancellation_accepted'] and summary['cancellation_confirmed']
    assert summary['motion_stop_verified'] is True
    assert summary['stopped_measurement'] is measured[0] and summary['stopped_measurement']['iteration'] == 50
    assert 'stopped_window' not in summary          # the pre-motion window does not survive


def test_case_b_stale_post_cancel_telemetry_leaves_the_stop_unverified():
    action, client, summary, events, _ = stale_interruption(stop='stale')
    interrupt(action, client)
    assert summary['cancellation_accepted'] is True
    assert_uncertified(summary)
    assert summary['cancellation_confirmed'] is False
    assert STALE in summary['cancellation_failure']
    assert 'revoke' in events and 'reconcile' not in events


@pytest.mark.parametrize('fault', ['rejected', 'timed_out', 'terminal_missing'])
@pytest.mark.parametrize('stop', ['stale', 'fresh'])
def test_case_c_cancellation_failure_never_infers_or_reuses_a_stop(fault, stop):
    action, client, summary, events, measured = stale_interruption(
        stop=stop, accepted_cancel=fault != 'rejected', cancel_done=fault != 'timed_out',
        terminal=fault != 'terminal_missing')
    interrupt(action, client)
    assert summary['cancellation_confirmed'] is False
    assert (STALE if stop == 'stale' else 'CANCELLATION_UNCONFIRMED') in summary['cancellation_failure']
    assert 'revoke' in events and 'reconcile' not in events
    if stop == 'stale':
        assert_uncertified(summary)
    else:                                           # only the independently measured post-interruption stop
        assert summary['motion_stop_verified'] is True and summary['stopped_measurement'] is measured[0]


def test_case_d_each_interruption_needs_its_own_stop():
    action, client, summary, events, measured = stale_interruption()
    interrupt(action, client)
    assert summary['stopped_measurement'] is measured[0]
    action.__globals__['wait_stopped'] = lambda: (_ for _ in ()).throw(guards.StaleMeasurement(STALE))
    interrupt(action, client)
    assert events.count('send') == 2
    assert_uncertified(summary)                     # the first interruption's stop cannot certify the second
    assert summary['cancellation_confirmed'] is False


def test_a_completed_motion_also_invalidates_the_earlier_stop():
    """A failure after a completed motion must re-measure the stop, not reuse one from before it."""
    from moveit_msgs.action import ExecuteTrajectory
    action, client, summary, cycle, recovery, events = action_fixture()
    summary.update(motion_stop_verified=True, stopped_measurement=dict(OLD_STOP))
    def monitor():
        client.result_future.value = N(status=4, result=N(error_code=N(val=1)))
        client.result_future.ready = True
    action.__globals__['execution_monitor'] = monitor
    action(client, ExecuteTrajectory.Goal(), 1)
    assert 'cancel' not in events
    assert_uncertified(summary)


def failure_handler():
    """The executor's real top-level failure statements, up to the recovery_required decision."""
    import ast
    from test_grasp_recovery_boundary import SCRIPT
    main = next(n for n in ast.parse(SCRIPT.read_text()).body if isinstance(n, ast.FunctionDef) and n.name == 'main')
    handler = next(h for t in ast.walk(main) if isinstance(t, ast.Try) for h in t.handlers
                   if 'KeyboardInterrupt' in ast.unparse(h.type) and 'recovery_required' in ast.unparse(h))
    end = next(i for i, s in enumerate(handler.body) if 'recovery_required' in ast.unparse(s)) + 1
    fn = ast.parse('def handle(exc):\n    pass\n').body[0]
    fn.body = handler.body[:end]
    def run(summary, exc):
        namespace = dict(vars(executor), summary=summary, begin_recovery=lambda e: None,
                         stage=lambda name: summary.update(current_stage=name))
        exec(compile(ast.fix_missing_locations(ast.Module(body=[fn], type_ignores=[])), str(SCRIPT), 'exec'), namespace)
        namespace['handle'](exc)
        return summary
    return run


@pytest.mark.parametrize('case', ['stale_stop', 'cancel_rejected', 'verified_stop', 'repeated'])
def test_interrupted_motion_failure_always_requires_recovery(case):
    handle = failure_handler()
    action, client, summary, events, measured = stale_interruption(
        stop='stale' if case in ('stale_stop', 'repeated') else 'fresh', accepted_cancel=case != 'cancel_rejected')
    if case == 'repeated':                          # an earlier interruption with a verified, reconciled stop
        summary.update(motion_stop_verified=True, cancellation_confirmed=True, stopped_measurement=dict(OLD_STOP))
    summary['execution_attempted'] = True           # set by the executor before every owned send
    interrupt(action, client)
    handle(summary, guards.StaleMeasurement(STALE))
    assert summary['recovery_required'] is True and summary['current_stage'] == 'FAILED'
    assert summary['failure'] == STALE and summary['failed_stage'] == 'EXECUTE_LIFT'
    if case == 'verified_stop':                     # a measured stop alone never clears recovery
        assert summary['motion_stop_verified'] is True and summary['cancellation_confirmed'] is True
    elif case == 'cancel_rejected':                 # sending the cancel never confirms it
        assert summary['cancellation_accepted'] is False and summary['cancellation_confirmed'] is False
    else:                                           # the earlier stop never certifies the new failure
        assert summary['motion_stop_verified'] is False and 'stopped_measurement' not in summary


def test_failure_before_any_owned_motion_does_not_require_recovery():
    summary = {'current_stage': 'PREPLAN_LIFT', 'execution_attempted': False}
    failure_handler()(summary, RuntimeError('planning failed'))
    assert summary['recovery_required'] is False
