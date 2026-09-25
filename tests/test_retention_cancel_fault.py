"""Fault injection for the existing executor's nested owned-cancel request."""
import ast
from pathlib import Path
from types import SimpleNamespace as N

def test_stale_measurement_does_not_prevent_owned_goal_cancellation():
    source=Path(__file__).parents[1]/'scripts/perceived_object_grasp_execute.py'
    tree=ast.parse(source.read_text())
    fn=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='request_cancel')
    events=[];uuid=bytes(range(16));response=N(return_code=0,goals_canceling=[N(goal_id=N(uuid=uuid))])
    future=N(done=lambda:True,result=lambda:response)
    def stale():raise RuntimeError('stale telemetry')
    def send():events.append('cancel sent');return future
    summary={}
    namespace=dict(summary=summary,owned_uuid=uuid,measurements=N(fresh=stale),
        controlled_cancel=False,controller_audit=None,time=N(time_ns=lambda:1,monotonic_ns=lambda:1),
        handle=N(cancel_goal_async=send),node=None,
        rclpy=N(spin_until_future_complete=lambda *a,**kw:events.append('awaited cancellation')),
        cancel_response_matches=lambda owned,r:owned==uuid and r is response)
    exec(compile(ast.Module(body=[fn],type_ignores=[]),str(source),'exec'),namespace)
    assert namespace['request_cancel']()
    assert events==['cancel sent','awaited cancellation']
    assert summary['cancel_measurement_error']=='stale telemetry'
    assert 'cancel_measurement' not in summary


def test_failure_stop_does_not_require_controlled_trial_metrics():
    import time
    import xml.etree.ElementTree as ET
    import sys
    sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
    source=Path(__file__).parents[1]/'scripts/perceived_object_grasp_execute.py'
    fn=next(n for n in ast.walk(ast.parse(source.read_text()))
            if isinstance(n,ast.FunctionDef) and n.name=='wait_stopped')
    def drain():
        now=time.time_ns()
        return [dict(iteration=i,sim_ns=(i+1)*1000000,wall_ns=now) for i in range(301)]
    measurements=N(robot=ET.fromstring('<robot><ros2_control><joint name="joint"/></ros2_control></robot>'),
        fresh=lambda:{},drain=drain,record=lambda *a:None,joints=lambda s:{'joint':[0.,0.]})
    summary={'cancellation_request':{'uuid':'ordinary-fault'}}
    namespace=dict(summary=summary,measurements=measurements,time=time,node=None,
                   rclpy=N(spin_once=lambda *a,**kw:None))
    exec(compile(ast.Module(body=[fn],type_ignores=[]),str(source),'exec'),namespace)
    assert namespace['wait_stopped']()
    assert summary['stopped_window']['duration_sim_ns']==300000000
    assert 'cancellation_metrics' not in summary
