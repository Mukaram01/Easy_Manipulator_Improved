"""Compile the real patched callback tail; test doubles are unit evidence only."""
from pathlib import Path
import subprocess
import pytest

REPO=Path(__file__).resolve().parents[1]

@pytest.fixture(scope='module')
def completion_probe(tmp_path_factory):
    root=tmp_path_factory.mktemp('moveit-active-result-unit')
    patch=(REPO/'patches/moveit_humble_capability_teardown.patch').read_text()
    section=patch.split('diff --git a/src/default_capabilities/move_action_capability.cpp ',1)[1].split('diff --git ',1)[0]
    source='\n'.join(line[1:] if line else '' for line in section.splitlines()
                     if (line.startswith(('+',' ')) and not line.startswith('+++')) or not line)
    marker='goal->get_goal()->planning_options.plan_only));'
    start=source.index(marker)+len(marker)
    tail=source[start:source.index('\n}',start)]
    program=r'''
#include <memory>
#include <stdexcept>
#include <string>
#include <cassert>
struct Context { bool valid=true; };
struct Node { std::shared_ptr<Context> context=std::make_shared<Context>(); Node* get_node_base_interface(){return this;} std::shared_ptr<Context> get_context(){return context;} };
namespace rclcpp { bool ok(std::shared_ptr<Context> c){return c->valid;} }
namespace moveit_msgs { namespace msg { struct MoveItErrorCodes {static constexpr int SUCCESS=1,PREEMPTED=-7;}; } }
struct Result {struct Code {int val=1;} error_code;};
struct Goal {
 std::shared_ptr<Context> context; bool active=true; int mode=0;
 bool is_active(){return active;}
 void finish(){if(mode==3)throw std::runtime_error("transition failed");active=false;if(mode==1||mode==2)throw std::runtime_error("Asked to publish result for goal that does not exist");if(mode==4)context->valid=false;}
 void succeed(std::shared_ptr<Result>){finish();} void abort(std::shared_ptr<Result>){finish();} void canceled(std::shared_ptr<Result>){finish();}
};
constexpr int IDLE=0; int move_state_=1; bool preempt_requested_=true; int feedback=0; bool race_feedback=false;
void setMoveState(int state,std::shared_ptr<Goal> goal){move_state_=state;if(goal){feedback++;if(race_feedback){goal->context->valid=false;throw std::runtime_error("publisher context invalid");}if(!goal->context->valid)throw std::runtime_error("publisher context invalid");}}
#define RCLCPP_WARN(...) (void)0
#define RCLCPP_DEBUG(...) (void)0
void complete(std::shared_ptr<Goal> goal,std::shared_ptr<Result> action_res,std::shared_ptr<Node> node){
''' + tail + r'''
}
int main(int argc,char**argv){
 auto node=std::make_shared<Node>();auto goal=std::make_shared<Goal>();goal->context=node->context;
 std::string mode=argv[1];bool expect_throw=false;
 if(mode=="shutdown_result"){goal->mode=1;node->context->valid=false;}
 if(mode=="valid_error"){goal->mode=2;expect_throw=true;}
 if(mode=="nonterminal_error"){goal->mode=3;node->context->valid=false;expect_throw=true;}
 if(mode=="shutdown_before_feedback"){goal->mode=4;}
 if(mode=="shutdown_during_feedback"){race_feedback=true;}
 bool threw=false;try{complete(goal,std::make_shared<Result>(),node);}catch(const std::runtime_error&){threw=true;}
 assert(threw==expect_throw);
 if(!expect_throw){assert(!goal->active);assert(move_state_==IDLE);assert(!preempt_requested_);}
 if(mode=="shutdown_result"||mode=="shutdown_before_feedback")assert(feedback==0);
 if(mode=="success")assert(feedback==1);
}
'''
    cpp=root/'probe.cpp';cpp.write_text(program)
    binary=root/'probe'
    subprocess.run(['c++','-std=c++17',str(cpp),'-o',str(binary)],check=True)
    return binary

@pytest.mark.parametrize('case',['success','shutdown_result','valid_error','nonterminal_error','shutdown_before_feedback','shutdown_during_feedback'])
def test_callback_completion_boundary(completion_probe,case):
    result=subprocess.run([str(completion_probe),case],capture_output=True,text=True)
    assert result.returncode==0,result.stderr


def test_active_observation_uses_actual_signal_handler_record(monkeypatch):
    import importlib.util
    script=Path(__file__).parents[1]/'scripts/moveit_teardown_repro/verify_active_plan.py'
    monkeypatch.syspath_prepend(str(script.parent))
    spec=importlib.util.spec_from_file_location('active_probe',script)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    success='[workcell.controller_certificate]: result=0 reason=ALL_INTERVALS_CERTIFIED\nMotion plan was computed successfully.\n'
    marker='[rclcpp]: signal_handler(SIGINT/SIGTERM)\n'
    assert module.shutdown_completion(marker+success)==dict(signal_handler_observed=True,
        production_certificate_after_signal=True,plan_result_after_signal=True,interrupted_after_signal=False)
    for text in (success,success+marker):
        result=module.shutdown_completion(text)
        assert not result['production_certificate_after_signal']
        assert not result['plan_result_after_signal']


def test_active_shutdown_requires_certificate_and_owner_interruption(monkeypatch):
    import importlib.util
    script=REPO/'scripts/moveit_teardown_repro/verify_active_plan.py'
    monkeypatch.syspath_prepend(str(script.parent))
    spec=importlib.util.spec_from_file_location('active_interrupt_probe',script)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    marker='[rclcpp]: signal_handler(SIGINT/SIGTERM)\n'
    certificate='[workcell.controller_certificate]: result=2 reason=PLANNING_CONTEXT_SHUTDOWN\n'
    owner='[workcell.planning_lifecycle]: PLANNING_CONTEXT_SHUTDOWN\n'
    assert module.shutdown_completion(marker+certificate+owner)['interrupted_after_signal']
    for text in (certificate+owner,marker+certificate,marker+owner,certificate+owner+marker):
        assert not module.shutdown_completion(text)['interrupted_after_signal']
