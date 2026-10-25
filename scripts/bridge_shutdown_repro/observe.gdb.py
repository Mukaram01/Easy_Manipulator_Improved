# Observational GDB commands only: no inferior calls, writes, signal injection,
# scheduler locking, sleeps or source patch. All breakpoints auto-continue.
import gdb, json, os, time
from pathlib import Path
root=Path(os.environ['BRIDGE_OBSERVATION_CYCLE'])
trace=(root/'events.jsonl').open('w', buffering=1)
seq=0

def record(kind, **data):
    global seq
    seq+=1
    thread=gdb.selected_thread()
    inferior=gdb.selected_inferior()
    pid=inferior.pid
    if pid: (root/'inferior.pid').write_text(str(pid))
    row=dict(seq=seq,at_ns=time.monotonic_ns(),kind=kind,pid=pid,
             tid=thread.ptid[1] if thread else None,**data)
    trace.write(json.dumps(row)+'\n')

def stack():
    return gdb.execute('bt 12',to_string=True)

class Return(gdb.FinishBreakpoint):
    def __init__(self,name,address):
        self.label=name;self.address=address
        super().__init__(gdb.newest_frame(),internal=True)
        self.silent=True
    def stop(self):
        record('return',function=self.label,object=self.address,stack=stack())
        return False
    def out_of_scope(self):
        record('return_unobserved',function=self.label,object=self.address)

class Entry(gdb.Breakpoint):
    def __init__(self,name,finish=True):
        self.label=name;self.finish=finish
        super().__init__(name,internal=True)
        self.silent=True
    def stop(self):
        address=str(gdb.parse_and_eval('$rdi'))
        record('entry',function=self.label,object=address,stack=stack())
        if self.finish:
            try: Return(self.label,address)
            except gdb.error as exc: record('observer_error',message=str(exc))
        return False

for setting in ('pagination off','confirm off','breakpoint pending on',
                'disable-randomization off','debuginfod enabled off','print thread-events on'):
    gdb.execute('set '+setting)
gdb.execute('handle SIGINT nostop print pass')
gdb.execute('handle SIGTERM nostop print pass')
gdb.events.new_thread.connect(lambda e:record('thread_created',ptid=list(e.inferior_thread.ptid)))
if hasattr(gdb.events,'thread_exited'):
    gdb.events.thread_exited.connect(lambda e:record('thread_exited',ptid=list(e.inferior_thread.ptid)))
gdb.events.exited.connect(lambda e:record('inferior_exited',exit_code=getattr(e,'exit_code',None)))
for name in ('rcl_shutdown','rcl_logging_fini','rcl_logging_external_shutdown',
             'rclcpp::Context::shutdown(std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&)',
             'rclcpp::Context::~Context()', 'rclcpp::Node::~Node()',
             'rclcpp::SignalHandler::uninstall()',
             'spdlog::details::registry::drop(std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&)',
             'spdlog::details::registry::~registry()'):
    Entry(name)
Entry('rclcpp::SignalHandler::deferred_signal_handler()')
Entry('exit',finish=False)
def stopped(event):
    if isinstance(event,gdb.SignalEvent):
        record('signal',signal=event.stop_signal,stack=stack())
gdb.events.stop.connect(stopped)
gdb.execute('run')
# A naturally fatal signal stops GDB before normal signal delivery. Capture its
# stack, then resume without changing or injecting the pending signal so the
# actual inferior termination is recorded instead of GDB killing it at exit.
while gdb.selected_inferior().pid:
    gdb.execute('continue')
record('observer_finished')
