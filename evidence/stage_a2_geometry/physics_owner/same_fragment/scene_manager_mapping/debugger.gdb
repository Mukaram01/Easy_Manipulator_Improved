set pagination off
set confirm off
set disable-randomization off
handle SIGILL stop print pass
handle SIGSEGV stop print pass
python
import gdb,json
from pathlib import Path
out=Path('/tmp/stage_a2_scene_map_20261010_3ee79522')
def exited(event):
    if not (out/'child_exit.json').exists():
        (out/'child_exit.json').write_text(json.dumps({'exit_code':getattr(event,'exit_code',None)}))
def stopped(event):
    if isinstance(event,gdb.SignalEvent):
        pid=gdb.selected_inferior().pid
        (out/'child_exit.json').write_text(json.dumps({'signal':event.stop_signal,'pid':pid,'continued_after_signal':False}))
        try:(out/'crash_maps.txt').write_text(Path('/proc/'+str(pid)+'/maps').read_text())
        except OSError:pass
        gdb.execute('thread apply all bt full')
        gdb.execute('info sharedlibrary')
        gdb.execute('info registers')
gdb.events.exited.connect(exited)
gdb.events.stop.connect(stopped)
end
run
quit
