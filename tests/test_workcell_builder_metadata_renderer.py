import subprocess,sys

def test_metadata_renderer_help_runs():
    cp=subprocess.run([sys.executable,'scripts/render_workcell_builder_metadata.py','--help'],capture_output=True,text=True)
    assert cp.returncode==0
    assert 'ModuleNotFoundError' not in (cp.stdout+cp.stderr)
