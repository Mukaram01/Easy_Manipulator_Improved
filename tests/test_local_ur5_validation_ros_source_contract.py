from __future__ import annotations

import subprocess
from pathlib import Path


def test_local_validation_sources_ros_setup_without_leaking_nounset_failure():
    repo = Path(__file__).resolve().parents[1]
    script = repo / "scripts" / "run_local_ur5_2f_workcell_validation.sh"
    text = script.read_text(encoding="utf-8")

    assert "source_setup_safely()" in text
    assert "set +u" in text
    assert "set -u" in text
    assert 'source_setup_safely "${ROS_SETUP}"' in text
    assert 'source_setup_safely "${INSTALL_SETUP}"' in text

    subprocess.run(["bash", "-n", str(script)], check=True)
