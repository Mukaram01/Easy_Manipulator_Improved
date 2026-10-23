"""Regression checks for the ROS Humble installation/bootstrap entrypoint.

These checks are source-level checks; successful robot runtime qualification
still requires the real workspace/Stage-A gates.
"""
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "fix_and_build_humble.sh"


def test_bootstrap_bash_syntax():
    result = subprocess.run(["bash", "-n", str(SCRIPT)], capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr


def test_bootstrap_reports_new_flags_and_usage():
    result = subprocess.run(["bash", str(SCRIPT), "--help"], capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr
    assert "--skip-local-proof-packages" in result.stdout
    assert "--epd-underlay" in result.stdout
    assert "--install-prereqs" in result.stdout


def test_embedded_web3d_prerequisites_are_provisioned():
    text = SCRIPT.read_text(encoding="utf-8")
    for token in (
        "libqt5svg5-dev",
        "libqt5opengl5-dev",
        "qtwebengine5-dev",
        'nvm install 20',
        "node20_available",
        "ensure_node20",
    ):
        assert token in text


def test_osqp_eigen_reuses_or_repairs_v1_definitions():
    text = SCRIPT.read_text(encoding="utf-8")
    for token in (
        "OSQP_EIGEN_OSQP_IS_V1;OSQP_EIGEN_OSQP_IS_V1_FINAL",
        "-Dosqp_DIR=/usr/local/lib/cmake/osqp",
        "-UOSQP_IS_V1 -UOSQP_IS_V1_FINAL",
        "OSQPEIGEN_REPAIRED",
        "trajopt_sqp_cache",
    ):
        assert token in text


def test_local_stage_a_proofs_are_never_silently_accepted():
    text = SCRIPT.read_text(encoding="utf-8")
    assert "SKIP_LOCAL_PROOF_PACKAGES=0" in text
    assert "workcell_bridge_shutdown_proof workcell_tem_teardown_repro" in text
    assert "Stage-A motion and shutdown qualification remain UNQUALIFIED" in text
