import subprocess
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def test_no_generated_scene_json_artifacts_are_committed():
    result = subprocess.run(
        ["git", "ls-files", "scenes/*/generated/*.json"],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        check=True,
    )

    committed = [line for line in result.stdout.splitlines() if line.strip()]
    assert committed == []
