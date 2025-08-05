#!/usr/bin/env python3

"""Validate xacro files in the package.

The original test relied on the ROS ``rospkg`` package to locate the
``realsense2_description`` package and used nose-style ``yield`` tests.
Both assumptions break in a minimal environment: ``rospkg`` is often not
available and ``yield`` based tests are no longer supported by ``pytest``.

To make the check robust we compute the package path relative to this
file and generate individual parametrised tests for every ``.xacro``
file.  The test is skipped automatically when the ``xacro``/``roslaunch``
dependencies are missing.
"""

import glob
import os
import subprocess

import pytest

# Skip the entire test module if required ROS tools are not available.
pytest.importorskip("xacro")
pytest.importorskip("roslaunch")

# Determine the root of the realsense2_description package using the file
# location instead of relying on ROS environment variables.
PATH = os.path.dirname(os.path.dirname(__file__))


def run_xacro_in_file(filename: str) -> None:
    """Run ``xacro`` on a single file to ensure it parses correctly."""
    assert filename
    try:
        subprocess.check_output(
            [
                "xacro",
                "--inorder",
                os.path.join("tests", filename),
            ],
            cwd=PATH,
            stderr=subprocess.STDOUT,
        )
    except (FileNotFoundError, subprocess.CalledProcessError) as exc:  # pragma: no cover - error path
        # ``xacro`` relies on ROS tools such as ``roslaunch`` and ``rosgraph``.
        # When those tools are missing, the subprocess fails which would
        # otherwise mark the test as an error.  Instead, skip the test so that
        # environments without the ROS stack don't report a failure.
        output = getattr(exc, "output", b"").decode("utf-8", errors="ignore")
        if "No module named" in output or isinstance(exc, FileNotFoundError):
            pytest.skip("xacro or ROS dependencies not available")
        raise


# Collect all xacro files once for parametrisation below
XACRO_FILES = [
    os.path.basename(f)
    for f in glob.glob(os.path.join(PATH, "tests", "*.xacro"))
]


@pytest.mark.parametrize("file", XACRO_FILES)
def test_files(file: str) -> None:
    run_xacro_in_file(file)
