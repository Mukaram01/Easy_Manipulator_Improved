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
# location instead of relying on ROS environment variables.  ``__file__`` may
# be a relative path depending on how the test is invoked which would cause
# the computed root to be incorrect.  ``os.path.abspath`` normalises the value
# so the test works regardless of the current working directory.
PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


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


# Collect all xacro files once for parametrisation below.  ``glob.glob``
# does not guarantee ordering which can lead to non-deterministic test
# parametrisation.  Sorting ensures stable test order across platforms and
# Python versions.  Additionally, fail early if no files are discovered to
# avoid a false-positive test run where zero tests execute silently.
XACRO_FILES = sorted(
    os.path.basename(f)
    for f in glob.glob(os.path.join(PATH, "tests", "*.xacro"))
)
if not XACRO_FILES:  # pragma: no cover - defensive programming
    raise AssertionError("No xacro files found for validation")


@pytest.mark.parametrize("file", XACRO_FILES)
def test_files(file: str) -> None:
    run_xacro_in_file(file)
