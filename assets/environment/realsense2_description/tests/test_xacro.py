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
    subprocess.check_output([
        "xacro",
        "--inorder",
        os.path.join("tests", filename),
    ], cwd=PATH)


# Collect all xacro files once for parametrisation below
XACRO_FILES = [
    os.path.basename(f)
    for f in glob.glob(os.path.join(PATH, "tests", "*.xacro"))
]


@pytest.mark.parametrize("file", XACRO_FILES)
def test_files(file: str) -> None:
    run_xacro_in_file(file)
