import subprocess
import sys
from pathlib import Path

import importlib.util


MODULE_PATH = Path("scripts/check_colcon_build_warnings.py")
spec = importlib.util.spec_from_file_location("check_colcon_build_warnings", MODULE_PATH)
module = importlib.util.module_from_spec(spec)
assert spec and spec.loader
spec.loader.exec_module(module)
classify_warnings = module.classify_warnings


def test_boost_bind_warning_in_run_grasp_execution_is_owned():
    log = """
[5.432s] Building CXX object easy_manipulation_deployment/emd_demo_nodes/run_grasp_execution/CMakeFiles/demo_node.dir/src/demo_node.cpp.o
In file included from /usr/include/boost/bind.hpp:30:
#warning The practice of declaring the Bind placeholders (_1, _2, ...) in the global namespace is deprecated.
"""
    classified = classify_warnings(log)
    assert len(classified["owned_warnings"]) == 1


def test_osqp_warning_is_known_third_party():
    log = """
CMake Warning (dev) at cmake/OsqpEigenDependencies.cmake:37 (set):
  Cannot set \"OSQP_EIGEN_OSQP_TARGET_TO_LINK\": current scope has no parent.
Call Stack (most recent call first):
"""
    classified = classify_warnings(log)
    assert len(classified["known_third_party"]) == 1


def test_assimp_warning_from_tesseract_is_known_third_party():
    log = """
[12.111s] Building CXX object tesseract_geometry/CMakeFiles/foo.dir/bar.cpp.o
/usr/include/assimp/pbrmaterial.h: warning: pbrmaterial.h is deprecated. Please update to PBR materials in materials.h and glTF-specific items in GltfMaterial.h
"""
    classified = classify_warnings(log)
    assert len(classified["known_third_party"]) == 1


def test_unknown_warning_fails_with_fail_on_owned(tmp_path: Path):
    log_path = tmp_path / "colcon.log"
    log_path.write_text("/tmp/other_pkg/file.cpp:9: warning: unused variable 'x'\n", encoding="utf-8")
    result = subprocess.run(
        [sys.executable, "scripts/check_colcon_build_warnings.py", "--log", str(log_path), "--fail-on-owned"],
        check=False,
    )
    assert result.returncode == 1


def test_quiet_meta_only_contains_third_party_packages():
    meta_path = Path("colcon/quiet_third_party_warnings.meta")
    text = meta_path.read_text(encoding="utf-8")
    assert '"osqp_eigen"' in text
    assert '"tesseract_geometry"' in text
    assert '"tesseract_urdf"' in text
    assert '"tesseract_examples"' in text
    assert "run_grasp_execution" not in text
    assert "easy_manipulation_deployment" not in text
