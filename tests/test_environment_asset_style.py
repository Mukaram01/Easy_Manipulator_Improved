import json
import subprocess
from pathlib import Path

import pytest


ROOT=Path(__file__).resolve().parents[1]


def run_checker(root, tmp_path, strict=True):
    out=tmp_path/'report.json'
    cmd=[
        'python3', 'scripts/check_environment_asset_style.py',
        '--root', str(root), '--json-out', str(out),
    ]
    if strict:
        cmd.append('--strict')
    cp=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True)
    return cp,json.loads(out.read_text())


def _row(data,name):
    return next(r for r in data['results'] if r['name']==name)


def make_environment_package(tmp_path, name, build_type='ament_cmake'):
    package=tmp_path/name
    for directory in ('launch','meshes','rviz','urdf'):
        (package/directory).mkdir(parents=True,exist_ok=True)
    export='' if build_type is None else f'<export><build_type>{build_type}</build_type></export>'
    (package/'package.xml').write_text(
        f'<package format="3"><name>{name}</name><version>0.1.0</version>'
        '<description>test asset</description>'
        '<maintainer email="test@example.com">Test</maintainer>'
        '<license>Apache-2.0</license>'
        f'<buildtool_depend>ament_cmake</buildtool_depend>{export}</package>'
    )
    (package/'CMakeLists.txt').write_text(
        f'cmake_minimum_required(VERSION 3.5)\nproject({name})\n'
        'find_package(ament_cmake REQUIRED)\n'
        'install(DIRECTORY launch meshes rviz urdf DESTINATION share/${PROJECT_NAME})\n'
        'ament_package()\n'
    )
    return package


def test_valid_generic_ament_environment_package_passes(tmp_path):
    make_environment_package(tmp_path,'generic_fixture_description')
    cp,data=run_checker(tmp_path,tmp_path)
    assert cp.returncode==0,cp.stdout+cp.stderr
    assert not _row(data,'generic_fixture_description')['errors']


@pytest.mark.parametrize('build_type',[None,'catkin'])
def test_missing_or_incorrect_build_type_fails_strict_check(tmp_path,build_type):
    make_environment_package(tmp_path,'generic_fixture_description',build_type)
    cp,data=run_checker(tmp_path,tmp_path)
    assert cp.returncode==1
    errors=_row(data,'generic_fixture_description')['errors']
    assert any('must export build_type=ament_cmake' in error for error in errors)
    assert 'ros.catkin' in cp.stdout


def test_real_environment_packages_pass_strict_check(tmp_path):
    cp,data=run_checker(ROOT/'assets/environment',tmp_path)
    assert cp.returncode==0,cp.stdout+cp.stderr
    assert not _row(data,'sorting_bin_description')['errors']
    assert all(not row['errors'] for row in data['results'])
