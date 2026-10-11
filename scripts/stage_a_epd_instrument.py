#!/usr/bin/env python3
"""Prepare an isolated read-only EPD observer; explicit one-shot execution only.

Never modifies external EPD or existing build trees. Synthetic preparation opens
no model session. The inference claim is exclusive and remains after any failure.
"""
import argparse
import gzip
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import datetime

import stage_a_epd_retained as adapter
import stage_a_epd_spill as baseline

ROOT = Path(__file__).resolve().parents[1]
EPD = Path('/home/user/epd_ros2_ws/src/easy_perception_deployment/easy_perception_deployment')
MODEL = Path('/home/user/stage_a_cube_model/cube_maskrcnn.onnx')
LABELS = MODEL.with_name('labels.txt')
HEADER = ROOT / 'scripts/stage_a_rgbd/epd_observer.hpp'
SYNTHETIC = ROOT / 'scripts/stage_a_rgbd/epd_observer_synthetic.cpp'


def strip_observers(text):
    return re.sub(r'/\* EPD_OBSERVER_BEGIN \*/.*?/\* EPD_OBSERVER_END \*/\n', '', text, flags=re.S)


def instrument(name, text):
    pre = json.loads(gzip.decompress((baseline.RETAINED / 'epd/preflight.json.gz').read_bytes()))
    expected = pre['source_sha256'][str(EPD / 'src' / name)]
    adapter.require(adapter.sha(text.encode()) == expected, 'changed external source: ' + name)
    def insert(source, anchor, observation):
        adapter.require(source.count(anchor) == 1, 'ambiguous observer boundary')
        return source.replace(anchor, anchor + '/* EPD_OBSERVER_BEGIN */\n' + observation + '\n/* EPD_OBSERVER_END */\n')
    text = '/* EPD_OBSERVER_BEGIN */\n#include "epd_observer.hpp"\n/* EPD_OBSERVER_END */\n' + text
    if name == 'ort_base.cpp':
        # Insert before Run rather than inside its argument list.
        anchor = '  m_outputTensors = m_session.Run(\n'
        adapter.require(text.count(anchor) == 1, 'ambiguous Session::Run')
        text = text.replace(anchor, '/* EPD_OBSERVER_BEGIN */\n'
            '  epd_observer::tensors("session_inputs_before_Run",inputTensors,m_inputNodeNames);\n'
            '/* EPD_OBSERVER_END */\n' + anchor)
        text = insert(text, '    m_outputNodeNames.data(),\n    m_numOutputs);\n',
                      '  epd_observer::tensors("session_outputs_after_Run",m_outputTensors,m_outputNodeNames);')
    else:
        text = insert(text, '        masks.emplace_back(curMask(cv::Rect(xmin, ymin, xmax - xmin, ymax - ymin)).clone());\n',
            '        epd_observer::roi(i,bboxes.size()-1,inferenceOutput[0].first+i*4,scale_x,scale_y,\n'
            '                          xmin,ymin,xmax,ymax,curMask,masks.back());')
    return text


def claim_once(folder):
    with (Path(folder) / 'inference_claim.json').open('x') as file:
        json.dump({'maximum_inference_calls': 1, 'retry_allowed': False, 'simulator_runs': 0}, file)


def command(args, cwd, log, env=None, timeout=300):
    with Path(log).open('w') as file:
        file.write(json.dumps(args) + '\n'); file.flush()
        result = subprocess.run(args, cwd=cwd, env=env, stdout=file, stderr=subprocess.STDOUT, timeout=timeout)
    adapter.require(result.returncode == 0, 'diagnostic command failed; inspect ' + str(log))


def dependencies(binary):
    result = subprocess.check_output(['ldd', str(binary)], text=True)
    adapter.require('not found' not in result, 'unresolved runtime dependency')
    paths = re.findall(r'(?:=> )?(/[^\s]+)', result)
    return {p: adapter.sha(Path(p).read_bytes()) for p in paths}


def external_sources():
    return {str(p): adapter.sha(p.read_bytes()) for group in ('src', 'include')
            for p in sorted((EPD / group).rglob('*')) if p.is_file()}


def prepare(folder):
    import onnx  # static graph inspection only; no inference
    folder = Path(folder).resolve(); folder.mkdir()  # exclusive preparation
    baseline.load_retained()  # Verify original provenance before any build.
    old = json.loads(gzip.decompress((baseline.RETAINED / 'epd/preflight.json.gz').read_bytes()))
    for name, digest in old['source_sha256'].items():
        adapter.require(adapter.sha(Path(name).read_bytes()) == digest, 'changed source: ' + name)
    adapter.require(adapter.sha(MODEL.read_bytes()) == old['model_sha256'] and
                    adapter.sha(LABELS.read_bytes()) == old['labels_sha256'], 'changed model/labels')
    historical = json.loads(gzip.decompress((baseline.RETAINED / 'epd/result.json.gz').read_bytes()))
    for name, digest in historical['loaded_library_sha256'].items():
        adapter.require(adapter.sha(Path(name).read_bytes()) == digest, 'changed historical runtime dependency: ' + name)
    sources = folder / 'source'; (sources / 'src').mkdir(parents=True)
    (sources / 'include').symlink_to(EPD / 'include', target_is_directory=True)
    for name in ('ort_base.cpp', 'p3_ort_base.cpp'):
        original = (EPD / 'src' / name).read_text(); modified = instrument(name, original)
        adapter.require(strip_observers(modified) == original, 'instrumentation changed original source')
        (sources / 'src' / name).write_text(modified)
        (sources / ('original_' + name)).write_text(original)
    env = dict(os.environ, EPD_EXECUTION_BACKEND='cpu')
    # Same standalone EPD caller/build options; only external source location differs.
    command(['cmake', '-S', str(ROOT / 'scripts/stage_a_rgbd'), '-B', str(folder / 'build'),
        '-DEPD_SOURCE_DIR=' + str(sources), '-DCMAKE_CXX_FLAGS=-I' + str(HEADER.parent)], ROOT, folder / 'configure.log', env)
    command(['cmake', '--build', str(folder / 'build'), '--target', 'stage_a_epd_retained', '-j2'],
            ROOT, folder / 'build.log', env)
    build = folder / 'build'; target = build / 'CMakeFiles/stage_a_epd_retained.dir'
    flags = (target / 'flags.make').read_text()
    args = []
    for key in ('CXX_DEFINES', 'CXX_INCLUDES', 'CXX_FLAGS'):
        args += shlex.split(re.search(r'^' + key + r' = (.*)$', flags, re.M)[1])
    links = shlex.split((target / 'link.txt').read_text())
    libraries = links[links.index('-o') + 2:]
    # Exercise actual original and instrumented P3 infer paths using fake OrtBase.
    # Includes typed observer tests, but creates no Session and opens no model.
    for mode, p3 in [('original', sources / 'original_p3_ort_base.cpp'), ('observed', sources / 'src/p3_ort_base.cpp')]:
        binary = folder / ('synthetic_' + mode)
        command(['/usr/bin/c++', *args, '-UNDEBUG', str(SYNTHETIC), str(p3), '-o', str(binary), *libraries],
                ROOT, folder / ('synthetic_build_' + mode + '.log'), env)
        output = folder / ('synthetic_' + mode + '_output'); output.mkdir()
        command([str(binary), str(output / 'pass.txt')], ROOT, folder / ('synthetic_' + mode + '.log'),
                {**env, 'EPD_OBSERVER_DIR': str(output)}, 30)
    a, b = folder / 'synthetic_original_output', folder / 'synthetic_observed_output'
    for path in a.iterdir():
        adapter.require(path.read_bytes() == (b / path.name).read_bytes(), 'observer changed synthetic tensor/path output')
    # Also compare observed copied masks and crop bytes with known synthetic output.
    for i in (0, 1):
        adapter.require((b / f'detection_{i}.roi.f32').read_bytes() == (b / f'synthetic_result_{i}.f32').read_bytes(), 'synthetic ROI mismatch')
    model = onnx.load(str(MODEL), load_external_data=False)
    producers = {o: n for n in model.graph.node for o in n.output}
    contract = []
    for output in model.graph.output:
        node = producers[output.name]
        contract.append({'name': output.name, 'type': onnx.helper.printable_type(output.type),
                         'producer': {'name': node.name, 'op_type': node.op_type, 'inputs': list(node.input)}})
    (folder / 'model_contract.json').write_text(json.dumps({'outputs': contract,
        'stage': 'exported ONNX graph outputs; may include graph-internal mask pasting/postprocessing',
        'opset': [{'domain': o.domain, 'version': o.version} for o in model.opset_import]}, indent=2))
    rgb = gzip.decompress((baseline.RETAINED / 'epd/input.rgb8.gz').read_bytes())
    (folder / 'input.rgb8').write_bytes(rgb)
    binary = build / 'stage_a_epd_retained'
    code = [Path(__file__), HEADER, SYNTHETIC, ROOT / 'scripts/stage_a_rgbd/epd_retained.cpp',
            ROOT / 'scripts/stage_a_rgbd/CMakeLists.txt', *sources.glob('src/*.cpp'), *sources.glob('original_*.cpp')]
    preflight = dict(baseline='6640cea2', preprocessing=adapter.preprocessing(),
        original_preflight=old, input_sha256=adapter.sha(rgb), binary=str(binary), binary_sha256=adapter.sha(binary.read_bytes()),
        source_sha256={str(p): adapter.sha(p.read_bytes()) for p in code},
        dependency_sha256=dependencies(binary), historical_loaded_library_sha256=historical['loaded_library_sha256'],
        external_source_sha256=external_sources(),
        external_git_status=subprocess.check_output(['git', '-C', str(EPD), 'status', '--porcelain=v1'], text=True),
        validation_sha256={str(p): adapter.sha(p.read_bytes()) for p in folder.glob('synthetic*') if p.is_file()},
        environment={k: env.get(k) for k in ('EPD_EXECUTION_BACKEND', 'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH')},
        synthetic_tests='PASS_ORIGINAL_AND_OBSERVED_IDENTICAL_NO_MODEL_INFERENCE', timeout_seconds=120)
    for group in (a, b):
        preflight['validation_sha256'].update({str(p): adapter.sha(p.read_bytes()) for p in group.iterdir()})
    (folder / 'preflight.json').write_text(json.dumps(preflight, indent=2) + '\n')
    return preflight


def execute(folder):
    folder = Path(folder).resolve(); pre = json.loads((folder / 'preflight.json').read_text())
    baseline.load_retained()
    adapter.require(pre['synthetic_tests'] == 'PASS_ORIGINAL_AND_OBSERVED_IDENTICAL_NO_MODEL_INFERENCE', 'synthetic gate failed')
    adapter.check_preprocessing(pre['preprocessing'])
    for name, digest in {**pre['source_sha256'], **pre['external_source_sha256'], **pre['validation_sha256'],
                         **pre['dependency_sha256'], **pre['historical_loaded_library_sha256'], pre['binary']: pre['binary_sha256'],
                         str(MODEL): pre['original_preflight']['model_sha256'], str(LABELS): pre['original_preflight']['labels_sha256'],
                         str(folder / 'input.rgb8'): pre['input_sha256']}.items():
        adapter.require(adapter.sha(Path(name).read_bytes()) == digest, 'changed preflight artifact: ' + name)
    adapter.require(dependencies(Path(pre['binary'])) == pre['dependency_sha256'], 'changed loader dependency resolution')
    adapter.require(external_sources() == pre['external_source_sha256'], 'changed external source inventory')
    adapter.require(subprocess.check_output(['git', '-C', str(EPD), 'status', '--porcelain=v1'], text=True) == pre['external_git_status'], 'changed external checkout status')
    for key, value in pre['environment'].items():
        adapter.require((('cpu' if key == 'EPD_EXECUTION_BACKEND' else os.environ.get(key))) == value, 'changed execution environment: ' + key)
    output = folder / 'output'; output.mkdir()
    claim_once(folder)  # Remains consumed even on failure; no retry path.
    args = [pre['binary'], str(MODEL), str(LABELS), str(folder / 'input.rgb8'), str(output)]
    env = dict(os.environ, EPD_EXECUTION_BACKEND='cpu', EPD_OBSERVER_DIR=str(output))
    started = datetime.datetime.now(datetime.timezone.utc).isoformat()
    with (folder / 'stdout.log').open('wb') as stdout, (folder / 'stderr.log').open('wb') as stderr:
        try:
            proc = subprocess.run(args, env=env, stdout=stdout, stderr=stderr, timeout=120)
            status = {'exit_code': proc.returncode, 'timeout': False}
        except subprocess.TimeoutExpired:
            status = {'exit_code': None, 'timeout': True}
    status.update(command=args, inference_attempts=1, execution_goals=0, simulator_runs=0,
                  started_utc=started, completed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
    (folder / 'process_exit.json').write_text(json.dumps(status, indent=2) + '\n')
    adapter.require(status['exit_code'] == 0, 'instrumented experiment failed; STOP, no retry')
    native = json.loads((output / 'inference.json').read_text())
    loaded = {p: adapter.sha(Path(p).read_bytes()) for p in native['loaded_library_paths']}
    (folder / 'loaded_library_sha256.json').write_text(json.dumps(loaded, indent=2) + '\n')
    # These are hashes of actual /proc/self/maps paths recorded by the caller.
    # Require every loaded dependency to be among the verified loader closure.
    resolved = {str(Path(p).resolve()): h for p, h in {**pre['historical_loaded_library_sha256'], **pre['dependency_sha256']}.items()}
    adapter.require(all(p in resolved and resolved[p] == h
                        for p, h in loaded.items()), 'unexpected loaded runtime library; no retry')
    after = dict(source_sha256=external_sources(), git_status=subprocess.check_output(['git', '-C', str(EPD), 'status', '--porcelain=v1'], text=True))
    (folder / 'external_after.json').write_text(json.dumps(after, indent=2) + '\n')
    adapter.require(after['source_sha256'] == pre['external_source_sha256'] and
                    after['git_status'] == pre['external_git_status'], 'external checkout changed during inference')
    return status


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('mode', choices=['prepare', 'execute']); parser.add_argument('folder', type=Path)
    args = parser.parse_args()
    print(json.dumps(prepare(args.folder) if args.mode == 'prepare' else execute(args.folder), indent=2))
