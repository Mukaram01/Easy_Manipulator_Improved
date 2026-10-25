"""Evidence checks for bounded bridge shutdown qualification methods.

The independent mechanism report supplies causal judgment; hashes and raw
records make that judgment inspectable and reject stale or partial evidence.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex

LEGACY_METHOD = 'legacy-installed-baseline/v1'
LIFETIME_METHOD = 'deterministic-source-lifetime/v1'
OBSERVED_METHOD = 'observed-shutdown-order/v1'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def identity(path):
    path = Path(path).resolve(strict=True)
    return dict(path=str(path), sha256=digest(path))


def checked_file(record):
    if not isinstance(record, dict) or not isinstance(record.get('path'), str):
        raise ValueError('missing evidence file identity')
    path = Path(record['path'])
    if not path.is_absolute() or digest(path) != record.get('sha256'):
        raise ValueError('evidence file hash mismatch: ' + str(path))
    return path


def method(manifest):
    if 'qualification' not in manifest:
        return LEGACY_METHOD
    qualification = manifest['qualification']
    if not isinstance(qualification, dict) or qualification.get('method') not in (LEGACY_METHOD, LIFETIME_METHOD, OBSERVED_METHOD):
        raise ValueError('unknown or missing bridge qualification method')
    if qualification['method'] == LEGACY_METHOD and set(qualification) != {'method'}:
        raise ValueError('alternative evidence requires its explicit qualification method')
    return qualification['method']


# Repository-tracked admission, harness and observation files. A record of one is
# bound to its checkout-relative name and bytes and is checked against the running
# checkout, so the same reviewed bytes in a relocated checkout are the same
# implementation while any changed byte still fails closed. Every other record is
# evidence, checked in place at its absolute path.
REPOSITORY_FILES = (
    'patches/ros_gz_bridge_humble_shutdown.patch',
    'scripts/build_ros_gz_bridge_shutdown_overlay.py', 'scripts/simulator_backend.py',
    'scripts/bridge_shutdown_repro/qualification.py', 'scripts/bridge_shutdown_repro/verify.py',
    'scripts/bridge_shutdown_repro/observe.py', 'scripts/bridge_shutdown_repro/observe.gdb.py',
    'scripts/bridge_shutdown_repro/CMakeLists.txt', 'scripts/bridge_shutdown_repro/publisher.cpp',
    'scripts/bridge_shutdown_repro/bridges.yaml', 'scripts/bridge_shutdown_repro/shutdown_context_test.cpp',
    'tests/test_bridge_shutdown_overlay.py')


def bound_identity(record):
    """Identity key of a record: its repository name, or its checked absolute evidence path."""
    if not isinstance(record, dict) or not isinstance(record.get('path'), str):
        raise ValueError('missing evidence file identity')
    path = Path(record['path'])
    for name in REPOSITORY_FILES:
        parts = Path(name).parts
        if path.is_absolute() and path.parts[-len(parts):] == parts:
            if digest(Path(__file__).resolve().parents[2] / name) != record.get('sha256'):
                raise ValueError('repository implementation differs from the reviewed bytes: ' + name)
            return name
    return str(checked_file(record).resolve())


def _identities(records):
    if not isinstance(records, list) or not records:
        raise ValueError('nonempty source/build identities required')
    paths = [bound_identity(item) for item in records]
    if len(set(paths)) != len(paths):
        raise ValueError('duplicate source/build identity')
    return dict(zip(paths, (item['sha256'] for item in records)))


def _text(value):
    return isinstance(value, str) and bool(value.strip())


def _trace(phase, corrected, bound):
    if (phase['timed_out'] is not False or phase['exercised'] is not True or
        type(phase['pid']) is not int or phase['pid'] <= 0 or
        type(phase['returncode']) is not int or
        (corrected and phase['returncode'] != 0) or
        type(phase['elapsed_seconds']) not in (float, int) or
        not 0 < phase['elapsed_seconds'] <= bound or
        phase['classification'] != ('lifetime-ordering-prevented' if corrected else 'lifetime-use-after-destruction')):
        raise ValueError('unexercised, timed out, or unclassified lifetime experiment')
    executable = checked_file(phase['executable'])
    if (not isinstance(phase['command'], list) or not phase['command'] or
        any(not _text(arg) for arg in phase['command']) or str(executable) not in phase['command']):
        raise ValueError('actual lifetime command missing')
    maps = checked_file(phase['maps']).read_text()
    _maps_contain(maps, phase['executable'])
    _identities(phase['loaded_libraries'])
    for library in phase['loaded_libraries']:
        _maps_contain(maps, library)
    output = checked_file(phase['stdout']).read_text()
    if 'memory_diagnostics' in phase:
        output += checked_file(phase['memory_diagnostics']).read_text()
    if 'RCLError' in output:
        raise ValueError('unrelated RCLError is not a lifetime mechanism')
    events = [json.loads(line) for line in checked_file(phase['trace']).read_text().splitlines() if line.strip()]
    if not events:
        raise ValueError('lifetime trace contains no observed events')
    for seq, event in enumerate(events, 1):
        if (not isinstance(event, dict) or type(event['seq']) is not int or event['seq'] != seq or
            type(event['pid']) is not int or event['pid'] != phase['pid'] or type(event['tid']) is not int or event['tid'] <= 0 or
            not _text(event['object']) or not isinstance(event['stack'], list) or len(event['stack']) < 2 or
            any(not _text(frame) for frame in event['stack']) or
            event['kind'] not in ('object_destroyed', 'object_access', 'signal_thread_stopped')):
            raise ValueError('lifetime event lacks a process, object, order, or captured call stack')
    destroyed = [event for event in events if event['kind'] == 'object_destroyed']
    accesses = [event for event in events if event['kind'] == 'object_access']
    stopped = [event for event in events if event['kind'] == 'signal_thread_stopped']
    if len(destroyed) != 1 or not accesses or any(event['object'] != destroyed[0]['object'] for event in events):
        raise ValueError('lifetime events do not identify one exercised destroyed object')
    destruction = destroyed[0]
    if corrected:
        if (len(stopped) != 1 or not max(event['seq'] for event in accesses) < stopped[0]['seq'] < destruction['seq']):
            raise ValueError('corrected trace does not show access completion and signal-thread stop before destruction')
    elif not any(event['seq'] > destruction['seq'] and event['tid'] != destruction['tid'] for event in accesses):
        raise ValueError('uncorrected trace does not show another thread accessing the destroyed object')


def _dependency_builds(phase, evidence):
    """Instrumented dependencies keep distinct identities and explicit source bindings."""
    loaded = _identities(phase['loaded_libraries'])
    reviewed_files = []
    builds = phase.get('dependency_builds', [])
    if not isinstance(builds, list):
        raise ValueError('invalid instrumented dependency builds')
    for dependency in evidence['runtime']['dependencies']:
        if loaded.get(str(Path(dependency['path']).resolve())) == dependency['sha256']:
            continue
        matches = [build for build in builds if build['runtime_library'] == dependency]
        if len(matches) != 1:
            raise ValueError('instrumented dependency needs its pinned source and build binding')
        build = matches[0]
        checked_file(build['library'])
        if (loaded.get(str(Path(build['library']['path']).resolve())) != build['library']['sha256'] or
            build['source_dependency'] not in evidence['source']['dependencies']):
            raise ValueError('instrumented dependency build does not match its loaded library/source')
        checked_file(build['build_config'])
        _identities(build['instrumentation'])
        if any(item not in evidence['harness']['instrumentation'] for item in build['instrumentation']):
            raise ValueError('instrumented dependency changes were not declared by the harness')
        reviewed_files.extend([build['build_config'], *build['instrumentation']])
    return reviewed_files


def validate_lifetime(manifest, include_runtime=True):
    """Require inspectable causal traces plus a separately authored bound review.

    This validates evidence structure and identity, not the reviewer's scientific
    judgment. The report must explain the real instrumentation and causal limit.
    """
    try:
        if method(manifest) != LIFETIME_METHOD:
            raise ValueError('explicit deterministic lifetime method required')
        if ('baseline_returncode' in manifest['reproduction'] or
            any(key in manifest['baseline'] for key in ('results_path', 'results_sha256'))):
            raise ValueError('deterministic evidence cannot populate installed-baseline reproduction')
        qualification = manifest['qualification']
        evidence_path = checked_file(qualification['evidence'])
        evidence = json.loads(evidence_path.read_text())
        if (evidence['schema'] != 'workcell_bridge_lifetime_evidence/v1' or
            evidence['method'] != LIFETIME_METHOD or not _text(evidence['producer']) or
            evidence['source']['bridge_commit'] != manifest['source']['commit'] or
            evidence['patch'] != manifest['patch']):
            raise ValueError('lifetime evidence source, method, or patch mismatch')
        _identities(evidence['source']['files'])
        for name in ('rclcpp', 'rcl', 'rcutils', 'rcl_logging_spdlog', 'spdlog'):
            dependency = evidence['source']['dependencies'][name]
            if not _text(dependency['revision']):
                raise ValueError('unpinned lifetime dependency: ' + name)
            _identities(dependency['files'])
        runtime = evidence['runtime']
        for name in ('executable', 'library'):
            checked_file(runtime[name])
            if runtime[name] != manifest[name]:
                raise ValueError('runtime binary differs from the reviewed lifetime record')
        checked_file(runtime['source_main'])
        if runtime['source_main']['sha256'] != manifest['source']['patched_main_sha256']:
            raise ValueError('runtime source differs from the reviewed lifetime record')
        if runtime['dependencies'] != qualification['runtime_dependencies']:
            raise ValueError('runtime dependencies differ from the reviewed lifetime record')
        _identities(runtime['dependencies'])
        harness = evidence['harness']
        implementation = [*harness['files'], *harness['instrumentation'], harness['instrumentation_notes'],
                          harness['lifetime_patch'], harness['build_config'], manifest['patch']]
        _identities(harness['files'])
        _identities(harness['instrumentation'])
        for item in implementation:
            bound_identity(item)
        bound = evidence['finite_bound_seconds']
        if type(bound) not in (int, float) or not math.isfinite(bound) or bound <= 0:
            raise ValueError('a positive finite lifetime test bound is required')
        if evidence['before']['pid'] == evidence['after']['pid']:
            raise ValueError('matched lifetime experiments require separate fresh processes')
        for name, corrected in (('before', False), ('after', True)):
            phase = evidence[name]
            _trace(phase, corrected, bound)
            if phase['executable']['sha256'] == runtime['executable']['sha256']:
                raise ValueError('instrumented harness cannot be the runtime executable')
            implementation.extend(_dependency_builds(phase, evidence))
        review = json.loads(checked_file(qualification['review']).read_text())
        if (review['schema'] != 'workcell_bridge_lifetime_review/v1' or review['disposition'] != 'accepted' or
            not _text(review['reviewer']) or review['reviewer'] == evidence['producer'] or
            review['evidence_sha256'] != qualification['evidence']['sha256'] or
            type(review['reviewed_at_ns']) is not int or review['reviewed_at_ns'] <= 0):
            raise ValueError('missing, rejected, or mismatched independent mechanism review')
        report = checked_file(review['report']).read_text()
        if len(report.strip()) < 200 or any(not _text(review['assessment'][key]) for key in
                ('causal_adequacy', 'scheduling', 'correction', 'runtime_isolation')):
            raise ValueError('a substantive mechanism report is required; a reviewed boolean is insufficient')
        validation = review['validation']
        if (not _text(validation['source_inspection']) or
            not isinstance(validation['independently_executed_tests'], list) or
            not isinstance(validation['inspected_test_logs'], list)):
            raise ValueError('review must distinguish inspected source from independently executed tests')
        repo = Path(__file__).resolve().parents[2]
        for field, name in (('build_script_sha256', 'build_ros_gz_bridge_shutdown_overlay.py'),
                            ('proof_script_sha256', 'bridge_shutdown_repro/verify.py'),
                            ('publisher_source_sha256', 'bridge_shutdown_repro/publisher.cpp'),
                            ('configuration_sha256', 'bridge_shutdown_repro/bridges.yaml')):
            if manifest[field] != digest(repo / 'scripts' / name):
                raise ValueError('runtime implementation hash mismatch: ' + name)
        context = manifest['shutdown_context_test']
        if context['source_sha256'] != digest(repo / 'scripts/bridge_shutdown_repro/shutdown_context_test.cpp'):
            raise ValueError('context regression implementation hash mismatch')
        if include_runtime:
            checked_file(dict(path=context['log_path'], sha256=context['log_sha256']))
        implementation.extend(identity(repo / name) for name in (
            'scripts/build_ros_gz_bridge_shutdown_overlay.py', 'scripts/simulator_backend.py',
            'scripts/bridge_shutdown_repro/qualification.py', 'scripts/bridge_shutdown_repro/verify.py',
            'scripts/bridge_shutdown_repro/CMakeLists.txt', 'scripts/bridge_shutdown_repro/publisher.cpp',
            'scripts/bridge_shutdown_repro/bridges.yaml', 'scripts/bridge_shutdown_repro/shutdown_context_test.cpp',
            'tests/test_bridge_shutdown_overlay.py'))
        reviewed = _identities(review['implementation'])
        if any(reviewed.get(bound_identity(item)) != item['sha256'] for item in implementation):
            raise ValueError('independent review is not bound to the exact admission/harness implementation')
        if include_runtime:
            validate_runtime(manifest, review['reviewed_at_ns'])
        return evidence
    except (KeyError, TypeError, OSError, IndexError, AttributeError) as exc:
        raise ValueError('incomplete lifetime evidence: ' + str(exc)) from exc


def _maps_contain(maps, record):
    path = checked_file(record).resolve(strict=True)
    entries = [line.split() for line in maps.splitlines() if line.split()[-1:] == [str(path)]]
    if not entries or any(len(row) != 6 or int(row[4]) != path.stat().st_ino for row in entries):
        raise ValueError('loaded mapping identity mismatch: ' + str(path))


def validate_runtime(manifest, reviewed_at_ns=None):
    """Recheck every raw row; summary success never substitutes for a full batch."""
    try:
        if method(manifest) in (LIFETIME_METHOD, OBSERVED_METHOD):
            _runtime_environment(manifest)
        started = manifest['runtime_started_ns']
        if type(started) is not int or started <= 0 or (reviewed_at_ns is not None and started < reviewed_at_ns):
            raise ValueError('runtime must follow the bound independent review')
        binary = checked_file(manifest['executable']).resolve()
        library = checked_file(manifest['library']).resolve()
        pids = set()
        for name, count in (('native', 20), ('memcheck', 5)):
            summary = manifest['reproduction'][name]
            path = checked_file(dict(path=summary['results_path'], sha256=summary['results_sha256']))
            record = json.loads(path.read_text())
            if (any(type(summary[key]) is not int or summary[key] != expected
                    for key, expected in (('cycles', count), ('passed', count), ('errors', 0))) or
                any(type(record[key]) is not int or record[key] != expected
                    for key, expected in (('cycles', count), ('completed', count), ('passed', count), ('errors', 0))) or
                record['all_success'] is not True or record['expected_abort'] is not False or
                record['binary'] != str(binary) or record['binary_sha256'] != manifest['executable']['sha256'] or
                record['library'] != str(library) or record['library_sha256'] != manifest['library']['sha256'] or
                len(record['results']) != count):
                raise ValueError('complete identity-bound runtime ' + name + ' batch required')
            artifacts = {item['path']: item for item in summary['artifacts']}
            expected_paths = {str(path.parent / f'{cycle:02}-{suffix}')
                              for cycle in range(1, count + 1)
                              for suffix in ('bridge.log', 'publisher.log', 'bridge.maps') +
                              (('memcheck.log',) if name == 'memcheck' else ())}
            if set(artifacts) != expected_paths:
                raise ValueError('runtime raw log/map inventory is incomplete')
            for artifact in artifacts.values():
                checked_file(artifact)
            if any(Path(item).stat().st_mtime_ns < started for item in [str(path), *artifacts]):
                raise ValueError('runtime evidence predates this fresh qualification')
            for cycle, row in enumerate(record['results'], 1):
                if (type(row['cycle']) is not int or row['cycle'] != cycle or row['passed'] is not True or
                    type(row['received']) is not int or row['received'] < 300 or
                    not isinstance(row['returncodes'], list) or len(row['returncodes']) != 2 or
                    any(type(code) is not int or code != 0 for code in row['returncodes']) or
                    row['loaded_libraries'] != [str(library)] or row['mapped_executable'] is not True or
                    (name == 'native' and row['loaded_executable'] != str(binary)) or
                    type(row['memcheck_errors']) is not int or row['memcheck_errors'] != 0 or
                    row['core_inventory_changed'] is not False or row['remaining_processes'] != [] or
                    row.get('forced_kill') or 'exception' in row or
                    len(row['pids']) != 2 or any(type(pid) is not int or pid <= 0 or pid in pids for pid in row['pids']) or
                    len(set(row['pids'])) != 2):
                    raise ValueError('runtime cycle lacks clean active traffic and owned shutdown')
                pids.update(row['pids'])
                maps = (path.parent / f'{cycle:02}-bridge.maps').read_text()
                _maps_contain(maps, manifest['executable'])
                _maps_contain(maps, manifest['library'])
                for dependency in manifest.get('qualification', {}).get('runtime_dependencies', []):
                    _maps_contain(maps, dependency)
                log = (path.parent / f'{cycle:02}-bridge.log').read_text()
                if 'RCLError' in log or 'corrupted unsorted chunks' in log:
                    raise ValueError('runtime bridge log records a shutdown error')
                if name == 'memcheck':
                    errors = re.findall(r'ERROR SUMMARY: ([\d,]+) errors',
                                        (path.parent / f'{cycle:02}-memcheck.log').read_text())
                    if not errors or any(int(error.replace(',', '')) != 0 for error in errors):
                        raise ValueError('runtime Memcheck errors or missing final summary')
    except (KeyError, TypeError, OSError, IndexError) as exc:
        raise ValueError('incomplete runtime evidence: ' + str(exc)) from exc


def invalidate_publication(root):
    for name in ('setup.bash', 'provenance.json'):
        (Path(root) / name).unlink(missing_ok=True)


def publish(root, manifest, patch):
    """The manifest replacement commits admission; any failure removes discovery."""
    from simulator_backend import read_bridge_overlay
    root = Path(root).resolve()
    pending = root / '.provenance.json.tmp'
    setup = root / '.setup.bash.tmp'
    invalidate_publication(root)
    try:
        executable = Path(manifest['executable']['path']).resolve(strict=True)
        if not executable.is_relative_to(root / 'install'):
            raise ValueError('runtime binary must be installed inside this overlay')
        validate_runtime(manifest)
        pending.write_text(json.dumps(manifest, indent=2) + '\n')
        read_bridge_overlay(pending, patch)
        setup.write_text('export ROS_GZ_BRIDGE_SHUTDOWN_OVERLAY=' + shlex.quote(str(root)) + '\n')
        os.replace(setup, root / 'setup.bash')
        os.replace(pending, root / 'provenance.json')
    except BaseException as exc:
        invalidate_publication(root)
        if isinstance(exc, (KeyError, TypeError, OSError)):
            raise ValueError('incomplete bridge qualification; no publication') from exc
        raise
    finally:
        pending.unlink(missing_ok=True)
        setup.unlink(missing_ok=True)


def _observed_mapping(maps, record):
    """Bind a real mapping to bytes, inode and device, including unmodified deps."""
    path = checked_file(record).resolve(strict=True)
    stat = path.stat()
    for field, value in (('inode', stat.st_ino), ('device', stat.st_dev)):
        if field in record and (type(record[field]) is not int or record[field] != value):
            raise ValueError('observed mapped file identity changed: ' + str(path))
    rows = [line.split() for line in maps.splitlines() if line.split()[-1:] == [str(path)]]
    device = (os.major(stat.st_dev), os.minor(stat.st_dev))
    if (not rows or any(len(row) != 6 or int(row[4]) != stat.st_ino or
                       tuple(int(part, 16) for part in row[3].split(':')) != device for row in rows)):
        raise ValueError('observed mapping device/inode mismatch: ' + str(path))


def _observed_events(path, row, experiment, corrected, inventory, terminal=None, expected_binary=None, dependencies=()):
    events = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    pid = row['pids']['bridge']
    last_time = -1
    for seq, event in enumerate(events, 1):
        if (type(event['seq']) is not int or event['seq'] != seq or
            type(event['at_ns']) is not int or event['at_ns'] <= last_time or
            event['kind'] not in ('entry', 'return', 'return_unobserved', 'thread_created',
                                  'inferior_exited', 'observer_finished', 'signal', 'thread_exited') or
            event['pid'] != (0 if event['kind'] == 'observer_finished' else pid)):
            raise ValueError('invalid observed process/event ordering')
        last_time = event['at_ns']
        if event['kind'] in ('entry', 'return'):
            if (type(event['tid']) is not int or str(event['tid']) not in inventory or
                not _text(event['object']) or not _text(event['function']) or
                not _text(event['stack']) or len(re.findall(r'^#\d+\s', event['stack'], re.M)) < 2):
                raise ValueError('observed event lacks its thread, object or captured stack')
    def selected(kind, function):
        return [e for e in events if e['kind'] == kind and e.get('function', '').startswith(function)]
    def one(kind, function):
        matches = selected(kind, function)
        if len(matches) != 1:
            raise ValueError('missing or ambiguous observed ' + kind + ': ' + function)
        return matches[0]
    deferred = 'rclcpp::SignalHandler::deferred_signal_handler()'
    uninstall = 'rclcpp::SignalHandler::uninstall()'
    drop = 'spdlog::details::registry::drop('
    destroy = 'spdlog::details::registry::~registry()'
    worker = one('entry', deferred)
    logging = one('entry', 'rcl_logging_fini')
    exiting = one('entry', 'exit')
    if terminal is not None:
        _observed_fatal_terminal(terminal, row, expected_binary, dependencies, events, worker, logging, exiting)
        exits = [e for e in events if e['kind'] == 'inferior_exited']
        signal_time = row['signal_sent_monotonic_ns']
        if (type(signal_time) is not int or not events[0]['at_ns'] <= signal_time < logging['at_ns'] or
            exits[0]['at_ns'] - signal_time > experiment['shutdown_timeout_seconds'] * 1e9 or
            events[-1]['at_ns'] - events[0]['at_ns'] >
                (experiment['traffic_timeout_seconds'] + experiment['shutdown_timeout_seconds']) * 1e9):
            raise ValueError('fatal observation exceeded its finite bound')
        return
    if any(e['kind'] == 'signal' for e in events):
        raise ValueError('normal observations cannot contain fatal signal stops')
    access = one('entry', drop)
    finished = one('return', drop)
    exiting = one('entry', 'exit')
    destroyed = one('entry', destroy)
    destroyed_return = one('return', destroy)
    logging = one('entry', 'rcl_logging_fini')
    if (worker['tid'] == pid or any(e['tid'] != worker['tid'] for e in (logging, access, finished)) or
        any(e['tid'] != pid for e in (exiting, destroyed, destroyed_return)) or
        access['object'] != finished['object'] or access['object'] != destroyed['object'] or
        destroyed_return['object'] != destroyed['object'] or exiting['object'] != '0' or
        not worker['seq'] < logging['seq'] < access['seq'] < finished['seq'] < destroyed['seq'] < destroyed_return['seq'] or
        not exiting['seq'] < destroyed['seq'] or
        any(deferred not in e['stack'] for e in (logging, access, finished)) or
        'rcl_logging_external_shutdown' not in access['stack'] or
        '__run_exit_handlers' not in destroyed['stack'] or
        '__libc_start_call_main' not in exiting['stack']):
        raise ValueError('observed logging access/destruction identity, stack or thread mismatch')
    if corrected:
        stopped = one('return', deferred)
        join = one('entry', uninstall)
        joined = one('return', uninstall)
        if (stopped['tid'] != worker['tid'] or join['tid'] != pid or joined['tid'] != pid or
            any(e['object'] != worker['object'] for e in (stopped, join, joined)) or
            not finished['seq'] < stopped['seq'] < joined['seq'] < exiting['seq'] or
            not join['seq'] < joined['seq'] or
            any('rclcpp::shutdown' not in e['stack'] or 'spin_bridge_node' not in e['stack'] for e in (join, joined))):
            raise ValueError('observed corrected shutdown lacks joined ordering before exit')
    elif (not logging['seq'] < exiting['seq'] < finished['seq'] or
          any(e['seq'] < exiting['seq'] for e in selected('return', deferred)) or
          selected('return', uninstall)):
        raise ValueError('baseline does not show natural exit during active deferred logging')
    # Nested rcl_logging_fini/external/Context returns may share a PC. Their
    # callback order is not promoted into a function-return total order.
    exits = [e for e in events if e['kind'] == 'inferior_exited']
    endings = [e for e in events if e['kind'] == 'observer_finished']
    signal_time = row['signal_sent_monotonic_ns']
    if (len(exits) != 1 or type(exits[0]['exit_code']) is not int or exits[0]['exit_code'] != 0 or
        len(endings) != 1 or endings[0] != events[-1] or
        not destroyed_return['seq'] < exits[0]['seq'] < endings[0]['seq'] or
        type(signal_time) is not int or not events[0]['at_ns'] <= signal_time < logging['at_ns'] or
        exits[0]['at_ns'] - signal_time > experiment['shutdown_timeout_seconds'] * 1e9 or
        events[-1]['at_ns'] - events[0]['at_ns'] >
            (experiment['traffic_timeout_seconds'] + experiment['shutdown_timeout_seconds']) * 1e9):
        raise ValueError('observed shutdown did not finish normally within its finite bound')


def _observed_batch(batch, manifest, dependencies, corrected, seen_pids):
    experiment_path = checked_file(batch['experiment'])
    results_path = checked_file(batch['results'])
    experiment = json.loads(experiment_path.read_text())
    rows = json.loads(results_path.read_text())
    expected_binary = manifest['executable'] if corrected else dict(
        path=manifest['baseline']['executable'], sha256=manifest['baseline']['sha256'])
    for name, expected in (('binary', expected_binary), ('library', manifest['library'])):
        checked_file(experiment[name])
        if any(experiment[name][key] != expected[key] for key in ('path', 'sha256')):
            raise ValueError('observed binary differs from manifest: ' + name)
    runtime_environment = _runtime_environment(manifest)
    environment = experiment['environment']
    if (not isinstance(environment, dict) or
        any(environment.get(key) != runtime_environment[key] for key in ('RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY')) or
        any(environment.get(key) for key in ('LD_PRELOAD', 'LD_AUDIT', 'VALGRIND_LIB', 'VALGRIND_OPTS'))):
        raise ValueError('observed environment differs from unmodified runtime qualification')
    terminal = batch.get('terminal')
    if terminal is not None and corrected:
        raise ValueError('patched observations must all terminate normally')
    count = experiment['cycles_limit']
    if (type(count) is not int or count < 5 or not isinstance(rows, list) or
        (not 1 <= len(rows) <= count if terminal is not None else len(rows) != count) or
        type(experiment['traffic_threshold']) is not int or experiment['traffic_threshold'] < 300 or
        any(type(experiment[key]) not in (int, float) or not math.isfinite(experiment[key]) or
            experiment[key] <= 0 for key in ('traffic_timeout_seconds', 'shutdown_timeout_seconds'))):
        raise ValueError('five complete bounded observations and active traffic required')
    for key in ('publisher', 'config', 'verifier'):
        bound_identity(experiment[key])
    repo = Path(__file__).resolve().parents[2]
    observer_names = ['scripts/bridge_shutdown_repro/' + name for name in ('observe.py', 'observe.gdb.py')]
    if _identities(experiment['instrumentation']) != _identities([identity(repo / name) for name in observer_names]):
        raise ValueError('observational scripts differ from reviewed implementation')
    # The commands bind the debugger script that actually ran, which the check above
    # proves byte-identical to this checkout's reviewed observer.
    debugger_script = next(item['path'] for item in experiment['instrumentation'] if bound_identity(item) == observer_names[1])
    if (experiment['config']['sha256'] != digest(repo / 'scripts/bridge_shutdown_repro/bridges.yaml') or
        experiment['verifier']['sha256'] != digest(repo / 'scripts/bridge_shutdown_repro/verify.py')):
        raise ValueError('observational verifier/configuration changed')
    artifacts = _identities(batch['artifacts'])
    suffixes = ('debugger.log', 'publisher.log', 'bridge.maps', 'events.jsonl', 'inferior.pid', 'task_inventory.json')
    expected_paths = {str(results_path.parent / f'{cycle:02}' / suffix)
                      for cycle in range(1, len(rows) + 1) for suffix in suffixes}
    if set(artifacts) != expected_paths:
        raise ValueError('observational raw evidence inventory incomplete')
    expected_loaded = _identities([manifest['library'], *dependencies])
    for cycle, row in enumerate(rows, 1):
        fatal = terminal is not None and cycle == len(rows)
        pids = row['pids']
        if (type(row['cycle']) is not int or row['cycle'] != cycle or
            type(row['received']) is not int or row['received'] < experiment['traffic_threshold'] or
            any(type(row[key]) is not int or row[key] != 0 for key in
                ('publisher_returncode', 'debugger_returncode')) or
            (row['bridge_returncode'] is not None if fatal else
             type(row['bridge_returncode']) is not int or row['bridge_returncode'] != 0) or
            any(row[key] is not False for key in ('debugger_terminated_inferior', 'access_after_destruction')) or
            row['core_inventory_changed'] is not fatal or row['clean_observation'] is not (not fatal) or row['observer_errors'] != [] or row['remaining_processes'] != [] or
            any(row.get(key) for key in ('forced_kill', 'forced_cleanup', 'timed_out')) or 'exception' in row or
            set(pids) != {'publisher', 'debugger', 'bridge'} or len(set(pids.values())) != 3 or
            any(type(pid) is not int or pid <= 0 or pid in seen_pids for pid in pids.values()) or
            set(row['groups']) != set(pids.values()) or row['loaded_executable'] != expected_binary['path'] or
            _identities(row['loaded_identities']) != expected_loaded):
            raise ValueError('observational cycle lacks clean identity-bound traffic and shutdown')
        seen_pids.update(pids.values())
        expected_commands = [
            ['publisher', [experiment['publisher']['path']]],
            ['debugger', ['/usr/bin/gdb', '-q', '-nx', '-batch', '-x', debugger_script,
                          '--args', expected_binary['path'], '--ros-args', '-p',
                          'config_file:=' + experiment['config']['path']]]]
        if row['commands'] != expected_commands:
            raise ValueError('observed commands contain unbound debugger intervention')
        folder = results_path.parent / f'{cycle:02}'
        maps = (folder / 'bridge.maps').read_text()
        _observed_dependency_inventory(maps, expected_loaded)
        for item in [experiment['binary'], *row['loaded_identities']]:
            _observed_mapping(maps, item)
        if int((folder / 'inferior.pid').read_text()) != pids['bridge']:
            raise ValueError('observed inferior identity mismatch')
        inventory = json.loads((folder / 'task_inventory.json').read_text())
        debugger_log = (folder / 'debugger.log').read_text()
        expected_log = 'received signal SIGSEGV' if fatal else f"(process {pids['bridge']}) exited normally"
        if (expected_log not in debugger_log or
            re.search(r'RCLError|SIGABRT|corrupted unsorted chunks|Inferior .*will be killed', debugger_log) or
            (not fatal and 'SIGSEGV' in debugger_log) or
            'PUBLISHER_READY' not in (folder / 'publisher.log').read_text()):
            raise ValueError('observed raw logs do not record normal active shutdown')
        _observed_events(folder / 'events.jsonl', row, experiment, corrected, inventory,
                         terminal if fatal else None, expected_binary, [manifest['library'], *dependencies])
    return [*(terminal.values() if terminal is not None else []), batch['experiment'], batch['results'], *batch['artifacts'], *experiment['instrumentation'],
            experiment['publisher'], experiment['config'], experiment['verifier']]


def validate_observed(manifest, include_runtime=True):
    """Validate observed shutdown order, without claiming UAF or abort reproduction."""
    try:
        if method(manifest) != OBSERVED_METHOD:
            raise ValueError('explicit observed shutdown method required')
        if ('baseline_returncode' in manifest['reproduction'] or
            any(key in manifest['baseline'] for key in ('results_path', 'results_sha256'))):
            raise ValueError('observed ordering cannot populate installed-baseline abort reproduction')
        qualification = manifest['qualification']
        evidence = json.loads(checked_file(qualification['evidence']).read_text())
        if (evidence['schema'] != 'workcell_bridge_observed_shutdown/v1' or evidence['method'] != OBSERVED_METHOD or
            not _text(evidence['producer']) or evidence['source']['bridge_commit'] != manifest['source']['commit'] or
            evidence['patch'] != manifest['patch']):
            raise ValueError('observed evidence source, method or patch mismatch')
        implementation = [manifest['patch'], *evidence['source']['files']]
        _identities(evidence['source']['files'])
        for name in ('rclcpp', 'rcl', 'rcutils', 'rcl_logging_spdlog', 'spdlog'):
            dependency = evidence['source']['dependencies'][name]
            if not _text(dependency['revision']):
                raise ValueError('unpinned observed source dependency: ' + name)
            _identities(dependency['files'])
            implementation.extend(dependency['files'])
        runtime = evidence['runtime']
        for name in ('executable', 'library'):
            checked_file(runtime[name])
            if runtime[name] != manifest[name]:
                raise ValueError('runtime differs from observed executable/library')
        checked_file(runtime['source_main'])
        if (runtime['source_main']['sha256'] != manifest['source']['patched_main_sha256'] or
            runtime['source_main'] not in evidence['source']['files'] or
            runtime['dependencies'] != qualification['runtime_dependencies']):
            raise ValueError('runtime source/dependencies differ from observed evidence')
        _identities(runtime['dependencies'])
        seen_pids = set()
        for name, corrected in (('baseline', False), ('patched', True)):
            implementation.extend(_observed_batch(evidence['observations'][name], manifest,
                                                  runtime['dependencies'], corrected, seen_pids))
        review = json.loads(checked_file(qualification['review']).read_text())
        if (review['schema'] != 'workcell_bridge_observed_shutdown_review/v1' or review['disposition'] != 'accepted' or
            not _text(review['reviewer']) or review['reviewer'] == evidence['producer'] or
            review['evidence_sha256'] != qualification['evidence']['sha256'] or
            type(review['reviewed_at_ns']) is not int or review['reviewed_at_ns'] <= 0):
            raise ValueError('missing or mismatched independent observed-shutdown review')
        report = checked_file(review['report']).read_text()
        if len(report.strip()) < 200 or any(not _text(review['assessment'][key]) for key in
                ('causal_adequacy', 'scheduling', 'correction', 'runtime_isolation', 'claim_limits', 'uninstall_join')):
            raise ValueError('substantive ordering/join/claim-limits review required')
        validation = review['validation']
        if (not _text(validation['source_inspection']) or
            not isinstance(validation['independently_executed_tests'], list) or
            not isinstance(validation['inspected_test_logs'], list)):
            raise ValueError('independent review must distinguish inspection and executed tests')
        repo = Path(__file__).resolve().parents[2]
        for field, name in (('build_script_sha256', 'build_ros_gz_bridge_shutdown_overlay.py'),
                            ('proof_script_sha256', 'bridge_shutdown_repro/verify.py'),
                            ('publisher_source_sha256', 'bridge_shutdown_repro/publisher.cpp'),
                            ('configuration_sha256', 'bridge_shutdown_repro/bridges.yaml')):
            if manifest[field] != digest(repo / 'scripts' / name):
                raise ValueError('runtime implementation hash mismatch: ' + name)
        context = manifest['shutdown_context_test']
        if context['source_sha256'] != digest(repo / 'scripts/bridge_shutdown_repro/shutdown_context_test.cpp'):
            raise ValueError('context implementation hash mismatch')
        implementation.extend(identity(repo / name) for name in (
            'scripts/build_ros_gz_bridge_shutdown_overlay.py', 'scripts/simulator_backend.py',
            'scripts/bridge_shutdown_repro/qualification.py', 'scripts/bridge_shutdown_repro/verify.py',
            'scripts/bridge_shutdown_repro/observe.py', 'scripts/bridge_shutdown_repro/observe.gdb.py',
            'scripts/bridge_shutdown_repro/CMakeLists.txt', 'scripts/bridge_shutdown_repro/publisher.cpp',
            'scripts/bridge_shutdown_repro/bridges.yaml', 'scripts/bridge_shutdown_repro/shutdown_context_test.cpp',
            'tests/test_bridge_shutdown_overlay.py'))
        reviewed = _identities(review['implementation'])
        if any(reviewed.get(bound_identity(item)) != item['sha256'] for item in implementation):
            raise ValueError('independent review is not bound to observed evidence/source/admission implementation')
        if include_runtime:
            checked_file(dict(path=context['log_path'], sha256=context['log_sha256']))
            validate_runtime(manifest, review['reviewed_at_ns'])
        return evidence
    except (KeyError, TypeError, OSError, IndexError, AttributeError) as exc:
        raise ValueError('incomplete observed shutdown evidence: ' + str(exc)) from exc


def _observed_dependency_inventory(maps, expected):
    """Declared lists cannot silently omit an actually mapped causal dependency."""
    relevant = re.compile(r'libros_gz_bridge|librclcpp|librcl\.|librcutils|librcl_logging|libspdlog|'
                          r'librmw|libfastcdr|libfastrtps|libddsc|libc\.so\.|libstdc\+\+|libignition|libsdformat')
    mapped = {str(Path(row[-1]).resolve()) for line in maps.splitlines()
              if len(row := line.split()) == 6 and row[-1].startswith('/') and relevant.search(Path(row[-1]).name)}
    if mapped != set(expected):
        raise ValueError('observed dependency inventory does not match raw mappings')
    for library in ('rclcpp', 'rcl', 'rcutils', 'rcl_logging_spdlog', 'spdlog'):
        if not any(re.fullmatch('lib' + library + r'\.so(?:\..*)?', Path(path).name) for path in mapped):
            raise ValueError('observed causal dependency missing: ' + library)


def _observed_fatal_terminal(terminal, row, executable, dependencies, events, worker, logging, exiting):
    """One naturally fatal baseline is a witness, never a fabricated clean batch.

    The archived core and independently reviewed all-thread trace attribute this
    narrow FastDDS/rosout shutdown failure. It is not the historical SIGABRT.
    """
    required = {'signal', 'executable_path', 'proc_status', 'proc_maps', 'core', 'backtrace', 'registers'}
    if not isinstance(terminal, dict) or set(terminal) != required:
        raise ValueError('complete attributed baseline terminal evidence required')
    files = {name: checked_file(item) for name, item in terminal.items()}
    pid = row['pids']['bridge']
    if files['signal'].read_text().strip() != '11' or files['executable_path'].read_text().strip() != executable['path']:
        raise ValueError('baseline core signal/executable mismatch')
    status = dict(re.findall(r'^(Pid|Tgid|PPid|TracerPid):\s*(\d+)\s*$', files['proc_status'].read_text(), re.M))
    if status != {key: str(value) for key, value in (
            ('Pid', pid), ('Tgid', pid), ('PPid', row['pids']['debugger']), ('TracerPid', row['pids']['debugger']))}:
        raise ValueError('baseline core process attribution mismatch')
    with files['core'].open('rb') as stream:
        header = stream.read(64)
    if len(header) < 64 or header[:4] != b'\x7fELF' or header[5] not in (1, 2) or int.from_bytes(
            header[16:18], 'little' if header[5] == 1 else 'big') != 4:
        raise ValueError('baseline terminal file is not an ELF core')
    maps = files['proc_maps'].read_text()
    for item in [executable, *dependencies]:
        _observed_mapping(maps, item)
    signals = [event for event in events if event['kind'] == 'signal']
    exits = [event for event in events if event['kind'] == 'inferior_exited']
    endings = [event for event in events if event['kind'] == 'observer_finished']
    deferred = 'rclcpp::SignalHandler::deferred_signal_handler()'
    chain = ('RTPSWriter::new_change', '__rmw_publish', '__rmw_destroy_publisher',
             'rcl_logging_fini', 'rclcpp::Context::shutdown', deferred)
    if (len(signals) != 1 or signals[0].get('signal') != 'SIGSEGV' or
        signals[0]['tid'] != worker['tid'] or worker['tid'] == pid or
        logging['tid'] != worker['tid'] or exiting['tid'] != pid or exiting['object'] != '0' or
        not worker['seq'] < logging['seq'] < exiting['seq'] < signals[0]['seq'] or
        any(part not in signals[0].get('stack', '') for part in chain) or deferred not in logging['stack'] or
        '__libc_start_call_main' not in exiting['stack'] or
        any(event['kind'] == 'return' and event.get('function') == deferred and
            event['seq'] < signals[0]['seq'] for event in events) or
        len(exits) != 1 or exits[0]['exit_code'] is not None or not signals[0]['seq'] < exits[0]['seq'] or
        (endings and (len(endings) != 1 or endings[0] != events[-1] or endings[0]['seq'] < exits[0]['seq'])) or
        (not endings and events[-1] != exits[0])):
        raise ValueError('baseline fatal signal is not an attributed deferred-logging ordering witness')
    backtrace = files['backtrace'].read_text()
    sections = dict(re.findall(r'^Thread \d+ \([^\n]*\(LWP (\d+)\)\):\n(.*?)(?=^Thread \d+ |\Z)', backtrace, re.M | re.S))
    if ('Program terminated with signal SIGSEGV' not in backtrace or
        any(part not in sections.get(str(worker['tid']), '') for part in chain) or
        any(part not in sections.get(str(pid), '') for part in
            ('DomainParticipantFactory::~DomainParticipantFactory', '__run_exit_handlers'))):
        raise ValueError('independent core trace does not bind fault worker and concurrent main-thread destruction')
    instruction = re.search(r'^#0\s+(0x[0-9a-fA-F]+)', signals[0]['stack'], re.M)
    registers = files['registers'].read_text()
    rip = re.search(r'^rip\s+(0x[0-9a-fA-F]+)', registers, re.M)
    if (instruction is None or rip is None or int(instruction[1], 16) != int(rip[1], 16) or
        'RTPSWriter::new_change' not in registers or 'Program terminated with signal SIGSEGV' not in registers):
        raise ValueError('independent core fault instruction/registers mismatch')


def _runtime_environment(manifest):
    environment = manifest['runtime_environment']
    interposition = ('LD_PRELOAD', 'LD_AUDIT', 'VALGRIND_LIB', 'VALGRIND_OPTS')
    if (not isinstance(environment, dict) or
        set(environment) != {'RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY', *interposition} or
        not _text(environment['RMW_IMPLEMENTATION']) or environment['ROS_LOCALHOST_ONLY'] != '1' or
        any(environment[key] != '' for key in interposition)):
        raise ValueError('runtime environment must bind middleware and exclude interposition')
    return environment
