"""vcpu_pause_detector (standalone diagnostic) and its report: classification and correlation."""
import json, os, shutil, signal, subprocess, sys, time
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))


@pytest.fixture(scope='session')
def detector(tmp_path_factory):
    if shutil.which('cc') is None: pytest.skip('no C compiler')
    binary = tmp_path_factory.mktemp('vcpu') / 'vcpu_pause_detector'
    subprocess.run(['cc', '-O2', '-Wall', '-Wextra', '-Werror', '-pthread', '-o', str(binary),
                    str(SCRIPTS / 'vcpu_pause_detector.c')], check=True)
    return binary


def _records(path):
    from vcpu_pause_report import parse_detector
    return parse_detector(path.read_text())


def test_detector_runs_one_pinned_thread_per_cpu_and_reports_its_coverage(tmp_path, detector):
    log = tmp_path / 'idle.log'
    started = time.monotonic()
    subprocess.run([str(detector), '--output', str(log), '--duration', '2'], check=True, timeout=30)
    assert time.monotonic() - started < 10
    config, events, summary, beats = _records(log)
    cpus = len(os.sched_getaffinity(0))
    assert summary['cpus'] == cpus and config['period_ms'] == '20.000' and config['resolve_ms'] == '30.000'
    # about 2 s / 20 ms windows per CPU; never a burst of catch-up windows
    assert cpus * 60 <= summary['windows'] <= cpus * 110
    assert all(e['cls'] in ('guest_switch', 'guest_irq', 'runqueue', 'stopped', 'migrated', 'short_unresolved',
                            'invisible', 'invisible_unverified') for e in events)


def test_a_guest_stop_is_classified_as_guest_side_never_invisible(tmp_path, detector):
    seen = []
    for attempt in range(4):
        log = tmp_path / f'stop{attempt}.log'
        process = subprocess.Popen([str(detector), '--output', str(log), '--duration', '3'])
        time.sleep(1); process.send_signal(signal.SIGSTOP); time.sleep(.3); process.send_signal(signal.SIGCONT)
        assert process.wait(timeout=30) == 0
        _, events, summary, _ = _records(log)
        long = [e for e in events if e['ms'] >= 250]
        assert long, 'the 300 ms stop must be seen by a spinning or a sleeping thread'
        assert all(e['cls'] in ('guest_switch', 'stopped') for e in long), [(e['kind'], e['cls']) for e in long]
        seen += [e['kind'] for e in long]
    assert 'late' in seen


def test_detector_stops_on_sigterm_with_a_summary_and_rejects_bad_arguments(tmp_path, detector):
    log = tmp_path / 'term.log'
    process = subprocess.Popen([str(detector), '--output', str(log)])
    time.sleep(.5); process.terminate()
    assert process.wait(timeout=10) == 0 and _records(log)[2] is not None
    for bad in (['--output', str(tmp_path / 'x.log'), '--spin-us', '30000'], ['--period-ms', '20'], ['--output', 'y', '--gap-ms', '-1']):
        assert subprocess.run([str(detector), *bad], capture_output=True).returncode == 2


def test_report_labels_holds_by_what_the_detector_saw(tmp_path):
    from vcpu_pause_report import parse_detector, correlate, coincidences
    log = ('config start_wall_ns=1000 pid=1 period_ms=20.000 resolve_ms=30.000\n'
           'gap 3 11 100000000 300.000 300000000 0 0 0 0 0 invisible\n'              # covers hold A
           'gap 2 12 100500000 280.000 280000000 0 0 0 0 0 invisible\n'              # same time, another CPU
           'late 1 13 600000000 120.000 100000000 1 0 0 0 runqueue\n'               # guest contention during hold B
           'gap 4 14 900000000 5.000 5000000 0 0 0 0 0 short_unresolved\n'          # only a short one near hold C
           'summary 5000000000 8 100 3 2 0 0 0 1 300.000 0 120.000\n')
    config, events, summary, _ = parse_detector(log)
    session = tmp_path / 's'; session.mkdir()
    hold = lambda start, ms, event='gil_probe_hold': dict(event=event, start_wall_ns=start, end_wall_ns=start + int(ms * 1e6), hold_ms=ms,
                                                           thread='measurement_acquisition', attribution='verified',
                                                           holder_cpu=dict(state_begin='R', on_cpu_ms=ms), holder_syscall=dict(in_syscall=[]))
    (session / 'summary.measurements.timing.json').write_text(json.dumps(
        [hold(110_000_000, 250), hold(590_000_000, 130), hold(899_000_000, 41, 'gil_probe_brief_hold'), hold(2_000_000_000, 90),
         hold(500, 90)]))
    labels = [h['label'] for h in correlate(config, events, summary, [session])]
    assert labels == ['vcpu_pause_coincident', 'guest_explained', 'short_unresolved_only', 'no_detector_record', 'not_covered']
    cross, = coincidences(events)
    assert cross['cpus'] == [2, 3] and cross['classes'] == ['invisible']
