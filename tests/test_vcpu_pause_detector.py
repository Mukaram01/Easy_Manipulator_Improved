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
    config, events, summary, beats, _ = _records(log)
    cpus = len(os.sched_getaffinity(0))
    # every thread really pinned to its own CPU (the summary lists the CPUs whose affinity was set)
    assert summary['cpus'] == cpus and summary['covered'] == ','.join(str(c) for c in sorted(os.sched_getaffinity(0)))
    assert config['period_ms'] == '20.000' and config['resolve_ms'] == '30.000' and config['spin_us'] == '150.0'
    # about 2 s / 20 ms windows per CPU; never a burst of catch-up windows
    assert cpus * 60 <= summary['windows'] <= cpus * 110
    assert all(e['cls'] in ('guest_switch', 'guest_irq', 'runqueue', 'stopped', 'migrated', 'short_unresolved',
                            'invisible', 'invisible_unverified', 'running_gap') for e in events)


def test_a_guest_stop_is_classified_as_guest_side_never_invisible(tmp_path, detector):
    seen = []
    for attempt in range(4):
        log = tmp_path / f'stop{attempt}.log'
        process = subprocess.Popen([str(detector), '--output', str(log), '--duration', '3'])
        time.sleep(1); process.send_signal(signal.SIGSTOP); time.sleep(.3); process.send_signal(signal.SIGCONT)
        assert process.wait(timeout=30) == 0
        _, events, summary, _, _ = _records(log)
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
    for bad in (['--output', str(tmp_path / 'x.log'), '--spin-us', '30000'], ['--period-ms', '20'], ['--output', 'y', '--gap-ms', '-1'],
                ['--output', str(tmp_path / 'z.log'), '--duration', 'soon']):
        assert subprocess.run([str(detector), *bad], capture_output=True).returncode == 2


def test_report_labels_holds_by_what_the_detector_saw(tmp_path):
    from vcpu_pause_report import parse_detector, correlate, coincidences, base_rate
    log = ('config start_wall_ns=1000 pid=1 period_ms=20.000 resolve_ms=30.000\n'
           'gap 3 11 100000000 300.000 300000000 0 0 0 0 0 invisible 1 0\n'               # busy CPU, covers hold A
           'late 2 12 100500000 280.000 100000 1 0 0 0 invisible 0 28\n'                 # idle CPU at the same time
           'late 1 13 600000000 120.000 100000000 1 0 0 0 runqueue 12 0\n'              # guest contention during hold B
           'gap 4 14 900000000 5.000 5000000 0 0 0 0 0 short_unresolved 0 0\n'          # only a short one near hold C
           'gap 5 15 1500000000 40.000 30000000 0 0 0 0 0 invisible 0 0\n'              # covers a quarter of hold E
           'gap 6 16 3000000000 60.000 60000000 0 1 0 0 0 guest_switch 6 0\n'           # charged the whole gap: running_gap
           'beat 4900000000 0 100 1 0 300.000\n')                                         # no summary: coverage ends at the last beat
    config, events, summary, beats, _ = parse_detector(log)
    assert summary is None and [e['cpu_state'] for e in events][:2] == ['busy', 'idle'] and events[-1]['cls'] == 'running_gap'
    session = tmp_path / 's'; session.mkdir()
    hold = lambda start, ms, event='gil_probe_hold': dict(event=event, start_wall_ns=start, end_wall_ns=start + int(ms * 1e6), hold_ms=ms,
                                                           thread='measurement_acquisition', attribution='verified',
                                                           holder_cpu=dict(state_begin='R', on_cpu_ms=ms), holder_syscall=dict(in_syscall=[]))
    (session / 'summary.measurements.timing.json').write_text(json.dumps(
        [hold(110_000_000, 250), hold(590_000_000, 130), hold(899_000_000, 41, 'gil_probe_brief_hold'), hold(2_000_000_000, 90),
         hold(1_480_000_000, 160), hold(500, 90), hold(4_950_000_000, 90)]))
    labels = [h['label'] for h in correlate(config, events, summary, [session], beats)]
    assert labels == ['invisible_covers_half', 'guest_activity_covers_half', 'short_unresolved_only', 'no_detector_record',
                      'partial_overlap', 'not_covered', 'not_covered']
    cross, = coincidences(events)                       # a gap and a late record on two CPUs, grouped by overlap
    assert cross['cpus'] == [2, 3] and cross['kinds'] == ['gap', 'late'] and cross['cpu_states'] == ['busy', 'idle']
    rate = base_rate(config, events, summary, beats)
    assert abs(rate['invisible_s'] - .34) < 1e-6 and 0 < rate['fraction'] < .1
