#!/usr/bin/env python3
"""Join vcpu_pause_detector output with the GIL probe holds of executor sessions (diagnostic only).

usage: vcpu_pause_report.py <detector log> <session dir>... [--json out.json]

A session dir holds summary.measurements.timing.json (GilProbe events). For every probe hold
(>= threshold, brief and open) it lists the detector records overlapping the hold on any CPU,
and labels the hold:
  vcpu_pause_coincident  an 'invisible' pause (no guest switch, no guest irq time, >= the
                         detector's resolve threshold) covers at least half of the hold;
  guest_explained        only guest-side records (switch, run queue, irq, stop) overlap;
  short_unresolved_only  only pauses too short to classify overlap;
  no_detector_record     the detector saw nothing on any CPU during the hold;
  not_covered            the detector was not running then.
It also reports invisible pauses seen on two or more CPUs within 5 ms of each other (a
whole-VM event). It reads evidence only and changes nothing.
"""
import argparse, json
from collections import Counter
from pathlib import Path


def parse_detector(text):
    config, events, summary, beats = {}, [], None, []
    for line in text.splitlines():
        f = line.split()
        if not f: continue
        if f[0] == 'config':
            config = dict(kv.split('=', 1) for kv in f[1:] if '=' in kv)
        elif f[0] == 'gap' and len(f) == 12:
            start = int(f[3]); ms = float(f[4])
            events.append(dict(kind='gap', cpu=int(f[1]), tid=int(f[2]), start_wall_ns=start, end_wall_ns=start + int(ms * 1e6),
                               ms=ms, d_cpu_ns=int(f[5]), d_nvcsw=int(f[6]), d_nivcsw=int(f[7]), d_irq=int(f[8]),
                               d_softirq=int(f[9]), d_steal=int(f[10]), cls=f[11]))
        elif f[0] == 'late' and len(f) == 11:
            start = int(f[3]); ms = float(f[4])
            events.append(dict(kind='late', cpu=int(f[1]), tid=int(f[2]), start_wall_ns=start, end_wall_ns=start + int(ms * 1e6),
                               ms=ms, d_runqueue_ns=int(f[5]), d_nvcsw=int(f[6]), d_irq=int(f[7]), d_softirq=int(f[8]),
                               d_steal=int(f[9]), cls=f[10]))
        elif f[0] == 'beat' and len(f) == 7:
            beats.append(dict(wall_ns=int(f[1]), cpu=int(f[2])))
        elif f[0] == 'summary' and len(f) >= 11:
            keys = ('wall_ns', 'cpus', 'windows', 'gaps', 'invisible', 'guest_switch', 'guest_irq', 'migrated', 'lates', 'max_gap_ms',
                    'late_invisible', 'max_late_ms')
            summary = {k: (float(v) if '.' in v else int(v)) for k, v in zip(keys, f[1:])}
    return config, events, summary, beats


def coincidences(events, window_ns=5_000_000):
    invisible = sorted((e for e in events if e['cls'] in ('invisible', 'short_unresolved')), key=lambda e: e['start_wall_ns'])
    groups, current = [], []
    for e in invisible:
        if current and e['start_wall_ns'] - current[-1]['start_wall_ns'] > window_ns:
            if len({x['cpu'] for x in current}) >= 2: groups.append(current)
            current = []
        current.append(e)
    if len({x['cpu'] for x in current}) >= 2: groups.append(current)
    return [dict(start_wall_ns=g[0]['start_wall_ns'], cpus=sorted({x['cpu'] for x in g}),
                 classes=sorted({x['cls'] for x in g}), max_ms=max(x['ms'] for x in g)) for g in groups]


def correlate(config, events, summary, sessions):
    start = int(config.get('start_wall_ns', 0)); end = summary['wall_ns'] if summary else None
    out = []
    for session in sessions:
        timing = json.loads((Path(session) / 'summary.measurements.timing.json').read_text())
        for h in (e for e in timing if e['event'] in ('gil_probe_hold', 'gil_probe_brief_hold', 'gil_probe_open_hold')):
            hs, he = h['start_wall_ns'], h['end_wall_ns']
            if not start or hs < start or (end is not None and he > end):
                label, near = 'not_covered', []
            else:
                near = [e for e in events if e['start_wall_ns'] <= he + 5_000_000 and e['end_wall_ns'] >= hs - 5_000_000]
                overlap = lambda e: max(0, min(e['end_wall_ns'], he) - max(e['start_wall_ns'], hs))
                if any(e['cls'] == 'invisible' and overlap(e) * 2 >= he - hs for e in near): label = 'vcpu_pause_coincident'
                elif any(e['cls'] in ('guest_switch', 'guest_irq', 'runqueue', 'stopped', 'migrated') for e in near): label = 'guest_explained'
                elif near and all(e['cls'] == 'short_unresolved' for e in near): label = 'short_unresolved_only'
                elif near: label = 'other'
                else: label = 'no_detector_record'
            cpu = h.get('holder_cpu') or {}
            out.append(dict(session=str(session), event=h['event'].removeprefix('gil_probe_'), start_wall_ns=hs, hold_ms=h['hold_ms'],
                            thread=h.get('thread'), attribution=h.get('attribution'), holder_state=cpu.get('state_begin'),
                            holder_on_cpu_ms=cpu.get('on_cpu_ms'), holder_syscall=(h.get('holder_syscall') or {}).get('in_syscall'),
                            label=label, detector=[{k: e[k] for k in ('kind', 'cpu', 'ms', 'cls')} | dict(offset_ms=(e['start_wall_ns'] - hs) / 1e6)
                                                   for e in near]))
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('log', type=Path)
    parser.add_argument('sessions', nargs='*', type=Path)
    parser.add_argument('--json', type=Path)
    args = parser.parse_args(argv)
    config, events, summary, beats = parse_detector(args.log.read_text())
    holds = correlate(config, events, summary, args.sessions)
    result = dict(config=config, summary=summary, classes=dict(Counter((e['kind'], e['cls']) for e in events).most_common()),
                  invisible=[e for e in events if e['cls'] == 'invisible'], cross_cpu=coincidences(events), holds=holds,
                  beats_per_cpu=dict(Counter(b['cpu'] for b in beats)))
    if args.json: args.json.write_text(json.dumps(result, indent=2, default=str) + '\n')
    print('detector summary', summary)
    print('records by (kind, class):', result['classes'])
    print('invisible pauses:', [(e['kind'], e['cpu'], e['ms']) for e in result['invisible']][:20])
    print('cross-CPU coincidences:', result['cross_cpu'][:10])
    for h in holds:
        print(f"- {h['event']} {h['hold_ms']:.1f} ms {h['thread']} [{h['attribution']}] state {h['holder_state']} -> {h['label']}: "
              f"{[(d['kind'], d['cpu'], d['ms'], d['cls']) for d in h['detector']][:6]}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
