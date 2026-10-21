#!/usr/bin/env python3
"""Join vcpu_pause_detector output with the GIL probe holds of executor sessions (diagnostic only).

usage: vcpu_pause_report.py <detector log> <session dir>... [--json out.json]

'invisible' means the guest kept no trace of the time (no own switch, no run-queue wait, no
softirq ticks, >= the resolve threshold). It is consistent with the hypervisor descheduling
or slowly waking the vCPU, or firmware; with tick-sampled accounting a long hardirq/NMI or
an irq-disabled kernel section on that CPU is not excluded. Each record is also marked
busy or idle from its CPU's tick deltas (an idle vCPU woken late is a different host effect
from a running vCPU that stopped).

For every probe hold (>= threshold, brief and open) in the session dirs it lists the
detector records overlapping it and labels the hold:
  invisible_covers_half     invisible records (per-CPU union) cover >= half of the hold;
  guest_activity_covers_half  guest-side records (switch, run queue, irq, stop) do;
  partial_overlap           some records overlap, neither covers half;
  short_unresolved_only     only pauses too short to classify overlap;
  no_detector_record        nothing on any CPU during the hold (for a hold >= 50 ms this
                            is evidence AGAINST a vCPU pause: the detector thread on the
                            holder's CPU would have woken >= 30 ms late);
  not_covered               the detector was not running then.
The holder's CPU is not known, so the base rate of invisible time is printed with the
expected number of chance coincidences. It reads evidence only and changes nothing.
"""
import argparse, json
from collections import Counter, defaultdict
from pathlib import Path

GUEST = ('guest_switch', 'guest_irq', 'runqueue', 'stopped', 'migrated')


def parse_detector(text):
    config, events, summary, beats, overruns = {}, [], None, [], []
    for line in text.splitlines():
        f = line.split()
        if not f: continue
        if f[0] == 'config':
            config = dict(kv.split('=', 1) for kv in f[1:] if '=' in kv)
        elif (f[0] == 'gap' and len(f) in (12, 14)) or (f[0] == 'late' and len(f) in (11, 13)):   # without / with busy+idle ticks
            start = int(f[3]); ms = float(f[4]); gap = f[0] == 'gap'
            e = dict(kind=f[0], cpu=int(f[1]), tid=int(f[2]), start_wall_ns=start, end_wall_ns=start + int(ms * 1e6), ms=ms)
            tail = f[5:]
            if gap:
                e.update(d_cpu_ns=int(tail[0]), d_nvcsw=int(tail[1]), d_nivcsw=int(tail[2]), d_irq=int(tail[3]),
                         d_softirq=int(tail[4]), d_steal=int(tail[5]), cls=tail[6])
                rest = tail[7:]
            else:
                e.update(d_runqueue_ns=int(tail[0]), d_nvcsw=int(tail[1]), d_irq=int(tail[2]), d_softirq=int(tail[3]),
                         d_steal=int(tail[4]), cls=tail[5])
                rest = tail[6:]
            e['d_busy'], e['d_idle'] = (int(rest[0]), int(rest[1])) if len(rest) == 2 else (None, None)
            # A real pause on a busy CPU can be followed by a preemption before the next clock read:
            # such a gap shows a switch although the thread was charged the whole gap as run time.
            if gap and e['cls'] == 'guest_switch' and e['d_cpu_ns'] >= e['ms'] * 1e6 * .9: e['cls'] = 'running_gap'
            e['cpu_state'] = ('idle' if e['d_idle'] is not None and e['d_idle'] * 10 * 2 >= ms else
                              'busy' if e['d_busy'] is not None and e['d_busy'] > 0 else 'unknown')
            events.append(e)
        elif f[0] == 'overrun' and len(f) == 7:
            overruns.append(dict(cpu=int(f[1]), wall_ns=int(f[3]), late_by_ms=float(f[4]), d_nvcsw=int(f[5]), d_nivcsw=int(f[6]),
                                 explained=bool(int(f[5]) or int(f[6]))))
        elif f[0] == 'beat' and len(f) == 7:
            beats.append(dict(wall_ns=int(f[1]), cpu=int(f[2])))
        elif f[0] == 'summary' and len(f) >= 11:
            keys = ('wall_ns', 'cpus', 'windows', 'gaps', 'invisible', 'guest_switch', 'guest_irq', 'migrated', 'lates', 'max_gap_ms',
                    'late_invisible', 'max_late_ms', 'covered')
            summary = {k: (v if k == 'covered' else float(v) if '.' in v else int(v)) for k, v in zip(keys, f[1:])}
    return config, events, summary, beats, overruns


def coverage_end(summary, beats, events):
    if summary: return summary['wall_ns']
    ends = [b['wall_ns'] for b in beats] + [e['end_wall_ns'] for e in events]
    return max(ends) if ends else None


def union_ms(intervals, lo, hi):
    spans = sorted((max(a, lo), min(b, hi)) for a, b in intervals if b > lo and a < hi)
    total, cur = 0, None
    for a, b in spans:
        if cur is None or a > cur[1]:
            if cur: total += cur[1] - cur[0]
            cur = [a, b]
        else: cur[1] = max(cur[1], b)
    if cur: total += cur[1] - cur[0]
    return total


def coincidences(events, slack_ns=5_000_000):
    """Groups of overlapping (within slack) invisible/short_unresolved records on >= 2 CPUs."""
    picked = sorted((e for e in events if e['cls'] in ('invisible', 'short_unresolved')), key=lambda e: e['start_wall_ns'])
    groups, current, end = [], [], None
    for e in picked:
        if current and e['start_wall_ns'] > end + slack_ns:
            groups.append(current); current = []
        current.append(e); end = max(end or 0, e['end_wall_ns']) if len(current) > 1 else e['end_wall_ns']
    if current: groups.append(current)
    return [dict(start_wall_ns=g[0]['start_wall_ns'], cpus=sorted({x['cpu'] for x in g}), classes=sorted({x['cls'] for x in g}),
                 kinds=sorted({x['kind'] for x in g}), max_ms=max(x['ms'] for x in g),
                 cpu_states=sorted({x['cpu_state'] for x in g})) for g in groups if len({x['cpu'] for x in g}) >= 2]


def correlate(config, events, summary, sessions, beats=()):
    start = int(config.get('start_wall_ns', 0)); end = coverage_end(summary, list(beats), events)
    out = []
    for session in sessions:
        timing = json.loads((Path(session) / 'summary.measurements.timing.json').read_text())
        for h in (e for e in timing if e['event'] in ('gil_probe_hold', 'gil_probe_brief_hold', 'gil_probe_open_hold')):
            hs, he = h['start_wall_ns'], h['end_wall_ns']
            near = [e for e in events if e['start_wall_ns'] <= he + 5_000_000 and e['end_wall_ns'] >= hs - 5_000_000]
            if not start or hs < start or end is None or he > end: label = 'not_covered'
            else:
                per_cpu = defaultdict(list)
                for e in near:
                    if e['cls'] == 'invisible': per_cpu[e['cpu']].append((e['start_wall_ns'], e['end_wall_ns']))
                invisible = max((union_ms(v, hs, he) for v in per_cpu.values()), default=0)
                guest = union_ms([(e['start_wall_ns'], e['end_wall_ns']) for e in near if e['cls'] in GUEST], hs, he)
                if invisible * 2 >= he - hs: label = 'invisible_covers_half'
                elif guest * 2 >= he - hs: label = 'guest_activity_covers_half'
                elif near and all(e['cls'] == 'short_unresolved' for e in near): label = 'short_unresolved_only'
                elif near: label = 'partial_overlap'
                else: label = 'no_detector_record'
            cpu = h.get('holder_cpu') or {}
            out.append(dict(session=str(session), event=h['event'].removeprefix('gil_probe_'), start_wall_ns=hs, hold_ms=h['hold_ms'],
                            thread=h.get('thread'), attribution=h.get('attribution'), holder_state=cpu.get('state_begin'),
                            holder_on_cpu_ms=cpu.get('on_cpu_ms'), holder_syscall=(h.get('holder_syscall') or {}).get('in_syscall'),
                            label=label, detector=[{k: e[k] for k in ('kind', 'cpu', 'ms', 'cls', 'cpu_state')} |
                                                   dict(offset_ms=(e['start_wall_ns'] - hs) / 1e6) for e in near]))
    return out


def base_rate(config, events, summary, beats):
    start = int(config.get('start_wall_ns', 0)); end = coverage_end(summary, list(beats), events)
    if not start or not end or end <= start: return None
    invisible = union_ms([(e['start_wall_ns'], e['end_wall_ns']) for e in events if e['cls'] == 'invisible'], start, end)
    return dict(covered_s=(end - start) / 1e9, invisible_s=invisible / 1e9, fraction=invisible / (end - start))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('log', type=Path)
    parser.add_argument('sessions', nargs='*', type=Path)
    parser.add_argument('--json', type=Path)
    args = parser.parse_args(argv)
    config, events, summary, beats, overruns = parse_detector(args.log.read_text())
    holds = correlate(config, events, summary, args.sessions, beats)
    rate = base_rate(config, events, summary, beats)
    long_holds = [h for h in holds if h['event'] == 'hold' and h['label'] != 'not_covered']
    expected = (rate['fraction'] * len(long_holds)) if rate else None
    result = dict(config=config, summary=summary, overruns=overruns, base_rate=rate,
                  expected_chance_invisible_overlaps_of_long_holds=expected,
                  classes=dict(Counter(f"{e['kind']}:{e['cls']}:{e['cpu_state']}" for e in events).most_common()),
                  invisible=[e for e in events if e['cls'] == 'invisible'], cross_cpu=coincidences(events), holds=holds,
                  beats_per_cpu=dict(Counter(b['cpu'] for b in beats)))
    if args.json: args.json.write_text(json.dumps(result, indent=2, default=str) + '\n')
    print('detector summary', summary, '| overruns', len(overruns))
    print('records (kind:class:cpu_state):', result['classes'])
    print('base rate of invisible time:', rate, '| expected chance overlaps among', len(long_holds), 'long holds:', expected)
    print('invisible pauses:', [(e['kind'], e['cpu'], e['ms'], e['cpu_state']) for e in result['invisible']][:20])
    print('cross-CPU groups:', result['cross_cpu'][:10])
    for h in holds:
        print(f"- {h['event']} {h['hold_ms']:.1f} ms {h['thread']} [{h['attribution']}] state {h['holder_state']} -> {h['label']}: "
              f"{[(d['kind'], d['cpu'], d['ms'], d['cls'], d['cpu_state']) for d in h['detector']][:8]}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
