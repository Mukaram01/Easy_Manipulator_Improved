#!/usr/bin/env python3
"""Offline report of the GIL probe evidence of one executor session (diagnostic only).

usage: gil_probe_report.py <session dir containing summary.measurements.timing.json> [--json out.json]

For every probe hold it joins, from the session's own evidence files only:
  * the holder's identity and how it was established (simulator_execution.GilProbe.parse);
  * its CPU (user/system ticks, on-CPU and run-queue time) and syscall samples;
  * its native PC samples, symbolized from the session's maps snapshot with addr2line. A
    library without symbols (stripped, no installed debug file) is reported by library and
    "nearest exported symbol", marked unreliable; stack-scan candidates are heuristic;
  * its Python frames at the release, with the snapshot's exactness;
  * executor loop calls that took >= 20 ms, GC and probe lateness overlapping it;
  * the telemetry receive gap around it (simulator stamp, publish stamp, receive stamp).
It reads evidence only and changes nothing.
"""
import argparse, collections, json, subprocess
from pathlib import Path


def load_maps(path):
    maps = []
    for line in Path(path).read_text().splitlines():
        fields = line.split(maxsplit=5)
        if len(fields) < 5: continue
        start, end = (int(x, 16) for x in fields[0].split('-'))
        maps.append(dict(start=start, end=end, perms=fields[1], offset=int(fields[2], 16),
                         path=fields[5].strip() if len(fields) == 6 else ''))
    bases = {}
    for m in maps:
        if m['path'] and m['offset'] == 0 and m['path'] not in bases: bases[m['path']] = m['start']
    return maps, bases


def symbol_quality(path, cache={}):
    """'symbols' when the file or an installed debug file carries a symbol table, else 'exported only'."""
    if path in cache: return cache[path]
    quality = 'exported only'
    if path.startswith('/'):
        try:
            if 'no symbols' not in subprocess.run(['nm', path], capture_output=True, text=True, timeout=30).stderr:
                quality = 'symbols'
            else:
                note = subprocess.run(['readelf', '-n', path], capture_output=True, text=True, timeout=30).stdout
                build = next((l.split(':', 1)[1].strip() for l in note.splitlines() if 'Build ID' in l), None)
                if build and Path(f'/usr/lib/debug/.build-id/{build[:2]}/{build[2:]}.debug').is_file(): quality = 'symbols'
        except (OSError, subprocess.SubprocessError): quality = 'unknown'
    cache[path] = quality
    return quality


def symbolize(addresses, maps, bases):
    located = {}
    by_file = collections.defaultdict(list)
    for address in set(addresses):
        m = next((m for m in maps if m['start'] <= address < m['end']), None)
        if m is None: located[address] = dict(library=None, symbol=None, quality='unmapped'); continue
        if not m['path'].startswith('/'):
            located[address] = dict(library=m['path'] or 'anonymous', symbol=None, quality='no file'); continue
        offset = address - bases.get(m['path'], m['start'])
        by_file[m['path']].append((address, offset))
    for path, items in by_file.items():
        quality = symbol_quality(path)
        try:
            out = subprocess.run(['addr2line', '-f', '-C', '-e', path] + [hex(o) for _, o in items],
                                 capture_output=True, text=True, timeout=120).stdout.splitlines()
            names = out[0::2]
        except (OSError, subprocess.SubprocessError): names = []
        for i, (address, offset) in enumerate(items):
            name = names[i] if i < len(names) and names[i] not in ('??', '') else None
            located[address] = dict(library=Path(path).name, offset=offset, symbol=name,
                                    quality=quality if name else 'unresolved')
    return located


def label(entry):
    if entry['library'] is None: return 'unmapped'
    if entry['symbol'] is None: return entry['library']
    return f"{entry['library']}:{entry['symbol']}" + ('' if entry['quality'] == 'symbols' else ' (nearest exported, unreliable)')


def report(session):
    session = Path(session)
    timing = json.loads((session / 'summary.measurements.timing.json').read_text())
    maps_file = session / 'summary.measurements.gil-probe.maps'
    maps, bases = load_maps(maps_file) if maps_file.is_file() else ([], {})
    summary = next((e for e in timing if e['event'] == 'gil_probe_summary'), None)
    holds = [e for e in timing if e['event'] == 'gil_probe_hold']
    receives = sorted((e for e in timing if e['event'] == 'receive' and e.get('receive_wall_ns')), key=lambda e: e['receive_wall_ns'])
    slow = [e for e in timing if e['event'] == 'loop_slow']
    gcs = [e for e in timing if e['event'] == 'gc']
    out = dict(session=str(session), probe=summary, holds=[])
    for hold in holds:
        start, end = hold['start_wall_ns'], hold['end_wall_ns']
        pcs = hold.get('native_pc') or []
        located = symbolize([p['rip'] for p in pcs] + [a for p in pcs for a in p['candidates']], maps, bases) if maps else {}
        leaf = collections.Counter(label(located[p['rip']]) for p in pcs if p['rip'] in located)
        callers = collections.Counter(label(located[a]) for p in pcs for a in set(p['candidates']) if a in located)
        gap = None
        for a, b in zip(receives, receives[1:]):
            if a['receive_wall_ns'] <= end and b['receive_wall_ns'] >= start:
                if gap is None or b['receive_wall_ns'] - a['receive_wall_ns'] > gap['receive_gap_ms'] * 1e6:
                    gap = dict(receive_gap_ms=(b['receive_wall_ns'] - a['receive_wall_ns']) / 1e6,
                               source_gap_ms=((b.get('source_wall_ns') or 0) - (a.get('source_wall_ns') or 0)) / 1e6,
                               publish_to_receive_ms=(b['receive_wall_ns'] - (b.get('publish_wall_ns') or b['receive_wall_ns'])) / 1e6,
                               next_callback_ms=b.get('callback_ns', 0) / 1e6, next_parse_ms=(b.get('parse_ns') or 0) / 1e6)
        frames = hold.get('release_frames') or []
        out['holds'].append(dict(
            start_wall_ns=start, hold_ms=hold['hold_ms'], thread=hold.get('thread'), attribution=hold.get('attribution'),
            native_id=hold.get('native_id'), holder_proc_data=hold.get('holder_proc_data'),
            overlaps_probe_late=hold.get('overlaps_probe_late'), holder_cpu=hold.get('holder_cpu'),
            holder_syscall=hold.get('holder_syscall'),
            native_pc=dict(samples=len(pcs), leaf=leaf.most_common(8), stack_scan_heuristic=callers.most_common(8),
                           delivery_us_max=max((p['delivery_us'] for p in pcs), default=None)),
            release_frames_status=hold.get('release_frames_status'),
            release_frames=[f"{f['file']}:{f['line']} {f['function']} (lasti {f['lasti']})" for f in frames[:8]],
            loop_slow=[dict(call=e['call'], thread=e['thread'], duration_ms=e['duration_ns'] / 1e6,
                            offset_ms=(e['start_wall_ns'] - start) / 1e6) for e in slow
                       if e['start_wall_ns'] <= end and e['start_wall_ns'] + e['duration_ns'] >= start],
            gc=[dict(generation=e['generation'], duration_ms=e['duration_ns'] / 1e6) for e in gcs
                if start - e['duration_ns'] <= e['wall_ns'] <= end + 5_000_000],
            telemetry=gap))
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('session', type=Path)
    parser.add_argument('--json', type=Path)
    args = parser.parse_args(argv)
    result = report(args.session)
    if args.json: args.json.write_text(json.dumps(result, indent=2) + '\n')
    probe = result['probe'] or {}
    print(f"{args.session}: {len(result['holds'])} hold(s) >= {probe.get('threshold_ms')} ms; "
          f"lates {probe.get('lates')}, unmapped {probe.get('unmapped_holds')}, proc failures {probe.get('proc_read_failures')}, "
          f"pc sent/received/lost {probe.get('pc_sent')}/{probe.get('pc_received')}/{probe.get('pc_lost')}, "
          f"snapshots {probe.get('snapshots')} (failed {probe.get('snapshot_failures')}), sampler wakes {probe.get('sampler_wakes')}")
    for h in result['holds']:
        print(f"- {h['hold_ms']:.1f} ms by {h['thread']} [{h['attribution']}], frames {h['release_frames_status']}: "
              f"{h['release_frames'][0] if h['release_frames'] else '-'}")
        print(f"  cpu {h['holder_cpu']}  syscall {h['holder_syscall']}")
        print(f"  native leaf {h['native_pc']['leaf'][:4]}  telemetry {h['telemetry']}")
        if h['loop_slow']: print(f"  loop {h['loop_slow']}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
