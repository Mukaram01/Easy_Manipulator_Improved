#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from typing import Sequence

POLL_INTERVAL_SECONDS = 0.05
SIGINT_GRACE_SECONDS = 1.0
SIGTERM_GRACE_SECONDS = 0.7
SIGKILL_VERIFY_SECONDS = 0.4

_shutdown_signal = 0


def _request_shutdown(signum: int, _frame) -> None:
    global _shutdown_signal
    if _shutdown_signal == 0:
        _shutdown_signal = signum


def _group_alive(pgid: int) -> bool:
    if pgid <= 1:
        return False
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _wait_group_gone(proc: subprocess.Popen, pgid: int, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        proc.poll()
        if not _group_alive(pgid):
            return True
        time.sleep(POLL_INTERVAL_SECONDS)
    proc.poll()
    return not _group_alive(pgid)


def _signal_group(pgid: int, sig: int) -> bool:
    try:
        os.killpg(pgid, sig)
        return True
    except ProcessLookupError:
        return False
    except PermissionError as exc:
        print(
            f"Workcell Studio preview group ERROR: cannot signal pgid={pgid} signal={sig}: {exc}",
            file=sys.stderr,
            flush=True,
        )
        return False


def _shutdown_group(proc: subprocess.Popen, pgid: int) -> bool:
    stages = (
        (signal.SIGINT, SIGINT_GRACE_SECONDS, "SIGINT"),
        (signal.SIGTERM, SIGTERM_GRACE_SECONDS, "SIGTERM"),
        (signal.SIGKILL, SIGKILL_VERIFY_SECONDS, "SIGKILL"),
    )
    for sig, timeout, label in stages:
        if not _group_alive(pgid):
            break
        print(
            f"Workcell Studio preview group shutdown: pgid={pgid} signal={label}",
            file=sys.stderr,
            flush=True,
        )
        _signal_group(pgid, sig)
        if _wait_group_gone(proc, pgid, timeout):
            break

    try:
        proc.wait(timeout=0.2)
    except subprocess.TimeoutExpired:
        pass

    gone = not _group_alive(pgid)
    if gone:
        print(
            f"Workcell Studio preview group stopped: pgid={pgid}",
            file=sys.stderr,
            flush=True,
        )
    else:
        print(
            f"Workcell Studio preview group ERROR: pgid={pgid} still exists after SIGKILL fallback",
            file=sys.stderr,
            flush=True,
        )
    return gone


def _parse_args(argv: Sequence[str]) -> list[str]:
    parser = argparse.ArgumentParser(
        description=(
            "Own one Workcell Studio preview launch in an isolated POSIX session/process group."
        )
    )
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    command = list(args.command)
    if command and command[0] == "--":
        command = command[1:]
    if not command:
        parser.error("a preview command is required after --")
    return command


def main(argv: Sequence[str] | None = None) -> int:
    if os.name != "posix":
        print(
            "Workcell Studio preview group ERROR: POSIX process groups are required",
            file=sys.stderr,
            flush=True,
        )
        return 70

    command = _parse_args(sys.argv[1:] if argv is None else argv)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, _request_shutdown)

    proc = subprocess.Popen(command, start_new_session=True)
    pgid = os.getpgid(proc.pid)
    sid = os.getsid(proc.pid)
    supervisor_pgid = os.getpgrp()

    # start_new_session=True must make the child both session and process-group
    # leader. Refuse to supervise if that isolation contract is not true; this
    # prevents any later negative-PGID signal from reaching Workcell Studio.
    if pgid != proc.pid or sid != proc.pid or pgid == supervisor_pgid:
        print(
            "Workcell Studio preview group ERROR: failed to create an isolated preview session "
            f"leader_pid={proc.pid} pgid={pgid} sid={sid} supervisor_pgid={supervisor_pgid}",
            file=sys.stderr,
            flush=True,
        )
        try:
            proc.terminate()
            proc.wait(timeout=1.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                proc.kill()
            except ProcessLookupError:
                pass
        return 71

    print(
        f"Workcell Studio preview group started: leader_pid={proc.pid} pgid={pgid} sid={sid}",
        file=sys.stderr,
        flush=True,
    )

    while True:
        if _shutdown_signal:
            return 0 if _shutdown_group(proc, pgid) else 72

        return_code = proc.poll()
        if return_code is None:
            time.sleep(POLL_INTERVAL_SECONDS)
            continue

        # ros2 launch can exit before descendants. Treat that as a failed preview,
        # clean only the owned group, and never let the supervisor disappear while
        # RViz/MoveIt/ros2_control remain alive.
        if _group_alive(pgid):
            print(
                "Workcell Studio preview group ERROR: leader exited "
                f"rc={return_code} while pgid={pgid} still had descendants; cleaning owned group",
                file=sys.stderr,
                flush=True,
            )
            return 75 if _shutdown_group(proc, pgid) else 72

        return return_code


if __name__ == "__main__":
    raise SystemExit(main())
