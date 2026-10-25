"""The single clock for measured-evidence freshness, ordering and latency.

CLOCK_MONOTONIC_RAW is shared by every process of one boot, is never stepped
and is never rate-adjusted. On LENLEG (WSL2, 2026-10-23) CLOCK_REALTIME was
stepped forward ~1.6-2.1 s every 32 s while the kernel tick was held at
9167 us, so CLOCK_MONOTONIC ran up to 8.3 % slow against the Windows host and
CLOCK_MONOTONIC_RAW. Wall time made a 1 ms-old sample "stale"; CLOCK_MONOTONIC
would have stretched the 250 ms guard to ~273 ms of real time. Wall-clock
stamps remain evidence only.
"""
import time

CLOCK_NAME = 'CLOCK_MONOTONIC_RAW'


def now_ns():
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
