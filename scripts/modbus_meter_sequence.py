"""Sequential Modbus meter polling state machine.

This module models the PLC/MB_CLIENT ownership contract for DB99 meter polling:
one controller owns Request/Disconnect, emits one-scan Request pulses, waits for
DONE/BUSY/ERROR or watchdog timeout, cools down after failures, and never
advances to another meter/building until the active meter is terminal.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Iterable

BUILDING_ORDER = (5, 12, 8, 4, 9)
READS = ((47681, 58), (40513, 18))
TERMINAL_RESULTS = {"DONE", "ERROR", "TIMEOUT", "SKIPPED"}


class Step(str, Enum):
    IDLE = "IDLE"
    REQUEST_PULSE = "REQUEST_PULSE"
    WAIT_FEEDBACK = "WAIT_FEEDBACK"
    DISCONNECT_PULSE = "DISCONNECT_PULSE"
    COOLDOWN = "COOLDOWN"
    BETWEEN_BUILDINGS = "BETWEEN_BUILDINGS"
    COMPLETE = "COMPLETE"


@dataclass(frozen=True)
class Meter:
    building: int
    meter: int
    enabled: bool = True


@dataclass(frozen=True)
class Feedback:
    done: bool = False
    busy: bool = False
    error: bool = False
    status: int = 0


@dataclass
class Diagnostics:
    active_building: int | None = None
    active_meter: int | None = None
    active_read_step: int | None = None
    retry_count: int = 0
    watchdog_elapsed_ms: int = 0
    last_failed_meter: tuple[int, int] | None = None
    last_modbus_status: int = 0
    cooldown_active: bool = False
    active_request_count: int = 0
    active_busy_count: int = 0
    safety_assertion_ok: bool = True


@dataclass
class ScanOutput:
    request: bool = False
    disconnect: bool = False
    request_register: int | None = None
    request_length: int | None = None
    building_done_pulse: int | None = None
    meter_result: str | None = None
    diagnostics: Diagnostics = field(default_factory=Diagnostics)


class SequentialMeterPoller:
    """Single-active-meter scheduler for DB99-backed MB_CLIENT polling."""

    def __init__(
        self,
        meters_by_building: dict[int, Iterable[int]],
        *,
        enabled_buildings: Iterable[int] = BUILDING_ORDER,
        max_retries: int = 2,
        watchdog_ms: int = 10_000,
        cooldown_ms: int = 1_000,
        inter_building_delay_ms: int = 2_000,
    ) -> None:
        self.max_retries = max_retries
        self.watchdog_ms = watchdog_ms
        self.cooldown_ms = cooldown_ms
        self.inter_building_delay_ms = inter_building_delay_ms
        enabled = set(enabled_buildings)
        self.plan: list[Meter] = [
            Meter(building, meter)
            for building in BUILDING_ORDER
            if building in enabled
            for meter in meters_by_building.get(building, ())
        ]
        self.index = 0
        self.step = Step.IDLE if self.plan else Step.COMPLETE
        self.read_index = 0
        self.retry_count = 0
        self.watchdog_elapsed_ms = 0
        self.cooldown_elapsed_ms = 0
        self.inter_building_elapsed_ms = 0
        self.last_failed_meter: tuple[int, int] | None = None
        self.last_modbus_status = 0
        self.failed_meters: list[tuple[int, int, str, int]] = []
        self.completed_meters: list[tuple[int, int]] = []
        self._pending_building_done: int | None = None

    @property
    def active_meter(self) -> Meter | None:
        if self.index < len(self.plan):
            return self.plan[self.index]
        return None

    def scan(self, feedback: Feedback | None = None, *, elapsed_ms: int = 100) -> ScanOutput:
        feedback = feedback or Feedback()
        out = ScanOutput()
        if self.step is Step.COMPLETE:
            out.diagnostics = self._diagnostics(False, feedback.busy)
            return out

        if self.step is Step.IDLE:
            self.step = Step.REQUEST_PULSE

        if self.step is Step.REQUEST_PULSE:
            register, length = READS[self.read_index]
            out.request = True
            out.request_register = register
            out.request_length = length
            self.watchdog_elapsed_ms = 0
            self.step = Step.WAIT_FEEDBACK
        elif self.step is Step.WAIT_FEEDBACK:
            self.watchdog_elapsed_ms += elapsed_ms
            self.last_modbus_status = feedback.status
            if feedback.done:
                self._finish_read_or_meter("DONE")
            elif feedback.error:
                self._start_failure_recovery("ERROR", feedback.status)
            elif self.watchdog_elapsed_ms >= self.watchdog_ms:
                self._start_failure_recovery("TIMEOUT", feedback.status)
        elif self.step is Step.DISCONNECT_PULSE:
            out.disconnect = True
            self.cooldown_elapsed_ms = 0
            self.step = Step.COOLDOWN
        elif self.step is Step.COOLDOWN:
            self.cooldown_elapsed_ms += elapsed_ms
            if self.cooldown_elapsed_ms >= self.cooldown_ms:
                self.step = Step.REQUEST_PULSE
        elif self.step is Step.BETWEEN_BUILDINGS:
            self.inter_building_elapsed_ms += elapsed_ms
            if self._pending_building_done is not None:
                out.building_done_pulse = self._pending_building_done
                self._pending_building_done = None
            if self.inter_building_elapsed_ms >= self.inter_building_delay_ms:
                self.step = Step.IDLE if self.active_meter else Step.COMPLETE
                self.inter_building_elapsed_ms = 0

        out.diagnostics = self._diagnostics(out.request, feedback.busy)
        return out

    def _finish_read_or_meter(self, result: str) -> None:
        self.watchdog_elapsed_ms = 0
        self.last_modbus_status = 0
        if self.read_index + 1 < len(READS):
            self.read_index += 1
            self.step = Step.REQUEST_PULSE
            return
        meter = self.active_meter
        if meter is not None:
            self.completed_meters.append((meter.building, meter.meter))
        self._advance_meter()

    def _start_failure_recovery(self, result: str, status: int) -> None:
        meter = self.active_meter
        if meter is not None:
            self.last_failed_meter = (meter.building, meter.meter)
        self.last_modbus_status = status
        if self.retry_count >= self.max_retries:
            if meter is not None:
                self.failed_meters.append((meter.building, meter.meter, result, status))
            self.retry_count = 0
            self.read_index = 0
            self._advance_meter()
        else:
            self.retry_count += 1
            self.step = Step.DISCONNECT_PULSE

    def _advance_meter(self) -> None:
        old = self.active_meter
        self.index += 1
        self.read_index = 0
        next_meter = self.active_meter
        if old is not None and (next_meter is None or next_meter.building != old.building):
            self._pending_building_done = old.building
            self.step = Step.BETWEEN_BUILDINGS
        else:
            self.step = Step.IDLE if next_meter else Step.COMPLETE

    def _diagnostics(self, request: bool, busy: bool) -> Diagnostics:
        active = self.active_meter
        active_request_count = int(request)
        active_busy_count = int(busy and self.step is Step.WAIT_FEEDBACK and not request)
        assertion_ok = active_request_count + active_busy_count <= 1
        return Diagnostics(
            active_building=active.building if active else None,
            active_meter=active.meter if active else None,
            active_read_step=self.read_index + 1 if active and self.step is not Step.BETWEEN_BUILDINGS else None,
            retry_count=self.retry_count,
            watchdog_elapsed_ms=self.watchdog_elapsed_ms,
            last_failed_meter=self.last_failed_meter,
            last_modbus_status=self.last_modbus_status,
            cooldown_active=self.step is Step.COOLDOWN,
            active_request_count=active_request_count,
            active_busy_count=active_busy_count,
            safety_assertion_ok=assertion_ok,
        )

    def assert_single_active_transaction(self, output: ScanOutput) -> None:
        diag = output.diagnostics
        if diag.active_request_count + diag.active_busy_count > 1:
            raise AssertionError("More than one meter transaction is active")
