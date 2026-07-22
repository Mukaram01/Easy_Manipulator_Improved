from scripts.modbus_meter_sequence import Feedback, SequentialMeterPoller


def scan(poller, feedback=None, elapsed_ms=100):
    output = poller.scan(feedback, elapsed_ms=elapsed_ms)
    poller.assert_single_active_transaction(output)
    return output


def test_one_meter_runs_at_a_time_and_preserves_building_order():
    poller = SequentialMeterPoller({5: [1, 2], 12: [1], 8: [1], 4: [1], 9: [1]}, inter_building_delay_ms=100)
    seen = []
    for _ in range(60):
        out = scan(poller)
        if out.request:
            seen.append((out.diagnostics.active_building, out.diagnostics.active_meter, out.request_register))
            scan(poller, Feedback(done=True))
        elif out.diagnostics.active_read_step is not None:
            followup = scan(poller, Feedback(done=True))
            if followup.request:
                seen.append((followup.diagnostics.active_building, followup.diagnostics.active_meter, followup.request_register))
    assert seen[:4] == [(5, 1, 47681), (5, 1, 40513), (5, 2, 47681), (5, 2, 40513)]
    assert [building for building, _, reg in seen if reg == 47681][:5] == [5, 5, 12, 8, 4]


def test_next_meter_waits_for_terminal_result():
    poller = SequentialMeterPoller({5: [1, 2]})
    assert scan(poller).request
    for _ in range(3):
        out = scan(poller, Feedback(busy=True), elapsed_ms=100)
        assert out.diagnostics.active_meter == 1
        assert not out.request
    assert scan(poller, Feedback(done=True)).diagnostics.active_meter == 1
    assert scan(poller).request  # second read, still meter 1
    scan(poller, Feedback(done=True))
    assert scan(poller).diagnostics.active_meter == 2


def test_timeout_causes_disconnect_cooldown_and_retry_same_meter():
    poller = SequentialMeterPoller({5: [1]}, watchdog_ms=200, cooldown_ms=300)
    assert scan(poller).request
    out = scan(poller, Feedback(busy=True, status=0x7002), elapsed_ms=200)
    assert out.diagnostics.last_failed_meter == (5, 1)
    assert out.diagnostics.retry_count == 1
    assert scan(poller).disconnect
    assert scan(poller, elapsed_ms=100).diagnostics.cooldown_active
    assert scan(poller, elapsed_ms=200).diagnostics.active_meter == 1
    retry = scan(poller)
    assert retry.request
    assert retry.request_register == 47681


def test_max_retries_cause_sequential_skip_to_next_meter():
    poller = SequentialMeterPoller({5: [1, 2]}, max_retries=1, watchdog_ms=100, cooldown_ms=100)
    scan(poller)  # request meter 1
    scan(poller, Feedback(error=True, status=0x7002), elapsed_ms=100)
    scan(poller)  # disconnect
    scan(poller, elapsed_ms=100)  # cooldown complete
    scan(poller)  # retry request
    scan(poller, Feedback(error=True, status=0x7002), elapsed_ms=100)
    assert poller.failed_meters == [(5, 1, "ERROR", 0x7002)]
    assert scan(poller).diagnostics.active_meter == 2


def test_next_building_cannot_start_early_and_done_pulse_precedes_delay():
    poller = SequentialMeterPoller({5: [1], 12: [1]}, inter_building_delay_ms=500)
    scan(poller)
    scan(poller, Feedback(done=True))
    scan(poller)
    out = scan(poller, Feedback(done=True))
    assert out.diagnostics.active_building == 12
    pulse = scan(poller, elapsed_ms=100)
    assert pulse.building_done_pulse == 5
    assert not pulse.request
    assert pulse.diagnostics.active_building == 12
    assert scan(poller, elapsed_ms=300).diagnostics.active_building == 12
    request = scan(poller, elapsed_ms=100)
    assert not request.request
    request = scan(poller)
    assert request.request
    assert request.diagnostics.active_building == 12


def test_no_parallel_meter_execution_is_introduced():
    poller = SequentialMeterPoller({5: [1, 2], 12: [1]}, watchdog_ms=1000)
    for _ in range(50):
        out = scan(poller, Feedback(busy=True), elapsed_ms=10)
        diag = out.diagnostics
        assert diag.active_request_count + diag.active_busy_count <= 1
        assert diag.safety_assertion_ok
        if out.request:
            assert out.request_register in (47681, 40513)
