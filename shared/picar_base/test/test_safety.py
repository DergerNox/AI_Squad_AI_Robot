"""Testing plan, test 2: SafetyGate rules, no hardware or ROS needed.

Run with:  cd ~/picar_ws/src/picar/shared/picar_base && python3 -m pytest test
"""

import pytest

from picar_base import safety
from picar_base.safety import SafetyGate


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


@pytest.fixture
def clock(monkeypatch):
    fake = FakeClock()
    monkeypatch.setattr(safety.time, 'monotonic', fake)
    return fake


@pytest.fixture
def gate(clock):
    return SafetyGate(max_speed=0.4, cmd_timeout_s=0.5,
                      obstacle_stop_m=0.25, range_timeout_s=0.3)


def fresh(gate, range_m=2.0):
    gate.on_command()
    gate.on_range(range_m)


def test_speed_is_capped_both_ways(gate):
    fresh(gate)
    assert gate.filter(1.0) == (0.4, 'ok')
    assert gate.filter(-1.0) == (-0.4, 'ok')


def test_no_command_yet_means_stop(gate):
    gate.on_range(2.0)
    assert gate.filter(0.3) == (0.0, 'timeout')


def test_command_timeout_stops(gate, clock):
    fresh(gate)
    clock.t += 0.6
    gate.on_range(2.0)
    assert gate.filter(0.3) == (0.0, 'timeout')


def test_obstacle_blocks_forward_but_not_reverse(gate):
    fresh(gate, 0.2)
    assert gate.filter(0.3) == (0.0, 'obstacle')
    assert gate.filter(-0.3) == (-0.3, 'ok')


def test_too_close_reading_counts_as_obstacle(gate):
    fresh(gate, float('-inf'))
    assert gate.filter(0.3) == (0.0, 'obstacle')


def test_nothing_in_range_is_clear(gate):
    fresh(gate, float('inf'))
    assert gate.filter(0.3) == (0.3, 'ok')


def test_stale_range_blocks_forward_but_not_reverse(gate, clock):
    fresh(gate)
    clock.t += 0.4
    gate.on_command()
    assert gate.filter(0.3) == (0.0, 'no_range')
    assert gate.filter(-0.3) == (-0.3, 'ok')


def test_estop_latches_until_cleared(gate):
    fresh(gate)
    gate.estop = True
    assert gate.filter(0.3) == (0.0, 'estop')
    gate.on_command()
    assert gate.filter(-0.3) == (0.0, 'estop')
    gate.estop = False
    assert gate.filter(0.3) == (0.3, 'ok')
