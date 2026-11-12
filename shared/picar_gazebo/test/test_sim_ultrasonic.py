"""sim_ultrasonic: the twin's exact readings made like a real HC-SR04's.

    source ~/picar_ws/install/setup.bash
    cd ~/picar_ws/src/picar/shared/picar_gazebo && python3 -m pytest test
"""

import importlib.util
import math
import pathlib
import random
import statistics
from types import SimpleNamespace

import pytest
import rclpy
from sensor_msgs.msg import Range

SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "sim_ultrasonic.py"
spec = importlib.util.spec_from_file_location("sim_ultrasonic", SCRIPT)
sim_ultrasonic = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sim_ultrasonic)
shape = sim_ultrasonic.shape


def readings(range_m, n=4000, noise_m=0.01, dropout=0.02, seed=1):
    rng = random.Random(seed)
    return [shape(range_m, 0.02, 4.0, noise_m, dropout, rng) for _ in range(n)]


def test_readings_scatter_by_the_noise_around_the_true_distance():
    values = [v for v in readings(1.0) if not math.isnan(v)]
    assert statistics.mean(values) == pytest.approx(1.0, abs=0.002)
    assert statistics.stdev(values) == pytest.approx(0.01, rel=0.1)


def test_about_the_dropout_share_of_readings_has_no_echo():
    values = readings(1.0)
    assert sum(math.isnan(v) for v in values) / len(values) == pytest.approx(0.02, abs=0.007)


def test_without_noise_and_dropout_the_reading_is_exact():
    assert set(readings(1.234, n=50, noise_m=0.0, dropout=0.0)) == {1.234}


def test_nothing_in_range_and_too_close_stay_as_they_are():
    assert set(readings(math.inf, n=200, dropout=0.0)) == {math.inf}
    assert set(readings(-math.inf, n=200, dropout=0.0)) == {-math.inf}


def test_noise_cannot_carry_a_reading_past_the_sensors_limits():
    near = readings(0.021, n=500, dropout=0.0)          # 1 mm above the 2 cm minimum
    assert set(near) - {-math.inf} and -math.inf in near
    assert all(v == -math.inf or v >= 0.02 for v in near)
    far = readings(3.995, n=500, dropout=0.0)
    assert math.inf in far and all(v == math.inf or v < 4.0 for v in far)


def test_the_same_seed_gives_the_same_readings():
    assert readings(1.0, n=100, seed=7) == readings(1.0, n=100, seed=7)
    assert readings(1.0, n=100, seed=7) != readings(1.0, n=100, seed=8)


@pytest.fixture
def node():
    rclpy.init(args=["--ros-args", "-p", "seed:=3", "-p", "dropout:=0.0"])
    made = sim_ultrasonic.SimUltrasonic()
    yield made
    made.destroy_node()
    rclpy.try_shutdown()


def test_the_node_republishes_as_an_ultrasonic_reading(node):
    sent = []
    node.publisher = SimpleNamespace(publish=sent.append)
    msg = Range(radiation_type=Range.INFRARED, min_range=0.02, max_range=4.0, range=1.5)
    msg.header.frame_id = "ultrasonic_link"
    node.on_reading(msg)
    assert len(sent) == 1 and sent[0].header.frame_id == "ultrasonic_link"
    assert sent[0].radiation_type == Range.ULTRASOUND
    assert sent[0].range == pytest.approx(1.5, abs=0.05) and sent[0].range != 1.5
