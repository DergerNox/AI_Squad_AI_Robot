"""The ultrasonic node with gpiozero's mock pins: a simulated HC-SR04 echo at set distances.

Needs gpiozero (on the Pi: sudo apt install python3-gpiozero); skipped without it.
Takes about 10 seconds, because the mock sensor answers in real time.

    source ~/picar_ws/install/setup.bash
    cd picar_car && python3 -m pytest test
"""

import math
import statistics
import time
from types import SimpleNamespace

import pytest

pytest.importorskip('gpiozero', reason='needs gpiozero: sudo apt install python3-gpiozero')

from gpiozero import Device  # noqa: E402
from gpiozero.pins.mock import MockFactory, MockTriggerPin  # noqa: E402
from picar_car.ultrasonic_node import Ultrasonic  # noqa: E402
import rclpy  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Range  # noqa: E402

SPEED_OF_SOUND = 343.0   # m/s


def echo_time(distance):
    """How long the echo pulse lasts for something this far away."""
    return 2 * distance / SPEED_OF_SOUND


@pytest.fixture(scope='module')
def sensor():
    """The node on mock pins (trigger 11, echo 8), and a way to collect what it publishes."""
    Device.pin_factory = MockFactory()
    echo = Device.pin_factory.pin(8)
    trigger = Device.pin_factory.pin(11, pin_class=MockTriggerPin, echo_pin=echo,
                                     echo_time=echo_time(1.0))
    rclpy.init()
    node = Ultrasonic()
    got = []
    node.create_subscription(Range, '/ultrasonic/range', got.append, qos_profile_sensor_data)

    def listen(seconds):
        """The messages published in the next `seconds`."""
        got.clear()
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
        return list(got)

    listen(1.0)   # let the first readings arrive
    yield SimpleNamespace(trigger=trigger, echo=echo, listen=listen)
    node.close()
    node.destroy_node()
    rclpy.try_shutdown()
    Device.pin_factory.close()


def readings(sensor, seconds=1.0):
    """The ranges published in the next `seconds`, without the first one (it may be
    from before the last change)."""
    return [message.range for message in sensor.listen(seconds)][1:]


@pytest.mark.parametrize('distance', [0.10, 0.30, 1.00, 2.50, 3.90])
def test_distance_within_3_cm_about_10_times_a_second(sensor, distance):
    sensor.trigger.echo_time = echo_time(distance)
    values = readings(sensor)
    assert 7 <= len(values) <= 11
    # The mock echo is timed by a thread: about 1.5 cm long on the PC, more on a slow Pi
    assert statistics.median(values) == pytest.approx(distance, abs=0.03)


def test_nothing_within_4_m_is_inf(sensor):
    sensor.trigger.echo_time = echo_time(6.0)
    values = readings(sensor)
    assert values and all(value == math.inf for value in values)


def test_no_echo_is_nan(sensor):
    sensor.trigger.echo_pin = Device.pin_factory.pin(5)   # the echo misses the node's pin
    try:
        values = readings(sensor)
    finally:
        sensor.trigger.echo_pin = sensor.echo
    assert values and all(math.isnan(value) for value in values)


def test_message_fields(sensor):
    message = sensor.listen(0.5)[-1]
    assert message.header.frame_id == 'ultrasonic_link'
    assert message.radiation_type == Range.ULTRASOUND
    assert message.min_range == pytest.approx(0.02) and message.max_range == 4.0
