"""ultrasonic: the PiCar-B's distance sensor (HC-SR04) as a ROS 2 node.

Publishes /ultrasonic/range (sensor_msgs/Range) rate_hz times a second:
    a number   metres from the sensor to the nearest thing straight ahead
    +inf       nothing within max_range_m: also when the sound does not come back at all,
               which the sensor says with a long echo pulse (its own timeout: about 38 ms on
               a genuine HC-SR04, 134 ms on the car's, measured 2026-11-14). Soft or thin
               things (cloth, a chair leg) send no sound back either, so they count as
               nothing too: the camera has to watch for them.
    -inf       closer than the sensor can measure (2 cm)
    NaN        no echo pulse at all: the sensor or its wiring failed
picar_base stops the car when something is too close, and also when no valid reading
has arrived for 0.3 s. So every reading here is a fresh one: if the sensor or this
node stops working, the car stops instead of driving on an old reading.

The pins are BCM numbers, by default Adeept's wiring (their server/ultra.py); change
them in config/car.yaml. Needs: sudo apt install python3-gpiozero python3-lgpio
On a PC without the sensor: GPIOZERO_PIN_FACTORY=mock ros2 run picar_car ultrasonic
(it then publishes NaN, because nothing echoes).
"""

import math
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

SPEED_OF_SOUND = 343.0   # m/s at about 20 degrees C
ECHO_WAIT_S = 0.2        # longer than any echo pulse, the sensor's own timeout pulse included


class HCSR04:
    """One reading at a time; the echo times come from the GPIO driver's edge timestamps."""

    def __init__(self, trigger_pin, echo_pin):
        from gpiozero import InputDevice, OutputDevice   # here, so a missing library gets a clear message
        self.trigger = OutputDevice(trigger_pin)
        self.echo = InputDevice(echo_pin)
        self.ticks_diff = self.echo.pin_factory.ticks_diff
        self.rise = self.fall = None
        self.done = threading.Event()
        self.echo.pin.edges = 'both'
        self.echo.pin.when_changed = self.on_edge

    def on_edge(self, ticks, state):
        if state:                       # the echo pulse starts when the sound leaves
            self.rise = ticks
        elif self.rise is not None:     # and ends when it comes back
            self.fall = ticks
            self.done.set()

    def read(self):
        """Metres to the nearest thing (from a timeout pulse: far beyond any real echo), or
        None if no echo pulse came or the last one never ended."""
        if self.echo.is_active:         # still high from the last reading: a fault
            return None
        self.rise = self.fall = None
        self.done.clear()
        self.trigger.on()               # a pulse of at least 10 microseconds starts a reading
        time.sleep(0.00001)
        self.trigger.off()
        # Until 2026-11-14 this waited 60 ms: the car's sensor sends 134 ms when nothing comes
        # back, so every such reading became NaN and picar_base let the car drive nowhere.
        if not self.done.wait(ECHO_WAIT_S):
            return None
        return self.ticks_diff(self.fall, self.rise) * SPEED_OF_SOUND / 2

    def close(self):
        self.echo.pin.when_changed = None
        self.trigger.close()
        self.echo.close()


class Ultrasonic(Node):

    def __init__(self):
        super().__init__('ultrasonic')
        param = self.declare_parameter
        trigger_pin = param('trigger_pin', 11).value
        echo_pin = param('echo_pin', 8).value
        rate_hz = param('rate_hz', 10.0).value

        self.msg = Range()
        self.msg.header.frame_id = param('frame_id', 'ultrasonic_link').value
        self.msg.radiation_type = Range.ULTRASOUND
        self.msg.field_of_view = param('field_of_view_rad', 0.26).value   # about 15 degrees
        self.msg.min_range = 0.02
        self.msg.max_range = param('max_range_m', 4.0).value

        try:
            self.sensor = HCSR04(trigger_pin, echo_pin)
        except ImportError:
            self.get_logger().error('gpiozero is missing: sudo apt install python3-gpiozero python3-lgpio')
            raise
        except Exception as error:
            self.get_logger().error(
                f'Cannot use GPIO {trigger_pin} and {echo_pin}: {error}. If the pins are busy, '
                'turn SPI off (dtparam=spi=off in /boot/firmware/config.txt, then reboot). If '
                'permission is denied, check the group of /dev/gpiochip0 and add yourself to it.')
            raise

        self.publisher = self.create_publisher(Range, '/ultrasonic/range', qos_profile_sensor_data)
        self.create_timer(1.0 / rate_hz, self.publish)
        self.get_logger().info(
            f'ultrasonic ready (trigger GPIO {trigger_pin}, echo GPIO {echo_pin}, {rate_hz:.0f} Hz)')

    def publish(self):
        distance = self.sensor.read()
        if distance is None:
            value = math.nan                  # no echo: not a reading at all
        elif distance >= self.msg.max_range:
            value = math.inf                  # nothing within range
        elif distance < self.msg.min_range:
            value = -math.inf                 # too close to measure
        else:
            value = distance
        self.msg.range = value
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.msg)

    def close(self):
        self.sensor.close()


def main():
    rclpy.init()
    node = None
    try:
        node = Ultrasonic()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
