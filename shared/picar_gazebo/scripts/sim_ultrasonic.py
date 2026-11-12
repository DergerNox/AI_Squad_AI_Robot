#!/usr/bin/env python3
"""sim_ultrasonic: the twin's distance sensor node (node name: ultrasonic).

Gazebo's sensor is exact; a real HC-SR04 is not. This node takes the twin's reading from
sim_bridge (/sim/ultrasonic/range: the shortest ray of the lidar fan in the car model) and
publishes it on /ultrasonic/range the way the real car's ultrasonic node does:
    a number   metres, with some noise (noise_m)
    +inf       nothing within the sensor's range (4 m)
    -inf       closer than the sensor can measure (2 cm)
    NaN        now and then no echo comes back (dropout: the share of readings)
so the agent and picar_base meet in the twin what they will meet on the car.

Parameters: noise_m (0.01), dropout (0.02), seed (0: a different series each run).
These are estimates: measure them on the real sensor.
"""

import math
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

MIN_RANGE_M = 0.02   # HC-SR04, used when the bridge does not say
MAX_RANGE_M = 4.0


def shape(range_m, min_m, max_m, noise_m, dropout, rng):
    """One exact reading as the real sensor would give it. rng: a random.Random."""
    if rng.random() < dropout:
        return math.nan                    # no echo came back
    if math.isnan(range_m) or math.isinf(range_m):
        return range_m                     # no reading, or out of range already
    value = range_m + (rng.gauss(0.0, noise_m) if noise_m > 0 else 0.0)
    if value >= max_m:
        return math.inf
    if value < min_m:
        return -math.inf
    return value


class SimUltrasonic(Node):

    def __init__(self):
        super().__init__('ultrasonic')
        self.noise_m = self.declare_parameter('noise_m', 0.01).value
        self.dropout = self.declare_parameter('dropout', 0.02).value
        self.rng = random.Random(self.declare_parameter('seed', 0).value or None)
        self.publisher = self.create_publisher(Range, '/ultrasonic/range', qos_profile_sensor_data)
        self.create_subscription(Range, '/sim/ultrasonic/range', self.on_reading,
                                 qos_profile_sensor_data)
        self.get_logger().info(f'ultrasonic ready (twin: noise {self.noise_m} m, '
                               f'{self.dropout:.0%} missing echoes)')

    def on_reading(self, msg):
        msg.range = shape(msg.range, msg.min_range or MIN_RANGE_M, msg.max_range or MAX_RANGE_M,
                          self.noise_m, self.dropout, self.rng)
        msg.radiation_type = Range.ULTRASOUND
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = SimUltrasonic()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
