#!/usr/bin/env python3
"""
command_listener.py: example ROS 2 node that receives drive commands.

It listens on /cmd_vel and prints every command it gets. It also publishes
odometry on /odom: where the car should be now. The PiCar-B has no wheel
encoders, so the position is worked out from the commands themselves.
Later the printing gets replaced by real motor and steering control.

Interface (standard ROS messages, so it runs on any ROS 2 setup):
    in   /cmd_vel   geometry_msgs/Twist     linear.x = speed in m/s (+ forward),
                                            angular.z = turn rate in rad/s (+ left)
    out  /odom      nav_msgs/Odometry       10 times a second, frame "odom" -> "base_link"

The car's numbers are ROS parameters, so they can be changed without editing
this file (defaults: the twin's values):
    wheelbase_m        rear axle to front axle               0.145
    max_steering_deg   steering limit                        30.0
    command_timeout_s  stop when no command came this long   0.5

Run it:
    source /opt/ros/jazzy/setup.bash
    python3 command_listener.py
    python3 command_listener.py --ros-args -p wheelbase_m:=0.15    # other numbers

Test it without the car, in a second terminal:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ros2 topic echo /odom --field pose.pose.position
"""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

RATE_HZ = 10.0   # how often /odom is published


class CommandListener(Node):
    def __init__(self):
        super().__init__('command_listener')
        self.wheelbase = self.declare_parameter('wheelbase_m', 0.145).value
        self.max_steering_deg = self.declare_parameter('max_steering_deg', 30.0).value
        self.command_timeout = self.declare_parameter('command_timeout_s', 0.5).value

        self.speed = 0.0        # m/s, from the last command
        self.turn_rate = 0.0    # rad/s, from the last command
        self.x = self.y = self.heading = 0.0   # where the car is (m, m, rad)
        self.moving = False
        self.last_command = self.get_clock().now()
        self.last_update = self.get_clock().now()

        self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_timer(1.0 / RATE_HZ, self.update)
        self.get_logger().info(
            f'Waiting for commands on /cmd_vel (wheelbase {self.wheelbase} m, '
            f'steering limit {self.max_steering_deg} deg)')

    def on_command(self, msg):
        """Take every message on /cmd_vel as the new command."""
        self.speed = msg.linear.x
        self.turn_rate = msg.angular.z
        self.last_command = self.get_clock().now()
        self.moving = True
        self.get_logger().info(
            f'Command: speed {self.speed:+.2f} m/s, '
            f'turn {math.degrees(self.turn_rate):+.0f} deg/s')

    def update(self):
        """Move the estimated position and publish it (10 times a second)."""
        now = self.get_clock().now()
        dt = (now - self.last_update).nanoseconds / 1e9
        self.last_update = now

        # No new command for a while: stop, like the real car's safety timeout
        if self.moving and (now - self.last_command).nanoseconds / 1e9 > self.command_timeout:
            self.speed = self.turn_rate = 0.0
            self.moving = False
            self.get_logger().info(
                f'No commands, stopped at x {self.x:.2f} m, y {self.y:.2f} m, '
                f'heading {math.degrees(self.heading):.0f} deg')

        # A car can only turn while it drives, and only as sharply as the steering allows
        tan_steering = math.tan(math.radians(self.max_steering_deg))
        max_turn_rate = abs(self.speed) * tan_steering / self.wheelbase
        turn_rate = max(-max_turn_rate, min(max_turn_rate, self.turn_rate))

        # Dead reckoning: assume the car did exactly what the commands said
        self.heading += turn_rate * dt
        self.x += self.speed * math.cos(self.heading) * dt
        self.y += self.speed * math.sin(self.heading) * dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        # The heading as a quaternion (a rotation around the vertical axis)
        odom.pose.pose.orientation.z = math.sin(self.heading / 2)
        odom.pose.pose.orientation.w = math.cos(self.heading / 2)
        odom.twist.twist.linear.x = self.speed
        odom.twist.twist.angular.z = turn_rate
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = CommandListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():   # works on older ROS 2 versions too
            rclpy.shutdown()


if __name__ == '__main__':
    main()
