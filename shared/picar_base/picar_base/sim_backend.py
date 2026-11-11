"""sim_backend.py: picar_base's backend for the Gazebo twin.

The twin's Ackermann controller takes a speed and a yaw rate on /sim/cmd_vel, so the
steering angle becomes v * tan(steering) / wheelbase. The camera head's servos take
angles in radians on /sim/head_pan and /sim/head_tilt. sim_bridge carries all three
to Gazebo (picar_gazebo/config/picar_bridge.yaml).
"""

import math

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

from picar_base.backend import Backend


class SimBackend(Backend):

    def __init__(self, node):
        super().__init__(node)
        self.full_speed = node.get_parameter('full_speed_mps').value   # m/s at full power
        self.wheelbase = node.get_parameter('wheelbase_m').value
        self.cmd_pub = node.create_publisher(Twist, '/sim/cmd_vel', 10)
        self.pan_pub = node.create_publisher(Float64, '/sim/head_pan', 10)
        self.tilt_pub = node.create_publisher(Float64, '/sim/head_tilt', 10)

    def drive(self, power, steering_deg):
        v = power * self.full_speed
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = v * math.tan(math.radians(steering_deg)) / self.wheelbase
        self.cmd_pub.publish(msg)

    def look(self, pan_deg, tilt_deg):
        self.pan_pub.publish(Float64(data=math.radians(pan_deg)))
        self.tilt_pub.publish(Float64(data=math.radians(tilt_deg)))
