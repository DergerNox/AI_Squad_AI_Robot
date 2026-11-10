"""picar_base: drives the PiCar-B through the SafetyGate.

Only the 'sim' backend exists so far: the filtered command goes to the Gazebo
twin on /sim/cmd_vel (steering becomes a yaw rate, v * tan(steering) / wheelbase)
and head angles go to /sim/head_pan and /sim/head_tilt.

Interface (same as the real car):
  /cmd_vel           geometry_msgs/Twist     teleop; must keep arriving (cmd_timeout_s)
  /ultrasonic/range  sensor_msgs/Range       feeds the obstacle stop
  /picar/move        picar_interfaces/Move   drive for up to max_move_s
  /picar/look        picar_interfaces/Look   point the camera head
  /picar/stop        std_srvs/Trigger        stop now
  /picar/estop       std_srvs/SetBool        latch / clear the emergency stop
  /picar/status      std_msgs/String, 2 Hz   ok, timeout, obstacle, no_range or estop
"""

import math
import signal
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from picar_interfaces.srv import Look, Move
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Range
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool, Trigger

from picar_base.safety import SafetyGate


def clamp(value, low, high):
    return max(low, min(high, value))


class PicarBase(Node):

    def __init__(self):
        super().__init__('picar_base')
        param = self.declare_parameter
        self.backend = param('backend', 'sim').value
        self.wheelbase = param('wheelbase_m', 0.145).value
        self.full_speed = param('full_speed_mps', 0.5).value   # speed at full motor power
        self.max_steering = param('max_steering_deg', 30.0).value
        self.max_move_s = param('max_move_s', 3.0).value
        self.max_pan = param('max_pan_deg', 90.0).value
        self.max_tilt = param('max_tilt_deg', 40.0).value
        self.gate = SafetyGate(
            max_speed=param('max_speed', 0.4).value,
            cmd_timeout_s=param('cmd_timeout_s', 0.5).value,
            obstacle_stop_m=param('obstacle_stop_m', 0.25).value,
            range_timeout_s=param('range_timeout_s', 0.3).value)
        if self.backend != 'sim':
            raise RuntimeError(f"backend '{self.backend}' is not implemented yet; use 'sim'")

        self.teleop_power = 0.0
        self.teleop_steering = 0.0
        self.status = 'timeout'
        self.moving = threading.Event()
        self.cancel = threading.Event()
        self.output_lock = threading.Lock()
        self.closing = False

        self.cmd_pub = self.create_publisher(Twist, '/sim/cmd_vel', 10)
        self.pan_pub = self.create_publisher(Float64, '/sim/head_pan', 10)
        self.tilt_pub = self.create_publisher(Float64, '/sim/head_tilt', 10)
        self.status_pub = self.create_publisher(String, '/picar/status', 10)

        # A move blocks for up to max_move_s, so it gets its own group and the
        # range updates, stop/estop and the 20 Hz loop keep running beside it.
        loop_group = MutuallyExclusiveCallbackGroup()
        move_group = ReentrantCallbackGroup()
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10,
                                 callback_group=loop_group)
        self.create_subscription(Range, '/ultrasonic/range', self.on_range,
                                 qos_profile_sensor_data, callback_group=loop_group)
        self.create_service(Move, '/picar/move', self.on_move, callback_group=move_group)
        self.create_service(Look, '/picar/look', self.on_look, callback_group=loop_group)
        self.create_service(Trigger, '/picar/stop', self.on_stop, callback_group=loop_group)
        self.create_service(SetBool, '/picar/estop', self.on_estop, callback_group=loop_group)
        self.create_timer(0.05, self.loop, callback_group=loop_group)              # 20 Hz
        self.create_timer(0.5, self.publish_status, callback_group=loop_group)     # 2 Hz

        self.zero_outputs(center_head=True)
        self.get_logger().info('picar_base ready (backend: sim)')

    # ---------- outputs ----------

    def drive(self, power, steering_deg):
        """power is motor power from -1 to 1, already filtered by the gate."""
        v = power * self.full_speed
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = v * math.tan(math.radians(steering_deg)) / self.wheelbase
        with self.output_lock:
            if not self.closing:
                self.cmd_pub.publish(msg)

    def zero_outputs(self, center_head=False):
        self.drive(0.0, 0.0)
        if center_head:
            self.pan_pub.publish(Float64(data=0.0))
            self.tilt_pub.publish(Float64(data=0.0))

    def final_stop(self):
        """Send one last stop and block every later drive command."""
        self.cancel.set()
        with self.output_lock:
            self.closing = True
            self.cmd_pub.publish(Twist())

    def clamp_steering(self, steering_deg):
        return clamp(steering_deg, -self.max_steering, self.max_steering)

    # ---------- inputs ----------

    def on_cmd_vel(self, msg):
        v, w = msg.linear.x, msg.angular.z
        self.teleop_power = clamp(v / self.full_speed, -1.0, 1.0)
        if abs(v) > 1e-3:
            steering = math.degrees(math.atan(self.wheelbase * w / v))
        else:
            steering = math.copysign(self.max_steering, w) if abs(w) > 1e-3 else 0.0
        self.teleop_steering = self.clamp_steering(steering)
        self.gate.on_command()

    def on_range(self, msg):
        if not math.isnan(msg.range):
            self.gate.on_range(msg.range)

    def loop(self):
        if self.moving.is_set():
            return
        power, why = self.gate.filter(self.teleop_power)
        self.status = why
        self.drive(power, self.teleop_steering if power != 0.0 else 0.0)

    def publish_status(self):
        self.status_pub.publish(String(data=self.status))

    # ---------- services ----------

    def on_move(self, req, res):
        if self.moving.is_set():
            res.completed, res.reason = False, 'busy'
            res.min_range_m = float(min(self.gate.range_m, 4.0))
            return res
        self.moving.set()
        self.cancel.clear()
        self.teleop_power = 0.0
        speed = clamp(req.speed, -1.0, 1.0)
        steering = self.clamp_steering(req.steering_deg)
        end = time.monotonic() + clamp(req.duration_s, 0.0, self.max_move_s)
        reason, closest = 'done', 4.0
        try:
            while time.monotonic() < end:
                if self.cancel.is_set():
                    reason = 'estop' if self.gate.estop else 'stopped'
                    break
                self.gate.on_command()
                power, why = self.gate.filter(speed * self.gate.max_speed)
                self.status = why
                self.drive(power, steering)
                closest = min(closest, self.gate.range_m)
                if why != 'ok':
                    reason = why
                    break
                time.sleep(0.05)
        finally:
            self.drive(0.0, 0.0)
            self.moving.clear()
        res.completed = reason == 'done'
        res.reason = reason
        res.min_range_m = float(closest)
        return res

    def on_look(self, req, res):
        pan = clamp(req.pan_deg, -self.max_pan, self.max_pan)
        tilt = clamp(req.tilt_deg, -self.max_tilt, self.max_tilt)
        self.pan_pub.publish(Float64(data=math.radians(pan)))
        self.tilt_pub.publish(Float64(data=math.radians(tilt)))
        res.success = True
        res.message = f'pan {pan:.0f} deg, tilt {tilt:.0f} deg'
        return res

    def on_stop(self, req, res):
        self.teleop_power = 0.0
        self.cancel.set()
        self.drive(0.0, 0.0)
        res.success, res.message = True, 'stopped'
        return res

    def on_estop(self, req, res):
        self.gate.estop = req.data
        if req.data:
            self.teleop_power = 0.0
            self.cancel.set()
            self.drive(0.0, 0.0)
        res.success = True
        res.message = 'estop latched' if req.data else 'estop cleared'
        return res


def main():
    # Handle Ctrl+C and SIGTERM ourselves so the car is stopped before exit
    # (set explicitly: a process started in the background inherits SIGINT ignored).
    stop = threading.Event()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, lambda signum, frame: stop.set())
    signal.signal(signal.SIGTERM, lambda signum, frame: stop.set())
    node = PicarBase()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        while rclpy.ok() and not stop.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        node.final_stop()
        time.sleep(0.2)          # let the stop command go out
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
