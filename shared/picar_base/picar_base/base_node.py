"""picar_base: drives the car through the SafetyGate.

The node keeps the safety rules, the services and the timing. What it drives comes from a
backend (backend.py), chosen by the `backend` parameter: sim drives the Gazebo twin
(sim_backend.py), hw will drive the real PiCar-B and esp32 the PC car.

Interface (the same on every car):
  /cmd_vel           geometry_msgs/Twist     teleop; must keep arriving (cmd_timeout_s)
  /ultrasonic/range  sensor_msgs/Range       feeds the obstacle stop (range_topics: every
                                             distance sensor that guards the way ahead)
  /picar/move        picar_interfaces/Move   drive for up to max_move_s
  /picar/look        picar_interfaces/Look   point the camera head
  /picar/stop        std_srvs/Trigger        stop now
  /picar/estop       std_srvs/SetBool        latch / clear the emergency stop
  /picar/status      std_msgs/String, 2 Hz   ok, timeout, obstacle, no_range or estop
  /picar/odom        nav_msgs/Odometry, 10 Hz  where the car should be, worked out from
                                             what picar_base really sent (no encoders)

Time: the twin (use_sim_time: true in sim.yaml) times everything by Gazebo's clock, so a
move drives as far when Gazebo runs slower than real time. Elsewhere the steady system
clock (time.monotonic) is used: it never jumps when the Pi sets its clock.
"""

import math
import signal
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from picar_interfaces.srv import Look, Move
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Range
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from picar_base.backend import BackendError, load_backend
from picar_base.safety import SafetyGate

MAX_RANGE_M = 4.0   # what a move reports when nothing was in range


def report_range(metres):
    """A reading for Move's min_range_m: 0 when too close to measure, at most MAX_RANGE_M."""
    return float(max(0.0, min(metres, MAX_RANGE_M)))


def wall_limit_s(duration_s):
    """How long a move may take by the wall clock, in case the twin's clock stands still
    (Gazebo paused or gone)."""
    return 2 * duration_s + 1.0


def clamp(value, low, high):
    return max(low, min(high, value))


class PicarBase(Node):

    def __init__(self):
        super().__init__('picar_base')
        param = self.declare_parameter
        backend_name = param('backend', 'sim').value
        self.wheelbase = param('wheelbase_m', 0.145).value
        self.full_speed = param('full_speed_mps', 0.5).value   # speed at full motor power
        self.max_steering = param('max_steering_deg', 30.0).value
        self.max_move_s = param('max_move_s', 3.0).value
        self.max_pan = param('max_pan_deg', 90.0).value
        self.max_tilt = param('max_tilt_deg', 40.0).value
        range_topics = list(param('range_topics', ['/ultrasonic/range']).value)
        # The clock for every timing rule: Gazebo's in the twin, else the steady system clock
        self.sim_time = self.get_parameter('use_sim_time').value
        self.now = self.ros_now if self.sim_time else time.monotonic
        self.gate = SafetyGate(
            max_speed=param('max_speed', 0.4).value,
            cmd_timeout_s=param('cmd_timeout_s', 0.5).value,
            obstacle_stop_m=param('obstacle_stop_m', 0.25).value,
            range_timeout_s=param('range_timeout_s', 0.3).value,
            sensors=range_topics,
            clock=self.now)
        self.backend = load_backend(backend_name, self)

        self.teleop_power = 0.0
        self.teleop_steering = 0.0
        self.status = 'timeout'
        self.move_lock = threading.Lock()   # held while a move runs: one move at a time
        self.cancel = threading.Event()
        self.output_lock = threading.Lock()   # around the outputs and the odometry
        self.closing = False
        # The odometry: where the car should be (x, y in metres, heading in radians), the
        # speed and turn rate being sent, and when that was last worked out
        self.pose = [0.0, 0.0, 0.0]
        self.sent = (0.0, 0.0)
        self.sent_since = None

        self.status_pub = self.create_publisher(String, '/picar/status', 10)
        self.odom_pub = self.create_publisher(Odometry, '/picar/odom', 10)

        # A move blocks for up to max_move_s, so it gets its own group and the
        # range updates, stop/estop and the 20 Hz loop keep running beside it.
        loop_group = MutuallyExclusiveCallbackGroup()
        move_group = ReentrantCallbackGroup()
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10,
                                 callback_group=loop_group)
        for topic in range_topics:
            self.create_subscription(Range, topic, lambda msg, topic=topic: self.on_range(msg, topic),
                                     qos_profile_sensor_data, callback_group=loop_group)
        self.create_service(Move, '/picar/move', self.on_move, callback_group=move_group)
        self.create_service(Look, '/picar/look', self.on_look, callback_group=loop_group)
        self.create_service(Trigger, '/picar/stop', self.on_stop, callback_group=loop_group)
        self.create_service(SetBool, '/picar/estop', self.on_estop, callback_group=loop_group)
        self.create_timer(0.05, self.loop, callback_group=loop_group)              # 20 Hz
        self.create_timer(0.5, self.publish_status, callback_group=loop_group)     # 2 Hz
        self.create_timer(0.1, self.publish_odom, callback_group=loop_group)       # 10 Hz

        self.drive(0.0, 0.0)
        self.backend.look(0.0, 0.0)
        clock = "Gazebo's clock" if self.sim_time else 'the system clock'
        self.get_logger().info(f'picar_base ready (backend: {backend_name}, timed by {clock})')

    def ros_now(self):
        """The ROS clock in seconds: with use_sim_time, Gazebo's (0 until /clock arrives)."""
        return self.get_clock().now().nanoseconds / 1e9

    # ---------- outputs ----------

    def drive(self, power, steering_deg):
        """power is motor power from -1 to 1, already filtered by the gate."""
        with self.output_lock:
            if not self.closing:
                self.update_pose()
                v = power * self.full_speed
                self.sent = (v, v * math.tan(math.radians(steering_deg)) / self.wheelbase)
                self.backend.drive(power, steering_deg)

    def final_stop(self):
        """Send one last stop and block every later drive command."""
        self.cancel.set()
        with self.output_lock:
            self.closing = True
            self.update_pose()
            self.sent = (0.0, 0.0)
            self.backend.drive(0.0, 0.0)

    def update_pose(self):
        """Move the odometry on by what was being sent since the last update (output_lock held)."""
        now = self.now()
        if self.sent_since is not None:
            dt = max(0.0, now - self.sent_since)
            v, w = self.sent
            x, y, heading = self.pose
            heading += w * dt
            self.pose = [x + v * math.cos(heading) * dt, y + v * math.sin(heading) * dt, heading]
        self.sent_since = now

    def publish_odom(self):
        with self.output_lock:
            self.update_pose()
            (x, y, heading), (v, w) = self.pose, self.sent
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(heading / 2)   # the heading as a quaternion
        odom.pose.pose.orientation.w = math.cos(heading / 2)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

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

    def on_range(self, msg, topic='/ultrasonic/range'):
        if not math.isnan(msg.range):   # NaN: no echo, so the sensor counts as silent
            self.gate.on_range(msg.range, topic)

    def loop(self):
        if self.move_lock.locked():
            return
        power, why = self.gate.filter(self.teleop_power)
        self.status = why
        self.drive(power, self.teleop_steering if power != 0.0 else 0.0)

    def publish_status(self):
        self.status_pub.publish(String(data=self.status))

    # ---------- services ----------

    def on_move(self, req, res):
        if not self.move_lock.acquire(blocking=False):
            res.completed, res.reason = False, 'busy'
            res.min_range_m = report_range(self.gate.range_m)
            return res
        reason, closest = 'done', MAX_RANGE_M
        try:
            self.cancel.clear()
            self.teleop_power = 0.0
            speed = clamp(req.speed, -1.0, 1.0)
            steering = self.clamp_steering(req.steering_deg)
            duration = clamp(req.duration_s, 0.0, self.max_move_s)
            end = self.now() + duration
            wall_end = time.monotonic() + wall_limit_s(duration)
            while self.now() < end:
                if self.cancel.is_set():
                    reason = 'estop' if self.gate.estop else 'stopped'
                    break
                if time.monotonic() > wall_end:   # the twin's clock stands still
                    reason = 'timeout'
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
            self.move_lock.release()
        res.completed = reason == 'done'
        res.reason = reason
        res.min_range_m = report_range(closest)
        return res

    def on_look(self, req, res):
        pan = clamp(req.pan_deg, -self.max_pan, self.max_pan)
        tilt = clamp(req.tilt_deg, -self.max_tilt, self.max_tilt)
        self.backend.look(pan, tilt)
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
    try:
        node = PicarBase()
    except BackendError as error:
        print(f'picar_base cannot start: {error}', file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(1)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        while rclpy.ok() and not stop.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        node.final_stop()
        time.sleep(0.2)          # let the stop command go out
        node.backend.close()
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
