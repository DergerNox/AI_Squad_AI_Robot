"""picar_base's node, without a ROS graph: teleop, moves, the camera head, stop and estop.

The node's publishers, subscriptions, services and timers are recorded instead of being
created, so the tests call the callbacks themselves and read what the node published.
A fake clock makes the waits instant. Needs ROS and the workspace sourced:

    source ~/picar_ws/install/setup.bash
    cd ~/picar_ws/src/picar/shared/picar_base && python3 -m pytest test
"""

import math
from collections import defaultdict
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import Twist
from picar_interfaces.srv import Look, Move
from std_srvs.srv import SetBool, Trigger

from rosgraph_msgs.msg import Clock

from picar_base import base_node, safety
from picar_base.backend import NAMED_BACKENDS, Backend, BackendError
from picar_base.base_node import PicarBase

WHEELBASE_M = 0.145
FULL_SPEED_MPS = 0.5
TOP_SPEED_MPS = FULL_SPEED_MPS * 0.4   # max_speed 0.4 of full motor power


class FakeTime:
    """Stands in for the time module: sleep() moves the clock on at once, and calls
    on_tick, so a test can deliver sensor readings while a move runs."""

    def __init__(self):
        self.now = 100.0
        self.on_tick = None

    def monotonic(self):
        return self.now

    def time(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds
        if self.now > 100.0 + 3600:   # a loop that never ends: fail instead of hanging
            raise AssertionError("the fake clock ran for an hour: a loop never ends")
        if self.on_tick:
            self.on_tick(self.now)


class Base:
    """A picar_base node with its ROS entities recorded."""

    def __init__(self, monkeypatch, params):
        self.clock = FakeTime()
        monkeypatch.setattr(base_node, "time", self.clock)
        monkeypatch.setattr(safety, "time", self.clock)
        self.published = defaultdict(list)    # topic -> messages
        self.subscriptions = {}               # topic -> callback
        self.services = {}                    # name -> callback
        self.timers = {}                      # period -> callback
        published, subscriptions, services, timers = (
            self.published, self.subscriptions, self.services, self.timers)

        def create_publisher(node, msg_type, topic, qos, **kwargs):
            return SimpleNamespace(publish=published[topic].append)

        def create_subscription(node, msg_type, topic, callback, qos, **kwargs):
            subscriptions[topic] = callback

        def create_service(node, srv_type, name, callback, **kwargs):
            services[name] = callback

        def create_timer(node, period, callback, **kwargs):
            timers[period] = callback

        for name, fake in [("create_publisher", create_publisher),
                           ("create_subscription", create_subscription),
                           ("create_service", create_service), ("create_timer", create_timer)]:
            monkeypatch.setattr(PicarBase, name, fake)
        args = ["--ros-args"]
        for name, value in params.items():
            args += ["-p", f"{name}:={value}"]
        rclpy.init(args=args)
        self.node = PicarBase()

    # ---------- the outside world ----------

    def range(self, metres, topic="/ultrasonic/range"):
        self.subscriptions[topic](SimpleNamespace(range=metres))

    def cmd_vel(self, v, w=0.0):
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        self.subscriptions["/cmd_vel"](msg)

    def loop(self):
        self.timers[0.05]()

    def teleop(self, seconds, v, w=0.0, range_m=2.0):
        """Teleop keeps sending (v, w) for `seconds`, the sensor reads range_m."""
        for _ in range(round(seconds / 0.05)):
            self.clock.now += 0.05
            if range_m is not None:
                self.range(range_m)
            self.cmd_vel(v, w)
            self.loop()

    def sensor(self, range_m, then=None):
        """From now on (also while a move runs) the distance sensor reads range_m(now) at
        every step; None: no reading. then(now) runs after each reading."""
        def tick(now):
            value = range_m(now)
            if value is not None:
                self.range(value)
            if then:
                then(now)
        tick(self.clock.now)   # a reading has already arrived
        self.clock.on_tick = tick

    def move(self, speed, steering_deg=0.0, duration_s=1.0):
        return self.services["/picar/move"](
            Move.Request(speed=speed, steering_deg=steering_deg, duration_s=duration_s),
            Move.Response())

    def drives(self):
        """Every drive command sent to the twin, as (linear.x, angular.z)."""
        return [(m.linear.x, m.angular.z) for m in self.published["/sim/cmd_vel"]]


@pytest.fixture
def make_base(monkeypatch):
    made = []

    def make(**params):
        made.append(Base(monkeypatch, params))
        return made[-1]

    yield make
    for base in made:
        base.node.destroy_node()
    rclpy.try_shutdown()


@pytest.fixture
def base(make_base):
    return make_base()


def yaw_rate(v, steering_deg):
    return v * math.tan(math.radians(steering_deg)) / WHEELBASE_M


# ---------- start ----------

def test_starts_stopped_with_the_camera_head_straight(base):
    assert base.drives() == [(0.0, 0.0)]
    assert [m.data for m in base.published["/sim/head_pan"]] == [0.0]
    assert [m.data for m in base.published["/sim/head_tilt"]] == [0.0]


def test_hw_is_the_real_picars_backend():
    # not loaded here: on the car it would open the real motors (picar_car tests it on fakes)
    assert NAMED_BACKENDS["hw"] == "picar_car.hw_backend:HwBackend"


# ---------- teleop (/cmd_vel) ----------

def test_teleop_is_limited_to_the_top_speed(base):
    base.teleop(0.2, v=0.5)
    assert base.drives()[-1] == pytest.approx((TOP_SPEED_MPS, 0.0))


def test_teleop_steering_is_limited_to_30_degrees(base):
    base.teleop(0.2, v=0.1, w=0.5)    # would need 36 degrees of steering
    assert base.drives()[-1] == pytest.approx((0.1, yaw_rate(0.1, 30.0)))


def test_teleop_without_a_distance_reading_only_reverses(base):
    base.teleop(0.5, v=0.1, range_m=None)
    assert base.drives()[-1] == (0.0, 0.0)
    base.timers[0.5]()
    assert base.published["/picar/status"][-1].data == "no_range"
    base.teleop(0.2, v=-0.1, range_m=None)
    assert base.drives()[-1] == pytest.approx((-0.1, 0.0))


def test_teleop_stops_half_a_second_after_the_last_command(base):
    base.teleop(0.2, v=0.1)
    assert base.drives()[-1][0] == pytest.approx(0.1)
    for _ in range(11):               # 0.55 s with sensor readings but no commands
        base.clock.now += 0.05
        base.range(2.0)
        base.loop()
    assert base.drives()[-1] == (0.0, 0.0)
    base.timers[0.5]()
    assert base.published["/picar/status"][-1].data == "timeout"


def test_teleop_stops_in_front_of_an_obstacle(base):
    base.teleop(0.2, v=0.1, range_m=0.2)
    assert base.drives()[-1] == (0.0, 0.0)
    base.timers[0.5]()
    assert base.published["/picar/status"][-1].data == "obstacle"


# ---------- /picar/move ----------

def test_a_move_drives_for_its_time_then_stops(base):
    base.sensor(lambda now: 2.0)
    start = base.clock.now
    res = base.move(1.0, steering_deg=10.0, duration_s=1.0)
    assert (res.completed, res.reason) == (True, "done")
    assert res.min_range_m == pytest.approx(2.0)
    assert base.clock.now - start == pytest.approx(1.0, abs=0.06)   # to one 50 ms step
    during = base.drives()[1:-1]
    assert during and all(d == pytest.approx((TOP_SPEED_MPS, yaw_rate(TOP_SPEED_MPS, 10.0)))
                          for d in during)
    assert base.drives()[-1] == (0.0, 0.0)


def test_a_half_speed_move_drives_at_half_the_top_speed(base):
    base.sensor(lambda now: 2.0)
    base.move(0.5, duration_s=0.2)
    assert base.drives()[1] == pytest.approx((TOP_SPEED_MPS / 2, 0.0))


def test_a_move_reports_4_m_when_nothing_is_in_range(base):
    base.sensor(lambda now: math.inf)
    assert base.move(1.0).min_range_m == pytest.approx(4.0)


def test_a_move_is_limited_to_3_seconds(base):
    base.sensor(lambda now: 2.0)
    start = base.clock.now
    base.move(1.0, duration_s=10.0)
    assert base.clock.now - start == pytest.approx(3.0, abs=0.06)


def test_a_move_stops_for_an_obstacle(base):
    start = base.clock.now
    base.sensor(lambda now: 2.0 if now - start < 0.5 else 0.2)
    res = base.move(1.0, duration_s=2.0)
    assert (res.completed, res.reason) == (False, "obstacle")
    assert res.min_range_m == pytest.approx(0.2)
    assert base.clock.now - start == pytest.approx(0.55, abs=0.06)
    assert base.drives()[-1] == (0.0, 0.0)


def test_a_move_reverses_away_from_an_obstacle(base):
    base.sensor(lambda now: 0.1)
    base.range(0.1)
    res = base.move(-1.0, duration_s=0.5)
    assert (res.completed, res.reason) == (True, "done")
    assert base.drives()[1] == pytest.approx((-TOP_SPEED_MPS, 0.0))


def test_a_move_without_a_distance_reading_does_not_drive_forward(base):
    res = base.move(1.0)
    assert (res.completed, res.reason) == (False, "no_range")
    assert base.drives()[-1] == (0.0, 0.0)


def test_a_second_move_during_a_move_is_busy(base):
    answers = []

    def second_move(now):
        if not answers and now > 100.1:
            answers.append(base.move(1.0))
    base.sensor(lambda now: 2.0, then=second_move)
    res = base.move(1.0)
    assert (answers[0].completed, answers[0].reason) == (False, "busy")
    assert (res.completed, res.reason) == (True, "done")


def test_stop_ends_a_move(base):
    def stop(now):
        if now > 100.3:
            base.services["/picar/stop"](Trigger.Request(), Trigger.Response())
    base.sensor(lambda now: 2.0, then=stop)
    res = base.move(1.0)
    assert (res.completed, res.reason) == (False, "stopped")
    assert base.drives()[-1] == (0.0, 0.0)


def test_estop_ends_a_move_and_holds_until_cleared(base):
    def estop(now):
        if now > 100.3 and not base.node.gate.estop:
            base.services["/picar/estop"](SetBool.Request(data=True), SetBool.Response())
    base.sensor(lambda now: 2.0, then=estop)
    assert base.move(1.0).reason == "estop"
    base.sensor(lambda now: 2.0)
    assert base.move(1.0).reason == "estop"
    res = base.services["/picar/estop"](SetBool.Request(data=False), SetBool.Response())
    assert res.message == "estop cleared"
    assert base.move(1.0).reason == "done"


def test_a_move_ends_teleop(base):
    base.teleop(0.2, v=0.1)
    base.sensor(lambda now: 2.0)
    base.move(1.0, duration_s=0.2)
    base.range(2.0)
    base.loop()                       # teleop's last command is not resumed
    assert base.drives()[-1] == (0.0, 0.0)


# ---------- /picar/look ----------

def test_look_is_limited_and_sent_in_radians(base):
    res = base.services["/picar/look"](Look.Request(pan_deg=120.0, tilt_deg=-60.0), Look.Response())
    assert res.success and res.message == "pan 90 deg, tilt -40 deg"
    assert base.published["/sim/head_pan"][-1].data == pytest.approx(math.pi / 2)
    assert base.published["/sim/head_tilt"][-1].data == pytest.approx(math.radians(-40))


# ---------- shutting down ----------

def test_the_final_stop_blocks_every_later_drive(base):
    base.teleop(0.2, v=0.1)
    base.node.final_stop()
    assert base.drives()[-1] == (0.0, 0.0)
    count = len(base.drives())
    base.teleop(0.2, v=0.1)
    assert len(base.drives()) == count


def test_a_reading_too_close_to_measure_is_reported_as_0_m(base):
    base.sensor(lambda now: -math.inf)
    res = base.move(-1.0, duration_s=0.2)    # reversing is allowed
    assert res.reason == "done" and res.min_range_m == 0.0


# ---------- backends ----------

class RecordingBackend(Backend):
    """A backend that writes down what picar_base asks of it."""

    def __init__(self, node):
        super().__init__(node)
        self.calls = []

    def drive(self, power, steering_deg):
        self.calls.append(("drive", power, steering_deg))

    def look(self, pan_deg, tilt_deg):
        self.calls.append(("look", pan_deg, tilt_deg))


def test_a_backend_gets_motor_power_steering_and_head_angles_in_degrees(make_base):
    base = make_base(backend="test_base_node:RecordingBackend")
    calls = base.node.backend.calls
    assert calls == [("drive", 0.0, 0.0), ("look", 0.0, 0.0)]
    base.teleop(0.1, v=0.1, w=0.5)            # 0.2 of full power, 36 degrees -> 30
    assert calls[-1] == ("drive", pytest.approx(0.2), pytest.approx(30.0))
    base.services["/picar/look"](Look.Request(pan_deg=45.0, tilt_deg=-10.0), Look.Response())
    assert calls[-1] == ("look", 45.0, -10.0)


def test_sim_is_the_twins_backend(make_base):
    base = make_base(backend="picar_base.sim_backend:SimBackend")
    assert base.drives() == [(0.0, 0.0)]


@pytest.mark.parametrize("name, message", [
    ("esp32", "backend 'esp32': pccar_car.esp32_backend is not there"),
    ("no_such_car.backend:Backend", "no_such_car.backend is not there"),
    ("warp", "use one of sim, hw, esp32 or package.module:ClassName"),
    ("picar_base.safety:SafetyGate", "picar_base.safety has no Backend class SafetyGate"),
])
def test_a_backend_that_cannot_be_loaded_says_why(make_base, name, message):
    with pytest.raises(BackendError, match=message):
        make_base(backend=name)


# ---------- several distance sensors ----------

LEFT = "/ultrasonic/left"


def test_every_sensor_in_range_topics_guards_the_way_ahead(make_base):
    base = make_base(range_topics="[/ultrasonic/range, /ultrasonic/left]")
    assert "/ultrasonic/range" in base.subscriptions and LEFT in base.subscriptions

    def teleop_forward(front, left):
        for _ in range(8):
            base.clock.now += 0.05
            base.range(front)
            if left is not None:
                base.range(left, LEFT)
            base.cmd_vel(0.1)
            base.loop()
        base.timers[0.5]()
        return base.drives()[-1], base.published["/picar/status"][-1].data

    drive, status = teleop_forward(2.0, 2.0)
    assert drive == pytest.approx((0.1, 0.0)) and status == "ok"
    assert teleop_forward(2.0, 0.2) == ((0.0, 0.0), "obstacle")
    assert teleop_forward(2.0, None) == ((0.0, 0.0), "no_range")   # the left one went quiet


def test_a_move_reports_the_closest_of_all_sensors(make_base):
    base = make_base(range_topics="[/ultrasonic/range, /ultrasonic/left]")
    base.sensor(lambda now: 2.0, then=lambda now: base.range(1.0, LEFT))
    res = base.move(1.0, duration_s=0.5)
    assert res.reason == "done" and res.min_range_m == pytest.approx(1.0)


# ---------- time: Gazebo's clock in the twin ----------

def set_sim_clock(base, seconds):
    msg = Clock()
    msg.clock.sec, msg.clock.nanosec = int(seconds), int(round(seconds % 1 * 1e9))
    base.subscriptions["/clock"](msg)


@pytest.fixture
def twin(make_base):
    return make_base(use_sim_time="true")


def test_the_real_car_does_not_listen_to_a_sim_clock(base):
    assert "/clock" not in base.subscriptions


def test_the_twin_times_a_move_by_gazebos_clock(twin):
    set_sim_clock(twin, 10.0)
    start = twin.clock.now
    # Gazebo runs at half speed: 0.025 s of its time for every 0.05 s of wall time
    twin.sensor(lambda now: 2.0, then=lambda now: set_sim_clock(twin, 10.0 + (now - start) / 2))
    res = twin.move(1.0, duration_s=1.0)
    assert (res.completed, res.reason) == (True, "done")
    assert twin.clock.now - start == pytest.approx(2.0, abs=0.06)   # 1 s of Gazebo's time


def test_a_paused_twin_gives_up_on_a_move(twin):
    set_sim_clock(twin, 10.0)
    twin.sensor(lambda now: 2.0)              # Gazebo's clock stands still at 10 s
    start = twin.clock.now
    res = twin.move(1.0, duration_s=1.0)
    assert (res.completed, res.reason) == (False, "timeout")
    assert twin.clock.now - start == pytest.approx(3.0, abs=0.06)   # 2 x 1 s + 1 s of wall time
    assert twin.drives()[-1] == (0.0, 0.0)


def test_before_gazebos_clock_starts_nothing_counts_as_fresh(twin):
    twin.loop()                               # the ROS clock reads 0: no command ever came
    twin.timers[0.5]()
    assert twin.drives()[-1] == (0.0, 0.0)
    assert twin.published["/picar/status"][-1].data == "timeout"


# ---------- /picar/odom ----------

def odom(base):
    """Publish the odometry now; returns x, y and the heading."""
    base.timers[0.1]()
    msg = base.published["/picar/odom"][-1]
    heading = 2 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    return msg.pose.pose.position.x, msg.pose.pose.position.y, heading


def test_odometry_follows_what_was_really_sent(base):
    assert odom(base) == (0.0, 0.0, 0.0)
    base.teleop(1.0, v=0.5)                   # the top speed, 0.2 m/s, for 1 s
    base.clock.now += 0.05
    x, y, heading = odom(base)
    assert x == pytest.approx(0.2, abs=0.011) and y == pytest.approx(0.0) and heading == 0.0
    msg = base.published["/picar/odom"][-1]
    assert (msg.header.frame_id, msg.child_frame_id) == ("odom", "base_link")
    assert msg.twist.twist.linear.x == pytest.approx(0.2)


def test_odometry_stays_put_while_the_safety_stop_holds_the_car(base):
    base.teleop(1.0, v=0.5, range_m=0.2)      # commands keep coming, the gate stops them
    assert odom(base) == (0.0, 0.0, 0.0)


def test_odometry_turns_as_the_steering_does(base):
    base.teleop(1.0, v=0.1, w=0.5)            # 0.1 m/s, steering limited to 30 degrees
    base.clock.now += 0.05
    x, y, heading = odom(base)
    assert heading == pytest.approx(yaw_rate(0.1, 30.0) * 1.0, abs=0.03)
    assert y > 0                              # it went left


def test_odometry_of_a_move(base):
    base.sensor(lambda now: 2.0)
    base.move(1.0, duration_s=1.0)
    x, y, heading = odom(base)
    assert x == pytest.approx(0.2, abs=0.011) and heading == 0.0
