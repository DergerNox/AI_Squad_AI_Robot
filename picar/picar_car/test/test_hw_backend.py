"""The hw backend with a fake GPIO and a fake I2C bus: no Raspberry Pi needed.

The fakes record what the backend sets: pin levels, PWM duty and the PCA9685's registers,
from which the servo values are read back. Needs ROS and the workspace sourced (picar_base):

    source ~/picar_ws/install/setup.bash
    cd picar_car && python3 -m pytest test
"""

import math
import pathlib
import sys
from collections import defaultdict
from types import SimpleNamespace

import pytest

from picar_base.backend import BackendError, load_backend
from picar_car import hw_backend
from picar_car.hw_backend import HwBackend

CAR_YAML = pathlib.Path(__file__).resolve().parents[1] / 'config' / 'car.yaml'
A_ENABLE, A_FORWARD, A_BACKWARD = 4, 14, 15
B_ENABLE, B_FORWARD, B_BACKWARD = 17, 27, 18
STEERING, PAN, TILT = 2, 6, 10


class FakePWM:

    def __init__(self, pin, hz):
        self.pin, self.hz = pin, hz
        self.duty = None
        self.running = False

    def start(self, duty):
        self.running, self.duty = True, duty

    def ChangeDutyCycle(self, duty):
        assert self.running, 'duty changed on a stopped PWM'
        self.duty = duty

    def stop(self):
        self.running = False


class FakeGPIO:
    BCM, OUT, LOW, HIGH = 'BCM', 'OUT', 0, 1

    def __init__(self):
        self.mode = None
        self.levels = {}          # pin -> 0 or 1, for pins set up as outputs
        self.pwms = {}            # pin -> FakePWM
        self.cleaned_up = False

    def setwarnings(self, flag):
        pass

    def setmode(self, mode):
        self.mode = mode

    def setup(self, pin, direction, initial=0):
        assert direction == self.OUT
        self.levels[pin] = initial

    def output(self, pin, level):
        assert pin in self.levels, f'GPIO {pin} written before setup'
        self.levels[pin] = level

    def PWM(self, pin, hz):
        self.pwms[pin] = FakePWM(pin, hz)
        return self.pwms[pin]

    def cleanup(self, *pins):
        self.cleaned_up = True


class FakeBus:
    """The PCA9685 at 0x40: remembers every register."""

    def __init__(self, fail=False):
        self.registers = defaultdict(int)
        self.writes = []
        self.fail = fail

    def write_byte_data(self, address, register, value):
        if self.fail:
            raise OSError(121, 'Remote I/O error')
        assert address == 0x40 and 0 <= value <= 255
        self.registers[register] = value
        self.writes.append((register, value))

    def read_byte_data(self, address, register):
        return self.registers[register]

    def servo(self, channel):
        """The channel's off step: the servo value last written."""
        base = 0x06 + 4 * channel
        assert (self.registers[base], self.registers[base + 1]) == (0, 0)   # on at step 0
        return self.registers[base + 2] | self.registers[base + 3] << 8


class FakeNode:
    """Stands in for picar_base: its parameters and its logger."""

    def __init__(self, **params):
        self.params = {'max_steering_deg': 30.0, **params}
        self.warnings = []

    def declare_parameter(self, name, default):
        return SimpleNamespace(value=self.params.get(name, default))

    def get_parameter(self, name):
        return SimpleNamespace(value=self.params[name])

    def get_logger(self):
        return SimpleNamespace(warning=self.warnings.append, info=print, error=print)


class FakeTime:

    def __init__(self):
        self.now = 1000.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Car:
    """The backend on fake hardware, and what the hardware does."""

    def __init__(self, monkeypatch, bus=None, **params):
        self.gpio, self.bus, self.time = FakeGPIO(), bus or FakeBus(), FakeTime()
        monkeypatch.setattr(hw_backend, 'open_gpio', lambda: self.gpio)
        monkeypatch.setattr(hw_backend, 'open_i2c', lambda number: self.bus)
        monkeypatch.setattr(hw_backend, 'time', self.time)
        self.node = FakeNode(**params)
        self.backend = HwBackend(self.node)

    def duties(self):
        return self.gpio.pwms[A_ENABLE].duty, self.gpio.pwms[B_ENABLE].duty

    def directions(self):
        """The direction pins: (A forward, A backward, B forward, B backward)."""
        pins = (A_FORWARD, A_BACKWARD, B_FORWARD, B_BACKWARD)
        return tuple(self.gpio.levels[pin] for pin in pins)

    def head(self):
        return self.bus.servo(PAN), self.bus.servo(TILT)


@pytest.fixture
def make_car(monkeypatch):
    made = []

    def make(**kwargs):
        made.append(Car(monkeypatch, **kwargs))
        return made[-1]

    yield make
    for car in made:
        car.backend.closed.set()     # ends the watchdog thread


@pytest.fixture
def car(make_car):
    return make_car()


# ---------- start ----------

def test_the_servo_board_runs_at_50_hz(car):
    writes = car.bus.writes
    at = writes.index((0xFE, 121))                        # 25 MHz / 4096 / 50 Hz - 1
    mode1 = [value for register, value in writes if register == 0x00]
    before = [value for register, value in writes[:at] if register == 0x00]
    assert before[-1] & 0x10                              # asleep while the prescaler changes
    after = mode1[len(before):]
    assert after[0] & 0x10 == 0 and after[-1] & 0x80      # then awake and restarted
    assert car.bus.registers[0x00] & 0x10 == 0


def test_starts_stopped_steering_and_camera_straight(car):
    assert car.gpio.mode == 'BCM'
    assert car.duties() == (0, 0)
    assert car.directions() == (0, 0, 0, 0)
    assert car.gpio.levels[A_ENABLE] == car.gpio.levels[B_ENABLE] == 0
    assert (car.gpio.pwms[A_ENABLE].hz, car.gpio.pwms[B_ENABLE].hz) == (1000, 1000)
    assert car.bus.servo(STEERING) == 300
    assert car.head() == (311, 340)


def test_hw_names_this_backend():
    node = FakeNode()
    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(hw_backend, 'open_gpio', FakeGPIO)
        patch.setattr(hw_backend, 'open_i2c', lambda number: FakeBus())
        backend = load_backend('hw', node)
    try:
        assert isinstance(backend, HwBackend)
    finally:
        backend.closed.set()


# ---------- motors ----------

def test_forward_drives_both_motors_forward(car):
    car.backend.drive(0.4, 0.0)               # picar_base's top speed
    assert car.directions() == (1, 0, 1, 0)
    assert car.duties() == (pytest.approx(76.0), pytest.approx(76.0))   # 60 + 0.4 x 40


def test_backward_drives_both_motors_backward(car):
    car.backend.drive(-0.4, 0.0)
    assert car.directions() == (0, 1, 0, 1)
    assert car.duties() == (pytest.approx(76.0), pytest.approx(76.0))


@pytest.mark.parametrize('power, duty', [(0.1, 64.0), (0.5, 80.0), (1.0, 100.0), (3.0, 100.0)])
def test_any_power_is_above_the_stall_duty(car, power, duty):
    car.backend.drive(power, 0.0)
    assert car.duties() == (pytest.approx(duty), pytest.approx(duty))


def test_no_power_stops_with_the_direction_pins_low(car):
    car.backend.drive(0.4, 0.0)
    for power in (0.0, 0.0001):
        car.backend.drive(power, 0.0)
        assert car.duties() == (0, 0)
        assert car.directions() == (0, 0, 0, 0)


def test_the_stall_duty_is_a_setting(make_car):
    car = make_car(**{'hw.min_duty_percent': 50.0})
    car.backend.drive(0.5, 0.0)
    assert car.duties() == (pytest.approx(75.0), pytest.approx(75.0))


# ---------- steering ----------

@pytest.mark.parametrize('degrees, value', [
    (0.0, 300), (30.0, 242), (-30.0, 350), (15.0, 271), (-15.0, 325), (45.0, 242), (-90.0, 350)])
def test_steering_left_is_lower_with_its_own_full_lock(car, degrees, value):
    car.backend.drive(0.0, degrees)
    assert car.bus.servo(STEERING) == value


def test_reversing_keeps_the_wheels_where_they_point(car):
    car.backend.drive(0.4, 20.0)
    forward = car.bus.servo(STEERING)
    car.backend.drive(-0.4, 20.0)
    assert car.bus.servo(STEERING) == forward < 300   # still left: no teleop-style swap


def test_full_lock_follows_picar_bases_steering_limit(make_car):
    car = make_car(max_steering_deg=20.0)
    car.backend.drive(0.0, 20.0)
    assert car.bus.servo(STEERING) == 242


def test_a_servo_is_written_only_when_its_value_changes(car):
    car.backend.drive(0.0, 10.0)
    count = len(car.bus.writes)
    for _ in range(20):                       # picar_base sends 20 commands a second
        car.backend.drive(0.0, 10.0)
    assert len(car.bus.writes) == count


# ---------- camera head ----------

@pytest.mark.parametrize('pan, tilt, values', [
    (0.0, 0.0, (311, 340)),
    (20.0, 0.0, (267, 340)),                  # left is lower: 311 - 20 x 2.2
    (-20.0, 10.0, (355, 318)),                # right is higher, up is lower
    (90.0, 90.0, (180, 180)),                 # stops at the safe limits
    (-90.0, -90.0, (420, 430)),
])
def test_the_head_turns_by_degrees_within_safe_limits(car, pan, tilt, values):
    car.backend.look(pan, tilt)
    assert car.head() == values


def test_the_head_numbers_are_settings(make_car):
    car = make_car(**{'hw.pan_centre': 300, 'hw.pan_per_deg': -2.0, 'hw.tilt_limits': [300, 350]})
    car.backend.look(10.0, 40.0)
    assert car.head() == (320, 300)           # a negative step turns the other way


# ---------- watchdog ----------

def test_the_watchdog_stops_motors_that_hear_nothing(car):
    car.backend.drive(0.4, 0.0)
    car.time.sleep(0.4)
    car.backend.check_watchdog()
    assert car.duties() == (pytest.approx(76.0), pytest.approx(76.0))   # 0.4 s: still fine
    car.time.sleep(0.2)
    car.backend.check_watchdog()
    assert car.duties() == (0, 0)
    assert car.directions() == (0, 0, 0, 0)
    assert car.node.warnings == ['hw: no drive command for 0.6 s, motors stopped']


def test_the_watchdog_leaves_a_car_that_keeps_getting_commands(car):
    for _ in range(40):                       # 2 s at 20 commands a second
        car.backend.drive(0.4, 0.0)
        car.time.sleep(0.05)
        car.backend.check_watchdog()
    assert car.duties() == (pytest.approx(76.0), pytest.approx(76.0))
    assert car.node.warnings == []


def test_the_watchdog_is_quiet_while_the_car_stands(car):
    car.time.sleep(10.0)
    car.backend.check_watchdog()
    assert car.node.warnings == []


# ---------- close ----------

def test_close_stops_and_leaves_the_motor_pins_driven_low(car):
    car.backend.drive(0.4, 25.0)
    car.backend.close()
    assert car.duties() == (0, 0)
    assert not car.gpio.pwms[A_ENABLE].running and not car.gpio.pwms[B_ENABLE].running
    assert all(level == 0 for level in car.gpio.levels.values())
    assert not car.gpio.cleaned_up            # cleaned-up pins would float
    assert car.bus.servo(STEERING) == 300
    assert car.backend.closed.is_set()


# ---------- what goes wrong ----------

def test_without_rpi_gpio_it_says_it_runs_only_on_the_pi(monkeypatch):
    monkeypatch.setitem(sys.modules, 'RPi', None)
    monkeypatch.setitem(sys.modules, 'RPi.GPIO', None)
    with pytest.raises(BackendError, match="python3-rpi.gpio.*only runs on the car's Raspberry Pi"):
        hw_backend.open_gpio()


def test_without_smbus_it_says_what_to_install(monkeypatch):
    monkeypatch.setitem(sys.modules, 'smbus', None)
    with pytest.raises(BackendError, match='sudo apt install python3-smbus'):
        hw_backend.open_i2c(1)


def test_a_silent_servo_board_says_how_to_check_it(monkeypatch):
    with pytest.raises(BackendError, match=r'PCA9685\) at I2C address 0x40.*i2cdetect -y 1'):
        Car(monkeypatch, bus=FakeBus(fail=True))


# ---------- the real picar_base node with car.yaml ----------

def test_picar_base_drives_through_it_with_car_yaml(monkeypatch):
    # car.yaml's hw settings reach the backend with the types it declares (ROS refuses a
    # whole number where a decimal was declared), and teleop comes out at the motors
    import rclpy
    from geometry_msgs.msg import Twist, Vector3
    from picar_base.base_node import PicarBase
    from sensor_msgs.msg import Range

    gpio, bus = FakeGPIO(), FakeBus()
    monkeypatch.setattr(hw_backend, 'open_gpio', lambda: gpio)
    monkeypatch.setattr(hw_backend, 'open_i2c', lambda number: bus)
    rclpy.init(args=['--ros-args', '--params-file', str(CAR_YAML)])
    node = None
    try:
        node = PicarBase()
        backend = node.backend
        assert isinstance(backend, HwBackend)
        assert backend.steering_left == 242
        assert (backend.pan_limits, backend.watchdog_s) == ([180, 420], 0.5)
        node.on_range(Range(range=2.0))
        node.on_cmd_vel(Twist(linear=Vector3(x=0.1), angular=Vector3(z=0.3)))
        node.loop()
        assert gpio.pwms[A_ENABLE].duty == pytest.approx(68.0)     # 0.2 of full power
        steering = math.degrees(math.atan(0.145 * 0.3 / 0.1))
        assert bus.servo(STEERING) == round(300 - 58 * steering / 30)
    finally:
        if node is not None:
            node.backend.close()
            node.destroy_node()
        rclpy.try_shutdown()
