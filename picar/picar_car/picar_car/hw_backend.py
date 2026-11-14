"""hw_backend.py: picar_base's backend for the real PiCar-B (Adeept's motor and servo board).

    rear motors   two DC motors on the Pi's own pins, always driven together:
                  A: enable GPIO 4, forward 14, backward 15; B: enable 17, forward 27,
                  backward 18 (BCM numbers); RPi.GPIO software PWM at 1000 Hz on the enables
    steering      servo on channel 2 of the PCA9685 (I2C bus 1, address 0x40, 50 Hz)
    camera head   pan servo on channel 6, tilt servo on channel 10

The numbers come from the team's picar_control/motor_steering_node.py, which drives this car;
every one of them is a parameter in config/car.yaml (picar_base's section, under hw:).
A servo value is the PCA9685's pulse length in 1/4096 of its 20 ms cycle (300 = about 1.5 ms):
lower values turn the steering and the camera left, and tilt the camera up.

Speed: below about 60 % duty the motors do not turn, so power 0..1 becomes 60..100 % duty.
picar_base's SafetyGate already limits the power (max_speed 0.4, so 76 % at most).

Safety: picar_base sends a drive command 20 times a second. If none arrives for
hw.watchdog_s (0.5 s), for example because picar_base hangs, the motors stop here. On exit
the motor pins stay outputs driven low (released pins would float).

Needs on the Pi: python3-rpi.gpio and python3-smbus (Ubuntu packages), I2C on
(dtparam=i2c_arm=on in /boot/firmware/config.txt), and the user in the gpio and i2c groups.
"""

import threading
import time

from picar_base.backend import Backend, BackendError

PWM_HZ_SERVOS = 50          # servos want a pulse every 20 ms


def clamp(value, low, high):
    return max(low, min(high, value))


def open_gpio():
    """RPi.GPIO, or a BackendError that says what is missing. Tests replace this."""
    try:
        import RPi.GPIO as gpio
    except ImportError:
        raise BackendError("backend 'hw': RPi.GPIO is missing (sudo apt install python3-rpi.gpio); "
                           "the hw backend only runs on the car's Raspberry Pi") from None
    except RuntimeError as error:   # RPi.GPIO raises this on a computer that is not a Pi
        raise BackendError(f"backend 'hw': RPi.GPIO cannot run here: {error}") from None
    return gpio


def open_i2c(bus_number):
    """The I2C bus (smbus.SMBus), or a BackendError that says what is wrong. Tests replace this."""
    try:
        import smbus
    except ImportError:
        raise BackendError(
            "backend 'hw': smbus is missing (sudo apt install python3-smbus)") from None
    try:
        return smbus.SMBus(bus_number)
    except (OSError, PermissionError) as error:
        raise BackendError(f"backend 'hw': cannot open I2C bus {bus_number}: {error}. Is I2C on "
                           "(dtparam=i2c_arm=on), and is the user in the i2c group?") from None


class PCA9685:
    """The servo board's PWM chip, written register by register (as the team's node does)."""

    MODE1, MODE2, PRESCALE, LED0_ON_L = 0x00, 0x01, 0xFE, 0x06
    SLEEP, RESTART, ALLCALL, OUTDRV = 0x10, 0x80, 0x01, 0x04

    def __init__(self, bus, address, hz=PWM_HZ_SERVOS, sleep=time.sleep):
        self.bus, self.address = bus, address
        self.write(self.MODE2, self.OUTDRV)               # totem-pole outputs
        self.write(self.MODE1, self.ALLCALL)
        sleep(0.005)                                      # the oscillator starts
        mode1 = bus.read_byte_data(address, self.MODE1) & ~self.SLEEP
        self.write(self.MODE1, mode1)
        sleep(0.005)
        prescale = int(25_000_000 / 4096 / hz - 1 + 0.5)  # 25 MHz clock, 4096 steps a cycle
        self.write(self.MODE1, (mode1 & 0x7F) | self.SLEEP)   # the prescaler changes only asleep
        self.write(self.PRESCALE, prescale)
        self.write(self.MODE1, mode1)
        sleep(0.005)
        self.write(self.MODE1, mode1 | self.RESTART)

    def write(self, register, value):
        self.bus.write_byte_data(self.address, register, value)

    def set(self, channel, value):
        """The pulse of one channel: on at step 0, off at step `value` (0..4095)."""
        base = self.LED0_ON_L + 4 * channel
        for offset, byte in enumerate((0, 0, value & 0xFF, value >> 8)):
            self.write(base + offset, byte)


class Motors:
    """The two rear motors: an enable pin with software PWM and two direction pins each."""

    def __init__(self, gpio, motor_pins, pwm_hz):
        self.gpio = gpio
        gpio.setwarnings(False)
        gpio.setmode(gpio.BCM)
        self.motors = []                       # (pwm, forward pin, backward pin)
        self.pins = []
        for enable, forward, backward in motor_pins:
            for pin in (enable, forward, backward):
                gpio.setup(pin, gpio.OUT, initial=gpio.LOW)
                self.pins.append(pin)
            pwm = gpio.PWM(enable, pwm_hz)
            pwm.start(0)
            self.motors.append((pwm, forward, backward))
        self.state = (0.0, True)               # (duty in percent, forward)

    def run(self, duty, forward):
        """duty 0..100 percent; 0 stops with both direction pins low."""
        if (duty, forward) == self.state:
            return
        gpio = self.gpio
        for pwm, forward_pin, backward_pin in self.motors:
            if duty <= 0:
                pwm.ChangeDutyCycle(0)
                gpio.output(forward_pin, gpio.LOW)
                gpio.output(backward_pin, gpio.LOW)
            else:
                gpio.output(forward_pin, gpio.HIGH if forward else gpio.LOW)
                gpio.output(backward_pin, gpio.LOW if forward else gpio.HIGH)
                pwm.ChangeDutyCycle(duty)
        self.state = (duty, forward)

    def close(self):
        self.run(0.0, True)
        for pwm, _, _ in self.motors:
            pwm.stop()
        for pin in self.pins:                  # no gpio.cleanup(): that would let the pins float
            self.gpio.output(pin, self.gpio.LOW)


class HwBackend(Backend):

    def __init__(self, node):
        super().__init__(node)

        def param(name, default):
            return node.declare_parameter('hw.' + name, default).value

        motor_pins = [list(param('motor_a_pins', [4, 14, 15])),   # enable, forward, backward
                      list(param('motor_b_pins', [17, 27, 18]))]
        pwm_hz = param('motor_pwm_hz', 1000)
        self.min_duty = param('min_duty_percent', 60.0)          # the motors stall below this
        i2c_bus = param('i2c_bus', 1)
        address = param('pca9685_address', 64)                   # 0x40
        self.steering_channel = param('steering_channel', 2)
        self.steering_centre = param('steering_centre', 300)
        self.steering_left = param('steering_full_left', 242)    # at +max_steering_deg
        self.steering_right = param('steering_full_right', 350)  # at -max_steering_deg
        self.pan_channel = param('pan_channel', 6)
        self.pan_centre = param('pan_centre', 311)
        self.pan_per_deg = param('pan_per_deg', 2.2)             # measure; negative: other way
        self.pan_limits = sorted(param('pan_limits', [180, 420]))
        self.tilt_channel = param('tilt_channel', 10)
        self.tilt_centre = param('tilt_centre', 340)
        self.tilt_per_deg = param('tilt_per_deg', 2.2)
        self.tilt_limits = sorted(param('tilt_limits', [180, 430]))
        self.watchdog_s = param('watchdog_s', 0.5)
        self.max_steering = node.get_parameter('max_steering_deg').value

        gpio = open_gpio()
        bus = open_i2c(i2c_bus)
        try:
            self.servos = PCA9685(bus, address)
        except OSError as error:
            raise BackendError(f"backend 'hw': no answer from the servo board (PCA9685) at "
                               f"I2C address {address:#04x} on bus {i2c_bus}: {error}. "
                               f"Check it with: i2cdetect -y {i2c_bus}") from None
        self.motors = Motors(gpio, motor_pins, pwm_hz)
        self.sent = {}                         # servo channel -> last value written
        self.lock = threading.Lock()
        self.last_drive = time.monotonic()
        self.set_servo(self.steering_channel, self.steering_centre)
        self.look(0.0, 0.0)

        self.closed = threading.Event()
        self.watchdog = threading.Thread(target=self.watch, name='hw_watchdog', daemon=True)
        self.watchdog.start()

    # ---------- what picar_base calls ----------

    def drive(self, power, steering_deg):
        with self.lock:
            self.last_drive = time.monotonic()
            self.set_servo(self.steering_channel, self.steering_value(steering_deg))
            power = clamp(power, -1.0, 1.0)
            self.motors.run(self.duty(power), power > 0)

    def look(self, pan_deg, tilt_deg):
        with self.lock:
            pan = round(self.pan_centre - pan_deg * self.pan_per_deg)
            tilt = round(self.tilt_centre - tilt_deg * self.tilt_per_deg)
            self.set_servo(self.pan_channel, clamp(pan, *self.pan_limits))
            self.set_servo(self.tilt_channel, clamp(tilt, *self.tilt_limits))

    def close(self):
        self.closed.set()
        with self.lock:
            self.motors.close()
            self.set_servo(self.steering_channel, self.steering_centre)

    # ---------- the numbers ----------

    def duty(self, power):
        """Motor duty in percent: 0 for no power, else min_duty .. 100."""
        if abs(power) < 1e-3:
            return 0.0
        return self.min_duty + abs(power) * (100.0 - self.min_duty)

    def steering_value(self, steering_deg):
        """The steering servo's value; left and right have their own full-lock values."""
        share = clamp(steering_deg / self.max_steering, -1.0, 1.0)
        full = self.steering_left if share >= 0 else self.steering_right
        return round(self.steering_centre + (full - self.steering_centre) * abs(share))

    def set_servo(self, channel, value):
        if self.sent.get(channel) != value:
            self.servos.set(channel, value)
            self.sent[channel] = value

    # ---------- the watchdog ----------

    def watch(self):
        while not self.closed.wait(0.1):
            self.check_watchdog()

    def check_watchdog(self):
        """Stop the motors if no drive command came for watchdog_s while they were running."""
        with self.lock:
            silent = time.monotonic() - self.last_drive
            if self.motors.state[0] > 0 and silent > self.watchdog_s:
                self.motors.run(0.0, True)
                self.node.get_logger().warning(
                    f'hw: no drive command for {silent:.1f} s, motors stopped')
