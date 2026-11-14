"""car_client.py: the car as the tools see it: picar_base's services, the camera head, the
camera and the distance sensor.

Every move goes through picar_base (/picar/move and /picar/look), so its SafetyGate checks
it. The client remembers where the camera points, and when the car and the head last
stopped moving, so that a picture always shows where they are now.
"""

import math
import time

from rai.communication.ros2 import ROS2Message
from rclpy.qos import qos_profile_sensor_data

from camera import CAMERA_DOWN, CameraDown, as_picture, newest_frame

RANGE_TOPIC = "/ultrasonic/range"


def clamp(value, low, high):
    return max(low, min(high, float(value)))


class CarClient:
    """One car: connector is RAI's ROS2Connector, car the car's numbers (car_config.Car),
    clock the time module (or a stand-in for it in the tests)."""

    def __init__(self, connector, car, clock=time):
        self.connector = connector
        self.car = car
        self.clock = clock
        self.pan = self.tilt = None   # where the camera points: unknown until the first look
        self.still_since = 0.0        # when the car and the head last stopped (clock.time())

    def point_camera(self, pan_deg, tilt_deg):
        """Turn the camera head, and wait until it is there."""
        car = self.car
        pan = clamp(pan_deg, -car.max_pan_deg, car.max_pan_deg)
        tilt = clamp(tilt_deg, -car.max_tilt_deg, car.max_tilt_deg)
        if self.pan is None:
            turn = 2 * car.max_pan_deg   # we don't know where it points: wait for the longest turn
        else:
            turn = max(abs(pan - self.pan), abs(tilt - self.tilt))
        self.connector.service_call(
            ROS2Message(payload={"pan_deg": pan, "tilt_deg": tilt}),
            target="/picar/look", msg_type="picar_interfaces/srv/Look")
        self.pan, self.tilt = pan, tilt
        self.clock.sleep(0.4 + turn / car.head_deg_per_s)   # let the head get there and settle
        self.still_since = self.clock.time()                # pictures from now on show where it points

    def look_ahead(self):
        """Point the camera straight ahead, unless it already does."""
        if self.pan != 0.0 or self.tilt != 0.0:
            self.point_camera(0.0, 0.0)

    def move(self, distance, steering):
        """Drive with a fixed steering angle, the camera looking ahead. Returns how far it
        went (metres, negative backward) and picar_base's answer (None if it did not answer):
        .reason is done, obstacle, no_range, timeout, estop or busy, and .min_range_m the
        closest distance reading during the move."""
        self.look_ahead()
        speed = self.car.speed_mps
        duration = abs(distance) / speed
        request = {"speed": math.copysign(1.0, distance), "steering_deg": steering,
                   "duration_s": duration}
        start = self.clock.monotonic()
        answer = self.connector.service_call(
            ROS2Message(payload=request), target="/picar/move",
            msg_type="picar_interfaces/srv/Move", timeout_sec=duration + 5.0).payload
        # No wheel encoders: work out how far it went from the time it drove.
        driven = math.copysign(min(abs(distance), speed * (self.clock.monotonic() - start)), distance)
        self.clock.sleep(0.3)                  # let the car settle
        self.still_since = self.clock.time()   # pictures from now on show where it stopped
        return driven, answer

    def take_picture(self):
        """A camera picture taken after the car and the camera head stopped moving."""
        return as_picture(newest_frame(self.connector, self.still_since, self.clock,
                                       self.car.camera_topic))

    def require_camera(self):
        """Never drive blind: the camera must have sent a picture in the last second."""
        try:
            newest_frame(self.connector, self.clock.time() - 1.0, self.clock,
                         self.car.camera_topic)
        except CameraDown:
            raise CameraDown(f"The car did not move. {CAMERA_DOWN}") from None

    def range_ahead(self):
        """The distance sensor's reading in metres: inf if nothing is in range, None if no reading."""
        try:
            reading = self.connector.receive_message(
                RANGE_TOPIC, timeout_sec=1.0, msg_type="sensor_msgs/msg/Range",
                qos_profile=qos_profile_sensor_data, auto_qos_matching=False).payload
        except TimeoutError:
            return None
        if math.isnan(reading.range):
            return None
        if reading.range == -math.inf:   # closer than the sensor can measure
            return 0.0
        if math.isinf(reading.range) or reading.range >= reading.max_range:
            return math.inf
        return reading.range
