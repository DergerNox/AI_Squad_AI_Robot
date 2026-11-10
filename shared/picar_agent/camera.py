"""camera.py: pictures from the car's camera (/camera/image_raw), never an old one.

RAI keeps the last message of a topic and hands it out again even after the camera has
stopped sending, so newest_frame checks each picture's arrival time. as_picture turns a
ROS image into the picture the model gets.
"""

import time

import numpy as np
from rai.messages import MultimodalArtifact, preprocess_image
from rclpy.qos import qos_profile_sensor_data

CAMERA_TOPIC = "/camera/image_raw"
CAMERA_WAIT_S = 5.0       # how long to wait for a new camera picture before giving up

# Told to the AI when the camera sends no new pictures (camera node stopped, bad connection)
CAMERA_DOWN = (f"The camera has sent no new picture for {CAMERA_WAIT_S:.0f} seconds, so you cannot "
               "see. Do not drive or turn: finish with DONE and tell the user that the camera is "
               "not working.")


class CameraDown(Exception):
    """No new picture from the camera."""


def newest_frame(connector, newer_than, clock=time):
    """The newest camera frame (a sensor_msgs/Image) that arrived after newer_than, a
    clock.time() value. Raises CameraDown if none comes within CAMERA_WAIT_S."""
    deadline = clock.time() + CAMERA_WAIT_S
    while True:
        try:
            # A fixed message type and sensor-data QoS work with any camera publisher,
            # even before ROS has discovered the topic.
            message = connector.receive_message(
                CAMERA_TOPIC, timeout_sec=1.0, msg_type="sensor_msgs/msg/Image",
                qos_profile=qos_profile_sensor_data, auto_qos_matching=False)
            if message.timestamp > newer_than:
                return message.payload
        except TimeoutError:   # no frame at all yet
            pass
        if clock.time() > deadline:
            print(f"\n[camera: no new picture for {CAMERA_WAIT_S:.0f} s, check it with: "
                  f"ros2 topic hz {CAMERA_TOPIC}]")
            raise CameraDown(CAMERA_DOWN)
        clock.sleep(0.05)


CHANNELS = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}   # the encodings it reads


def as_picture(image):
    """A ROS image (sensor_msgs/Image) as the picture the model gets: red, green, blue.

    Reads rgb8, bgr8, rgba8, bgra8 (camera_ros can send 4 channels) and mono8, also when
    each row is padded (step longer than width x channels)."""
    channels = CHANNELS.get(image.encoding)
    if channels is None:
        raise ValueError(f"the camera sends {image.encoding} pictures, which the agent cannot "
                         f"read: set it to rgb8 or bgr8 (camera_ros: format RGB888)")
    row = image.width * channels
    step = getattr(image, "step", 0) or row   # bytes per row, padding included
    pixels = np.frombuffer(bytes(image.data), np.uint8).reshape(image.height, step)[:, :row]
    pixels = pixels.reshape(image.height, image.width, channels)
    if image.encoding.startswith("bgr"):
        pixels = pixels[:, :, 2::-1]              # blue, green, red (alpha) -> red, green, blue
    elif channels == 4:
        pixels = pixels[:, :, :3]                 # without alpha
    elif channels == 1:
        pixels = np.repeat(pixels, 3, axis=2)     # grey in all three colours
    return MultimodalArtifact(images=[preprocess_image(np.ascontiguousarray(pixels))], audios=[])
