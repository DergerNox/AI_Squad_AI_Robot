"""camera.as_picture: ROS images in every encoding the agent reads, as red-green-blue pictures."""

import base64
import io
from types import SimpleNamespace

import numpy as np
import pytest
from PIL import Image

from camera import as_picture, message_type, newest_frame


def ros_image(encoding, pixels, padding=0):
    """A sensor_msgs/Image stand-in: pixels is a list of rows, each a list of pixels (byte tuples)."""
    height, width, channels = len(pixels), len(pixels[0]), len(pixels[0][0])
    data = b"".join(bytes(v for pixel in row for v in pixel) + bytes(padding) for row in pixels)
    return SimpleNamespace(encoding=encoding, height=height, width=width,
                           step=width * channels + padding, data=data)


def colours(picture):
    """The picture's pixels as (red, green, blue) tuples, row by row. The picture must be
    plain red-green-blue: no alpha, not grey."""
    png = Image.open(io.BytesIO(base64.b64decode(picture["images"][0])))
    assert png.mode == "RGB"
    return [[tuple(int(v) for v in pixel) for pixel in row] for row in np.array(png)]


RED, BLUE = (255, 0, 0), (0, 0, 255)


@pytest.mark.parametrize("encoding, red, blue", [
    ("rgb8", (255, 0, 0), (0, 0, 255)),
    ("bgr8", (0, 0, 255), (255, 0, 0)),
    ("rgba8", (255, 0, 0, 128), (0, 0, 255, 128)),
    ("bgra8", (0, 0, 255, 128), (255, 0, 0, 128)),
])
def test_colour_encodings_come_out_as_red_green_blue(encoding, red, blue):
    assert colours(as_picture(ros_image(encoding, [[red, blue]]))) == [[RED, BLUE]]


def test_grey_pictures_come_out_grey():
    picture = as_picture(ros_image("mono8", [[(10,), (200,)]]))
    assert colours(picture) == [[(10, 10, 10), (200, 200, 200)]]


def test_padded_rows_lose_their_padding():
    image = ros_image("bgr8", [[(0, 0, 255), (255, 0, 0)], [(255, 0, 0), (0, 0, 255)]], padding=2)
    assert len(image.data) == 2 * (2 * 3 + 2)
    assert colours(as_picture(image)) == [[RED, BLUE], [BLUE, RED]]


def test_an_encoding_it_cannot_read_says_what_to_set():
    image = SimpleNamespace(encoding="yuv422", height=1, width=2, step=4, data=bytes(4))
    with pytest.raises(ValueError, match="the camera sends yuv422 pictures.*format RGB888"):
        as_picture(image)


# ---------- compressed pictures (the real car: /camera/image_raw/compressed) ----------

def compressed(pixels, file_format):
    """A sensor_msgs/CompressedImage stand-in: the picture saved as a JPEG or PNG file."""
    out = io.BytesIO()
    Image.fromarray(np.array(pixels, np.uint8)).save(out, file_format)
    return SimpleNamespace(format=file_format.lower(), data=out.getvalue())


def test_a_png_comes_out_exactly():
    assert colours(as_picture(compressed([[RED, BLUE]], "PNG"))) == [[RED, BLUE]]


def test_a_jpeg_comes_out_red_green_blue():
    # left half red, right half blue; JPEG is lossy, so only close to the colours
    pixels = [[RED] * 8 + [BLUE] * 8 for _ in range(16)]
    out = colours(as_picture(compressed(pixels, "JPEG")))
    assert (len(out), len(out[0])) == (16, 16)
    for (r, g, b), want in ((out[8][2], RED), (out[8][13], BLUE)):
        assert max(abs(r - want[0]), abs(g - want[1]), abs(b - want[2])) < 40


def test_a_picture_that_cannot_be_decoded_says_what_to_check():
    broken = SimpleNamespace(format="jpeg", data=b"not a picture")
    with pytest.raises(ValueError, match="cannot be decoded: is the camera_topic"):
        as_picture(broken)


@pytest.mark.parametrize("topic, msg_type", [
    ("/camera/image_raw", "sensor_msgs/msg/Image"),
    ("/camera/image_raw/compressed", "sensor_msgs/msg/CompressedImage"),
])
def test_the_topic_decides_the_message_type(topic, msg_type):
    assert message_type(topic) == msg_type


class OneFrame:
    """A connector stand-in with one frame; it writes down what it was asked for."""

    def __init__(self):
        self.asked = []

    def receive_message(self, topic, timeout_sec, msg_type, qos_profile, auto_qos_matching):
        self.asked.append((topic, msg_type))
        return SimpleNamespace(timestamp=101.0, payload="the frame")


def test_newest_frame_reads_the_cars_topic_with_its_type():
    connector, clock = OneFrame(), SimpleNamespace(time=lambda: 100.0, sleep=lambda s: None)
    assert newest_frame(connector, 100.0, clock, "/camera/image_raw/compressed") == "the frame"
    assert connector.asked == [("/camera/image_raw/compressed", "sensor_msgs/msg/CompressedImage")]
