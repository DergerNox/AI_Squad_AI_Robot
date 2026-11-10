"""camera.as_picture: ROS images in every encoding the agent reads, as red-green-blue pictures."""

import base64
import io
from types import SimpleNamespace

import numpy as np
import pytest
from PIL import Image

from camera import as_picture


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
