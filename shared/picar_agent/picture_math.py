"""picture_math.py: working with the tools' pictures and the model's bounding boxes, without
the model: cut-outs, how much two pictures differ, how far the view turned, where a box is.

A tool picture is a dict {"images": [base64 PNG], "audios": []}, as camera.as_picture makes it.
Boxes are (label, x1, y1, x2, y2) in fractions of the picture's width and height.
"""

import base64
import io

import cv2
import numpy as np
from PIL import Image

CROP_MIN_PX = 448   # a cut-out smaller than this (short side) is enlarged before the model sees it
TURN_MATCH = 0.2    # how well two pictures must match for a measured turn to count
# Below this spread of grey levels the upper picture is too plain to tell (a bare wall); the real
# car's pictures had 19-52 (2026-11-24)
TURN_TEXTURE = 10.0


def image_of(picture, mode="RGB"):
    """The PIL image in a tool picture."""
    data = picture["images"][0]
    data = data.split(",", 1)[1] if data.startswith("data:") else data
    return Image.open(io.BytesIO(base64.b64decode(data))).convert(mode)


def both(first, picture):
    """Two pictures for one question: the first sighting, then the picture now."""
    return {"images": first["images"] + picture["images"], "audios": []}


def cut_out(picture, x1, y1, x2, y2, margin=0.05):
    """A tool picture's part inside a bounding box (fractions of the width and height), with a
    small margin, as a picture of its own. Small parts are enlarged to CROP_MIN_PX: a box 20 %
    of the picture wide is only about 130 pixels, too little for the model to tell a wardrobe
    from a bed (real car, 2026-11-20)."""
    image = image_of(picture)
    w, h = image.size
    box = (max(0, int((x1 - margin) * w)), max(0, int((y1 - margin) * h)),
           min(w, int((x2 + margin) * w) + 1), min(h, int((y2 + margin) * h) + 1))
    part = image.crop(box)
    scale = CROP_MIN_PX / min(part.size)
    if scale > 1:
        part = part.resize((round(part.size[0] * scale), round(part.size[1] * scale)), Image.LANCZOS)
    buffer = io.BytesIO()
    part.save(buffer, format="PNG")
    return {"images": [base64.b64encode(buffer.getvalue()).decode()], "audios": []}


def picture_change(a, b):
    """How different two tool pictures are: the mean grey-level change of 32 x 24 copies."""
    def small(picture):
        return np.asarray(image_of(picture, "L").resize((32, 24)), dtype=float)
    return float(np.abs(small(a) - small(b)).mean())


def turned_deg(before, after, fov_deg):
    """How far the view turned between two tool pictures, in degrees (positive = left), from how
    far the upper 40 % of the picture (the far things) moved sideways; None if the two do not
    match well enough to tell (the car really moved, or the pictures are too small or plain)."""
    def band(picture):
        grey = np.asarray(image_of(picture, "L"), dtype=np.float32)
        return grey[: int(grey.shape[0] * 0.4)]
    a, b = band(before), band(after)
    if a.shape != b.shape or min(a.shape) < 16 or min(a.std(), b.std()) < TURN_TEXTURE:
        return None
    (dx, _), match = cv2.phaseCorrelate(a, b, cv2.createHanningWindow(a.shape[::-1], cv2.CV_32F))
    if not match >= TURN_MATCH:
        return None
    return dx * fov_deg / a.shape[1]   # things move right in the picture when the car turns left


def nearest_boxes(boxes, where):
    """The boxes, the one nearest a part of the picture ("left", "middle", "right") first."""
    was = {"left": 1 / 6, "middle": 0.5, "right": 5 / 6}.get(where, 0.5)
    return sorted(boxes, key=lambda b: abs((b[1] + b[3]) / 2 - was))


def part_of(box):
    """left, middle or right: the third of the picture a box's centre is in."""
    centre = (box[1] + box[3]) / 2
    return "left" if centre < 1 / 3 else "right" if centre > 2 / 3 else "middle"


def floor_distance(floor, row):
    """Metres from the bumper to where the floor shows at this row of the picture (row 1 is
    the bottom edge), from the car's floor table; outside the table, its nearest end."""
    points = sorted(floor, reverse=True)   # bottom edge first: closest first
    if row >= points[0][0]:
        return points[0][1]
    for (row_a, dist_a), (row_b, dist_b) in zip(points, points[1:]):
        if row >= row_b:
            return dist_a + (dist_b - dist_a) * (row_a - row) / (row_a - row_b)
    return points[-1][1]
