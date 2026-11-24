"""A stand-in for the vision model in tests: answers the tools' picture questions by colour.

It sees the twin's red, blue and yellow objects exactly, so a test measures how the
tools drive, not how well a real model sees.
"""
import base64
import io

import re

import numpy as np
from PIL import Image

COLOURS = {
    "red": lambda r, g, b: (r > 60) & (r > 2 * g) & (r > 2 * b),
    "blue": lambda r, g, b: (b > 60) & (b > 1.6 * r) & (b > 1.3 * g),
    "yellow": lambda r, g, b: (r > 90) & (g > 90) & (b < 0.6 * np.minimum(r, g)),
}


def pixels(b64):
    if b64.startswith("data:"):
        b64 = b64.split(",", 1)[1]
    return np.asarray(Image.open(io.BytesIO(base64.b64decode(b64))).convert("RGB")).astype(int)


def mask_for(question, b64):
    colour = next((c for c in COLOURS if c in question.lower()), None)
    if colour is None:
        return None, None
    px = pixels(b64)
    return colour, COLOURS[colour](px[..., 0], px[..., 1], px[..., 2])


TWIN_OBJECTS = {"red": "red box", "blue": "blue cylinder", "yellow": "yellow box"}


def box(mask, label):
    """A bounding box around the mask's pixels, as qwen3-vl writes it (0 to 1000), or [] if
    there are too few of them."""
    if mask.sum() < 40:
        return "[]"
    rows, cols = np.where(mask.any(axis=1))[0], np.where(mask.any(axis=0))[0]
    height, width = mask.shape
    x1, x2 = round(1000 * cols.min() / width), round(1000 * (cols.max() + 1) / width)
    y1, y2 = round(1000 * rows.min() / height), round(1000 * (rows.max() + 1) / height)
    return f'[{{"bbox_2d": [{x1}, {y1}, {x2}, {y2}], "label": "{label}"}}]'


def answer(question, b64):
    located = re.match(r"Locate the (\w+) ", question)
    if located:   # picar_tools.LOCATE: a box around the colour asked about
        colour = located.group(1)
        if colour not in COLOURS:
            return "[]"
        px = pixels(b64)
        return box(COLOURS[colour](px[..., 0], px[..., 1], px[..., 2]), TWIN_OBJECTS.get(colour, colour))
    named = re.search(r"What is the (\w+) thing", question)
    if named:   # naming a sighting (picar_tools.NAME_IT; its examples name other colours)
        return TWIN_OBJECTS.get(named.group(1), f"{named.group(1)} object")
    colour, mask = mask_for(question, b64)
    if mask is None:
        return "white wall"
    width = mask.shape[1]
    if "the same" in question and "Is it the" not in question:   # picar_tools.SAME: a cut-out
        return "yes" if mask.mean() > 0.08 else "no"
    if "Is it the" in question:   # what the car has stopped in front of
        middle = mask[:, width // 3: 2 * width // 3]
        return f"yes, {colour} object" if middle.mean() > 0.08 else "no, white wall"
    if mask.sum() < 40:
        return "no"
    cols = np.where(mask.any(axis=0))[0]
    centre = (cols.min() + cols.max()) / 2
    if centre < width / 3:
        return "left"
    if centre > 2 * width / 3:
        return "right"
    return "middle"


class ColourVision:
    """Has the same invoke() as a LangChain chat model, for make_picar_tools(vision=...)."""

    def __init__(self):
        self.calls = 0

    def invoke(self, messages, config=None):
        self.calls += 1
        message = messages[0]
        question = message.content if isinstance(message.content, str) else " ".join(
            part.get("text", "") for part in message.content if isinstance(part, dict))
        # with two pictures (a first sighting, then the picture now) the question is about the last
        return type("Answer", (), {"content": answer(question, message.images[-1])})()
