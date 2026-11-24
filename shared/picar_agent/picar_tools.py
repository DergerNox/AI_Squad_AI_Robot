"""picar_tools.py: what the AI can do with the PiCar: its tools.

drive        drives forward or backward with a steering angle, then takes a picture
turn         turns the car by an angle (a drive at full steering), then takes a picture
look         points the camera and takes a picture
look_around  looks left, ahead and right for a target and says how far to turn to face it
approach     drives to a target it can see in steps, looking and steering after each step
reached      notes that the car has arrived at one of the targets
remember     saves a fact for later tasks (in memory.json)

drive, turn and the camera go through picar_base (/picar/move and /picar/look),
so the SafetyGate still checks every move the AI makes. Questions about pictures
("is there a blue cylinder here?") are answered by the same AI model.

The car's own sensors have the last word: reached only counts a target when the
safety system has stopped the car in front of it, and drive and turn do not move
the car while the camera sends no new pictures.

The other parts: car_client.py talks to the car, camera.py gets its pictures,
car_config.py reads the car's numbers, prompt.py tells the AI about the car and
history.py keeps the conversation short.
"""

import base64
import io
import json
import math
import os
import re
import time
from dataclasses import dataclass, field

import cv2
import numpy as np
from langchain_core.tools import tool
from PIL import Image
from rai.communication.ros2 import ROS2Connector
from rai.messages import HumanMultimodalMessage

from camera import CAMERA_DOWN, CameraDown
from car_client import CarClient, clamp
from car_config import load_car, metres
from picture_log import PictureLog

# The car's numbers (speed, limits, size, camera) come from its settings file: car_config.py.
# These are how the tools work, the same for every car.
MIN_MOVE_M = 0.05         # less than this counts as "did not move"
# Next to agent.py, from whichever folder the agent is started
MEMORY_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "memory.json")


def look_around_pans(max_pan):
    """Where look_around points the camera: ahead, left, right, and further out where the head
    turns that far. The twin's head turns to 90 degrees (pictures together about 240 degrees);
    the real car's only to about 50: asked for 90 it stopped there, and look_around told the
    AI to turn about 40 degrees too far (2026-11-20)."""
    if max_pan >= 75:
        return (0, 45, -45, min(90, max_pan), -min(90, max_pan))
    return (0, max_pan, -max_pan)


SIDE_OFFSET_DEG = 20      # something on the left/right of a picture is about this far from its middle
# Nothing sees behind the car, and on the real car reversing bumped into things (2026-11-17):
# the tools reverse only when the way forward is blocked, and then at most this far.
BACK_UP_M = 0.2

APPROACH_MAX_STEPS = 12   # steps per approach call
APPROACH_MIN_M = 0.2      # shortest step straight at the target
# Close to a target the sensor does not hear (a low box on the real car: its beam passed over
# it and the car hit the box, 2026-11-16): steps this long at most, side corrections at half
# the angle and only while reversing, so they swing the car away from the target, not into it.
APPROACH_CLOSE_M = 0.1
# The sensor hears the target if it reads at most this much further than the camera puts it
# (the real car's sensor read 1.3 m, the cabinet behind the low box, with the box at the bumper)
HEAR_MARGIN_M = 0.3
APPROACH_BACK_M = BACK_UP_M   # how far to back up when the target slips out of the picture

# What the safety system's stop reasons mean
STOP_REASONS = {
    "no_range": "the distance sensor had no fresh reading, just try again",
    "timeout": "the drive command arrived too late, just try again",
    "busy": "another move was still running",
}

# look_around without a real target ("look around and tell me what you see"): the model
# called look_around("unknown") and then drove at it (real car, 2026-11-14). It describes now.
NO_TARGET = {"", "unknown", "anything", "something", "everything", "nothing", "object", "objects",
             "thing", "things", "surroundings", "around", "room", "area", "environment", "scene",
             "view", "what i see", "what you see"}
DESCRIBE_PANS = ((0, "ahead"), (45, "to the left"), (-45, "to the right"))
DESCRIBE = ("This picture is from a small robot car's camera. Describe it in one short sentence: "
            "the main things you see and their colours.")


def no_target(target):
    """True if a look_around target names nothing to search for."""
    return re.sub(r"^(the|a|an|any)\s+", "", target.strip().lower()) in NO_TARGET


# Asked to name a sighting. The model says "yes, there" to anything of the target's colour
# (on the real car it took a blue bag for a blue box, 3 times out of 3), but names things
# right ("blue bag", "red stick"; 2026-11-14), so a sighting counts only if named right.
NAME_IT = ("This picture is from a small robot car's camera. What is the {thing} at the {where} of "
           "the picture? Answer with two or three words, colour first, like 'red ball' or 'white wall'.")
# Asked about a cut-out of the picture: the target's bounding box. With a white plastic bag in
# front of a white bed, "the white thing in the middle" was always the bag (real car, 2026-11-20).
NAME_CROP = ("This is part of a picture from a small robot car's camera. What is the {thing} in it? "
             "Answer with two or three words, colour first, like 'red ball' or 'white wall'.")
# How far down the camera looks for a target that was straight ahead and has gone: at tilt 0
# the floor within about 0.4 m of the bumper is below the picture.
LOOK_DOWN_DEG = -30
# Ferenc's idea (2026-11-20): the first time a task's target is seen and named, its part of the
# picture is kept, and later sightings that a name cannot settle are compared with it. The real
# model on cut-outs (3 tries each): the white bed from another run "yes"; the plastic bag that
# had fooled it, a slipper and a white door "no".
SAME = ("The first picture shows a {target} that a small robot car saw earlier. The second picture "
        "is from the car's camera now. Is the thing in the second picture the same {target}? "
        "Answer only 'yes' or 'no'.")
COLOUR_WORDS = {"red", "blue", "green", "yellow", "white", "black", "orange", "purple", "pink",
                "brown", "grey", "gray"}
KINDS = {   # what may stand for the target's kind in the model's name for it
    "box": ("box", "cube", "carton", "crate", "package", "block", "container"),
    "bed": ("bed", "mattress", "sheet", "blanket", "duvet", "bedding", "pillow"),
    "stick": ("stick", "pole", "rod", "bar", "pipe"),
    "cylinder": ("cylinder", "can", "tube", "pillar", "column"),
    "ball": ("ball", "sphere"),
    "shoe": ("shoe", "slipper", "sandal", "sneaker", "boot", "trainer"),
}


# What the model calls a target it has just seen and named, once the car is close: the real
# car's blue box became "blue wall" and "blue object" (2026-11-17). Accepted only right after
# the target was seen and named (TaskState.just_seen), never while searching: blue look-alikes
# get names of their own ("blue backpack", "blue books").
VAGUE_NAMES = {"object", "thing", "item", "wall", "surface", "panel", "shape", "rectangle",
               "square", "board", "cardboard", "packaging"}


def vague_name(answer):
    """True if the model's name for a thing says only its colour and that it is a thing."""
    words = re.findall(r"[a-z]+", answer)
    return bool(words) and words[-1] in VAGUE_NAMES


def word_forms(word):
    """A word and its singular forms: boxes -> {boxes, boxe, box}."""
    forms = {word}
    if word.endswith("s") and len(word) > 3:
        forms.add(word[:-1])
    if word.endswith("es") and len(word) > 4:
        forms.add(word[:-2])
    return forms


def one_letter_off(a, b):
    """True if a and b differ by one letter: one changed, added or left out."""
    if abs(len(a) - len(b)) > 1:
        return False
    if len(a) == len(b):
        return sum(x != y for x, y in zip(a, b)) == 1
    short, long_ = sorted((a, b), key=len)
    return any(long_[:i] + long_[i + 1:] == short for i in range(len(long_)))


def same_word(a, b):
    """The same word, also as a plural, or with one letter wrong in a word of five letters or
    more ("sliper" for "slipper": the user's typo made the slipper impossible to confirm)."""
    for x in word_forms(a):
        for y in word_forms(b):
            if x == y or (min(len(x), len(y)) >= 5 and one_letter_off(x, y)):
                return True
    return False


def names_target(answer, target):
    """True if the model's name for a thing says it is the target's kind (box, stick, ...)."""
    kind = target.split()[-1] if target.split() else ""
    if not kind:
        return False
    family = next((words for words in KINDS.values() if any(same_word(kind, w) for w in words)), (kind,))
    return any(same_word(word, f) for word in re.findall(r"[a-z]+", answer) for f in family)


# Told to the AI from the second move in a row without a distance reading. One is a passing
# blip (the twin's sensor misses 2 % of echoes); several mean the sensor cannot see what is in
# front: on the real car a soft blanket and a crumpled bag gave no echo at all (2026-11-14).
NO_ECHO = ("The distance sensor gets no echo from what is in front: probably something soft "
           "(cloth, foam, a crumpled bag) or a surface at an angle. The safety system does not let "
           "the car drive forward without a reading, so trying again does not help: turn towards "
           "something else, back up, or finish with DONE and tell the user.")
# Where things are in a picture, as bounding boxes: qwen3-vl answers with numbers (0 to 1000
# across and down the picture), much more exactly than in words. On the real car the words
# failed (2026-11-17): "bottom edge" for a box 70 % down the picture (approach stopped 0.7 m
# early: "arrived by the camera") and "left" for a power strip that reached the car's path.
LOCATE = ('Locate the {target} in the image. Output its bounding box in JSON: '
          '[{{"bbox_2d": [x1, y1, x2, y2], "label": "{target}"}}], or [] if there is none.')
BOTTOM_ROW = 0.95   # a box reaching this far down touches the bottom edge of the picture
CLOSE_ROW = 2 / 3   # the lower third: close (with tilt 0 the middle row is the horizon)


def nearest_boxes(boxes, where):
    """The boxes, the one nearest a part of the picture ("left", "middle", "right") first."""
    was = {"left": 1 / 6, "middle": 0.5, "right": 5 / 6}.get(where, 0.5)
    return sorted(boxes, key=lambda b: abs((b[1] + b[3]) / 2 - was))


def part_of(box):
    """left, middle or right: the third of the picture a box's centre is in."""
    centre = (box[1] + box[3]) / 2
    return "left" if centre < 1 / 3 else "right" if centre > 2 / 3 else "middle"


def boxes_in(answer):
    """The bounding boxes in the model's answer: [(label, x1, y1, x2, y2)], as fractions of
    the picture's width and height (0 is the left or top edge, 1 the right or bottom edge)."""
    boxes = []
    for item in re.finditer(r"\{[^{}]*\}", answer):
        numbers = re.search(r'"bbox_2d"\s*:\s*\[\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,'
                            r'\s*([-\d.]+)\s*\]', item.group(0))
        if not numbers:
            continue
        x1, y1, x2, y2 = (min(max(float(v) / 1000, 0.0), 1.0) for v in numbers.groups())
        label = re.search(r'"label"\s*:\s*"([^"]*)"', item.group(0))
        boxes.append(((label.group(1) if label else "").strip().lower(),
                      min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)))
    return boxes


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


# Two pictures of the real car standing still differ by about 0.3 grey levels on average (32 x 24
# copies), pictures of different places by 45-60 (2026-11-17). A drive that picar_base finished
# but that changed the picture less than this did not move the car: that day a loose motor cable
# let approach "drive 0.49 m" twelve times while the car stood still.
STILL_CHANGE = 1.0
CHECK_MOVED_M = 0.1   # drives at least this long are checked
# Not only dead motors: on the real car, pressed against cables and a slipper the safety system
# cannot see, the AI was told the motors may have no power and gave up (2026-11-24).
STILL_ONCE = (" The camera picture did not change, so the car may not have moved: something low or soft"
              " may block it, or the motors may have no power.")
NOT_MOVING = ("The car does not seem to move: the camera picture stayed the same after {n} drives in a "
              "row that picar_base finished. Something low or soft may block it (the safety system "
              "cannot see such things), or the motors may have no power (a loose motor cable, flat "
              "batteries).")
NOT_MOVING_TRY = (" Back up {back:g} m once (drive with distance_m {neg:g}); if the picture still does not"
                  " change, finish with DONE and tell the user to check the car.")
NOT_MOVING_END = " Do not drive again: finish with DONE and tell the user to check the car."
FILLS_VIEW = 0.9   # a target box this wide (of the picture's width), low down: the car is at it
# approach calls to one target without arriving: on the real car the AI looped approach, back up,
# approach for four minutes with a slipper lying in front of the bed (2026-11-24)
APPROACH_TRIES = 3


def not_moving(n):
    """What the AI is told after n finished drives in a row that did not change the picture."""
    then = NOT_MOVING_TRY.format(back=BACK_UP_M, neg=-BACK_UP_M) if n == 2 else NOT_MOVING_END
    return NOT_MOVING.format(n=n) + then


CROP_MIN_PX = 448   # a cut-out smaller than this (short side) is enlarged before the model sees it

# Driven straight at the target, it is suddenly gone and something is at most this far beyond the
# safety stop distance: probably the target itself, filling the picture. On the real car the bed's
# drawer filled the picture at about 0.27 m, just outside the 0.25 m stop, and the model said "no
# white bed" about it (2026-11-24). If the picture looks like the first sighting, it has arrived.
# The real model on that run's pictures, 2 tries each: the bed's drawer and top close up "yes" 6/6,
# a white plush toy in front "no" 6/6 (asked "part of the same bed?" it explained instead).
RIGHT_IN_FRONT_M = 0.1
CLOSE_UP = ("The first picture shows a {target} that a small robot car saw earlier. The second picture "
            "is from the car's camera now, very close to something. Is the second picture a close-up "
            "of the same {target}? Answer only 'yes' or 'no'.")

# A turn the camera says hardly happened: the view turned less than this part of the asked angle
# (and less than TURN_MIN_DEG). On the real car, pressed against the bed and a plush toy, two turns
# of 120 did not move it, and the AI was told it had turned (2026-11-24).
TURN_PART = 0.25
TURN_MIN_DEG = 8.0
TURN_MATCH = 0.2   # how well the two pictures must match for the measured turn to count
# Below this spread of grey levels the upper picture is too plain to tell (a bare wall); the real
# car's pictures had 19-52 (2026-11-24)
TURN_TEXTURE = 10.0
TURN_STUCK = (" The camera picture says the car turned only about {real:.0f} degrees, not {asked:.0f}: "
              "something low or soft may block it, which the safety system cannot see. Back up "
              "{back:g} m (drive with distance_m {neg:g}) before you turn again.")


def turned_deg(before, after, fov_deg):
    """How far the view turned between two tool pictures, in degrees (positive = left), from how
    far the upper 40 % of the picture (the far things) moved sideways; None if the two do not
    match well enough to tell (the car really moved, or the pictures are too small)."""
    def band(picture):
        data = picture["images"][0]
        data = data.split(",", 1)[1] if data.startswith("data:") else data
        grey = np.asarray(Image.open(io.BytesIO(base64.b64decode(data))).convert("L"), dtype=np.float32)
        return grey[: int(grey.shape[0] * 0.4)]
    a, b = band(before), band(after)
    if a.shape != b.shape or min(a.shape) < 16 or min(a.std(), b.std()) < TURN_TEXTURE:
        return None
    (dx, _), match = cv2.phaseCorrelate(a, b, cv2.createHanningWindow(a.shape[::-1], cv2.CV_32F))
    if not match >= TURN_MATCH:
        return None
    return dx * fov_deg / a.shape[1]   # things move right in the picture when the car turns left


def cut_out(picture, x1, y1, x2, y2, margin=0.05):
    """A tool picture's part inside a bounding box (fractions of the width and height), with a
    small margin, as a picture of its own. Small parts are enlarged to CROP_MIN_PX: a box 20 %
    of the picture wide is only about 130 pixels, too little for the model to tell a wardrobe
    from a bed (real car, 2026-11-20)."""
    data = picture["images"][0]
    data = data.split(",", 1)[1] if data.startswith("data:") else data
    image = Image.open(io.BytesIO(base64.b64decode(data))).convert("RGB")
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
        data = picture["images"][0]
        data = data.split(",", 1)[1] if data.startswith("data:") else data
        image = Image.open(io.BytesIO(base64.b64decode(data))).convert("L").resize((32, 24))
        return np.asarray(image, dtype=float)
    return float(np.abs(small(a) - small(b)).mean())


# Told to the AI when picar_base's emergency stop holds the car. Without "finish", the model
# kept trying approach and backing up for minutes (first run on the real car, 2026-11-14).
ESTOP = ("The emergency stop is on: the car cannot move at all until a person releases it. "
         "Do not call drive, turn or approach again: finish with DONE and tell the user that "
         "the emergency stop is on.")

# Asked about the picture when the safety system has stopped the car
IN_FRONT = ("This picture is from a small robot car's camera, looking straight ahead. "
            "The car has stopped because something is very close in front of it.")


def says_yes(answer):
    """True if an answer to a yes/no question starts with yes."""
    words = re.findall(r"[a-z]+", answer.lower())
    return bool(words) and words[0] == "yes"


def says_no(answer):
    """True if an answer to a yes/no question starts with no."""
    words = re.findall(r"[a-z]+", answer.lower())
    return bool(words) and words[0] in ("no", "not")


def load_memory(path=None):
    """The facts the AI saved with remember, from memory.json (or path)."""
    try:
        with open(path or MEMORY_FILE, encoding="utf-8") as f:
            facts = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return []
    return facts if isinstance(facts, list) else []


def save_memory(memory, path=None):
    with open(path or MEMORY_FILE, "w", encoding="utf-8") as f:
        json.dump(memory, f, indent=2, ensure_ascii=False)


@dataclass
class FirstSight:
    """The first time a target was seen and named in a task: the picture and where in it.
    crop: its part of the picture (cut out when first needed); tried: the model was asked
    where exactly it is in the picture (if it did not say, there is nothing to compare with)."""

    picture: dict
    where: str
    crop: dict = None
    tried: bool = False


@dataclass
class TaskState:
    """What the tools know about the task in progress."""

    reached: list = field(default_factory=list)   # the targets reached so far, in order
    next_target: str = ""     # the next target, as reached was told
    searched: str = ""        # the last target look_around or approach was asked for
    # Where the car is stopped. All of these clear when the car moves:
    blocked_m: float = None   # something this close in front stops it (None: nothing does)
    seen: str = ""            # what the camera check says that is
    standing_at: str = ""     # the target it has just reached and still stands at
    tries: int = 0            # drives forward that did not move it
    doubted: bool = False     # reached was turned down once here because of the camera check
    close_by_camera: str = ""  # the target the camera sees right in front while the sensor cannot
    no_echo: int = 0          # moves in a row that ended without a distance reading
    misses: int = 0           # look_arounds in a row that did not find the searched target
    just_seen: str = ""       # the target the last look_around or approach saw and named
    not_moving: int = 0       # finished drives in a row that did not change the camera picture
    approaches: int = 0       # approach calls for the searched target without arriving
    # The first sighting of each target ({target: FirstSight}), to compare later ones with.
    # Kept only in memory, and only until the target is reached or the task ends.
    first_seen: dict = field(default_factory=dict)

    def new_task(self):
        """Forget the last task: its targets, their first sightings and where the car stopped."""
        self.reached.clear()
        self.first_seen.clear()
        self.next_target = self.searched = self.just_seen = ""
        self.no_echo = self.not_moving = self.misses = self.approaches = 0
        self.leave_spot()

    def leave_spot(self):
        """The car has moved: whatever it was standing at is behind it now."""
        self.blocked_m, self.seen, self.standing_at, self.close_by_camera = None, "", "", ""
        self.tries, self.doubted = 0, False

    def target_now(self):
        """The target the AI is going for, as far as the tools know."""
        return self.next_target or self.searched

    def progress_note(self):
        note = f" Reached so far: {', '.join(self.reached) if self.reached else 'nothing yet'}."
        if self.next_target:
            note += f" Next target: {self.next_target}."
        return note

    def status_note(self):
        """Added to every picture: progress, and what to do if the car is stuck."""
        note = self.progress_note()
        if self.standing_at:
            note += (f" You are still at the {self.standing_at}: back up {BACK_UP_M:g} m (drive with"
                     f" distance_m {-BACK_UP_M:g}) and turn away from it.")
        elif self.close_by_camera:
            note += (f" The camera shows the {self.close_by_camera} right in front of the car, which"
                     f" the distance sensor cannot see: you have arrived, call reached now.")
        elif self.blocked_m is not None:
            target = f"the {self.target_now()}" if self.target_now() else "your target"
            note += " The car is stopped right in front of something and cannot drive forward."
            if self.seen:
                note += f' The camera check says: "{self.seen}".'
            note += f" If that is {target}, you have arrived: call reached now. If not, back up or turn."
        return note


def make_picar_tools(connector: ROS2Connector, vision=None, car=None, clock=None, memory_file=None,
                     picture_log=None):
    """Create the tools. They share one car client, which remembers where the camera points.

    vision: the chat model, used for short questions about pictures.
    car: the car's numbers (car_config.Car); None reads them from PICAR_CONFIG's file.
    clock: the time module (the default), or a stand-in for it in the tests.
    memory_file: where remember keeps its facts; None: MEMORY_FILE, next to agent.py.
    picture_log: where the run's pictures are kept (picture_log.PictureLog); None: as .env says.
    Returns the tools, the function that starts a new task (with its text), and the remembered facts.
    """
    car = car or load_car()
    client = CarClient(connector, car, clock or time)
    task = TaskState()
    memory_file = memory_file or MEMORY_FILE
    memory = load_memory(memory_file)   # facts kept between tasks and restarts
    pictures = picture_log or PictureLog.from_env()

    def start_task(text=""):
        task.new_task()
        pictures.start(text)
    start_task.task = task   # what the tools know about the task, for the tests

    def ask_about(picture, question, label="camera", limit=60, echo=True):
        """One short question to the vision model about a picture, without the chat. The
        answer comes back in lower case, at most limit characters. echo: print it."""
        if vision is None:
            return ""
        message = HumanMultimodalMessage(content=question, images=picture["images"])
        try:
            # No callbacks: the answer is not streamed to the terminal or /to_human
            answer = vision.invoke([message], config={"callbacks": []}).content
        except Exception as error:
            print(f"\n[camera question failed: {error}]")
            return ""
        answer = " ".join(str(answer).strip(" .'\"").lower().split())[:limit]
        pictures.save(picture, label, question, answer)
        if echo:
            print(f"\n[{label}: {answer}]")
        return answer

    def locate(picture, question, label):
        """The bounding boxes the model gives for a LOCATE question; prints them
        short: "green bag 20-50% across, down to 72%"."""
        boxes = boxes_in(ask_about(picture, question, label, limit=1500, echo=False))
        shown = "; ".join(f"{name or '?'} {x1:.0%}-{x2:.0%} across, down to {y2:.0%}"
                          for name, x1, _, x2, y2 in boxes)
        print(f"\n[{label}: {shown or 'none'}]")
        return boxes

    def check_in_front(picture):
        """Ask what the car has stopped in front of (the target? Compared with its first
        sighting, if there is one)."""
        first = reference(task.target_now()) if task.target_now() else None
        if first is not None:
            question = (f"These pictures are from a small robot car's camera. The first one shows a "
                        f"{task.target_now()} it saw earlier. In the second one, looking straight ahead, "
                        "the car has stopped because something is very close in front of it. Is it the "
                        f"same {task.target_now()}? Answer 'yes' or 'no', then say what it is in two or "
                        "three words, colour first.")
            return ask_about(both(first, picture), question)
        if task.target_now():
            question = (f"{IN_FRONT} Is it the {task.target_now()}? Answer 'yes' or 'no', then say "
                        "what it is in two or three words, colour first.")
        else:
            question = (f"{IN_FRONT} What is that thing? Answer with two or three words, "
                        "colour first, like 'green ball' or 'white wall'.")
        return ask_about(picture, question)

    def where_is(picture, target, label, confirm=True, lenient=False):
        """Where the vision model sees the target in a picture: "left", "middle",
        "right", or "" if it is not there. confirm: the model must also name the thing
        it sees as the target's kind (see NAME_IT), or a part of the picture it locates must
        look like the target's first sighting; lenient: a vague name will do (VAGUE_NAMES)."""
        question = (f"Is there a {target} in this picture? Answer only 'no', or where it is "
                    "in the picture: 'left', 'middle' or 'right'.")
        where = place(ask_about(picture, question, label))
        if not where or not confirm:
            return where
        thing = thing_of(target)
        name = ask_about(picture, NAME_IT.format(thing=thing, where=where), f"{label}, named")
        if names_target(name, target) or (lenient and vague_name(name)):
            seen_first(picture, target, where)
            return where
        # Perhaps something else of the target's colour stands in front of it: ask where the
        # target is exactly, and whether that part of the picture shows it
        boxed = confirm_by_box(picture, target, label, where)
        if boxed:
            return boxed
        # Something else of the target's colour. The model points at one thing only: with a blue
        # bag in front it never pointed at the blue box (real car, 2026-11-17). Ask once more.
        question = (f"Is there a {target} in this picture, other than the {name}? Answer only "
                    "'no', or where it is in the picture: 'left', 'middle' or 'right'.")
        other = place(ask_about(picture, question, f"{label}, other"))
        if not other or other == where:
            return ""
        name = ask_about(picture, NAME_IT.format(thing=thing, where=other), f"{label}, named")
        if names_target(name, target) or (lenient and vague_name(name)):
            seen_first(picture, target, other)
            return other
        return ""

    def confirm_by_box(picture, target, label, near):
        """left, middle or right if a bounding box the model gives for the target (the box nearest
        `near` first, at most two), cut out of the picture, looks like the target's first sighting,
        or without one, shows the target's kind; else ""."""
        boxes = locate(picture, LOCATE.format(target=target), f"{label}, where exactly")
        for box in nearest_boxes(boxes, near)[:2]:
            if is_target(picture, box, target, label):
                return part_of(box)
        return ""

    def is_target(picture, box, target, label, vague=False):
        """True if a bounding box shows the target: like its first sighting, or without one, named
        as its kind in a close look (vague: a vague name will do)."""
        crop = cut_out(picture, *box[1:])
        first = reference(target)
        if first is not None:
            return says_yes(ask_about(both(first, crop), SAME.format(target=target),
                                      f"{label}, same as the first sighting"))
        name = ask_about(crop, NAME_CROP.format(thing=thing_of(target)), f"{label}, close look")
        if names_target(name, target) or (vague and vague_name(name)):
            seen_first(picture, target, part_of(box), box)
            return True
        return False

    def thing_of(target):
        """What to call the target in a question that must not give away what it is."""
        first = target.split()[0] if target.split() else ""
        return f"{first} thing" if first in COLOUR_WORDS else "object"

    def both(first, picture):
        """Two pictures for one question: the first sighting, then the picture now."""
        return {"images": first["images"] + picture["images"], "audios": []}

    def seen_first(picture, target, where, box=None):
        """Keep the target's first sighting in this task (later ones are compared with it)."""
        if not target or target in task.first_seen:
            return
        sight = task.first_seen[target] = FirstSight(picture, where)
        if box is not None:
            sight.crop, sight.tried = cut_out(picture, *box[1:]), True
            pictures.save(sight.crop, f"first sighting of the {target}")

    def reference(target):
        """The target's part of its first sighting, to compare with; None if there is none."""
        sight = task.first_seen.get(target)
        if sight is None:
            return None
        if not sight.tried:   # cut out when first needed: often the names settle everything
            sight.tried = True
            boxes = locate(sight.picture, LOCATE.format(target=target), "first sighting, where exactly")
            if boxes:
                box = nearest_boxes(boxes, sight.where)[0]
                sight.crop = cut_out(sight.picture, *box[1:])
                pictures.save(sight.crop, f"first sighting of the {target}")
        return sight.crop

    def place(answer):
        """left, middle, right or "" (not there) from an answer to where_is's question."""
        words = re.findall(r"[a-z]+", answer)
        if not words or words[0] in ("no", "not", "none", "nothing"):
            return ""
        where = next((w for w in ("left", "middle", "centre", "center", "right") if w in words), "")
        if where in ("centre", "center") or (not where and words[0] == "yes"):
            where = "middle"   # in the middle, or seen without saying where
        return where

    def picture_with_notes(text, before=None, turn_deg=None):
        """Take the picture for a tool result. If the car has just got stuck, ask
        the camera what it is stuck at. before: the picture from before a finished drive,
        to check that the car really moved; turn_deg: how far that drive should have turned it."""
        try:
            picture = client.take_picture()
        except CameraDown:   # the car has moved: still tell the AI what it did
            return f"{text} {CAMERA_DOWN}", None
        if task.blocked_m is not None and not task.seen:
            task.seen = check_in_front(picture)
        note = moved_note(before, picture)
        if turn_deg and before is not None and not note:
            real = turned_deg(before, picture, car.camera_fov_deg)
            if real is not None and abs(real) < max(TURN_MIN_DEG, TURN_PART * abs(turn_deg)):
                note = TURN_STUCK.format(real=abs(real), asked=abs(turn_deg), back=BACK_UP_M, neg=-BACK_UP_M)
        text = text + note + task.status_note()
        pictures.save(picture, "shown to the AI", answer=text)
        return text, picture

    def picture_before(distance):
        """A picture straight ahead before a drive long enough to check, else None."""
        if abs(distance) < CHECK_MOVED_M:
            return None
        client.look_ahead()
        try:
            return client.take_picture()
        except CameraDown:
            return None

    def finished(driven, stop):
        return not stop and abs(driven) >= CHECK_MOVED_M - 1e-6

    def moved_note(before, after):
        """"" if the camera picture changed after a finished drive; else a warning, and from
        the second drive in a row that did not change it, NOT_MOVING."""
        if before is None or after is None:
            return ""
        if picture_change(before, after) >= STILL_CHANGE:
            task.not_moving = 0
            return ""
        task.not_moving += 1
        return f" {not_moving(task.not_moving)}" if task.not_moving >= 2 else STILL_ONCE

    def move(distance, steering):
        """Drive with a fixed steering angle. Returns how far it went, why it stopped
        early ("" if it drove the whole way), and whether something in front stopped it."""
        driven, answer = client.move(distance, steering)
        moved = abs(driven) >= MIN_MOVE_M
        if moved:   # whatever the car was standing at is behind it now
            task.leave_spot()
        if answer is None:
            return driven, "picar_base did not answer.", False
        if answer.reason == "no_range":
            task.no_echo += 1
            if task.no_echo >= 2:
                return driven, NO_ECHO, False
            return driven, f"Stopped early: {STOP_REASONS['no_range']}.", False
        task.no_echo = 0
        if answer.reason == "obstacle":
            task.blocked_m = answer.min_range_m
            if not moved:
                task.tries += 1
            stop = f"Safety stop: something is {answer.min_range_m:.2f} m in front of the bumper."
            return driven, stop, True
        if answer.reason == "estop":
            return driven, ESTOP, False
        if answer.reason != "done":
            return driven, f"Stopped early: {STOP_REASONS.get(answer.reason, answer.reason)}.", False
        return driven, "", False

    def drove(driven, steering, stop):
        """A drive in words."""
        turned = car.turn_deg_per_m * driven * steering / car.max_steering_deg
        return (f"Drove {driven:+.2f} m with steering {steering:+.0f}, the heading changed "
                f"about {turned:+.0f} degrees (positive = left). {stop}").strip()

    # The descriptions of look and drive hold the car's limits, so they are made here
    pan, tilt = car.max_pan_deg, car.max_tilt_deg

    @tool(response_format="content_and_artifact", description=f"""Point the camera and take a picture.

pan_deg: -{pan:.0f} to {pan:.0f}, positive turns the camera left.
tilt_deg: -{tilt:.0f} to {tilt:.0f}, positive looks up.
pan_deg 0 and tilt_deg 0 look straight ahead.""")
    def look(pan_deg: float = 0.0, tilt_deg: float = 0.0):
        client.point_camera(pan_deg, tilt_deg)
        pan, tilt = client.pan, client.tilt
        text = f"Picture with the camera at pan {pan:.0f}, tilt {tilt:.0f}."
        if abs(pan) >= 10:
            side = "left" if pan > 0 else "right"
            text += (f" The camera points {abs(pan):.0f} degrees to the {side} of the car. If your"
                     f" target is in this picture, call turn with angle_deg {pan:.0f} before you drive.")
        picture = client.take_picture()
        pictures.save(picture, "look, shown to the AI", answer=text + task.status_note())
        return text + task.status_note(), picture

    @tool(response_format="content_and_artifact")
    def look_around(target: str):
        """Search for a target with the camera, without moving the car. The camera looks
        to the left, ahead and to the right, and you are told how far to turn to face it.

        target: what you are looking for, for example "blue cylinder".
        """
        target = target.strip().lower()
        if no_target(target):
            seen = []
            for pan, side in DESCRIBE_PANS:
                client.point_camera(pan, 0)
                answer = ask_about(client.take_picture(), DESCRIBE, f"camera {side}", limit=160)
                seen.append(f"{side}: {answer or 'nothing clear'}")
            client.point_camera(0, 0)
            text = ("No target was named, so the camera described what it sees: " + "; ".join(seen)
                    + ". Do not drive: answer the user now, starting with DONE, with what you saw.")
            return text, client.take_picture()
        if target in task.reached:
            return (f"You already reached the {target}. Do not look for it again: look for"
                    f" the next target.{task.status_note()}", client.take_picture())
        if target != task.searched:
            task.misses = 0
        task.searched = target
        shots, seen = {}, []
        for pan in look_around_pans(car.max_pan_deg):
            client.point_camera(pan, 0)
            shots[pan] = client.take_picture()
            where = where_is(shots[pan], target, f"camera at pan {pan}")
            if where:
                seen.append((pan, where))
                if where == "middle":
                    break   # in the middle of a picture: good enough, stop looking
        client.point_camera(0, 0)
        looked = ", ".join(str(p) for p in shots)
        task.just_seen = target if seen else ""
        task.misses = 0 if seen else task.misses + 1
        if not seen:
            # Without an end the AI turned and looked for minutes (real car, 2026-11-17): after
            # a full circle (three looks, two turns of 120) open space, then give up
            text = f"Looked around for the {target} with the camera at pan {looked}: not seen."
            if task.misses < 3:
                text += " It may be behind the car: turn 120 and call look_around again."
            elif task.misses == 3:
                text += (f" You have now looked all the way around. Drive {metres(car.max_drive_m)} m"
                         " straight into open space, then call look_around again.")
            else:
                text += (f" It is not anywhere around the car: finish with DONE and tell the user you"
                         f" could not find the {target}.")
            return text + task.status_note(), shots[0]
        pan, where = min(seen, key=lambda s: s[1] in ("left", "right"))   # prefer the middle
        angle = pan + {"left": SIDE_OFFSET_DEG, "right": -SIDE_OFFSET_DEG}.get(where, 0)
        text = f"Looked around for the {target}: seen with the camera at pan {pan}, {where} of the picture"
        if abs(angle) < 10:
            text += ", about straight ahead. Call approach to drive to it."
        else:
            side = "left" if angle > 0 else "right"
            text += (f", about {abs(angle)} degrees to the {side} of the car. Call turn with"
                     f" angle_deg {angle} to face it, then call approach.")
        return text + task.status_note(), shots[pan]

    @tool(response_format="content_and_artifact", description=f"""Drive, then take a picture straight ahead.

distance_m: metres to drive, positive forward, negative backward, at most {metres(car.max_drive_m)} per call.
steering_deg: -{car.max_steering_deg:.0f} to {car.max_steering_deg:.0f}, positive steers left.""")
    def drive(distance_m: float, steering_deg: float = 0.0):
        distance = clamp(distance_m, -car.max_drive_m, car.max_drive_m)
        steering = clamp(steering_deg, -car.max_steering_deg, car.max_steering_deg)
        client.require_camera()
        before = picture_before(distance)
        driven, stop, blocked = move(distance, steering)
        if blocked and abs(driven) < MIN_MOVE_M:
            text = (f"Did not move: something is {task.blocked_m:.2f} m in front of the bumper,"
                    " so driving forward does nothing.")
            if task.tries > 1:
                text += f" You have tried {task.tries} times."
        else:
            text = drove(driven, steering, stop)
        if abs(distance_m) > car.max_drive_m:
            text += f" {metres(car.max_drive_m)} m is the most per call."
        return picture_with_notes(text, before if finished(driven, stop) else None)

    @tool(response_format="content_and_artifact")
    def turn(angle_deg: float):
        """Turn the car by an angle, then take a picture straight ahead.

        angle_deg: -120 to 120, positive turns left, negative turns right. The car drives
        forward at full steering while it turns, so it also moves forward and to that side.
        To face something you saw with the camera at pan P, call turn with angle_deg P.
        """
        # The AI picks the angle; this turns it into a drive at full steering, forward: nothing
        # sees behind the car (until 2026-11-17 big turns went half forward, half reversing).
        angle = clamp(angle_deg, -120.0, 120.0)
        distance = abs(angle) / car.turn_deg_per_m
        steering = math.copysign(car.max_steering_deg, angle)
        client.require_camera()
        before = picture_before(distance)
        driven, stop, blocked = move(distance, steering)
        rest = distance - abs(driven)
        if blocked and rest >= MIN_MOVE_M and stop != ESTOP:
            # Reversing with the steering the other way turns the same way; not far, as
            # nothing sees behind the car
            back, stop, _ = move(-min(rest, BACK_UP_M), -steering)
            turned = math.copysign((abs(driven) + abs(back)) * car.turn_deg_per_m, angle)
            how = ("Blocked in front, so I turned the rest while reversing." if rest <= BACK_UP_M else
                   f"Blocked in front, so I turned a little more while reversing ({BACK_UP_M:g} m: "
                   "nothing sees behind the car).")
            text = (f"{how} The heading changed about {turned:+.0f} degrees (positive = left)."
                    f" {stop}").strip()
        else:
            text = drove(driven, steering, stop)
            return picture_with_notes(text, before if finished(driven, stop) else None,
                                      turn_deg=math.copysign(abs(driven) * car.turn_deg_per_m, angle))
        return picture_with_notes(text)

    @tool(response_format="content_and_artifact")
    def approach(target: str):
        """Drive to a target you can see, until the safety system stops the car right in
        front of it. It drives in steps, looks after every step and steers at the target,
        so it does not drive past it. Then call reached.

        target: what to drive to, for example "blue cylinder".
        """
        target = target.strip().lower()
        if target in task.reached:
            return (f"You already reached the {target}. Do not drive to it again: look for the"
                    f" next target.{task.status_note()}", client.take_picture())
        if target != task.searched:
            task.approaches = 0
        task.searched = target
        task.approaches += 1
        if task.approaches > APPROACH_TRIES:
            return (f"You have tried to approach the {target} {APPROACH_TRIES} times without arriving. "
                    f"Do not call approach again: finish with DONE and tell the user how close you "
                    f"got.{task.status_note()}", client.take_picture())
        client.require_camera()
        client.look_ahead()
        steps, where, last, stop = [], "", "", ""
        last_row = None   # how far down the picture the target reached when last located
        filled_box = None  # the target's box when it filled the picture (x1, y1, x2, y2)
        close = False   # the camera has seen the target in the lower third or nearer
        deaf = False    # at the last straight step the distance sensor did not hear the target
        filled = False  # at the last straight step the target filled the picture's width, low down
        right_there = car.floor and floor_distance(car.floor, 1.0)   # the bottom edge, metres

        def target_row(picture, boxes=None):
            """How far down the picture the target reaches (1 = the bottom edge), None if the
            model does not locate it. Of several boxes, the one nearest the middle."""
            nonlocal filled, last_row, filled_box
            boxes = boxes if boxes is not None else locate(picture, LOCATE.format(target=target), "where exactly")
            if not boxes:
                filled, last_row = False, None
                return None
            _, x1, y1, x2, y2 = min(boxes, key=lambda b: abs((b[1] + b[3]) / 2 - 0.5))
            filled, last_row = x2 - x1 >= FILLS_VIEW and y2 >= CLOSE_ROW, y2
            filled_box = (x1, y1, x2, y2) if filled else None
            return y2

        def big_and_near(ahead):
            """The target filled the picture's width low down, and something is just beyond the
            safety stop: the car is at it, without asking the model (Ferenc's choice, 2026-11-24:
            faster, though a wrong arrival is more likely)."""
            return filled and ahead is not None and ahead <= car.stop_distance_m + RIGHT_IN_FRONT_M

        def close_up_of_first(picture, why):
            """True if the picture is a close-up of the target's first sighting (False without one)."""
            first = reference(target)
            return first is not None and says_yes(ask_about(
                both(first, picture), CLOSE_UP.format(target=target),
                f"approach, {why}, a close-up of the first sighting?"))

        def box_close_up_of_first(picture, box):
            """True if only the target's box, cut out of the picture, is a close-up of its first
            sighting: something lying in front of the target (a slipper at the bed) made the whole
            picture look different. On the real car's pictures the box said "yes" at every close
            view of the bed's drawer and "no" for 11 of 15 boxes around the slipper (2026-11-24)."""
            first = reference(target)
            return first is not None and box is not None and says_yes(ask_about(
                both(first, cut_out(picture, *box)), CLOSE_UP.format(target=target),
                "approach, fills the view, its box a close-up of the first sighting?"))

        def arrived_by_camera(picture):
            task.close_by_camera = target
            moves = ", ".join(steps) if steps else "did not move"
            return (f"Approach to the {target}: {moves}. The distance sensor does not hear the "
                    f"{target}, but the camera shows it right in front of the car."
                    + task.status_note()), picture

        def arrived_in_front(picture, ahead, compared=True, cluttered=False):
            task.close_by_camera = target
            task.approaches = 0
            moves = ", ".join(steps) if steps else "did not move"
            if cluttered:
                return (f"Approach to the {target}: {moves}. The camera shows the {target} filling the picture "
                        "right in front of the car, with something lying in front of it: this is as close as "
                        f"the car can get.{task.status_note()}"), picture
            like = ", like when you first saw it." if compared else ", filling the camera picture."
            if ahead is not None and ahead <= car.stop_distance_m + RIGHT_IN_FRONT_M:
                seen = (f"Something is {ahead:.2f} m in front of the bumper, and the camera shows it is "
                        f"the {target}{like}")
            else:   # the sensor does not hear it (it reads further, under or past the target)
                seen = f"The camera shows the {target} right in front of the car{like}"
            return f"Approach to the {target}: {moves}. {seen}{task.status_note()}", picture

        before = None   # the picture from before the last step, if that step finished
        for _ in range(APPROACH_MAX_STEPS):
            try:
                picture = client.take_picture()
            except CameraDown:   # say how far it got before the camera stopped
                return f"Approach to the {target}: {', '.join(steps) or 'did not move'}. {CAMERA_DOWN}", None
            moved_note(before, picture)   # did the last step move the car?
            if task.not_moving >= 2:
                return (f"Approach to the {target}: {', '.join(steps)}. "
                        f"{not_moving(task.not_moving)}{task.status_note()}"), picture
            # named once, at the first sighting: after that it follows the same thing. Right after
            # look_around saw and named it, or after losing it for a moment, a vague name will do.
            where = where_is(picture, target, "approach", confirm=last in ("", "lost"),
                             lenient=last == "lost" or task.just_seen == target)
            located = None
            if not where and last in ("left", "middle", "right"):
                # It was there a moment ago. Close up the model may not see it as a box any more
                # (real car, 2026-11-17): ask where exactly before giving it up.
                located = locate(picture, LOCATE.format(target=target), "where exactly")
                if located:
                    box = nearest_boxes(located, last)[0]
                    # The box must show the target (like its first sighting, or something vague): on
                    # the real car a plastic bag in front of the lens came back as the "white bed"
                    # (2026-11-20)
                    if is_target(picture, box, target, "approach", vague=True):
                        where, located = part_of(box), [box]
                    else:
                        located = None
            if not where and last == "middle":
                # Straight ahead a moment ago and gone, and something is just beyond the safety
                # stop: probably the target filling the picture. Like its first sighting: arrived.
                # (or it filled the picture's width a step ago: the bed in front of the real car)
                ahead = client.range_ahead()
                near = ahead is not None and ahead <= car.stop_distance_m + RIGHT_IN_FRONT_M
                if big_and_near(ahead):
                    return arrived_in_front(picture, ahead, compared=False)
                if (near or filled) and close_up_of_first(picture, "right in front"):
                    return arrived_in_front(picture, ahead)
            if not where and last == "middle":
                # Straight ahead a moment ago and gone: most likely below the picture, right in
                # front. Look down before backing up blind (the head tilts too).
                client.point_camera(0, LOOK_DOWN_DEG)
                try:
                    down = client.take_picture()
                except CameraDown:
                    down = None
                below = down is not None and where_is(down, target, "approach, looking down", confirm=False)
                if below and reference(target) is not None:   # the same thing as at the start?
                    below = confirm_by_box(down, target, "approach, looking down", below)
                client.look_ahead()
                if below:
                    ahead = client.range_ahead()
                    if ahead is None or ahead - car.stop_distance_m >= APPROACH_CLOSE_M:
                        return arrived_by_camera(down)   # right in front; the sensor does not hear it
                    driven, stop, _ = move(APPROACH_CLOSE_M, 0.0)   # heard: the safety stop ends it
                    steps.append(f"saw it below the camera, drove {driven:.2f} m")
                    before = None
                    if stop:
                        break
                    continue
            if not where:
                if last in ("", "lost"):
                    break   # not in the picture, or still not after backing up
                # It was there a moment ago, so now it is beside the car or below the camera:
                # back up, swinging the front towards the side it was on.
                swing = {"left": -car.max_steering_deg / 2, "right": car.max_steering_deg / 2}.get(last, 0.0)
                driven, stop, _ = move(-APPROACH_BACK_M, swing)
                steps.append(f"lost it, backed up {abs(driven):.2f} m")
                last, close, filled = "lost", False, False
            elif where == "middle":
                # Straight at it, as far as the distance sensor says is free
                ahead = client.range_ahead()
                longest = max(APPROACH_MIN_M, car.approach_max_m)   # then it looks again
                distance = (min(2 * APPROACH_MIN_M, longest) if ahead is None else
                            clamp(ahead - car.stop_distance_m + 0.05, APPROACH_MIN_M, longest))
                # and never past where the camera sees the target
                row = target_row(picture, located)
                # The target fills the picture's width, low down: the car is at it. Check before
                # driving on: the real car drove another 0.3 m into a slipper lying at the bed, and
                # close up its pictures no longer looked like the bed (2026-11-24)
                if big_and_near(ahead):
                    return arrived_in_front(picture, ahead, compared=False)
                if filled and close_up_of_first(picture, "fills the view"):
                    return arrived_in_front(picture, ahead)
                if filled and box_close_up_of_first(picture, filled_box):
                    return arrived_in_front(picture, ahead, cluttered=True)
                seen_at = floor_distance(car.floor, row) if row is not None and right_there else None
                # the sensor hears the target if it reads about where the camera sees it
                hears_it = ahead is not None and ahead < (
                    seen_at + HEAR_MARGIN_M if seen_at is not None else car.max_drive_m)
                if row is not None and row >= BOTTOM_ROW and not hears_it:
                    return arrived_by_camera(picture)   # right in front, but the sensor is deaf to it
                close, deaf = close or (row is not None and row >= CLOSE_ROW), not hears_it
                if seen_at is not None:
                    distance = min(distance, max(APPROACH_MIN_M, seen_at - right_there))
                if close and deaf:   # only the camera judges how close: small steps
                    distance = min(distance, APPROACH_CLOSE_M)
                driven, stop, _ = move(distance, 0.0)
                steps.append(f"drove {driven:.2f} m")
                last = where
            else:
                # It filled the picture's width a step ago: "left" or "right" says little now (the
                # real car corrected six times with its nose at the bed, 2026-11-24). At it?
                if filled:
                    ahead = client.range_ahead()
                    if big_and_near(ahead):
                        return arrived_in_front(picture, ahead, compared=False)
                    if close_up_of_first(picture, "fills the view"):
                        return arrived_in_front(picture, ahead)
                # On one side: turn towards it. If the last turn to this side did not bring it
                # to the middle, it is close and to the side, so turn while reversing instead.
                # Close to a target the sensor does not hear, always reverse, and turn half as
                # much: a forward arc at full steering pushes the front into the target.
                careful = close and deaf
                offset = SIDE_OFFSET_DEG / 2 if careful else SIDE_OFFSET_DEG
                angle = offset if where == "left" else -offset
                arc = abs(angle) / car.turn_deg_per_m
                steering = math.copysign(car.max_steering_deg, angle)
                backward = last == where or careful
                driven, stop, _ = move(-arc, -steering) if backward else move(arc, steering)
                steps.append(f"turned {where}" + (" reversing" if backward else ""))
                last = where
            before = picture if finished(driven, stop) else None
            if stop:
                break   # the safety stop, or another early stop
        task.just_seen = target if where else ""
        moves = ", ".join(steps) if steps else "did not move"
        if stop and task.no_echo and where == "middle":
            # The distance sensor cannot see what is in front (something soft?), so the safety
            # stop will never say "arrived": let the camera judge whether the target is right there.
            try:
                picture = client.take_picture()
            except CameraDown:
                return f"Approach to the {target}: {moves}. {stop} {CAMERA_DOWN}", None
            row = target_row(picture)
            if row is not None and row >= BOTTOM_ROW:
                return arrived_by_camera(picture)
        if stop:
            text = f"Approach to the {target}: {moves}. {stop}"
        elif task.approaches >= APPROACH_TRIES and where and last_row is not None and last_row >= CLOSE_ROW:
            # Close (the lower third of the picture) after three tries: count it as arrived
            task.close_by_camera, task.approaches = target, 0
            text = (f"Approach to the {target}: {moves}. After {APPROACH_TRIES} tries the {target} is close "
                    "in front of the car (down in the lower third of the picture): count it as arrived.")
        elif task.approaches >= APPROACH_TRIES:
            text = (f"Approach to the {target}: {moves}. You have tried to approach the {target} "
                    f"{APPROACH_TRIES} times without arriving: do not call approach again. Finish with "
                    "DONE and tell the user how close you got.")
        elif not where:
            text = (f"The {target} is not in the picture straight ahead ({moves}). Call look_around"
                    " to find it.")
        else:
            text = f"Approach to the {target}: {moves}. Not there yet: call approach again."
        return picture_with_notes(text)

    def forget_first_sightings(target):
        """A target reached: its first sighting is not needed any more; the last one: none are."""
        task.approaches = 0
        task.first_seen.pop(target, None)
        if not task.next_target:
            task.first_seen.clear()

    @tool
    def reached(target: str, next_target: str = ""):
        """Call this when the safety system has stopped the car right in front of one of
        the targets the user asked for, or when a tool says the camera shows it right in front.
        The car's sensors check that it is really there.

        target: what you have reached, for example "red box".
        next_target: the next target the user asked for, or "" if none is left.
        """
        target = target.strip().lower()
        if target in task.reached:
            return (f"You already reached the {target}. Do not drive to it again: back up, "
                    f"then call look_around for the next target.{task.progress_note()}")
        # Arrived means stopped right in front of it: the distance sensor decides, not a guess.
        # Only where the sensor gets no echo, the camera's close-up check in approach decides.
        if task.close_by_camera and target == task.close_by_camera:
            task.standing_at = target
            task.reached.append(target)
            task.next_target = next_target.strip().lower()
            forget_first_sightings(target)
            how = f"Noted: {target} (by the camera: the distance sensor could not see it)."
            if not task.next_target:
                return f"{how}{task.progress_note()} If that was the last target, finish with DONE."
            return (f"{how}{task.progress_note()} Back up {BACK_UP_M:g} m from the {target} (drive "
                    f"with distance_m {-BACK_UP_M:g}), then call look_around for the {task.next_target}.")
        if task.blocked_m is None:
            return (f"Not reached yet: the safety system has not stopped the car, so the {target} "
                    f"is not right in front of it. Call approach for the {target}: it drives until "
                    f"the safety system stops the car in front of it. Then call reached again. If "
                    f"you cannot see the {target}, call look_around.{task.progress_note()}")
        # Stopped in front of something else? Turned down once; asked again here, it counts.
        if target == task.target_now() and says_no(task.seen) and not task.doubted:
            task.doubted = True
            return (f"Not reached yet: the camera check says the thing in front of the car is not "
                    f"the {target}: \"{task.seen}\". Back up, then call look_around for the "
                    f"{target}.{task.progress_note()}")
        task.standing_at = target   # until the car moves away
        task.reached.append(target)
        task.next_target = next_target.strip().lower()
        forget_first_sightings(target)
        if not task.next_target:
            return f"Noted: {target}.{task.progress_note()} If that was the last target, finish with DONE."
        return (f"Noted: {target}.{task.progress_note()} Back up {BACK_UP_M:g} m from the {target} (drive "
                f"with distance_m {-BACK_UP_M:g}), then call look_around for the {task.next_target}.")

    @tool
    def remember(fact: str):
        """Save one short fact for later tasks, for example where you found something."""
        memory.append(fact)
        del memory[:-20]   # keep the newest 20
        save_memory(memory, memory_file)
        return "Saved."

    return [drive, turn, look, look_around, approach, reached, remember], start_task, memory
