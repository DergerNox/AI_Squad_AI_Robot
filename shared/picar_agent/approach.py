"""approach.py: the approach tool's work: drive to a target the camera sees, in checked steps,
and decide when the car has arrived.

Each step: a picture; where is the target (left, middle, right, gone)? In the middle: drive
straight at it, as far as the distance sensor and the camera allow (at most the car's
approach step); at a side: turn towards it; gone: look for it below the camera, else back up.

The car has arrived when (checked in this order, see the arrived_* methods):
  the safety stop stops it in front of something (reached then asks the camera what it is), or
  1. the target is gone from the middle and something is just beyond the safety stop, or it
     filled the picture a step ago: the target filled the picture and something is near, or the
     picture is a close-up of its first sighting (arrived_when_gone);
  2. it is gone from the middle, and seen below the camera while the sensor does not hear it
     (look_down);
  3. it is in the middle and fills the picture's width low down: something is near, or the
     picture is a close-up of its first sighting, or only the target's box (something lying in
     front of it) looks like it; or its box reaches the bottom edge while the sensor does not hear it
     (arrived_in_middle);
  4. it filled the picture a step ago and is now at a side: something is near, or the picture
     is a close-up of its first sighting (arrived_at_side);
  5. a move ended without a distance reading and its box reaches the bottom edge (finish);
  6. the third approach to the same target ends with it in the lower third (finish).
"""

import math

from camera import CAMERA_DOWN, CameraDown
from car_client import clamp
from moves import finished, not_moving
from picture_math import floor_distance, nearest_boxes, part_of
from questions import CLOSE_UP
from task_state import BACK_UP_M

SIDE_OFFSET_DEG = 20      # something on the left/right of a picture is about this far from its middle
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
# How far down the camera looks for a target that was straight ahead and has gone: at tilt 0
# the floor within about 0.4 m of the bumper is below the picture.
LOOK_DOWN_DEG = -30
BOTTOM_ROW = 0.95   # a box reaching this far down touches the bottom edge of the picture
CLOSE_ROW = 2 / 3   # the lower third: close (with tilt 0 the middle row is the horizon)
FILLS_VIEW = 0.9    # a target box this wide (of the picture's width), low down: the car is at it
# Driven straight at the target, it is suddenly gone and something is at most this far beyond the
# safety stop distance: probably the target itself, filling the picture. On the real car the bed's
# drawer filled the picture at about 0.27 m, just outside the 0.25 m stop, and the model said "no
# white bed" about it (2026-11-24). If the picture looks like the first sighting, it has arrived.
RIGHT_IN_FRONT_M = 0.1
# approach calls to one target without arriving: on the real car the AI looped approach, back up,
# approach for four minutes with a slipper lying in front of the bed (2026-11-24)
APPROACH_TRIES = 3


class Approach:
    """One approach call. target: in lower case; client, car, task, eyes, moves: the tools' parts."""

    def __init__(self, target, client, car, task, eyes, moves):
        self.target, self.client, self.car, self.task, self.eyes, self.moves = (
            target, client, car, task, eyes, moves)
        self.steps = []          # what each step did, in words
        self.where = ""          # where the target is in the newest picture ("" not seen)
        self.last = ""           # where it was at the step before ("lost": backed up after losing it)
        self.stop = ""           # why the last move stopped early ("" it drove the whole way)
        self.before = None       # the picture from before the last step, if that step finished
        self.close = False       # the camera has seen the target in the lower third or nearer
        self.deaf = False        # at the last straight step the distance sensor did not hear it
        self.filled = False      # at the last straight step it filled the picture's width, low down
        self.filled_box = None   # its box then (x1, y1, x2, y2)
        self.last_row = None     # how far down the picture it reached when last located
        self.right_there = car.floor and floor_distance(car.floor, 1.0)   # the bottom edge, metres

    # ---------- the call ----------

    def run(self):
        """The tool's result: (text, picture)."""
        task, target = self.task, self.target
        if target in task.reached:
            return (f"You already reached the {target}. Do not drive to it again: look for the"
                    f" next target.{task.status_note()}", self.client.take_picture())
        if target != task.searched:
            task.approaches = 0
        task.searched = target
        task.approaches += 1
        if task.approaches > APPROACH_TRIES:
            return (f"You have tried to approach the {target} {APPROACH_TRIES} times without arriving. "
                    f"Do not call approach again: finish with DONE and tell the user how close you "
                    f"got.{task.status_note()}", self.client.take_picture())
        self.client.require_camera()
        self.client.look_ahead()
        for _ in range(APPROACH_MAX_STEPS):
            result = self.step()
            if isinstance(result, tuple):   # arrived, or cannot go on
                return result
            if result == "stop":
                break
        return self.finish()

    def step(self):
        """One look and one move: "next", "stop" (end the call), or the call's result."""
        task, target, client, eyes = self.task, self.target, self.client, self.eyes
        try:
            picture = client.take_picture()
        except CameraDown:   # say how far it got before the camera stopped
            return f"Approach to the {target}: {', '.join(self.steps) or 'did not move'}. {CAMERA_DOWN}", None
        self.moves.moved_note(self.before, picture)   # did the last step move the car?
        if task.not_moving >= 2:
            return (f"Approach to the {target}: {', '.join(self.steps)}. "
                    f"{not_moving(task.not_moving)}{task.status_note()}"), picture
        # named once, at the first sighting: after that it follows the same thing. Right after
        # look_around saw and named it, or after losing it for a moment, a vague name will do.
        self.where = eyes.where_is(picture, target, "approach", confirm=self.last in ("", "lost"),
                                   lenient=self.last == "lost" or task.just_seen == target)
        located = None
        if not self.where and self.last in ("left", "middle", "right"):
            # It was there a moment ago. Close up the model may not see it as a box any more
            # (real car, 2026-11-17): ask where exactly before giving it up.
            located = eyes.locate(picture, target, "where exactly")
            if located:
                box = nearest_boxes(located, self.last)[0]
                # The box must show the target (like its first sighting, or something vague): on
                # the real car a plastic bag in front of the lens came back as the "white bed"
                # (2026-11-20)
                if eyes.is_target(picture, box, target, "approach", vague=True):
                    self.where, located = part_of(box), [box]
                else:
                    located = None
        if not self.where and self.last == "middle":
            arrived = self.arrived_when_gone(picture)
            if arrived:
                return arrived
            result = self.look_down()
            if result is not None:
                return result
        if not self.where:
            if self.last in ("", "lost"):
                return "stop"   # not in the picture, or still not after backing up
            return self.back_up(picture)
        if self.where == "middle":
            return self.straight(picture, located)
        return self.to_the_side(picture)

    # ---------- the moves ----------

    def back_up(self, picture):
        """It was there a moment ago, so now it is beside the car or below the camera: back up,
        swinging the front towards the side it was on."""
        car = self.car
        swing = {"left": -car.max_steering_deg / 2, "right": car.max_steering_deg / 2}.get(self.last, 0.0)
        driven, self.stop, _ = self.moves.move(-APPROACH_BACK_M, swing)
        self.steps.append(f"lost it, backed up {abs(driven):.2f} m")
        self.last, self.close, self.filled = "lost", False, False
        return self.after_move(picture, driven)

    def straight(self, picture, located):
        """In the middle: straight at it, as far as the distance sensor says is free, and never
        past where the camera sees it."""
        car = self.car
        ahead = self.client.range_ahead()
        longest = max(APPROACH_MIN_M, car.approach_max_m)   # then it looks again
        distance = (min(2 * APPROACH_MIN_M, longest) if ahead is None else
                    clamp(ahead - car.stop_distance_m + 0.05, APPROACH_MIN_M, longest))
        row = self.target_row(picture, located)
        seen_at = floor_distance(car.floor, row) if row is not None and self.right_there else None
        # the sensor hears the target if it reads about where the camera sees it
        hears_it = ahead is not None and ahead < (
            seen_at + HEAR_MARGIN_M if seen_at is not None else car.max_drive_m)
        arrived = self.arrived_in_middle(picture, ahead, row, hears_it)
        if arrived:
            return arrived
        self.close, self.deaf = self.close or (row is not None and row >= CLOSE_ROW), not hears_it
        if seen_at is not None:
            distance = min(distance, max(APPROACH_MIN_M, seen_at - self.right_there))
        if self.close and self.deaf:   # only the camera judges how close: small steps
            distance = min(distance, APPROACH_CLOSE_M)
        driven, self.stop, _ = self.moves.move(distance, 0.0)
        self.steps.append(f"drove {driven:.2f} m")
        self.last = self.where
        return self.after_move(picture, driven)

    def to_the_side(self, picture):
        """On one side: turn towards it. If the last turn to this side did not bring it to the
        middle, it is close and to the side, so turn while reversing instead. Close to a target
        the sensor does not hear, always reverse, and turn half as much: a forward arc at full
        steering pushes the front into the target."""
        arrived = self.arrived_at_side(picture)
        if arrived:
            return arrived
        car, where = self.car, self.where
        careful = self.close and self.deaf
        offset = SIDE_OFFSET_DEG / 2 if careful else SIDE_OFFSET_DEG
        angle = offset if where == "left" else -offset
        arc = abs(angle) / car.turn_deg_per_m
        steering = math.copysign(car.max_steering_deg, angle)
        backward = self.last == where or careful
        if backward:
            driven, self.stop, _ = self.moves.move(-arc, -steering)
        else:
            driven, self.stop, _ = self.moves.move(arc, steering)
        self.steps.append(f"turned {where}" + (" reversing" if backward else ""))
        self.last = where
        return self.after_move(picture, driven)

    def look_down(self):
        """Straight ahead a moment ago and gone: most likely below the picture, right in front.
        Look down before backing up blind (the head tilts too). None: not seen below."""
        client, eyes, target, car = self.client, self.eyes, self.target, self.car
        client.point_camera(0, LOOK_DOWN_DEG)
        try:
            down = client.take_picture()
        except CameraDown:
            down = None
        below = down is not None and eyes.where_is(down, target, "approach, looking down", confirm=False)
        if below and eyes.reference(target) is not None:   # the same thing as at the start?
            below = eyes.confirm_by_box(down, target, "approach, looking down", below)
        client.look_ahead()
        if not below:
            return None
        ahead = client.range_ahead()
        if ahead is None or ahead - car.stop_distance_m >= APPROACH_CLOSE_M:
            return self.arrived_by_camera(down)   # right in front; the sensor does not hear it
        driven, self.stop, _ = self.moves.move(APPROACH_CLOSE_M, 0.0)   # heard: the safety stop ends it
        self.steps.append(f"saw it below the camera, drove {driven:.2f} m")
        self.before = None
        return "stop" if self.stop else "next"

    def after_move(self, picture, driven):
        """After a step's move: keep its picture to check that the car moved; stop early?"""
        self.before = picture if finished(driven, self.stop) else None
        return "stop" if self.stop else "next"

    # ---------- where the target is ----------

    def target_row(self, picture, boxes=None):
        """How far down the picture the target reaches (1 = the bottom edge), None if the
        model does not locate it. Of several boxes, the one nearest the middle. Also notes
        whether it fills the picture's width, low down."""
        boxes = boxes if boxes is not None else self.eyes.locate(picture, self.target, "where exactly")
        if not boxes:
            self.filled, self.last_row = False, None
            return None
        _, x1, y1, x2, y2 = min(boxes, key=lambda b: abs((b[1] + b[3]) / 2 - 0.5))
        self.filled, self.last_row = x2 - x1 >= FILLS_VIEW and y2 >= CLOSE_ROW, y2
        self.filled_box = (x1, y1, x2, y2) if self.filled else None
        return y2

    def near(self, ahead):
        """Something is just beyond the safety stop distance."""
        return ahead is not None and ahead <= self.car.stop_distance_m + RIGHT_IN_FRONT_M

    def big_and_near(self, ahead):
        """The target filled the picture's width low down, and something is just beyond the
        safety stop: the car is at it, without asking the model (Ferenc's choice, 2026-11-24:
        faster, though a wrong arrival is more likely)."""
        return self.filled and self.near(ahead)

    def close_up_of_first(self, picture, why):
        """True if the picture is a close-up of the target's first sighting (False without one)."""
        return self.eyes.close_up(picture, self.target, CLOSE_UP,
                                  f"approach, {why}, a close-up of the first sighting?")

    def box_like_first(self, picture, box):
        """True if only the target's box, cut out of the picture, shows the target (eyes.is_target:
        like its first sighting): something lying in front of the target (a slipper at the bed)
        made the whole picture look different. On run 4's 34 boxes (real car, 2026-11-24) the
        cut-outs compared with the first sighting were right 30 times."""
        if box is None:
            return False
        return self.eyes.is_target(picture, ("",) + tuple(box), self.target, "approach, fills the view, its box")

    # ---------- arrived? (the rules, in the order of the module's docstring) ----------

    def arrived_when_gone(self, picture):
        """1. Gone from the middle, and something just beyond the safety stop or it filled the
        picture a step ago (the bed in front of the real car)."""
        ahead = self.client.range_ahead()
        if self.big_and_near(ahead):
            return self.arrived_in_front(picture, ahead, compared=False)
        if (self.near(ahead) or self.filled) and self.close_up_of_first(picture, "right in front"):
            return self.arrived_in_front(picture, ahead)
        return None

    def arrived_in_middle(self, picture, ahead, row, hears_it):
        """3. In the middle. The target fills the picture's width, low down: the car is at it.
        Checked before driving on: the real car drove another 0.3 m into a slipper lying at the
        bed, and close up its pictures no longer looked like the bed (2026-11-24)."""
        if self.big_and_near(ahead):
            return self.arrived_in_front(picture, ahead, compared=False)
        if self.filled and self.close_up_of_first(picture, "fills the view"):
            return self.arrived_in_front(picture, ahead)
        if self.filled and self.box_like_first(picture, self.filled_box):
            return self.arrived_in_front(picture, ahead, cluttered=True)
        if row is not None and row >= BOTTOM_ROW and not hears_it:
            return self.arrived_by_camera(picture)   # right in front, but the sensor is deaf to it
        return None

    def arrived_at_side(self, picture):
        """4. It filled the picture's width a step ago: "left" or "right" says little now (the
        real car corrected six times with its nose at the bed, 2026-11-24)."""
        if not self.filled:
            return None
        ahead = self.client.range_ahead()
        if self.big_and_near(ahead):
            return self.arrived_in_front(picture, ahead, compared=False)
        if self.close_up_of_first(picture, "fills the view"):
            return self.arrived_in_front(picture, ahead)
        return None

    # ---------- the result ----------

    def moves_text(self):
        return ", ".join(self.steps) if self.steps else "did not move"

    def arrived_by_camera(self, picture):
        task, target = self.task, self.target
        task.close_by_camera = target
        return (f"Approach to the {target}: {self.moves_text()}. The distance sensor does not hear the "
                f"{target}, but the camera shows it right in front of the car."
                + task.status_note()), picture

    def arrived_in_front(self, picture, ahead, compared=True, cluttered=False):
        task, target = self.task, self.target
        task.close_by_camera = target
        task.approaches = 0
        moves = self.moves_text()
        if cluttered:
            return (f"Approach to the {target}: {moves}. The camera shows the {target} filling the picture "
                    "right in front of the car, with something lying in front of it: this is as close as "
                    f"the car can get.{task.status_note()}"), picture
        like = ", like when you first saw it." if compared else ", filling the camera picture."
        if self.near(ahead):
            seen = (f"Something is {ahead:.2f} m in front of the bumper, and the camera shows it is "
                    f"the {target}{like}")
        else:   # the sensor does not hear it (it reads further, under or past the target)
            seen = f"The camera shows the {target} right in front of the car{like}"
        return f"Approach to the {target}: {moves}. {seen}{task.status_note()}", picture

    def finish(self):
        """The call's result after its steps."""
        task, target, where, stop = self.task, self.target, self.where, self.stop
        task.just_seen = target if where else ""
        moves = self.moves_text()
        if stop and task.no_echo and where == "middle":
            # 5. The distance sensor cannot see what is in front (something soft?), so the safety
            # stop will never say "arrived": let the camera judge whether the target is right there.
            try:
                picture = self.client.take_picture()
            except CameraDown:
                return f"Approach to the {target}: {moves}. {stop} {CAMERA_DOWN}", None
            row = self.target_row(picture)
            if row is not None and row >= BOTTOM_ROW:
                return self.arrived_by_camera(picture)
        if stop:
            text = f"Approach to the {target}: {moves}. {stop}"
        elif (task.approaches >= APPROACH_TRIES and where and self.last_row is not None
              and self.last_row >= CLOSE_ROW):
            # 6. Close (the lower third of the picture) after three tries: count it as arrived
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
        return self.moves.picture_with_notes(text)
