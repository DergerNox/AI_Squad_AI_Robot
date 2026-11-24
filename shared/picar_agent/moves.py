"""moves.py: driving the car for the tools, and telling the AI what happened.

Moves drives through the car client (picar_base's SafetyGate checks every move), turns
picar_base's answer into words for the AI, and checks with the camera that the car really
moved (or turned) as far as it was told.
"""

from dataclasses import dataclass

from camera import CAMERA_DOWN, CameraDown
from picture_math import picture_change, turned_deg
from task_state import BACK_UP_M

MIN_MOVE_M = 0.05         # less than this counts as "did not move"

# What the safety system's stop reasons mean
STOP_REASONS = {
    "no_range": "the distance sensor had no fresh reading, just try again",
    "timeout": "the drive command arrived too late, just try again",
    "busy": "another move was still running",
}
# Told to the AI from the second move in a row without a distance reading. One is a passing
# blip (the twin's sensor misses 2 % of echoes); several mean the sensor cannot see what is in
# front: on the real car a soft blanket and a crumpled bag gave no echo at all (2026-11-14).
NO_ECHO = ("The distance sensor gets no echo from what is in front: probably something soft "
           "(cloth, foam, a crumpled bag) or a surface at an angle. The safety system does not let "
           "the car drive forward without a reading, so trying again does not help: turn towards "
           "something else, back up, or finish with DONE and tell the user.")
# Told to the AI when picar_base's emergency stop holds the car. Without "finish", the model
# kept trying approach and backing up for minutes (first run on the real car, 2026-11-14).
ESTOP = ("The emergency stop is on: the car cannot move at all until a person releases it. "
         "Do not call drive, turn or approach again: finish with DONE and tell the user that "
         "the emergency stop is on.")

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

# A turn the camera says hardly happened: the view turned less than this part of the asked angle
# (and less than TURN_MIN_DEG). On the real car, pressed against the bed and a plush toy, two turns
# of 120 did not move it, and the AI was told it had turned (2026-11-24).
TURN_PART = 0.25
TURN_MIN_DEG = 8.0
TURN_STUCK = (" The camera picture says the car turned only about {real:.0f} degrees, not {asked:.0f}: "
              "something low or soft may block it, which the safety system cannot see. Back up "
              "{back:g} m (drive with distance_m {neg:g}) before you turn again.")


def not_moving(n):
    """What the AI is told after n finished drives in a row that did not change the picture."""
    then = NOT_MOVING_TRY.format(back=BACK_UP_M, neg=-BACK_UP_M) if n == 2 else NOT_MOVING_END
    return NOT_MOVING.format(n=n) + then


def finished(driven, stop):
    """True if a drive went its whole way and was long enough to check with the camera."""
    return not stop and abs(driven) >= CHECK_MOVED_M - 1e-6


@dataclass
class Moves:
    """client: the CarClient; car: the car's numbers; task: the TaskState; eyes: Eyes;
    log: the PictureLog."""

    client: object
    car: object
    task: object
    eyes: object
    log: object

    def move(self, distance, steering):
        """Drive with a fixed steering angle. Returns how far it went, why it stopped
        early ("" if it drove the whole way), and whether something in front stopped it."""
        task = self.task
        driven, answer = self.client.move(distance, steering)
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

    def drove(self, driven, steering, stop):
        """A drive in words."""
        car = self.car
        turned = car.turn_deg_per_m * driven * steering / car.max_steering_deg
        return (f"Drove {driven:+.2f} m with steering {steering:+.0f}, the heading changed "
                f"about {turned:+.0f} degrees (positive = left). {stop}").strip()

    def picture_before(self, distance):
        """A picture straight ahead before a drive long enough to check, else None."""
        if abs(distance) < CHECK_MOVED_M:
            return None
        self.client.look_ahead()
        try:
            return self.client.take_picture()
        except CameraDown:
            return None

    def moved_note(self, before, after):
        """"" if the camera picture changed after a finished drive; else a warning, and from
        the second drive in a row that did not change it, NOT_MOVING."""
        if before is None or after is None:
            return ""
        if picture_change(before, after) >= STILL_CHANGE:
            self.task.not_moving = 0
            return ""
        self.task.not_moving += 1
        return f" {not_moving(self.task.not_moving)}" if self.task.not_moving >= 2 else STILL_ONCE

    def picture_with_notes(self, text, before=None, turn_deg=None):
        """Take the picture for a tool result. If the car has just got stuck, ask
        the camera what it is stuck at. before: the picture from before a finished drive,
        to check that the car really moved; turn_deg: how far that drive should have turned it."""
        task = self.task
        try:
            picture = self.client.take_picture()
        except CameraDown:   # the car has moved: still tell the AI what it did
            return f"{text} {CAMERA_DOWN}", None
        if task.blocked_m is not None and not task.seen:
            task.seen = self.eyes.check_in_front(picture)
        note = self.moved_note(before, picture)
        if turn_deg and before is not None and not note:
            real = turned_deg(before, picture, self.car.camera_fov_deg)
            if real is not None and abs(real) < max(TURN_MIN_DEG, TURN_PART * abs(turn_deg)):
                note = TURN_STUCK.format(real=abs(real), asked=abs(turn_deg), back=BACK_UP_M, neg=-BACK_UP_M)
        text = text + note + task.status_note()
        self.log.save(picture, "shown to the AI", answer=text)
        return text, picture
