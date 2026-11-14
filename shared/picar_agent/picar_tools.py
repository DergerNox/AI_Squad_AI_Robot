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

import json
import math
import os
import re
import time
from dataclasses import dataclass, field

from langchain_core.tools import tool
from rai.communication.ros2 import ROS2Connector
from rai.messages import HumanMultimodalMessage

from camera import CAMERA_DOWN, CameraDown
from car_client import CarClient, clamp
from car_config import load_car

# The car's numbers (speed, limits, size, camera) come from its settings file: car_config.py.
# These are how the tools work, the same for every car.
MIN_MOVE_M = 0.05         # less than this counts as "did not move"
# Next to agent.py, from whichever folder the agent is started
MEMORY_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "memory.json")

LOOK_AROUND_PANS = (0, 45, -45, 90, -90)   # 62 degree pictures: together about 240 degrees
SIDE_OFFSET_DEG = 20      # something on the left/right of a picture is about this far from its middle
K_TURN_DEG = 60           # turns this big go half forward, half reversing, to stay in place

APPROACH_MAX_STEPS = 12   # steps per approach call
APPROACH_MIN_M = 0.2      # shortest step straight at the target
APPROACH_BACK_M = 0.3     # how far to back up when the target slips out of the picture

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
COLOUR_WORDS = {"red", "blue", "green", "yellow", "white", "black", "orange", "purple", "pink",
                "brown", "grey", "gray"}
KINDS = {   # what may stand for the target's kind in the model's name for it
    "box": ("box", "cube", "carton", "crate", "package", "block"),
    "stick": ("stick", "pole", "rod", "bar", "pipe"),
    "cylinder": ("cylinder", "can", "tube", "pillar", "column"),
    "ball": ("ball", "sphere"),
}


def names_target(answer, target):
    """True if the model's name for a thing says it is the target's kind (box, stick, ...)."""
    kind = target.split()[-1] if target.split() else ""
    return any(word in answer for word in KINDS.get(kind, (kind,))) if kind else False


# Told to the AI from the second move in a row without a distance reading. One is a passing
# blip (the twin's sensor misses 2 % of echoes); several mean the sensor cannot see what is in
# front: on the real car a soft blanket and a crumpled bag gave no echo at all (2026-11-14).
NO_ECHO = ("The distance sensor gets no echo from what is in front: probably something soft "
           "(cloth, foam, a crumpled bag) or a surface at an angle. The safety system does not let "
           "the car drive forward without a reading, so trying again does not help: turn towards "
           "something else, back up, or finish with DONE and tell the user.")
# Asked when the sensor cannot see the target: is it close enough to count as arrived?
CLOSE_UP = ("This picture is from a small robot car's camera, looking straight ahead. Is there a "
            "{target} right in front of the car, so close that it reaches the bottom edge of the "
            "picture? Answer only 'yes' or 'no'.")

# Told to the AI when picar_base's emergency stop holds the car. Without "finish", the model
# kept trying approach and backing up for minutes (first run on the real car, 2026-11-14).
ESTOP = ("The emergency stop is on: the car cannot move at all until a person releases it. "
         "Do not call drive, turn or approach again: finish with DONE and tell the user that "
         "the emergency stop is on.")

# Asked about the picture when the safety system has stopped the car
IN_FRONT = ("This picture is from a small robot car's camera, looking straight ahead. "
            "The car has stopped because something is very close in front of it.")


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

    def new_task(self):
        """Forget the last task: its targets and where the car stopped."""
        self.reached.clear()
        self.next_target = self.searched = ""
        self.no_echo = 0
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
            note += f" You are still at the {self.standing_at}: back up and turn away from it."
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


def make_picar_tools(connector: ROS2Connector, vision=None, car=None, clock=None, memory_file=None):
    """Create the tools. They share one car client, which remembers where the camera points.

    vision: the chat model, used for short questions about pictures.
    car: the car's numbers (car_config.Car); None reads them from PICAR_CONFIG's file.
    clock: the time module (the default), or a stand-in for it in the tests.
    memory_file: where remember keeps its facts; None: MEMORY_FILE, next to agent.py.
    Returns the tools, the function that starts a new task, and the remembered facts.
    """
    car = car or load_car()
    client = CarClient(connector, car, clock or time)
    task = TaskState()
    memory_file = memory_file or MEMORY_FILE
    memory = load_memory(memory_file)   # facts kept between tasks and restarts

    def ask_about(picture, question, label="camera", limit=60):
        """One short question to the vision model about a picture, without the chat. The
        answer comes back in lower case, at most limit characters."""
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
        print(f"\n[{label}: {answer}]")
        return answer

    def check_in_front(picture):
        """Ask what the car has stopped in front of."""
        if task.target_now():
            question = (f"{IN_FRONT} Is it the {task.target_now()}? Answer 'yes' or 'no', then say "
                        "what it is in two or three words, colour first.")
        else:
            question = (f"{IN_FRONT} What is that thing? Answer with two or three words, "
                        "colour first, like 'green ball' or 'white wall'.")
        return ask_about(picture, question)

    def where_is(picture, target, label, confirm=True):
        """Where the vision model sees the target in a picture: "left", "middle",
        "right", or "" if it is not there. confirm: the model must also name the thing
        it sees as the target's kind (see NAME_IT)."""
        question = (f"Is there a {target} in this picture? Answer only 'no', or where it is "
                    "in the picture: 'left', 'middle' or 'right'.")
        words = re.findall(r"[a-z]+", ask_about(picture, question, label))
        if not words or words[0] in ("no", "not", "none", "nothing"):
            return ""
        where = next((w for w in ("left", "middle", "centre", "center", "right") if w in words), "")
        if where in ("centre", "center") or (not where and words[0] == "yes"):
            where = "middle"   # in the middle, or seen without saying where
        if confirm:
            first = target.split()[0] if target.split() else ""
            thing = f"{first} thing" if first in COLOUR_WORDS else "object"
            name = ask_about(picture, NAME_IT.format(thing=thing, where=where), f"{label}, named")
            if not names_target(name, target):
                return ""   # something else of the same colour
        return where

    def picture_with_notes(text):
        """Take the picture for a tool result. If the car has just got stuck, ask
        the camera what it is stuck at."""
        try:
            picture = client.take_picture()
        except CameraDown:   # the car has moved: still tell the AI what it did
            return f"{text} {CAMERA_DOWN}", None
        if task.blocked_m is not None and not task.seen:
            task.seen = check_in_front(picture)
        return text + task.status_note(), picture

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
        return text + task.status_note(), client.take_picture()

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
        task.searched = target
        pictures, seen = {}, []
        for pan in LOOK_AROUND_PANS:
            client.point_camera(pan, 0)
            pictures[pan] = client.take_picture()
            where = where_is(pictures[pan], target, f"camera at pan {pan}")
            if where:
                seen.append((pan, where))
                if where == "middle":
                    break   # in the middle of a picture: good enough, stop looking
        client.point_camera(0, 0)
        looked = ", ".join(str(p) for p in pictures)
        if not seen:
            text = (f"Looked around for the {target} with the camera at pan {looked}: not seen. It may"
                    " be behind the car: turn 120 and call look_around again.")
            return text + task.status_note(), pictures[0]
        pan, where = min(seen, key=lambda s: s[1] in ("left", "right"))   # prefer the middle
        angle = pan + {"left": SIDE_OFFSET_DEG, "right": -SIDE_OFFSET_DEG}.get(where, 0)
        text = f"Looked around for the {target}: seen with the camera at pan {pan}, {where} of the picture"
        if abs(angle) < 10:
            text += ", about straight ahead. Call approach to drive to it."
        else:
            side = "left" if angle > 0 else "right"
            text += (f", about {abs(angle)} degrees to the {side} of the car. Call turn with"
                     f" angle_deg {angle} to face it, then call approach.")
        return text + task.status_note(), pictures[pan]

    @tool(response_format="content_and_artifact", description=f"""Drive, then take a picture straight ahead.

distance_m: metres to drive, positive forward, negative backward, at most {car.max_drive_m:.1f} per call.
steering_deg: -{car.max_steering_deg:.0f} to {car.max_steering_deg:.0f}, positive steers left.""")
    def drive(distance_m: float, steering_deg: float = 0.0):
        distance = clamp(distance_m, -car.max_drive_m, car.max_drive_m)
        steering = clamp(steering_deg, -car.max_steering_deg, car.max_steering_deg)
        client.require_camera()
        driven, stop, blocked = move(distance, steering)
        if blocked and abs(driven) < MIN_MOVE_M:
            text = (f"Did not move: something is {task.blocked_m:.2f} m in front of the bumper,"
                    " so driving forward does nothing.")
            if task.tries > 1:
                text += f" You have tried {task.tries} times."
        else:
            text = drove(driven, steering, stop)
        if abs(distance_m) > car.max_drive_m:
            text += f" {car.max_drive_m:.1f} m is the most per call."
        return picture_with_notes(text)

    @tool(response_format="content_and_artifact")
    def turn(angle_deg: float):
        """Turn the car by an angle, then take a picture straight ahead.

        angle_deg: -120 to 120, positive turns left, negative turns right. Small turns
        drive forward; turns of 60 degrees or more go half forward and half reversing, so
        the car stays near where it is. To face something you saw with the camera at
        pan P, call turn with angle_deg P.
        """
        # The AI picks the angle; this turns it into drives at full steering.
        angle = clamp(angle_deg, -120.0, 120.0)
        distance = abs(angle) / car.turn_deg_per_m
        steering = math.copysign(car.max_steering_deg, angle)
        big = abs(angle) >= K_TURN_DEG
        client.require_camera()
        # A big turn in one forward arc moves the car about half a metre, and a close
        # target ends up beside it. Like a three-point turn, do half of it forward.
        driven, stop, blocked = move(distance / 2 if big else distance, steering)
        rest = distance - abs(driven)
        if (blocked or big) and rest >= MIN_MOVE_M and stop != ESTOP:
            # Reversing with the steering the other way turns the same way
            back, stop, _ = move(-rest, -steering)
            turned = math.copysign((abs(driven) + abs(back)) * car.turn_deg_per_m, angle)
            how = ("Blocked in front, so I turned the rest while reversing." if blocked else
                   "Turned half forward, half reversing.")
            text = (f"{how} The heading changed about {turned:+.0f} degrees (positive = left)."
                    f" {stop}").strip()
        else:
            text = drove(driven, steering, stop)
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
        task.searched = target
        client.require_camera()
        client.look_ahead()
        steps, where, last, stop = [], "", "", ""
        for _ in range(APPROACH_MAX_STEPS):
            try:
                picture = client.take_picture()
            except CameraDown:   # say how far it got before the camera stopped
                return f"Approach to the {target}: {', '.join(steps) or 'did not move'}. {CAMERA_DOWN}", None
            # named once, at the first sighting: after that it follows the same thing
            where = where_is(picture, target, "approach", confirm=last in ("", "lost"))
            if not where:
                if last in ("", "lost"):
                    break   # not in the picture, or still not after backing up
                # It was there a moment ago, so now it is beside the car or below the camera:
                # back up, swinging the front towards the side it was on.
                swing = {"left": -car.max_steering_deg / 2, "right": car.max_steering_deg / 2}.get(last, 0.0)
                driven, stop, _ = move(-APPROACH_BACK_M, swing)
                steps.append(f"lost it, backed up {abs(driven):.2f} m")
                last = "lost"
            elif where == "middle":
                # Straight at it, as far as the distance sensor says is free
                ahead = client.range_ahead()
                distance = (2 * APPROACH_MIN_M if ahead is None else
                            clamp(ahead - car.stop_distance_m + 0.05, APPROACH_MIN_M, car.max_drive_m))
                driven, stop, _ = move(distance, 0.0)
                steps.append(f"drove {driven:.2f} m")
                last = where
            else:
                # On one side: turn towards it. If the last turn to this side did not bring it
                # to the middle, it is close and to the side, so turn while reversing instead.
                angle = SIDE_OFFSET_DEG if where == "left" else -SIDE_OFFSET_DEG
                arc = abs(angle) / car.turn_deg_per_m
                steering = math.copysign(car.max_steering_deg, angle)
                backward = last == where
                driven, stop, _ = move(-arc, -steering) if backward else move(arc, steering)
                steps.append(f"turned {where}" + (" reversing" if backward else ""))
                last = where
            if stop:
                break   # the safety stop, or another early stop
        moves = ", ".join(steps) if steps else "did not move"
        if stop and task.no_echo and where == "middle":
            # The distance sensor cannot see what is in front (something soft?), so the safety
            # stop will never say "arrived": let the camera judge whether the target is right there.
            try:
                picture = client.take_picture()
            except CameraDown:
                return f"Approach to the {target}: {moves}. {stop} {CAMERA_DOWN}", None
            close = ask_about(picture, CLOSE_UP.format(target=target), "close up")
            if close.startswith("yes"):
                task.close_by_camera = target
                return (f"Approach to the {target}: {moves}. The distance sensor gets no echo here, "
                        f"but the camera shows the {target} right in front of the car."
                        + task.status_note()), picture
        if stop:
            text = f"Approach to the {target}: {moves}. {stop}"
        elif not where:
            text = (f"The {target} is not in the picture straight ahead ({moves}). Call look_around"
                    " to find it.")
        else:
            text = f"Approach to the {target}: {moves}. Not there yet: call approach again."
        return picture_with_notes(text)

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
            how = f"Noted: {target} (by the camera: the distance sensor could not see it)."
            if not task.next_target:
                return f"{how}{task.progress_note()} If that was the last target, finish with DONE."
            return (f"{how}{task.progress_note()} Back up from the {target}, then call "
                    f"look_around for the {task.next_target}.")
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
        if not task.next_target:
            return f"Noted: {target}.{task.progress_note()} If that was the last target, finish with DONE."
        return (f"Noted: {target}.{task.progress_note()} Back up from the {target}, then call "
                f"look_around for the {task.next_target}.")

    @tool
    def remember(fact: str):
        """Save one short fact for later tasks, for example where you found something."""
        memory.append(fact)
        del memory[:-20]   # keep the newest 20
        save_memory(memory, memory_file)
        return "Saved."

    return [drive, turn, look, look_around, approach, reached, remember], task.new_task, memory
