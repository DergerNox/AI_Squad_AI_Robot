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

The car's own sensors and camera have the last word: reached only counts a target when
the safety system has stopped the car in front of it, or approach has seen it right in
front with the camera; drive and turn do not move the car while the camera sends no new
pictures.

The parts the tools are made of:
  task_state.py    what the tools know about the task (TaskState)
  questions.py     what the vision model is asked, and how its answers are read
  picture_math.py  cut-outs, picture changes, turns and boxes, without the model
  eyes.py          the questions about pictures, and the first sighting of each target
  moves.py         driving, and telling the AI what happened
  approach.py      the approach tool's steps and its arrival rules
and car_client.py talks to the car, camera.py gets its pictures, car_config.py reads the
car's numbers, prompt.py tells the AI about the car, history.py keeps the conversation short.
"""

import json
import math
import os
import time

from langchain_core.tools import tool
from rai.communication.ros2 import ROS2Connector

from approach import (APPROACH_CLOSE_M, APPROACH_MAX_STEPS, LOOK_DOWN_DEG,  # noqa: F401 (the tests)
                      SIDE_OFFSET_DEG, Approach)
from car_client import CarClient, clamp
from car_config import load_car, metres
from eyes import Eyes
from moves import ESTOP, MIN_MOVE_M, Moves, finished
from picture_log import PictureLog
from picture_math import both, cut_out, floor_distance  # noqa: F401 (the tests)
from questions import (CLOSE_UP, DESCRIBE, IN_FRONT, LOCATE, NAME_CROP, NAME_IT,  # noqa: F401
                       SAME, boxes_in, names_target, no_target, says_no)
from task_state import BACK_UP_M, TaskState

# Next to agent.py, from whichever folder the agent is started
MEMORY_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "memory.json")
DESCRIBE_PANS = ((0, "ahead"), (45, "to the left"), (-45, "to the right"))


def look_around_pans(max_pan):
    """Where look_around points the camera: ahead, left, right, and further out where the head
    turns that far. The twin's head turns to 90 degrees (pictures together about 240 degrees);
    the real car's only to about 50: asked for 90 it stopped there, and look_around told the
    AI to turn about 40 degrees too far (2026-11-20)."""
    if max_pan >= 75:
        return (0, 45, -45, min(90, max_pan), -min(90, max_pan))
    return (0, max_pan, -max_pan)


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
    eyes = Eyes(vision, task, pictures)
    moves = Moves(client, car, task, eyes, pictures)
    move, drove, picture_before, picture_with_notes = (
        moves.move, moves.drove, moves.picture_before, moves.picture_with_notes)
    ask_about, where_is = eyes.ask, eyes.where_is

    def start_task(text=""):
        task.new_task()
        pictures.start(text)
    start_task.task = task   # what the tools know about the task, for the tests

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
        front of it or the camera shows it right in front. It drives in steps, looks after
        every step and steers at the target, so it does not drive past it. Then call reached.

        target: what to drive to, for example "blue cylinder".
        """
        return Approach(target.strip().lower(), client, car, task, eyes, moves).run()

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
        # Arrived means right in front of it: the safety stop says so, or approach's camera checks
        # (approach.py lists them). Not a guess of the AI's.
        if task.close_by_camera and target == task.close_by_camera:
            task.standing_at = target
            task.reached.append(target)
            task.next_target = next_target.strip().lower()
            forget_first_sightings(target)
            how = f"Noted: {target} (by the camera)."
            if not task.next_target:
                return f"{how}{task.progress_note()} If that was the last target, finish with DONE."
            return (f"{how}{task.progress_note()} Back up {BACK_UP_M:g} m from the {target} (drive "
                    f"with distance_m {-BACK_UP_M:g}), then call look_around for the {task.next_target}.")
        if task.blocked_m is None:
            return (f"Not reached yet: the safety system has not stopped the car, so the {target} "
                    f"is not right in front of it. Call approach for the {target}: it drives until "
                    f"the safety system stops the car in front of it, or until the camera shows it "
                    f"right in front. Then call reached again. If you cannot see the {target}, call "
                    f"look_around.{task.progress_note()}")
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
