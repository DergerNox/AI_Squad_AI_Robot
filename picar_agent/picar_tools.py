"""picar_tools.py: what the AI can do with the PiCar, and what it knows about it.

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
"""

import json
import math
import re
import time

import numpy as np
from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ToolMessage
from langchain_core.tools import tool
from rai.communication.ros2 import ROS2Connector, ROS2Message
from rai.messages import HumanMultimodalMessage, MultimodalArtifact, preprocess_image
from rclpy.qos import qos_profile_sensor_data

# The car. The first three must match picar_base/config/sim.yaml.
SPEED_MPS = 0.2           # full_speed_mps 0.5 x max_speed 0.4
MAX_MOVE_S = 3.0          # max_move_s
MAX_STEERING_DEG = 30.0   # max_steering_deg
MAX_PAN_DEG = 90.0
MAX_TILT_DEG = 40.0
TURN_DEG_PER_M = 200.0    # heading change per metre at full steering (measured in the twin)
HEAD_DEG_PER_S = 150.0    # how fast the camera head turns (measured in the twin)
MAX_DRIVE_M = SPEED_MPS * MAX_MOVE_S
MIN_MOVE_M = 0.05         # less than this counts as "did not move"
STOP_DISTANCE_M = 0.25    # obstacle_stop_m: the safety system stops the car this close
CAMERA_TOPIC = "/camera/image_raw"
CAMERA_WAIT_S = 5.0       # how long to wait for a new camera picture before giving up
RANGE_TOPIC = "/ultrasonic/range"
MEMORY_FILE = "memory.json"

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
    "estop": "the emergency stop is on",
    "busy": "another move was still running",
}

# Told to the AI when the camera sends no new pictures (camera node stopped, bad connection)
CAMERA_DOWN = (f"The camera has sent no new picture for {CAMERA_WAIT_S:.0f} seconds, so you cannot "
               "see. Do not drive or turn: finish with DONE and tell the user that the camera is "
               "not working.")

# Asked about the picture when the safety system has stopped the car
IN_FRONT = ("This picture is from a small robot car's camera, looking straight ahead. "
            "The car has stopped because something is very close in front of it.")

PICAR_PROMPT = f"""You drive a small robot car, a simulated Adeept PiCar-B in a room in Gazebo.
You can only control it with your tools: drive moves the car, turn turns the car by an
angle, look turns the camera, look_around searches for a target with the camera, and
approach drives to a target you can see. Each of them shows you a new camera picture.
reached notes that you have arrived at a target, and remember saves a fact for later tasks.

The car:
- 22 cm long and 10 cm wide, rear-wheel drive with front-wheel steering like a real car.
  It cannot turn on the spot: to change direction it has to drive while steering.
- It drives at {SPEED_MPS} m/s, at most {MAX_DRIVE_M:.1f} m per drive call. A negative distance reverses.
- Steering goes from -{MAX_STEERING_DEG:.0f} to +{MAX_STEERING_DEG:.0f} degrees, positive steers left.
  At full steering the car turns about {TURN_DEG_PER_M / 10:.0f} degrees for every 0.1 m it drives,
  so a 90 degree turn takes about {90 / TURN_DEG_PER_M:.2f} m and moves the car about 0.3 m forward
  and 0.3 m to the side. Half the steering turns half as much. Reversing with left steering
  turns the front of the car to the right.
- A safety system stops the car when something is 0.25 m or closer in front of the bumper.
  Reversing is always allowed.

The camera:
- It sits on a pan/tilt head at the front of the car, 17 cm above the floor. Pan goes from
  -{MAX_PAN_DEG:.0f} to {MAX_PAN_DEG:.0f} degrees (positive = left), tilt from -{MAX_TILT_DEG:.0f} to {MAX_TILT_DEG:.0f}
  (positive = up). drive points the camera straight ahead first.
- The picture is about 62 degrees wide. Something at the left or right edge is 31 degrees to
  that side of where the camera points, halfway between the middle and the edge about 16 degrees.
- With tilt 0 the middle row of the picture is the horizon, and how far down the picture an
  object touches the floor tells its distance from the bumper: at the bottom edge 0.35 m,
  90% of the way down 0.45 m, 80% 0.6 m, 75% 0.75 m, 70% 0.9 m, 65% 1.25 m, 60% 1.9 m.
  The floor closer than 0.35 m is below the picture; tilt down (-30) to see it.
- Looking down or to the side you may see parts of your own car: black wheels, green body.

How to drive:
- Start every task with look(pan_deg=0, tilt_deg=0) to see what is straight ahead.
- drive and turn show you a new picture straight ahead after they move.
- Whenever your target is not in the picture, call look_around with its name. It looks to
  the left, ahead and to the right and tells you how far to turn to face the target.
  Then call turn with that angle, then approach.
- If look_around does not find it, turn 120 degrees and call look_around again. If it still
  does not find it, drive {MAX_DRIVE_M:.1f} m into open space and call look_around again.
- To reach a target you can see, call approach with its name. It drives at the target in
  steps, looks after every step and steers at it, until the safety system stops the car right
  in front of it: then you have arrived. Call reached with its name and the name of the next
  target ("" if it was the last one). reached tells you what to do next. If approach loses
  the target, call look_around.
- If the safety system stops the car in front of something else, back up or turn: driving
  forward again will not move the car.
- If the user asks for several targets, go to them in order. Never drive to a target you
  have already reached. Every picture says what you have reached and which target is next.
- Use look only when you want to point the camera somewhere yourself.
- Every message must either call a tool or be your final answer. Never just say what you
  will do next: do it in the same message.
- When you learn something useful for later, like where you found something, save it with
  remember, in one short sentence.
- When the task is finished, or you are sure it cannot be done, start your final answer with DONE.
  With several targets, say DONE only after you have reached all of them.
- Keep your messages short and plain, without markdown: say what you see and what you do.
  Answer in the language the user writes in.
"""


class CameraDown(Exception):
    """No new picture from the camera."""


def clamp(value, low, high):
    return max(low, min(high, float(value)))


def says_no(answer):
    """True if an answer to a yes/no question starts with no."""
    words = re.findall(r"[a-z]+", answer.lower())
    return bool(words) and words[0] in ("no", "not")


def is_request(message):
    """True for a task from the user (not a picture, not the agent's nudge)."""
    return type(message) is HumanMessage and message.name != "nudge"


def caption_picture(messages):
    """Write the newest picture's tool result next to the picture itself.

    RAI sends each picture as a separate message after the tool result. The model
    reads that message last and pays the most attention to it, so the notes
    (stuck, reached, next target) go there instead of "Image returned by a tool call".
    """
    if not messages or not isinstance(messages[-1], HumanMultimodalMessage):
        return   # the newest message is not a picture: the model reads that one last anyway
    i = len(messages) - 1
    picture = messages[i]
    results = [m for m in messages[:i] if isinstance(m, ToolMessage)]
    same_call = [m for m in results if m.tool_call_id == getattr(picture, "tool_call_id", None)]
    result = (same_call or results or [None])[-1]
    if result is None:
        return
    text = result.content
    if isinstance(text, list):   # RAI stores it as a list of parts: [{"type": "text", "text": ...}]
        text = " ".join(part.get("text", "") for part in text if isinstance(part, dict))
    messages[i] = HumanMultimodalMessage(content=f"New camera picture. {text}", images=picture.images)


def compress_history(messages, old_tasks=5, steps=8, pictures=1):
    """Return a shorter copy of the conversation, so it fits the model's context.

    Finished tasks keep only the user's request and the final answer. The task
    in progress keeps its last `steps` tool calls and its newest `pictures`
    pictures. Only the last `old_tasks` finished tasks are kept.
    """
    requests = [i for i, m in enumerate(messages) if is_request(m)]
    if not requests:
        return list(messages)
    system = [m for m in messages[:1] if isinstance(m, SystemMessage)]

    # Finished tasks: request and final answer (an AI message without tool calls)
    old = [m for m in messages[:requests[-1]]
           if is_request(m) or (isinstance(m, AIMessage) and not m.tool_calls)]
    old_requests = [i for i, m in enumerate(old) if is_request(m)]
    if len(old_requests) > old_tasks:
        old = old[old_requests[-old_tasks]:]

    # Task in progress: cut at a tool call so every tool result stays after its call
    now = list(messages[requests[-1]:])
    calls = [i for i, m in enumerate(now) if isinstance(m, AIMessage) and m.tool_calls]
    if len(calls) > steps:
        now = now[:1] + now[calls[-steps]:]
    shots = [i for i, m in enumerate(now) if isinstance(m, HumanMultimodalMessage)]
    for i in reversed(shots[:-pictures]):
        del now[i]
    caption_picture(now)

    return system + old + now


def load_memory():
    """The facts the AI saved with remember, from memory.json."""
    try:
        with open(MEMORY_FILE, encoding="utf-8") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return []


def save_memory(memory):
    with open(MEMORY_FILE, "w", encoding="utf-8") as f:
        json.dump(memory, f, indent=2, ensure_ascii=False)


def make_picar_tools(connector: ROS2Connector, vision=None):
    """Create the tools. They share one connector and remember where the camera points.

    vision: the chat model, used for short questions about pictures.
    """
    head = {"pan": None, "tilt": None}   # unknown until the first command
    progress = []                        # targets reached in the current task
    goal = {"next": "", "searched": ""}  # the next target from reached, the last look_around target
    # Where the car is stopped: blocked_m is set while something stops it in front,
    # seen is what the camera says that is, reached names the target it is still
    # standing at, doubted is set once reached has been turned down here because
    # of the camera. All of them clear when the car moves.
    spot = {"blocked_m": None, "seen": "", "reached": "", "tries": 0, "doubted": False}
    still = {"since": 0.0}               # when the car and the camera head last stopped moving
    memory = load_memory()               # facts kept between tasks and restarts

    def leave_spot():
        spot.update(blocked_m=None, seen="", reached="", tries=0, doubted=False)

    def new_task():
        progress.clear()
        goal.update(next="", searched="")
        leave_spot()

    def target_now():
        """The target the AI is going for, as far as the tools know."""
        return goal["next"] or goal["searched"]

    def progress_note():
        note = f" Reached so far: {', '.join(progress) if progress else 'nothing yet'}."
        if goal["next"]:
            note += f" Next target: {goal['next']}."
        return note

    def status_note():
        """Added to every picture: progress, and what to do if the car is stuck."""
        note = progress_note()
        if spot["reached"]:
            note += f" You are still at the {spot['reached']}: back up and turn away from it."
        elif spot["blocked_m"] is not None:
            target = f"the {target_now()}" if target_now() else "your target"
            note += " The car is stopped right in front of something and cannot drive forward."
            if spot["seen"]:
                note += f' The camera check says: "{spot["seen"]}".'
            note += f" If that is {target}, you have arrived: call reached now. If not, back up or turn."
        return note

    def newest_frame(newer_than):
        """The newest camera frame that arrived after newer_than (a time.time() value).

        RAI keeps the last frame and hands it out again even after the camera has
        stopped sending, so the arrival time is checked here."""
        deadline = time.time() + CAMERA_WAIT_S
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
            if time.time() > deadline:
                print(f"\n[camera: no new picture for {CAMERA_WAIT_S:.0f} s, check it with: "
                      f"ros2 topic hz {CAMERA_TOPIC}]")
                raise CameraDown(CAMERA_DOWN)
            time.sleep(0.05)

    def require_camera():
        """Never drive blind: the camera must have sent a picture in the last second."""
        try:
            newest_frame(time.time() - 1.0)
        except CameraDown:
            raise CameraDown(f"The car did not move. {CAMERA_DOWN}") from None

    def take_picture():
        """A camera picture taken after the car and the camera head stopped moving."""
        image = newest_frame(still["since"])
        pixels = np.frombuffer(bytes(image.data), np.uint8)
        pixels = pixels.reshape(image.height, image.width, 3)
        if image.encoding == "bgr8":
            pixels = pixels[:, :, ::-1]
        return MultimodalArtifact(images=[preprocess_image(pixels)], audios=[])

    def ask_about(picture, question, label="camera"):
        """One short question to the vision model about a picture, without the chat."""
        if vision is None:
            return ""
        message = HumanMultimodalMessage(content=question, images=picture["images"])
        try:
            # No callbacks: the answer is not streamed to the terminal or /to_human
            answer = vision.invoke([message], config={"callbacks": []}).content
        except Exception as error:
            print(f"\n[camera question failed: {error}]")
            return ""
        answer = " ".join(str(answer).strip(" .'\"").lower().split())[:60]
        print(f"\n[{label}: {answer}]")
        return answer

    def check_in_front(picture):
        """Ask what the car has stopped in front of."""
        if target_now():
            question = (f"{IN_FRONT} Is it the {target_now()}? Answer 'yes' or 'no', then say "
                        "what it is in two or three words, colour first.")
        else:
            question = (f"{IN_FRONT} What is that thing? Answer with two or three words, "
                        "colour first, like 'green ball' or 'white wall'.")
        return ask_about(picture, question)

    def where_is(picture, target, label):
        """Where the vision model sees the target in a picture: "left", "middle",
        "right", or "" if it is not there."""
        question = (f"Is there a {target} in this picture? Answer only 'no', or where it is "
                    "in the picture: 'left', 'middle' or 'right'.")
        words = re.findall(r"[a-z]+", ask_about(picture, question, label))
        if not words or words[0] in ("no", "not", "none", "nothing"):
            return ""
        where = next((w for w in ("left", "middle", "centre", "center", "right") if w in words), "")
        if where in ("centre", "center") or (not where and words[0] == "yes"):
            where = "middle"   # in the middle, or seen without saying where
        return where

    def range_ahead():
        """The distance sensor's reading in metres: inf if nothing is in range, None if no reading."""
        try:
            reading = connector.receive_message(
                RANGE_TOPIC, timeout_sec=1.0, msg_type="sensor_msgs/msg/Range",
                qos_profile=qos_profile_sensor_data, auto_qos_matching=False).payload
        except TimeoutError:
            return None
        if math.isnan(reading.range):
            return None
        if reading.range == -math.inf:   # closer than the sensor can measure
            return 0.0
        if math.isinf(reading.range) or reading.range >= reading.max_range:
            return math.inf
        return reading.range

    def picture_with_notes(text):
        """Take the picture for a tool result. If the car has just got stuck, ask
        the camera what it is stuck at."""
        try:
            picture = take_picture()
        except CameraDown:   # the car has moved: still tell the AI what it did
            return f"{text} {CAMERA_DOWN}", None
        if spot["blocked_m"] is not None and not spot["seen"]:
            spot["seen"] = check_in_front(picture)
        return text + status_note(), picture

    def point_camera(pan_deg, tilt_deg):
        pan = clamp(pan_deg, -MAX_PAN_DEG, MAX_PAN_DEG)
        tilt = clamp(tilt_deg, -MAX_TILT_DEG, MAX_TILT_DEG)
        if head["pan"] is None:
            turn = 2 * MAX_PAN_DEG   # we don't know where it points: wait for the longest turn
        else:
            turn = max(abs(pan - head["pan"]), abs(tilt - head["tilt"]))
        connector.service_call(
            ROS2Message(payload={"pan_deg": pan, "tilt_deg": tilt}),
            target="/picar/look", msg_type="picar_interfaces/srv/Look")
        head["pan"], head["tilt"] = pan, tilt
        time.sleep(0.4 + turn / HEAD_DEG_PER_S)   # let the head get there and settle
        still["since"] = time.time()              # pictures from now on show where it points

    def move(distance, steering):
        """Drive with a fixed steering angle. Returns how far it went, why it stopped
        early ("" if it drove the whole way), and whether something in front stopped it."""
        if head["pan"] != 0.0 or head["tilt"] != 0.0:
            point_camera(0.0, 0.0)
        duration = abs(distance) / SPEED_MPS
        request = {"speed": math.copysign(1.0, distance), "steering_deg": steering,
                   "duration_s": duration}
        start = time.monotonic()
        result = connector.service_call(
            ROS2Message(payload=request), target="/picar/move",
            msg_type="picar_interfaces/srv/Move", timeout_sec=duration + 5.0).payload
        # No wheel encoders: work out how far it went from the time it drove.
        driven = math.copysign(min(abs(distance), SPEED_MPS * (time.monotonic() - start)), distance)
        time.sleep(0.3)                # let the car settle
        still["since"] = time.time()   # pictures from now on show where it stopped

        moved = abs(driven) >= MIN_MOVE_M
        if moved:   # whatever the car was standing at is behind it now
            leave_spot()
        if result is None:
            return driven, "picar_base did not answer.", False
        if result.reason == "obstacle":
            spot["blocked_m"] = result.min_range_m
            if not moved:
                spot["tries"] += 1
            stop = f"Safety stop: something is {result.min_range_m:.2f} m in front of the bumper."
            return driven, stop, True
        if result.reason != "done":
            return driven, f"Stopped early: {STOP_REASONS.get(result.reason, result.reason)}.", False
        return driven, "", False

    def drove(driven, steering, stop):
        """A drive in words."""
        turned = TURN_DEG_PER_M * driven * steering / MAX_STEERING_DEG
        return (f"Drove {driven:+.2f} m with steering {steering:+.0f}, the heading changed "
                f"about {turned:+.0f} degrees (positive = left). {stop}").strip()

    @tool(response_format="content_and_artifact")
    def look(pan_deg: float = 0.0, tilt_deg: float = 0.0):
        """Point the camera and take a picture.

        pan_deg: -90 to 90, positive turns the camera left. tilt_deg: -40 to 40, positive looks up.
        pan_deg 0 and tilt_deg 0 look straight ahead.
        """
        point_camera(pan_deg, tilt_deg)
        pan, tilt = head["pan"], head["tilt"]
        text = f"Picture with the camera at pan {pan:.0f}, tilt {tilt:.0f}."
        if abs(pan) >= 10:
            side = "left" if pan > 0 else "right"
            text += (f" The camera points {abs(pan):.0f} degrees to the {side} of the car. If your"
                     f" target is in this picture, call turn with angle_deg {pan:.0f} before you drive.")
        return text + status_note(), take_picture()

    @tool(response_format="content_and_artifact")
    def look_around(target: str):
        """Search for a target with the camera, without moving the car. The camera looks
        to the left, ahead and to the right, and you are told how far to turn to face it.

        target: what you are looking for, for example "blue cylinder".
        """
        target = target.strip().lower()
        if target in progress:
            return (f"You already reached the {target}. Do not look for it again: look for"
                    f" the next target.{status_note()}", take_picture())
        goal["searched"] = target
        pictures, seen = {}, []
        for pan in LOOK_AROUND_PANS:
            point_camera(pan, 0)
            pictures[pan] = take_picture()
            where = where_is(pictures[pan], target, f"camera at pan {pan}")
            if where:
                seen.append((pan, where))
                if where == "middle":
                    break   # in the middle of a picture: good enough, stop looking
        point_camera(0, 0)
        looked = ", ".join(str(p) for p in pictures)
        if not seen:
            text = (f"Looked around for the {target} with the camera at pan {looked}: not seen. It may"
                    " be behind the car: turn 120 and call look_around again.")
            return text + status_note(), pictures[0]
        pan, where = min(seen, key=lambda s: s[1] in ("left", "right"))   # prefer the middle
        angle = pan + {"left": SIDE_OFFSET_DEG, "right": -SIDE_OFFSET_DEG}.get(where, 0)
        text = f"Looked around for the {target}: seen with the camera at pan {pan}, {where} of the picture"
        if abs(angle) < 10:
            text += ", about straight ahead. Call approach to drive to it."
        else:
            side = "left" if angle > 0 else "right"
            text += (f", about {abs(angle)} degrees to the {side} of the car. Call turn with"
                     f" angle_deg {angle} to face it, then call approach.")
        return text + status_note(), pictures[pan]

    @tool(response_format="content_and_artifact")
    def drive(distance_m: float, steering_deg: float = 0.0):
        """Drive, then take a picture straight ahead.

        distance_m: metres to drive, positive forward, negative backward, at most 0.6 per call.
        steering_deg: -30 to 30, positive steers left.
        """
        distance = clamp(distance_m, -MAX_DRIVE_M, MAX_DRIVE_M)
        steering = clamp(steering_deg, -MAX_STEERING_DEG, MAX_STEERING_DEG)
        require_camera()
        driven, stop, blocked = move(distance, steering)
        if blocked and abs(driven) < MIN_MOVE_M:
            text = (f"Did not move: something is {spot['blocked_m']:.2f} m in front of the bumper,"
                    " so driving forward does nothing.")
            if spot["tries"] > 1:
                text += f" You have tried {spot['tries']} times."
        else:
            text = drove(driven, steering, stop)
        if abs(distance_m) > MAX_DRIVE_M:
            text += f" {MAX_DRIVE_M:.1f} m is the most per call."
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
        distance = abs(angle) / TURN_DEG_PER_M
        steering = math.copysign(MAX_STEERING_DEG, angle)
        big = abs(angle) >= K_TURN_DEG
        require_camera()
        # A big turn in one forward arc moves the car about half a metre, and a close
        # target ends up beside it. Like a three-point turn, do half of it forward.
        driven, stop, blocked = move(distance / 2 if big else distance, steering)
        rest = distance - abs(driven)
        if (blocked or big) and rest >= MIN_MOVE_M:
            # Reversing with the steering the other way turns the same way
            back, stop, _ = move(-rest, -steering)
            turned = math.copysign((abs(driven) + abs(back)) * TURN_DEG_PER_M, angle)
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
        if target in progress:
            return (f"You already reached the {target}. Do not drive to it again: look for the"
                    f" next target.{status_note()}", take_picture())
        goal["searched"] = target
        require_camera()
        if head["pan"] != 0.0 or head["tilt"] != 0.0:
            point_camera(0.0, 0.0)
        steps, where, last, stop = [], "", "", ""
        for _ in range(APPROACH_MAX_STEPS):
            try:
                picture = take_picture()
            except CameraDown:   # say how far it got before the camera stopped
                return f"Approach to the {target}: {', '.join(steps) or 'did not move'}. {CAMERA_DOWN}", None
            where = where_is(picture, target, "approach")
            if not where:
                if last in ("", "lost"):
                    break   # not in the picture, or still not after backing up
                # It was there a moment ago, so now it is beside the car or below the camera:
                # back up, swinging the front towards the side it was on.
                swing = {"left": -MAX_STEERING_DEG / 2, "right": MAX_STEERING_DEG / 2}.get(last, 0.0)
                driven, stop, _ = move(-APPROACH_BACK_M, swing)
                steps.append(f"lost it, backed up {abs(driven):.2f} m")
                last = "lost"
            elif where == "middle":
                # Straight at it, as far as the distance sensor says is free
                ahead = range_ahead()
                distance = (2 * APPROACH_MIN_M if ahead is None else
                            clamp(ahead - STOP_DISTANCE_M + 0.05, APPROACH_MIN_M, MAX_DRIVE_M))
                driven, stop, _ = move(distance, 0.0)
                steps.append(f"drove {driven:.2f} m")
                last = where
            else:
                # On one side: turn towards it. If the last turn to this side did not bring it
                # to the middle, it is close and to the side, so turn while reversing instead.
                angle = SIDE_OFFSET_DEG if where == "left" else -SIDE_OFFSET_DEG
                arc = abs(angle) / TURN_DEG_PER_M
                steering = math.copysign(MAX_STEERING_DEG, angle)
                backward = last == where
                driven, stop, _ = move(-arc, -steering) if backward else move(arc, steering)
                steps.append(f"turned {where}" + (" reversing" if backward else ""))
                last = where
            if stop:
                break   # the safety stop, or another early stop
        moves = ", ".join(steps) if steps else "did not move"
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
        the targets the user asked for. The car's sensors check that it is really there.

        target: what you have reached, for example "red box".
        next_target: the next target the user asked for, or "" if none is left.
        """
        target = target.strip().lower()
        if target in progress:
            return (f"You already reached the {target}. Do not drive to it again: back up, "
                    f"then call look_around for the next target.{progress_note()}")
        # Arrived means stopped right in front of it: the distance sensor decides, not a guess
        if spot["blocked_m"] is None:
            return (f"Not reached yet: the safety system has not stopped the car, so the {target} "
                    f"is not right in front of it. Call approach for the {target}: it drives until "
                    f"the safety system stops the car in front of it. Then call reached again. If "
                    f"you cannot see the {target}, call look_around.{progress_note()}")
        # Stopped in front of something else? Turned down once; asked again here, it counts.
        if target == target_now() and says_no(spot["seen"]) and not spot["doubted"]:
            spot["doubted"] = True
            return (f"Not reached yet: the camera check says the thing in front of the car is not "
                    f"the {target}: \"{spot['seen']}\". Back up, then call look_around for the "
                    f"{target}.{progress_note()}")
        spot["reached"] = target   # until the car moves away
        progress.append(target)
        goal["next"] = next_target.strip().lower()
        if not goal["next"]:
            return f"Noted: {target}.{progress_note()} If that was the last target, finish with DONE."
        return (f"Noted: {target}.{progress_note()} Back up from the {target}, then call "
                f"look_around for the {goal['next']}.")

    @tool
    def remember(fact: str):
        """Save one short fact for later tasks, for example where you found something."""
        memory.append(fact)
        del memory[:-20]   # keep the newest 20
        save_memory(memory)
        return "Saved."

    return [drive, turn, look, look_around, approach, reached, remember], new_task, memory