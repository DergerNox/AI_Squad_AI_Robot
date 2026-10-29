"""picar_tools.py: what the AI can do with the PiCar, and what it knows about it.

drive        drives forward or backward with a steering angle, then takes a picture
turn         turns the car by an angle (a drive at full steering), then takes a picture
look         points the camera and takes a picture
look_around  looks left, ahead and right for a target and says how far to turn to face it
reached      notes that the car has arrived at one of the targets
remember     saves a fact for later tasks (in memory.json)

drive, turn and the camera go through picar_base (/picar/move and /picar/look),
so the SafetyGate still checks every move the AI makes. Questions about pictures
("is there a blue cylinder here?") are answered by the same AI model.
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
CAMERA_TOPIC = "/camera/image_raw"
MEMORY_FILE = "memory.json"

LOOK_AROUND_PANS = (0, 45, -45, 90, -90)   # 62 degree pictures: together about 240 degrees
SIDE_OFFSET_DEG = 20      # something on the left/right of a picture is about this far from its middle

# What the safety system's stop reasons mean
STOP_REASONS = {
    "no_range": "the distance sensor had no fresh reading, just try again",
    "timeout": "the drive command arrived too late, just try again",
    "estop": "the emergency stop is on",
    "busy": "another move was still running",
}

# Asked about the picture when the safety system has stopped the car
IN_FRONT = ("This picture is from a small robot car's camera, looking straight ahead. "
            "The car has stopped because something is very close in front of it.")

PICAR_PROMPT = f"""You drive a small robot car, a simulated Adeept PiCar-B in a room in Gazebo.
You can only control it with your tools: drive moves the car, turn turns the car by an
angle, look turns the camera, look_around searches for a target with the camera. Each of
them shows you a new camera picture. reached notes that you have arrived at a target,
and remember saves a fact for later tasks.

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
  Then call turn with that angle and drive at the target.
- If look_around does not find it, turn 120 degrees and call look_around again. If it still
  does not find it, drive {MAX_DRIVE_M:.1f} m into open space and call look_around again.
- To reach a target, steer until it is in the middle of the picture, then drive straight at it
  in steps of {MAX_DRIVE_M:.1f} m or less. Keep going until the safety system stops the car right in
  front of it: then you have arrived. Call reached with its name and the name of the next
  target ("" if it was the last one). reached tells you what to do next.
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


def clamp(value, low, high):
    return max(low, min(high, float(value)))


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
    # standing at. All of them clear when the car moves.
    spot = {"blocked_m": None, "seen": "", "reached": "", "tries": 0}
    memory = load_memory()               # facts kept between tasks and restarts

    def new_task():
        progress.clear()
        goal.update(next="", searched="")
        spot.update(blocked_m=None, seen="", reached="", tries=0)

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

    def take_picture():
        # A fixed message type and sensor-data QoS work with any camera publisher,
        # even before ROS has discovered the topic.
        message = connector.receive_message(
            CAMERA_TOPIC, timeout_sec=3.0, msg_type="sensor_msgs/msg/Image",
            qos_profile=qos_profile_sensor_data, auto_qos_matching=False)
        image = message.payload
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

    def picture_with_notes(text):
        """Take the picture for a tool result. If the car has just got stuck, ask
        the camera what it is stuck at."""
        picture = take_picture()
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
        time.sleep(0.4 + turn / HEAD_DEG_PER_S)   # let the head get there and a new frame arrive

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
        time.sleep(0.3)   # let the car settle and a new frame arrive

        moved = abs(driven) >= MIN_MOVE_M
        if moved:   # whatever the car was standing at is behind it now
            spot.update(blocked_m=None, seen="", reached="", tries=0)
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
        question = (f"Is there a {target} in this picture? Answer only 'no', or where it is "
                    "in the picture: 'left', 'middle' or 'right'.")
        pictures, seen = {}, []
        for pan in LOOK_AROUND_PANS:
            point_camera(pan, 0)
            pictures[pan] = take_picture()
            answer = ask_about(pictures[pan], question, f"camera at pan {pan}")
            words = re.findall(r"[a-z]+", answer)
            where = next((w for w in ("left", "middle", "centre", "center", "right") if w in words), "")
            if not where and words[:1] == ["yes"]:
                where = "middle"   # seen, but the answer does not say where
            if where and words[0] not in ("no", "not", "none", "nothing"):
                seen.append((pan, where))
                if where != "left" and where != "right":
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
            text += ", about straight ahead. Drive at it."
        else:
            side = "left" if angle > 0 else "right"
            text += (f", about {abs(angle)} degrees to the {side} of the car. Call turn with"
                     f" angle_deg {angle} to face it, then drive at it.")
        return text + status_note(), pictures[pan]

    @tool(response_format="content_and_artifact")
    def drive(distance_m: float, steering_deg: float = 0.0):
        """Drive, then take a picture straight ahead.

        distance_m: metres to drive, positive forward, negative backward, at most 0.6 per call.
        steering_deg: -30 to 30, positive steers left.
        """
        distance = clamp(distance_m, -MAX_DRIVE_M, MAX_DRIVE_M)
        steering = clamp(steering_deg, -MAX_STEERING_DEG, MAX_STEERING_DEG)
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

        angle_deg: -120 to 120, positive turns left, negative turns right. The car drives
        forward while it turns. To face something you saw with the camera at pan P,
        call turn with angle_deg P.
        """
        # The AI picks the angle; this turns it into a drive at full steering.
        angle = clamp(angle_deg, -120.0, 120.0)
        distance = abs(angle) / TURN_DEG_PER_M
        steering = math.copysign(MAX_STEERING_DEG, angle)
        driven, stop, blocked = move(distance, steering)
        rest = distance - abs(driven)
        if blocked and rest >= MIN_MOVE_M:
            # Blocked in front: reversing with the steering the other way turns the same way
            back, stop, _ = move(-rest, -steering)
            turned = math.copysign((abs(driven) + abs(back)) * TURN_DEG_PER_M, angle)
            text = (f"Blocked in front, so I turned the rest while reversing. The heading changed "
                    f"about {turned:+.0f} degrees (positive = left). {stop}").strip()
        else:
            text = drove(driven, steering, stop)
        return picture_with_notes(text)

    @tool
    def reached(target: str, next_target: str = ""):
        """Call this when you have arrived at one of the targets the user asked for.

        target: what you have reached, for example "red box".
        next_target: the next target the user asked for, or "" if none is left.
        """
        target = target.strip().lower()
        spot["reached"] = target   # until the car moves away
        if target in progress:
            return (f"You already reached the {target}. Do not drive to it again: back up, "
                    f"then call look_around for the next target.{progress_note()}")
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

    return [drive, turn, look, look_around, reached, remember], new_task, memory