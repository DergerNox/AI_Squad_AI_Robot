"""The agent's tools (drive, turn, look, look_around, approach, reached, remember) against a fake car.

FakeCar answers like picar_base and the camera and distance sensor nodes, and records
every request. FakeVision gives scripted answers to the tools' picture questions, and
FakeTime makes the waits instant. The car's numbers are the twin's, from its sim.yaml.
"""

import base64
import dataclasses
import io
import json
import math
import pathlib
from datetime import datetime
from types import SimpleNamespace

import numpy as np
import pytest
from PIL import Image
from rai.communication.ros2 import ROS2Message

import approach
import camera
import car_client
import car_config
import picar_tools
import questions

SHARED = pathlib.Path(__file__).resolve().parents[2]   # shared/picar_agent/test -> shared
TWIN = car_config.load_car(str(SHARED / "picar_base" / "config" / "sim.yaml"))


class FakeTime:
    """Stands in for the time module inside picar_tools: sleep() returns at once and
    moves the clock on, so the tests are fast and the timing is exact."""

    def __init__(self):
        self.now = 1000.0
        self.slept = []

    def monotonic(self):
        return self.now

    def time(self):
        return self.now

    def sleep(self, seconds):
        self.slept.append(seconds)
        self.now += seconds


class FakeCar:
    """Stands in for the ROS2Connector: records what the tools send to picar_base
    and answers like picar_base, the camera and the distance sensor would.

    results: what the next /picar/move calls return, one entry per call:
        (reason, part, min_range_m)  the move ends with this reason after driving
                                     this part (0 to 1) of the asked time
        None                         picar_base does not answer
    Once the list is empty, every move drives the whole way ("done").

    frames: how many more new camera pictures arrive (None: the camera keeps sending).
    After the last one, RAI hands out the last picture again, as it does when the
    camera node stops.

    image: what the camera sees. next_image: what it sees after the next look or move.
    stale_first: after each look or move, RAI first hands out the last picture again
    (taken before the move), as it does when the next picture has not arrived yet.

    range_m: what the distance sensor reads.

    moving: a move that drives changes the picture (a new shade of grey), as a car that
    really moves does; False: the wheels do not turn (a loose motor cable).
    """

    def __init__(self, clock):
        self.clock = clock
        self.looks = []     # (pan_deg, tilt_deg) of every /picar/look call
        self.moves = []     # every /picar/move request: speed, steering_deg, duration_s
        self.results = []
        self.calls = []     # "look" and "move", in the order they came
        self.image = SimpleNamespace(height=2, width=2, encoding="rgb8", data=bytes(12))
        self.next_image = None
        self.stale_first = False
        self.stale_now = False
        self.frames = None
        self.last_frame = 0.0
        self.last_image = self.image
        self.range_m = math.inf
        self.moving = True

    def receive_message(self, topic, **kwargs):
        if topic == "/ultrasonic/range":
            return ROS2Message(payload=SimpleNamespace(range=self.range_m, max_range=4.0))
        assert topic == "/camera/image_raw"
        if self.stale_now:
            self.stale_now = False
        elif self.frames is None or self.frames > 0:
            self.clock.now += 0.1   # the next picture arrives (10 a second)
            self.last_frame, self.last_image = self.clock.now, self.image
            if self.frames is not None:
                self.frames -= 1
        return ROS2Message(payload=self.last_image, timestamp=self.last_frame)

    def service_call(self, message, target, msg_type, **kwargs):
        given = self.next_image is not None
        if given:   # the head or the car moves: the camera sees something new
            self.image, self.next_image = self.next_image, None
        self.stale_now = self.stale_first
        request = message.payload
        if target == "/picar/look":
            self.calls.append("look")
            self.looks.append((request["pan_deg"], request["tilt_deg"]))
            return ROS2Message(payload=SimpleNamespace(success=True, message=""))
        assert target == "/picar/move"
        self.calls.append("move")
        self.moves.append(request)
        result = self.results.pop(0) if self.results else ("done", 1.0, 4.0)
        if result is None:
            return ROS2Message(payload=None)
        reason, part, closest = result
        self.clock.now += request["duration_s"] * part
        if self.moving and part > 0 and not given:
            shade = (len(self.moves) * 40) % 256
            self.image = SimpleNamespace(height=2, width=2, encoding="rgb8", data=bytes([shade] * 12))
        return ROS2Message(payload=SimpleNamespace(
            completed=reason == "done", reason=reason, min_range_m=closest))


class FakeVision:
    """Stands in for the AI model's answers to the tools' picture questions. It knows each of
    the tools' questions (questions.py) and fails the test on any other: a changed or mixed-up
    question must not get an answer by chance (a new question once took the stop check's name).
    answers: for WHERE, OTHER, the stop checks and DESCRIBE, in order; after them every answer is "no"."""

    def __init__(self):
        self.answers = []
        self.questions = []
        self.name = ""   # the name for a sighting (NAME_IT); "": the colour asked about + " box";
        #                  a list: names in order, the last one repeats
        self.crop_name = ""   # the name for a close look at a bounding box (NAME_CROP); "": the
        #                       name a sighting would get now (without using it up)
        # Where exactly the target is (LOCATE): how far down the picture its box reaches, in
        # words (WORD_ROWS), in order, the last one repeats; the box is in the middle; "none": not located.
        self.box_bottom = ["upper half"]
        self.same = ["yes"]   # compared with the first sighting (SAME, CLOSE_UP): in order, the last repeats
        self.box_x = (0.4, 0.6)   # where across the picture the located box is (LOCATE)
        self.pictures = []    # how many pictures came with each question

    def invoke(self, messages, config=None):
        question = messages[0].content[0]["text"]
        self.questions.append(question)
        self.pictures.append(sum(part.get("type") == "image_url" for part in messages[0].content[1:]))
        if question.startswith(SAME_START):   # a cut-out compared with the first sighting
            return SimpleNamespace(content=self.same.pop(0) if len(self.same) > 1 else self.same[0])
        if question.startswith(NAME_IT_START):   # naming a sighting: not one of the answers
            colour = question[len(NAME_IT_START):].split()[0]
            if isinstance(self.name, list):
                return SimpleNamespace(content=self.name.pop(0) if len(self.name) > 1 else self.name[0])
            return SimpleNamespace(content=self.name or f"{colour} box")
        if question.startswith(NAME_CROP_START):   # a close look at a bounding box
            colour = question.split("What is the ")[1].split()[0]
            if self.crop_name:
                return SimpleNamespace(content=self.crop_name)
            name = self.name[0] if isinstance(self.name, list) else self.name
            return SimpleNamespace(content=name or f"{colour} box")
        if question.startswith(LOCATE_START):   # where exactly: not one of the answers
            word = self.box_bottom.pop(0) if len(self.box_bottom) > 1 else self.box_bottom[0]
            if word == "none":
                return SimpleNamespace(content="[]")
            return SimpleNamespace(content=f"```json\n[{box_json('target', *self.box_x, WORD_ROWS[word])}]\n```")
        if not question.startswith(ANSWERED_IN_ORDER):
            raise AssertionError(f"FakeVision does not know this question: {question[:100]!r}")
        return SimpleNamespace(content=self.answers.pop(0) if self.answers else "no")


NAME_IT_START = picar_tools.NAME_IT.split("{thing}")[0]
LOCATE_START = picar_tools.LOCATE.split("{target}")[0]
NAME_CROP_START = picar_tools.NAME_CROP.split("{thing}")[0]
SAME_START = picar_tools.SAME.split("{target}")[0]
assert picar_tools.CLOSE_UP.startswith(SAME_START)   # both compare with the first sighting
# The questions answered from FakeVision.answers, in order
ANSWERED_IN_ORDER = tuple(text.split("{")[0] for text in (
    questions.WHERE, questions.OTHER, questions.IN_FRONT, questions.IN_FRONT_FIRST, questions.DESCRIBE,
    questions.FREER))
WORD_ROWS = {"bottom edge": 0.99, "lower third": 0.83, "middle": 0.55, "upper half": 0.4}


def box_json(label, x1, x2, bottom):
    """One bounding box as qwen3-vl writes it (0 to 1000 across and down)."""
    return (f'{{"bbox_2d": [{round(x1 * 1000)}, {round((bottom - 0.15) * 1000)}, {round(x2 * 1000)}, '
            f'{round(bottom * 1000)}], "label": "{label}"}}')


AGENT_MEMORY_FILE = picar_tools.MEMORY_FILE   # the real one, before the fixture below


@pytest.fixture(autouse=True)
def memory_file(monkeypatch, tmp_path):
    """Every test keeps its facts in a memory.json of its own, never the agent's."""
    path = str(tmp_path / "memory.json")
    monkeypatch.setattr(picar_tools, "MEMORY_FILE", path)
    monkeypatch.delenv("PICAR_SAVE_PICTURES", raising=False)   # no picture log unless a test asks
    return path


@pytest.fixture
def clock():
    return FakeTime()


@pytest.fixture
def car(clock):
    return FakeCar(clock)


@pytest.fixture
def vision():
    return FakeVision()


@pytest.fixture
def tools(car, vision):
    return make_tools(car, vision)


def make_tools(car, vision, numbers=TWIN):
    """The tools for a fake car (car), with the car's numbers from its settings (numbers)."""
    tool_list, new_task, memory = picar_tools.make_picar_tools(car, vision, numbers, car.clock)
    return SimpleNamespace(**{t.name: t for t in tool_list}, new_task=new_task, memory=memory,
                           task=new_task.task)


def use(tool, **args):
    """Call a tool the way the agent does. Returns the tool message:
    .content is the text, .artifact the picture (for the tools that take one)."""
    return tool.invoke({"name": tool.name, "args": args, "id": "call-1", "type": "tool_call"})


def move(speed, steering_deg, duration_s):
    return {"speed": pytest.approx(speed), "steering_deg": steering_deg, "duration_s": pytest.approx(duration_s)}


def png_of(pixels):
    """PNG bytes of an array of pixels (rows x columns x 3)."""
    buffer = io.BytesIO()
    Image.fromarray(pixels).save(buffer, format="PNG")
    return buffer.getvalue()


def pixel(picture):
    """The top-left pixel of a tool's picture, as (red, green, blue)."""
    png = base64.b64decode(picture["images"][0])
    return tuple(int(v) for v in np.array(Image.open(io.BytesIO(png)).convert("RGB"))[0, 0])


def stop_in_front(tools, car, vision, seen="yes, red box"):
    """Drive into something: the safety stop holds the car and the camera check says `seen`."""
    car.results = [("obstacle", 0.0, 0.2)]
    vision.answers = [seen]
    use(tools.drive, distance_m=0.3)


# ---------- the test helpers ----------

def test_the_fake_model_fails_on_a_question_it_does_not_know(vision):
    # a new question once took the stop check's name, and every test still passed (2026-11-24)
    message = SimpleNamespace(content=[{"type": "text", "text": "Is this a trick question?"}])
    with pytest.raises(AssertionError, match="FakeVision does not know this question"):
        vision.invoke([message])


# ---------- the car's numbers ----------

# Another car: every number differs from the twin's
OTHER = dataclasses.replace(TWIN, speed_mps=0.25, turn_speed_mps=0.25, max_move_s=2.0, max_steering_deg=25.0,
                            max_pan_deg=60.0, max_tilt_deg=20.0, turn_deg_per_m=100.0,
                            head_deg_per_s=100.0, stop_distance_m=0.5)


def test_the_tools_use_the_cars_numbers(car, vision, clock):
    tools = make_tools(car, vision, OTHER)
    text = use(tools.drive, distance_m=2.0, steering_deg=45).content
    assert car.moves == [move(1.0, 25.0, 2.0)]    # 0.5 m at most (0.25 m/s x 2 s)
    assert "0.5 m is the most per call." in text
    use(tools.turn, angle_deg=30)                 # 30 deg at 100 deg/m = 0.3 m
    assert car.moves[-1] == move(1.0, 25.0, 1.2)
    use(tools.look, pan_deg=80, tilt_deg=30)
    assert car.looks[-1] == (60.0, 20.0)
    assert clock.slept[-1] == pytest.approx(0.4 + 60 / 100)   # the head turned 60 deg at 100 deg/s


def test_the_tool_descriptions_give_the_cars_limits(car, vision):
    tools = make_tools(car, vision, OTHER)
    assert "at most 0.5 per call" in tools.drive.description
    assert "steering_deg: -25 to 25" in tools.drive.description
    assert "pan_deg: -60 to 60" in tools.look.description
    assert "tilt_deg: -20 to 20" in tools.look.description


def test_approach_steps_stop_short_of_the_cars_stop_distance(car, vision):
    tools = make_tools(car, vision, OTHER)
    car.range_m = 0.9
    car.results = [("obstacle", 1.0, 0.5)]
    vision.answers = ["middle"]
    use(tools.approach, target="red box")
    assert car.moves == [move(1.0, 0.0, 0.45 / 0.25)]   # 0.9 - 0.5 stop distance + 0.05


# ---------- drive ----------

def test_drive_asks_picar_base_for_the_cars_power_for_the_right_time(tools, car):
    # the twin drives at max_speed: speed 1.0, as /picar/move's speed is a fraction of it
    text = use(tools.drive, distance_m=0.5, steering_deg=10).content
    assert car.moves == [move(1.0, 10.0, 2.5)]   # 0.5 m at 0.2 m/s
    assert text.startswith("Drove +0.50 m with steering +10, the heading changed about +33 degrees")


def test_drive_backward(tools, car):
    text = use(tools.drive, distance_m=-0.3).content
    assert car.moves == [move(-1.0, 0.0, 1.5)]
    assert text.startswith("Drove -0.30 m")


# The real car: slower in reverse (car.yaml forward_power 0.35, reverse_power 0.15)
SLOW_REVERSE = dataclasses.replace(TWIN, power=0.35, speed_mps=0.175, reverse_power=0.15,
                                   reverse_speed_mps=0.075)


def test_drive_uses_the_cars_power_forward_and_in_reverse(car, vision):
    tools = make_tools(car, vision, SLOW_REVERSE)
    use(tools.drive, distance_m=0.35)
    use(tools.drive, distance_m=-0.15)
    assert car.moves == [move(0.875, 0.0, 2.0), move(-0.375, 0.0, 2.0)]   # of max_speed 0.4


def test_every_move_is_longer_by_the_start_up_and_it_does_not_count_as_distance(car, vision):
    # the real car: 0.2 s moved it 4 mm, 0.5 s 5 cm, 0.7 s about 10 cm (2026-11-24)
    tools = make_tools(car, vision, dataclasses.replace(TWIN, start_lag_s=0.35))
    text = use(tools.drive, distance_m=0.1).content
    assert car.moves == [move(1.0, 0.0, 0.1 / 0.2 + 0.35)]
    assert text.startswith("Drove +0.10 m")


def test_with_a_start_up_a_long_move_goes_in_parts_each_with_its_start_up(car, vision):
    tools = make_tools(car, vision, dataclasses.replace(TWIN, start_lag_s=0.35, reverse_speed_mps=0.1))
    text = use(tools.drive, distance_m=-0.4).content   # 4 s of rolling: 2.65 s in the first part
    assert [round(m["duration_s"], 2) for m in car.moves] == [3.0, 1.7]
    assert text.startswith("Drove -0.40 m")


def test_a_long_slow_reverse_goes_in_parts_of_at_most_max_move_s(car, vision):
    tools = make_tools(car, vision, SLOW_REVERSE)
    text = use(tools.drive, distance_m=-0.3).content   # 4 s at 0.075 m/s; picar_base allows 3 s
    assert car.moves == [move(-0.375, 0.0, 3.0), move(-0.375, 0.0, 1.0)]
    assert text.startswith("Drove -0.30 m")


def test_a_move_in_parts_ends_at_the_first_early_stop(car, vision):
    tools = make_tools(car, vision, SLOW_REVERSE)
    car.results = [("estop", 0.5, 4.0)]
    text = use(tools.drive, distance_m=-0.3).content
    assert len(car.moves) == 1
    assert text.startswith("Drove -0.11 m") and "The emergency stop is on" in text   # 1.5 s at 0.075 m/s


def test_the_drive_limit_is_said_as_exactly_as_it_is_used(car, vision):
    # the real car: 0.22 m/s x 3 s = 0.66 m, which the texts called "0.7"
    tools = make_tools(car, vision, dataclasses.replace(TWIN, speed_mps=0.22))
    assert "at most 0.66 per call" in tools.drive.description
    text = use(tools.drive, distance_m=0.7).content
    assert text.endswith("0.66 m is the most per call. Reached so far: nothing yet.")


def test_drive_is_limited_to_0_6_m_and_30_degrees(tools, car):
    text = use(tools.drive, distance_m=2.0, steering_deg=45).content
    assert car.moves == [move(1.0, 30.0, 3.0)]
    assert "0.6 m is the most per call." in text


def test_drive_points_the_camera_straight_ahead_first(tools, car):
    use(tools.drive, distance_m=0.2)
    use(tools.drive, distance_m=0.2)
    assert car.calls == ["look", "move", "move"]   # the head was unknown only at first
    use(tools.look, pan_deg=45)
    use(tools.drive, distance_m=0.2)
    assert car.looks[-1] == (0.0, 0.0) and car.calls[-2:] == ["look", "move"]


def test_drive_returns_a_picture(tools):
    picture = use(tools.drive, distance_m=0.2).artifact
    assert len(picture["images"]) == 1


def test_moving_away_counts_the_tries_again_from_zero(tools, car, vision):
    car.results = [("obstacle", 0.0, 0.2), ("obstacle", 0.0, 0.2), ("done", 1.0, 4.0),
                   ("obstacle", 0.0, 0.2)]
    use(tools.drive, distance_m=0.4)
    assert "You have tried 2 times." in use(tools.drive, distance_m=0.4).content
    use(tools.drive, distance_m=-0.3)   # backs away: a new spot
    text = use(tools.drive, distance_m=0.4).content
    assert text.startswith("Did not move") and "You have tried" not in text


def test_blocked_drive_says_it_did_not_move_and_asks_the_camera_once(tools, car, vision):
    car.results = [("obstacle", 0.5, 0.2), ("obstacle", 0.0, 0.2), ("obstacle", 0.0, 0.2)]
    vision.answers = ["White wall."]
    first = use(tools.drive, distance_m=0.4).content   # 0.2 m, then the safety stop
    assert first.startswith("Drove +0.20 m") and 'The camera check says: "white wall".' in first
    second = use(tools.drive, distance_m=0.4).content
    assert second.startswith("Did not move: something is 0.20 m in front of the bumper")
    assert "cannot drive forward" in second and "You have tried" not in second
    third = use(tools.drive, distance_m=0.4).content
    assert "You have tried 2 times." in third
    checks = [q for q in vision.questions if q.startswith(picar_tools.IN_FRONT)]
    assert len(checks) == 1   # the car has not moved since, so it is the same thing


def test_the_camera_check_names_the_target_being_searched(tools, car, vision):
    vision.answers = ["middle"]
    vision.box_bottom = ["none"]   # the first sighting cannot be cut out: nothing to compare with
    use(tools.look_around, target="red box")
    car.results = [("obstacle", 0.5, 0.2)]
    text = use(tools.drive, distance_m=0.6).content
    assert "Is it the red box?" in vision.questions[-1] and vision.pictures[-1] == 1
    assert text.startswith("Drove +0.30 m") and "Safety stop: something is 0.20 m" in text
    assert "If that is the red box, you have arrived: call reached now." in text


def test_the_camera_check_compares_with_the_first_sighting(tools, car, vision):
    # the real model, stopped with a plastic bag in front of the lens: "is it the white bed?"
    # "yes" 3/3; shown the bed's first sighting too: "no" 3/3 (2026-11-20)
    vision.answers = ["middle", "no, white plastic bag"]
    vision.name = "white bed"
    use(tools.look_around, target="white bed")
    car.results = [("obstacle", 0.5, 0.2)]
    text = use(tools.drive, distance_m=0.6).content
    assert "Is it the same white bed?" in vision.questions[-1] and vision.pictures[-1] == 2
    assert 'The camera check says: "no, white plastic bag"' in text


def test_drive_stopped_early_says_why(tools, car):
    car.results = [("no_range", 0.5, 4.0)]
    text = use(tools.drive, distance_m=0.5).content
    assert text.startswith("Drove +0.25 m")
    assert "Stopped early: the distance sensor had no fresh reading, just try again." in text


def test_no_answer_from_picar_base(tools, car):
    car.results = [None]
    text = use(tools.drive, distance_m=0.5).content
    assert "picar_base did not answer." in text


# ---------- no echo from the distance sensor ----------

def test_a_second_move_without_a_distance_reading_explains_why(tools, car):
    # on the real car a soft blanket gave no echo at all, and "just try again" kept the model trying
    car.results = [("no_range", 0.0, 4.0), ("no_range", 0.0, 4.0)]
    first = use(tools.drive, distance_m=0.3).content
    second = use(tools.drive, distance_m=0.3).content
    assert "just try again" in first
    assert "The distance sensor gets no echo from what is in front" in second
    assert "trying again does not help" in second


def test_a_reading_in_between_starts_the_count_again(tools, car):
    car.results = [("no_range", 0.0, 4.0), ("done", 1.0, 4.0), ("no_range", 0.0, 4.0)]
    texts = [use(tools.drive, distance_m=0.3).content for _ in range(3)]
    assert "just try again" in texts[2]


def test_approach_lets_the_camera_judge_arrival_when_the_sensor_gets_no_echo(tools, car, vision):
    car.results = [("no_range", 0.3, 4.0)]
    vision.answers = ["middle"]
    vision.box_bottom = ["middle", "bottom edge"]   # before the step, and after the stop
    vision.name = "red stick"
    text = use(tools.approach, target="red stick").content
    assert vision.questions[-1].startswith("Locate the red stick in the image.")
    assert "the camera shows it right in front of the car" in text
    assert "call reached now" in text
    answer = use(tools.reached, target="red stick").content
    assert answer.startswith("Noted: red stick (by the camera).")


def test_without_the_camera_close_up_no_echo_is_not_arrival(tools, car, vision):
    car.results = [("no_range", 0.3, 4.0)]
    vision.answers = ["middle"]
    vision.box_bottom = ["middle", "lower third"]
    vision.name = "red stick"
    text = use(tools.approach, target="red stick").content
    assert "Stopped early: the distance sensor had no fresh reading" in text
    assert use(tools.reached, target="red stick").content.startswith("Not reached yet")


def test_the_camera_arrival_is_forgotten_when_the_car_moves(tools, car, vision):
    car.results = [("no_range", 0.3, 4.0)]
    vision.answers = ["middle"]
    vision.box_bottom = ["middle", "bottom edge"]
    vision.name = "red stick"
    use(tools.approach, target="red stick")
    use(tools.drive, distance_m=-0.3)
    assert use(tools.reached, target="red stick").content.startswith("Not reached yet")


# ---------- approach: step sizes from the sensor and the camera ----------

def test_a_target_the_sensor_does_not_hear_counts_by_the_camera_without_driving_into_it(
        tools, car, vision):
    # the real car's sensor never heard a small blue box: approach drove 0.58 m past it
    car.range_m = math.inf          # the sensor hears nothing ahead
    vision.answers = ["middle"]
    vision.box_bottom = ["bottom edge"]
    text = use(tools.approach, target="blue box").content
    assert car.moves == []
    assert "The distance sensor does not hear the blue box, but the camera shows it right in front" in text
    assert "call reached now" in text
    assert use(tools.reached, target="blue box").content.startswith("Noted: blue box (by the camera")


def test_a_target_the_sensor_hears_still_ends_with_the_safety_stop(tools, car, vision):
    car.range_m = 0.30              # the sensor hears it: arrival stays the safety stop's
    car.results = [("obstacle", 0.5, 0.24)]
    vision.answers = ["middle", "yes, blue box"]
    vision.box_bottom = ["bottom edge"]
    text = use(tools.approach, target="blue box").content
    assert len(car.moves) == 1
    assert "Safety stop: something is 0.24 m in front of the bumper." in text
    assert use(tools.reached, target="blue box").content.startswith("Noted: blue box.")


def test_close_to_a_target_the_sensor_does_not_hear_the_steps_are_10_cm(tools, car, vision):
    # a low box on the real car: the sensor heard the cabinet behind it and the car hit the box
    car.range_m = math.inf          # the sensor alone would allow the longest step (0.6 m)
    vision.answers = ["middle", "middle"]
    vision.box_bottom = ["lower third", "bottom edge"]
    use(tools.approach, target="blue box")
    assert car.moves == [move(1.0, 0.0, 0.1 / 0.2)]   # 10 cm, then arrived by the camera


def test_in_the_lower_third_with_the_sensor_hearing_it_the_step_goes_to_the_bottom_edge(
        tools, car, vision):
    car.range_m = 0.5               # the sensor hears it: its step would be 0.3 m
    vision.answers = ["middle"]
    vision.box_bottom = ["lower third"]
    use(tools.approach, target="blue box")
    step = picar_tools.floor_distance(TWIN.floor, 0.83) - 0.35   # to the bottom edge: about 0.2 m
    assert car.moves[0] == move(1.0, 0.0, step / 0.2)
    assert step == pytest.approx(0.205)


@pytest.mark.parametrize("reading, step", [
    (1.3, 0.1),      # the real car: the cabinet behind a low box, far beyond where the camera sees it
    (0.75, 0.205),   # the twin: the box itself, about where the camera sees it
])
def test_the_sensor_hears_the_target_if_it_reads_about_where_the_camera_sees_it(
        tools, car, vision, reading, step):
    car.range_m = reading
    vision.answers = ["middle"]
    vision.box_bottom = ["lower third"]
    use(tools.approach, target="blue box")
    assert car.moves[0]["duration_s"] == pytest.approx(step / 0.2, abs=0.01)


def test_close_to_a_target_the_sensor_does_not_hear_side_corrections_reverse(tools, car, vision):
    car.range_m = math.inf
    vision.answers = ["middle", "left", "middle"]
    vision.box_bottom = ["lower third", "bottom edge"]
    use(tools.approach, target="blue box")
    # 10 cm straight, then 10 degrees left while reversing (0.05 m at 200 deg/m), then arrived
    assert car.moves == [move(1.0, 0.0, 0.5), move(-1.0, -30.0, 0.05 / 0.2)]


def test_a_target_the_sensor_hears_gets_the_usual_side_correction(tools, car, vision):
    car.range_m = 0.5
    vision.answers = ["middle", "left"]
    vision.box_bottom = ["lower third"]
    use(tools.approach, target="blue box")
    assert car.moves[1] == move(1.0, 30.0, 0.1 / 0.2)   # 20 degrees forward, as before


@pytest.mark.parametrize("answer", ["middle", "upper half", "", "i do not know"])
def test_far_away_the_sensor_alone_sizes_the_step(tools, car, vision, answer):
    car.range_m = 0.5
    vision.answers = ["middle"]
    vision.box_bottom = [answer]
    use(tools.approach, target="blue box")
    assert car.moves[0] == move(1.0, 0.0, (0.5 - 0.25 + 0.05) / 0.2)


@pytest.mark.parametrize("row, metres", [(1.0, 0.35), (0.9, 0.45), (0.85, 0.525), (0.6, 1.9), (0.3, 1.9)])
def test_the_floor_table_gives_the_distance_of_a_picture_row(row, metres):
    assert picar_tools.floor_distance(TWIN.floor, row) == pytest.approx(metres)


@pytest.mark.parametrize("answer, boxes", [
    # the real qwen3-vl on the real car's picture (2026-11-17)
    ('```json [ {"bbox_2d": [402, 519, 714, 655], "label": "blue box"} ] ```',
     [("blue box", 0.402, 0.519, 0.714, 0.655)]),
    ('[{"bbox_2d": [12, 597, 430, 754], "label": "White power strip"}, '
     '{"bbox_2d": [500, 600, 450, 1200], "label": "x"}]',          # corners swapped, past the edge
     [("white power strip", 0.012, 0.597, 0.43, 0.754), ("x", 0.45, 0.6, 0.5, 1.0)]),
    ("There is no blue box in the image.", []),
    ("[]", []),
])
def test_bounding_boxes_from_the_models_answer(answer, boxes):
    assert picar_tools.boxes_in(answer) == [
        (label, *(pytest.approx(v) for v in numbers)) for label, *numbers in boxes]


# ---------- the car does not move, picar_base does not answer ----------

def test_a_drive_that_does_not_change_the_picture_warns_then_says_to_stop(tools, car):
    # 2026-11-17: a loose motor cable; approach "drove 0.49 m" twelve times, the car stood still
    car.moving = False
    first = use(tools.drive, distance_m=0.4).content
    assert "The camera picture did not change, so the car may not have moved" in first
    second = use(tools.drive, distance_m=0.4).content
    assert "The car does not seem to move" in second and "finish with DONE" in second
    # pressed against cables and a slipper, the real car was told only "no power" (2026-11-24)
    assert "Something low or soft may block it" in second and "Back up 0.2 m once" in second
    third = use(tools.drive, distance_m=-0.2).content
    assert "Do not drive again: finish with DONE" in third and "Back up" not in third


def test_a_drive_that_changes_the_picture_starts_the_count_again(tools, car):
    car.moving = False
    use(tools.drive, distance_m=0.4)
    car.moving = True
    assert "did not change" not in use(tools.drive, distance_m=0.4).content
    car.moving = False
    assert "does not seem to move" not in use(tools.drive, distance_m=0.4).content


def test_approach_stops_when_its_steps_do_not_move_the_car(tools, car, vision):
    car.moving = False
    vision.answers = ["middle"] * 12
    text = use(tools.approach, target="red box").content
    assert len(car.moves) == 2
    assert "The car does not seem to move" in text and "finish with DONE" in text


def test_short_or_blocked_drives_are_not_checked(tools, car):
    car.moving = False
    assert "did not change" not in use(tools.drive, distance_m=0.05).content
    car.results = [("obstacle", 0.0, 0.2)]
    assert "did not change" not in use(tools.drive, distance_m=0.4).content


def test_picar_base_not_answering_says_to_finish(tools, car):
    def not_there(*args, **kwargs):
        raise TimeoutError("Service /picar/look not ready within 5.0 seconds.")
    car.service_call = not_there
    with pytest.raises(car_client.BaseDown, match="picar_base on the car does not answer.*finish with DONE"):
        use(tools.look, pan_deg=45)


# ---------- the emergency stop ----------

def test_the_emergency_stop_tells_the_ai_to_finish(tools, car):
    # on the real car the model kept trying approach and backing up for minutes
    car.results = [("estop", 0.0, 4.0)]
    text = use(tools.drive, distance_m=0.5).content
    assert text.startswith("Drove +0.00 m")
    assert "The emergency stop is on: the car cannot move at all" in text
    assert "finish with DONE and tell the user" in text


def test_a_big_turn_does_not_try_the_reversing_half_at_the_emergency_stop(tools, car):
    car.results = [("estop", 0.0, 4.0)]
    text = use(tools.turn, angle_deg=90).content
    assert len(car.moves) == 1
    assert "finish with DONE" in text


def test_approach_ends_at_the_emergency_stop(tools, car, vision):
    car.results = [("estop", 0.0, 4.0)]
    vision.answers = ["middle"]
    text = use(tools.approach, target="red box").content
    assert len(car.moves) == 1
    assert text.startswith("Approach to the red box: drove 0.00 m. The emergency stop is on")


# ---------- never blind ----------

def test_drive_and_turn_do_not_move_without_new_pictures(tools, car):
    car.frames = 0   # the camera has stopped: only its last picture is left
    with pytest.raises(camera.CameraDown, match="The car did not move. The camera has sent"):
        use(tools.drive, distance_m=0.5)
    with pytest.raises(camera.CameraDown, match="The car did not move."):
        use(tools.turn, angle_deg=30)
    assert car.moves == []


def test_a_camera_that_stops_during_a_move_is_reported_with_the_move(tools, car):
    car.frames = 1   # one picture for the check before the move, none after it
    result = use(tools.drive, distance_m=0.5)
    assert car.moves == [move(1.0, 0.0, 2.5)]
    assert result.content.startswith("Drove +0.50 m")
    assert "the camera is not working" in result.content and result.artifact is None


# ---------- turn ----------

def test_a_small_turn_drives_forward_at_full_steering(tools, car):
    text = use(tools.turn, angle_deg=50).content
    assert car.moves == [move(1.0, 30.0, 1.25)]   # 50 deg at 200 deg/m = 0.25 m
    assert "the heading changed about +50 degrees" in text


def test_turn_right(tools, car):
    text = use(tools.turn, angle_deg=-45).content
    assert car.moves == [move(1.0, -30.0, 1.125)]
    assert "the heading changed about -45 degrees" in text


def test_a_big_turn_drives_forward_all_the_way(tools, car):
    # nothing sees behind the car: until 2026-11-17 it went half forward, half reversing
    text = use(tools.turn, angle_deg=90).content
    assert car.moves == [move(1.0, 30.0, 2.25)]   # 0.45 m
    assert text.startswith("Drove +0.45 m with steering +30, the heading changed about +90 degrees")


def test_turn_is_limited_to_120_degrees(tools, car):
    use(tools.turn, angle_deg=180)
    assert car.moves == [move(1.0, 30.0, 3.0)]


def test_a_turn_blocked_at_once_reverses_only_0_2_m(tools, car):
    car.results = [("obstacle", 0.0, 0.2)]
    text = use(tools.turn, angle_deg=120).content
    assert car.moves == [move(1.0, 30.0, 3.0), move(-1.0, -30.0, 1.0)]   # 0.2 m of the 0.6 m
    assert text.startswith("Blocked in front, so I turned a little more while reversing (0.2 m")
    assert "The heading changed about +40 degrees" in text


def test_blocked_turn_reverses_the_rest_with_the_steering_the_other_way(tools, car):
    car.results = [("obstacle", 0.4, 0.2)]   # 0.1 m of the 0.25 m, then blocked
    text = use(tools.turn, angle_deg=50).content
    assert car.moves == [move(1.0, 30.0, 1.25), move(-1.0, -30.0, 0.75)]
    assert text.startswith("Blocked in front, so I turned the rest while reversing. "
                           "The heading changed about +50 degrees")


def textured(seed, noise=0, shift=0):
    """A 64 x 48 camera picture of random texture (the same seed: the same scene), plus noise,
    the scene moved shift pixels to the right (as when the car turns left)."""
    rng = np.random.default_rng(seed)
    pixels = np.roll(rng.integers(0, 256, (48, 64, 3)).astype(int), shift, axis=1)
    if noise:
        pixels = np.clip(pixels + np.random.default_rng(seed + 99).integers(-noise, noise + 1, pixels.shape), 0, 255)
    return SimpleNamespace(height=48, width=64, encoding="rgb8", data=pixels.astype(np.uint8).tobytes())


def test_a_turn_the_camera_says_did_not_happen_says_back_up(tools, car):
    # the real car, pressed against the bed and a plush toy: two turns of 120 did not move it,
    # and the AI was told it had turned (2026-11-24)
    use(tools.look)                            # the camera points ahead: no look before the turn
    car.image = textured(1)
    car.next_image = textured(1, noise=8)    # the same view, a little noisier: it did not turn
    text = use(tools.turn, angle_deg=90).content
    assert "The camera picture says the car turned only about 0 degrees, not 90" in text
    assert "Back up 0.2 m (drive with distance_m -0.2) before you turn again." in text


def test_a_turn_that_changed_the_view_says_nothing_more(tools, car):
    use(tools.look)
    car.image = textured(1)
    car.next_image = textured(1, shift=30)    # the view slid 30 of 64 pixels: it turned about 29 deg
    assert "turned only about" not in use(tools.turn, angle_deg=90).content


def test_a_plain_view_cannot_tell_whether_the_car_turned(tools, car):
    use(tools.look)
    car.image = SimpleNamespace(height=48, width=64, encoding="rgb8", data=bytes([90] * 48 * 64 * 3))
    car.next_image = SimpleNamespace(height=48, width=64, encoding="rgb8", data=bytes([95] * 48 * 64 * 3))
    assert "turned only about" not in use(tools.turn, angle_deg=90).content   # a bare wall: no alarm


# ---------- look ----------

def test_look_is_limited_and_says_how_to_face_what_it_sees(tools, car):
    text = use(tools.look, pan_deg=120, tilt_deg=-60).content
    assert car.looks == [(90.0, -40.0)]
    assert text.startswith("Picture with the camera at pan 90, tilt -40.")
    assert "90 degrees to the left of the car" in text and "call turn with angle_deg 90" in text


def test_look_straight_ahead_has_no_turn_hint(tools):
    text = use(tools.look).content
    assert text.startswith("Picture with the camera at pan 0, tilt 0.") and "call turn" not in text


def test_the_camera_waits_for_the_head_to_get_there(tools, clock):
    use(tools.look)                 # where the head points is unknown: the longest wait
    use(tools.look, pan_deg=45)     # 45 degrees at 150 degrees/s
    assert clock.slept == [pytest.approx(0.4 + 180 / 150), pytest.approx(0.4 + 45 / 150)]


RED = SimpleNamespace(height=1, width=1, encoding="rgb8", data=bytes([255, 0, 0]))
BLUE = SimpleNamespace(height=1, width=1, encoding="rgb8", data=bytes([0, 0, 255]))


@pytest.mark.parametrize("call", [
    lambda tools: use(tools.look, pan_deg=45),
    lambda tools: use(tools.drive, distance_m=0.3),
], ids=["look", "drive"])
def test_a_picture_is_never_one_from_before_the_car_or_the_camera_moved(tools, car, call):
    car.image = RED
    use(tools.look)              # the newest picture RAI holds is red
    car.stale_first = True       # and it hands that one out first after the next move
    car.next_image = BLUE        # after which the camera sees blue
    assert pixel(call(tools).artifact) == (0, 0, 255)


def test_bgr_pictures_are_turned_into_rgb(tools, car):
    car.image = SimpleNamespace(height=1, width=1, encoding="bgr8", data=bytes([0, 0, 255]))
    assert pixel(use(tools.look).artifact) == (255, 0, 0)
    car.image = SimpleNamespace(height=1, width=1, encoding="rgb8", data=bytes([255, 0, 0]))
    assert pixel(use(tools.look).artifact) == (255, 0, 0)


# ---------- look_around ----------

def test_look_around_stops_when_the_target_is_in_the_middle(tools, car, vision):
    vision.answers = ["no", "yes, in the middle"]
    text = use(tools.look_around, target="Red Box").content
    assert car.looks == [(0.0, 0.0), (45.0, 0.0), (0.0, 0.0)]   # then back to straight ahead
    assert "seen with the camera at pan 45, middle of the picture" in text
    assert "about 45 degrees to the left of the car. Call turn with angle_deg 45" in text
    assert text.endswith("then call approach. Reached so far: nothing yet.")
    assert "Is there a red box in this picture?" in vision.questions[0]


def test_look_around_adds_20_degrees_for_the_side_of_the_picture(tools, car, vision):
    vision.answers = ["right"]
    text = use(tools.look_around, target="red box").content
    assert [pan for pan, tilt in car.looks] == [0, 45, -45, 90, -90, 0]   # a side: keeps looking
    assert "about 20 degrees to the right of the car. Call turn with angle_deg -20" in text


def test_look_around_prefers_the_middle_of_a_picture(tools, vision):
    vision.answers = ["on the left", "middle"]
    text = use(tools.look_around, target="red box").content
    assert "Call turn with angle_deg 45" in text


@pytest.mark.parametrize("answer", ["yes", "Yes, in the middle.", "centre", "center"])
def test_look_around_answers_that_mean_straight_ahead(tools, vision, answer):
    vision.answers = [answer]
    text = use(tools.look_around, target="red box").content
    assert "middle of the picture, about straight ahead. Call approach to drive to it." in text


def test_look_around_not_seen(tools, car, vision):
    vision.answers = ["no, it is not on the left", "none", "nothing", "not here", "no"]
    text = use(tools.look_around, target="red box").content
    assert "with the camera at pan 0, 45, -45, 90, -90: not seen" in text
    assert "turn 120 and call look_around again" in text
    assert car.looks[-1] == (0.0, 0.0)


def test_look_around_without_a_vision_model_sees_nothing(car):
    tools = make_tools(car, None)
    assert "not seen" in use(tools.look_around, target="red box").content


@pytest.mark.parametrize("target", ["unknown", "", "Anything", "the surroundings"])
def test_look_around_without_a_real_target_describes_and_does_not_drive(tools, car, vision, target):
    # "look around and tell me what you see": the model called look_around("unknown")
    vision.answers = ["A red stick on a blanket.", "a wooden wall", "a plastic bag"]
    text = use(tools.look_around, target=target).content
    assert car.looks == [(0.0, 0.0), (45.0, 0.0), (-45.0, 0.0), (0.0, 0.0)]
    assert car.moves == []
    assert vision.questions[0] == picar_tools.DESCRIBE and "Do not mention the camera" in vision.questions[0]
    assert ("ahead: a red stick on a blanket; to the left: a wooden wall; to the right: a plastic bag"
            in text)
    assert text.endswith("Do not drive: answer the user now, starting with DONE, with what you saw.")


def test_look_around_does_not_take_something_else_of_the_colour(tools, car, vision):
    # on the real car "Is there a blue box?" got "middle" for a blue bag; named, it was "blue bag"
    vision.answers = ["middle"]
    vision.name = "blue bag"
    text = use(tools.look_around, target="blue box").content
    assert "What is the blue thing at the middle of the picture?" in vision.questions[1]
    assert text.startswith("Looked around for the blue box with the camera at pan 0, 45, -45, 90, -90: not seen.")


@pytest.mark.parametrize("name", ["blue box", "a blue cube", "blue cardboard carton", "blue boxes"])
def test_a_sighting_named_as_the_targets_kind_counts(tools, car, vision, name):
    vision.answers = ["middle"]
    vision.name = name
    assert "seen with the camera at pan 0" in use(tools.look_around, target="blue box").content


def test_approach_names_only_its_first_sighting(tools, car, vision):
    car.range_m = 1.0
    car.results = [("done", 1.0, 4.0), ("obstacle", 0.5, 0.2)]
    vision.answers = ["middle", "middle", "yes, red box"]
    use(tools.approach, target="red box")
    assert sum(q.startswith(NAME_IT_START) for q in vision.questions) == 1


def test_look_around_does_not_search_for_a_reached_target(tools, car, vision):
    stop_in_front(tools, car, vision)
    use(tools.reached, target="red box")
    looks, questions = len(car.looks), len(vision.questions)
    text = use(tools.look_around, target="Red box").content
    assert text.startswith("You already reached the red box.")
    assert len(car.looks) == looks and len(vision.questions) == questions


def test_look_around_asks_again_past_something_else_of_the_targets_colour(tools, car, vision):
    # a blue backpack in front of the blue box: the model pointed at the backpack every time
    vision.answers = ["left", "middle"]            # where is the blue box? / other than the backpack?
    vision.name = ["blue backpack", "blue box"]
    vision.crop_name = "blue backpack"             # its bounding box shows the backpack too
    text = use(tools.look_around, target="blue box").content
    assert "seen with the camera at pan 0, middle of the picture" in text
    assert any("other than the blue backpack?" in q for q in vision.questions)


def test_asking_again_that_points_at_the_same_place_is_not_a_sighting(tools, car, vision):
    vision.answers = ["left", "left"] + ["no"] * 4
    vision.name = ["blue backpack"]
    assert "not seen" in use(tools.look_around, target="blue box").content


@pytest.mark.parametrize("name, target", [("blue container", "blue box"), ("white sheet", "white bed"),
                                          ("white mattress", "white bed")])
def test_names_the_real_model_gave_the_targets(name, target):
    assert picar_tools.names_target(name, target)


@pytest.mark.parametrize("name", ["white blanket", "white pillow", "white duvet"])
def test_things_of_their_own_are_not_the_bed(name):
    # the real car took a white blanket for the "white bed" (2026-11-20)
    assert not picar_tools.names_target(name, "white bed")


def test_look_around_stays_within_the_heads_range(car, vision):
    # the real car's head turns only about 50 degrees: asked for 90 it stopped there, and
    # look_around told the AI to turn about 40 degrees too far (2026-11-20)
    tools = make_tools(car, vision, dataclasses.replace(TWIN, max_pan_deg=50.0))
    use(tools.look_around, target="red box")
    assert [pan for pan, _ in car.looks] == [0.0, 50.0, -50.0, 0.0]


def test_a_search_ends_after_a_full_circle_and_open_space(tools, car, vision):
    # the AI turned and looked for minutes for a "white bed" it never found (real car, 2026-11-17)
    texts = [use(tools.look_around, target="red box").content for _ in range(4)]
    assert "turn 120 and call look_around again" in texts[0] and "turn 120" in texts[1]
    # then explore looks for it while driving around (before 2026-11-25: one blind drive into open space)
    assert 'looked all the way around. Call explore with target "red box"' in texts[2]
    assert "finish with DONE and tell the user you could not find the red box" in texts[3]


def test_finding_the_target_or_searching_another_starts_the_count_again(tools, car, vision):
    use(tools.look_around, target="red box")
    use(tools.look_around, target="red box")
    vision.answers = ["middle"]
    use(tools.look_around, target="red box")           # found
    texts = [use(tools.look_around, target="red box").content for _ in range(2)]
    assert all("turn 120" in t for t in texts)
    use(tools.look_around, target="red box")
    assert "turn 120" in use(tools.look_around, target="blue box").content   # another target


def test_a_close_look_at_the_targets_box_tells_it_from_what_stands_in_front(tools, car, vision):
    # a white plastic bag in front of the white bed: "the white thing in the middle" was always
    # the bag, so the bed was never accepted (real car, 2026-11-20)
    vision.answers = ["middle"]
    vision.name = "white bag"
    vision.crop_name = "white bed"
    vision.box_bottom = ["lower third"]               # the bed's box, in the middle of the picture
    text = use(tools.look_around, target="white bed").content
    assert "seen with the camera at pan 0, middle of the picture" in text
    assert any(q.startswith(NAME_CROP_START) for q in vision.questions)


def test_the_first_sighting_is_kept_and_cut_out_only_when_needed(tools, car, vision):
    vision.answers = ["middle"]
    vision.name = "white bed"
    use(tools.look_around, target="white bed")
    assert not any(q.startswith(LOCATE_START) for q in vision.questions)   # named: no more questions
    (sight,) = tools.task.first_seen.values()
    assert sight.where == "middle" and sight.crop is None


def test_later_a_sighting_the_name_cannot_settle_is_compared_with_the_first(tools, car, vision):
    # Ferenc's idea (2026-11-20): with a white bag in front of the white bed, the close look
    # is not asked for a name any more: the bed's box is compared with its first sighting
    vision.answers = ["middle", "middle"]
    vision.name = ["white bed", "white bag"]      # the first sighting, then the bag in front
    use(tools.look_around, target="white bed")
    text = use(tools.look_around, target="white bed").content
    assert "seen with the camera at pan 0, middle of the picture" in text
    assert sum(q.startswith(SAME_START) for q in vision.questions) == 1
    assert not any(q.startswith(NAME_CROP_START) for q in vision.questions)
    assert vision.pictures[vision.questions.index(next(q for q in vision.questions
                                                       if q.startswith(SAME_START)))] == 2


def test_a_box_unlike_the_first_sighting_is_no_sighting_whatever_its_name(tools, car, vision):
    # the real car took a narrow white box at pan -50 (the wardrobe?) for the "white bed"
    vision.answers = ["middle", "middle"]
    vision.name = ["white bed", "white thing"]
    vision.crop_name = "white bed"                # a close look would have accepted it
    vision.same = ["no"]
    use(tools.look_around, target="white bed")
    assert "not seen" in use(tools.look_around, target="white bed").content


def test_first_sightings_are_forgotten_when_reached_and_when_the_task_ends(tools, car, vision):
    vision.answers = ["middle", "yes, red box", "middle"]
    use(tools.look_around, target="red box")
    car.results = [("obstacle", 0.5, 0.2)]
    use(tools.drive, distance_m=0.6)
    use(tools.reached, target="red box", next_target="blue box")
    assert "red box" not in tools.task.first_seen
    use(tools.look_around, target="blue box")
    assert list(tools.task.first_seen) == ["blue box"]
    tools.new_task("go to the yellow box")
    assert tools.task.first_seen == {}


def test_the_last_target_reached_forgets_every_first_sighting(tools, car, vision):
    vision.answers = ["middle", "middle", "yes, red box"]
    use(tools.look_around, target="blue box")
    use(tools.look_around, target="red box")
    car.results = [("obstacle", 0.5, 0.2)]
    use(tools.drive, distance_m=0.6)
    use(tools.reached, target="red box")
    assert tools.task.first_seen == {}


def test_a_close_look_that_shows_something_else_is_no_sighting(tools, car, vision):
    vision.answers = ["middle"]
    vision.name = vision.crop_name = "white bag"
    assert "not seen" in use(tools.look_around, target="white bed").content


def test_the_close_look_is_at_the_box_only(tools, car, vision):
    picture = {"images": [base64.b64encode(png_of(np.zeros((100, 200, 3), np.uint8))).decode()]}
    part = picar_tools.cut_out(picture, 0.25, 0.5, 0.75, 1.0)
    png = Image.open(io.BytesIO(base64.b64decode(part["images"][0])))
    # x 0.2-0.8 of 200 (+1), y 0.45-1.0 of 100: 121 x 55 with the 5 % margin (cut at the
    # bottom edge), then enlarged so its short side is 448
    assert png.size == (round(121 * 448 / 55), 448)


def test_with_the_picture_log_on_a_task_keeps_every_picture_and_question(car, vision, tmp_path):
    log = picar_tools.PictureLog(tmp_path / "runs")
    tool_list, start_task, _ = picar_tools.make_picar_tools(car, vision, TWIN, car.clock, picture_log=log)
    tools = {t.name: t for t in tool_list}
    start_task("find the red box")
    use(tools["look_around"], target="red box")   # five pictures, one question each, not seen
    (folder,) = (tmp_path / "runs").iterdir()
    assert folder.name.endswith("_find-the-red-box")
    assert len(list(folder.glob("*.jpg"))) == 5
    assert "Is there a red box in this picture?" in (folder / "log.txt").read_text()


@pytest.mark.parametrize("name, target, right", [
    ("black slipper", "black sliper", True),      # the user's typo (real car, 2026-11-20)
    ("black shoe", "black slipper", True),        # the shoe family
    ("black slippers", "black slipper", True),    # a plural
    ("blue boxes", "blue box", True),
    ("black object", "black slipper", False),
    ("black cable", "black slipper", False),
    ("brown fox", "brown box", False),            # short words: no one-letter slack
    ("red candle", "red can", False),             # a whole word, not a part of one
])
def test_names_allow_a_typo_a_plural_and_a_family(name, target, right):
    assert picar_tools.names_target(name, target) == right


# ---------- explore ----------

def test_explore_looks_around_at_every_stop_and_drives_on_where_free(tools, car, vision):
    # "drive around and tell me what you saw" (Ferenc, 2026-11-25)
    car.range_m = 2.0
    vision.answers = ["a white bed", "a desk", "a door", "a red box", "a wall", "a chair"]
    text = use(tools.explore, stops=2).content
    assert car.moves == [move(1.0, 0.0, 0.5 / 0.2)]          # between the two stops only
    assert ("Stop 1: ahead: a white bed; to the left: a desk; to the right: a door. Then drove on 0.5 m. "
            "Stop 2: ahead: a red box; to the left: a wall; to the right: a chair.") in text
    assert "answer the user, starting with DONE, with a short summary" in text
    assert sum(q == picar_tools.DESCRIBE for q in vision.questions) == 6


def test_explore_turns_left_where_the_way_is_not_free_and_neither_side_is_clearly_freer(tools, car, vision):
    car.range_m = 0.6                                        # less than 1 m free: no drive on
    text = use(tools.explore, stops=2).content               # "no" to "which side is freer?"
    assert car.moves == [move(1.0, 30.0, 90 / 200 / 0.2)]     # a 90 degree turn left
    assert "it was not clear which side was freer, so it turned 90 degrees left." in text


def test_explore_turns_towards_the_side_with_more_free_floor(tools, car, vision):
    # "is the floor free?" got 5 of 10 right on the real car's pictures, comparing the two sides 9 of
    # 10 (2026-11-25): asked in both orders, the answers must agree
    car.range_m = 0.6
    vision.answers = ["a bed", "a chair close", "open floor", "second", "first"]   # the right side, both orders
    text = use(tools.explore, stops=2).content
    assert car.moves == [move(1.0, -30.0, 90 / 200 / 0.2)]    # a 90 degree turn right
    assert "the right side looked freer, so it turned 90 degrees right." in text
    asked = [i for i, q in enumerate(vision.questions) if q == picar_tools.FREER]
    assert len(asked) == 2 and all(vision.pictures[i] == 2 for i in asked)


def test_explore_turns_left_when_the_two_orders_disagree(tools, car, vision):
    car.range_m = 0.6
    vision.answers = ["a bed", "a chair close", "open floor", "second", "second"]   # always "the second"
    assert "it was not clear which side was freer" in use(tools.explore, stops=2).content
    assert car.moves[0]["steering_deg"] == 30.0


def test_explore_stops_early_at_the_emergency_stop(tools, car, vision):
    car.range_m = 2.0
    car.results = [("estop", 0.0, 4.0)]
    text = use(tools.explore, stops=4).content
    assert len(car.moves) == 1 and "Stop 2" not in text
    assert "It stopped exploring early" in text and "The emergency stop is on" in text


def test_explore_with_a_target_stops_where_it_sees_it(tools, car, vision):
    # "find the red box somewhere in the room"
    car.range_m = 2.0
    vision.answers = ["no", "no", "no", "no", "no", "middle"]   # stop 1: five pans; stop 2: ahead
    text = use(tools.explore, stops=4, target="Red box").content
    assert len(car.moves) == 1                                # drove on once, then saw it
    assert text.startswith("Explored for the red box: at stop 2 it was seen with the camera at pan 0, "
                           "middle of the picture, about straight ahead. Call approach")
    assert not any(q == picar_tools.DESCRIBE for q in vision.questions)   # looks for it, not around


def test_explore_with_a_target_it_does_not_see_says_finish(tools, car, vision):
    car.range_m = 2.0
    text = use(tools.explore, stops=2, target="red box").content
    assert "Explored from 2 stops looking for the red box: not seen. Stop 1: not seen. Then drove on" in text
    assert "Finish with DONE and tell the user you could not find the red box." in text
    assert "could not find the red box" in use(tools.look_around, target="red box").content   # no more searching


def test_explore_with_a_target_turns_by_its_side_pictures(car, vision):
    # the real car's head turns 40 degrees: its side pictures are at 40 and -40, not 45
    tools = make_tools(car, vision, dataclasses.replace(TWIN, max_pan_deg=40.0))
    car.range_m = 0.6
    vision.answers = ["no", "no", "no", "second", "first"]   # not seen; then the right side is freer
    use(tools.explore, stops=2, target="red box")
    assert car.moves[0]["steering_deg"] == -30.0


def test_explore_saves_the_main_things_it_saw_for_later_tasks(tools, car, vision):
    # the AI wrote "I remember ..." and never called remember (real car, 2026-11-25)
    car.range_m = 2.0
    vision.answers = ["a white bed with a beige sheet is far, while a blue box and black clothes are near",
                      "a wooden dresser is near", "a white bed is far, while a radiator is near"]
    text = use(tools.explore, stops=1).content
    assert "The main things are saved for later tasks." in text
    day = datetime.fromtimestamp(car.clock.time()).strftime("%Y-%m-%d")   # the fake clock's day, here
    assert picar_tools.load_memory()[-1] == (f"Seen while exploring on {day}: a white bed with a beige sheet, "
                                             "a blue box and black clothes, a wooden dresser, a white bed, a radiator")


@pytest.mark.parametrize("description, things", [
    ("a white bed with a beige sheet is far, while a blue box and black clothes are near on the floor",
     ["a white bed with a beige sheet", "a blue box and black clothes"]),
    ("a black slipper with white patterns rests near the floor, while a white drawer is farther back",
     ["a black slipper with white patterns", "a white drawer"]),
    ("a white cabinet door, near, and a dark brown floor, far", ["a white cabinet door", "a dark brown floor"]),
])
def test_the_things_a_description_names(description, things):
    assert picar_tools.main_things(description) == things


def test_explore_without_a_target_during_a_search_looks_for_that_target(tools, car, vision):
    # the AI called explore without the black slipper while finding it (real car, 2026-11-25)
    car.range_m = 2.0
    use(tools.look_around, target="red box")              # the search: not seen
    vision.answers = ["no", "no", "no", "no", "no", "middle"]
    text = use(tools.explore, stops=3).content
    assert text.startswith("Explored for the red box: at stop 2 it was seen")


def test_explore_looks_from_at_most_8_stops(tools, car, vision):
    car.range_m = 2.0
    use(tools.explore, stops=20)
    assert len(car.moves) == 7


# ---------- approach ----------

@pytest.mark.parametrize("range_m, step_m", [
    (1.0, 0.6),          # free up to the stop distance, at most 0.6 m a step
    (0.5, 0.3),          # 0.5 - 0.25 stop distance + 0.05
    (0.3, 0.2),          # never less than 0.2 m
    (math.inf, 0.6),     # nothing in range
    (-math.inf, 0.2),    # too close to measure
    (math.nan, 0.4),     # no reading: a careful step
])
def test_approach_sizes_a_straight_step_by_the_distance_sensor(tools, car, vision, range_m, step_m):
    car.range_m = range_m
    car.results = [("obstacle", 1.0, 0.2)]
    vision.answers = ["middle"]
    use(tools.approach, target="red box")
    assert car.moves == [move(1.0, 0.0, step_m / 0.2)]


def test_approach_looks_again_after_the_cars_approach_step(car, vision):
    # the real car: a 0.6 m step lost the white bed when the car pulled to one side (2026-11-20)
    tools = make_tools(car, vision, dataclasses.replace(TWIN, approach_step_m=0.3))
    car.range_m = 2.0                      # free far ahead: the twin would drive 0.6 m
    car.results = [("done", 1.0, 4.0), ("obstacle", 0.5, 0.2)]
    vision.answers = ["middle", "middle", "yes, red box"]
    text = use(tools.approach, target="red box").content
    assert car.moves[0] == move(1.0, 0.0, 0.3 / 0.2)
    assert text.startswith("Approach to the red box: drove 0.30 m, drove 0.15 m.")


def test_turns_drive_at_the_cars_turn_power_and_speed(car, vision):
    # at full lock the real car's wheels barely rolled at the forward power (2026-11-24)
    tools = make_tools(car, vision, dataclasses.replace(TWIN, turn_power=0.4, turn_speed_mps=0.1))
    use(tools.turn, angle_deg=20)         # 0.1 m at full steering
    use(tools.drive, distance_m=0.2)      # straight: the forward power
    assert car.moves == [move(1.0, 30.0, 1.0), move(1.0, 0.0, 1.0)]


def test_approach_drives_until_the_safety_stop_then_reached_counts(tools, car, vision):
    car.range_m = 1.0
    car.results = [("done", 1.0, 4.0), ("obstacle", 0.5, 0.2)]
    vision.answers = ["middle", "middle", "yes, red box"]
    text = use(tools.approach, target="Red box").content
    assert car.moves == [move(1.0, 0.0, 3.0), move(1.0, 0.0, 3.0)]
    assert text.startswith("Approach to the red box: drove 0.60 m, drove 0.30 m. Safety stop: "
                           "something is 0.20 m in front of the bumper.")
    assert 'The camera check says: "yes, red box". If that is the red box, you have arrived' in text
    assert use(tools.reached, target="red box").content.startswith("Noted: red box.")


def test_approach_turns_towards_a_target_at_the_side_and_reverses_if_that_did_not_help(
        tools, car, vision):
    car.results = [("done", 1.0, 4.0), ("done", 1.0, 4.0), ("obstacle", 1.0, 0.2)]
    vision.answers = ["left", "left", "middle"]
    text = use(tools.approach, target="red box").content
    # 20 degrees at 200 deg/m = 0.1 m at full steering, then the same while reversing
    assert car.moves == [move(1.0, 30.0, 0.5), move(-1.0, -30.0, 0.5), move(1.0, 0.0, 3.0)]
    assert text.startswith("Approach to the red box: turned left, turned left reversing, drove 0.60 m.")


def test_approach_backs_up_towards_a_target_it_lost(tools, car, vision):
    vision.answers = ["right", "no", "no"]
    vision.box_bottom = ["none"]   # and the model cannot locate it either
    text = use(tools.approach, target="red box").content
    # lost after a turn right: back up with the front swinging right (steering left)
    # (0.2 m since 2026-11-17, was 0.3: nothing sees behind the car)
    assert car.moves == [move(1.0, -30.0, 0.5), move(-1.0, 15.0, 1.0)]
    assert text == ("The red box is not in the picture straight ahead (turned right, lost it, "
                    "backed up 0.20 m). Call look_around to find it. Reached so far: nothing yet.")


def test_close_up_the_target_is_located_before_it_is_given_up(tools, car, vision):
    # the real car: after a drive at the blue box the model said "no blue box" (too close?)
    car.results = [("done", 1.0, 4.0), ("obstacle", 0.5, 0.24)]
    vision.answers = ["middle", "no", "yes, red box"]
    vision.box_bottom = ["middle", "lower third"]   # the box, then located after the "no"
    text = use(tools.approach, target="red box").content
    assert all(m["speed"] > 0 for m in car.moves) and len(car.moves) == 2   # no backing up
    assert "lost it" not in text and "Safety stop" in text


def test_a_box_that_shows_something_else_does_not_count_as_the_lost_target(tools, car, vision):
    # the real car: after a step the "white bed" box was the plastic bag in front of the lens,
    # and approach said "arrived by the camera" 1.5 m from the bed (2026-11-20)
    car.range_m = math.inf
    vision.answers = ["middle", "no", "no"]        # seen, then gone; not below the camera either
    vision.name = "white bed"
    vision.box_bottom = ["middle", "bottom edge"]     # the box, then the "bed" filling the bottom
    vision.same = ["no"]                           # not like the bed's first sighting
    text = use(tools.approach, target="white bed").content
    assert "right in front of the car" not in text and "lost it, backed up" in text
    assert any(q.startswith(SAME_START) for q in vision.questions)


def test_without_a_first_sighting_to_compare_the_close_look_names_the_lost_target(tools, car, vision):
    car.range_m = math.inf
    vision.answers = ["middle", "no", "no"]
    vision.name = "white bed"
    vision.box_bottom = ["middle", "bottom edge", "none"]   # its first sighting cannot be cut out
    vision.crop_name = "clear plastic bag"
    text = use(tools.approach, target="white bed").content
    assert "right in front of the car" not in text and "lost it, backed up" in text
    assert any(q.startswith(NAME_CROP_START) for q in vision.questions)


def test_right_after_look_around_named_it_a_vague_name_will_do(tools, car, vision):
    # close up, the real car's blue box became "blue object" and "blue wall" (2026-11-17)
    car.results = [("obstacle", 0.5, 0.24)]
    vision.answers = ["middle", "middle", "yes, blue box"]
    vision.name = ["blue box", "blue object"]   # look_around's sighting, then approach's
    use(tools.look_around, target="blue box")
    use(tools.approach, target="blue box")
    assert len(car.moves) == 1


@pytest.mark.parametrize("first", ["approach", "look_around"])
def test_without_a_sighting_just_before_a_vague_name_is_not_the_target(tools, car, vision, first):
    vision.answers = ["left"]
    vision.name = ["blue wall"]
    text = use(getattr(tools, first), target="blue box").content
    assert car.moves == [] and ("not in the picture" in text or "not seen" in text)


def test_a_target_gone_from_straight_ahead_is_looked_for_below_before_backing_up(tools, car, vision):
    # at tilt 0 the floor within ~0.4 m of the bumper is below the picture; the head tilts
    car.range_m = math.inf                    # the sensor does not hear it
    vision.answers = ["middle", "no", "middle"]   # seen, then gone, then seen looking down
    vision.box_bottom = ["middle", "none"]       # and its box is not found straight ahead either
    text = use(tools.approach, target="blue box").content
    assert (0.0, picar_tools.LOOK_DOWN_DEG) in car.looks
    assert all(m["speed"] > 0 for m in car.moves)   # did not back up
    assert "the camera shows it right in front of the car" in text


def test_looking_down_with_the_sensor_hearing_it_takes_one_small_step(tools, car, vision):
    car.range_m = 0.3                         # heard close: the safety stop will end the step
    car.results = [("done", 1.0, 0.3), ("obstacle", 0.5, 0.24)]
    vision.answers = ["middle", "no", "middle", "yes, blue box"]
    vision.box_bottom = ["middle", "none"]
    text = use(tools.approach, target="blue box").content
    assert car.moves[-1] == move(1.0, 0.0, picar_tools.APPROACH_CLOSE_M / 0.2)
    assert "saw it below the camera" in text and "Safety stop" in text


def test_below_the_camera_it_must_look_like_the_first_sighting(tools, car, vision):
    car.range_m = math.inf
    vision.answers = ["middle", "no", "middle"]   # seen, gone, "seen" looking down
    vision.box_bottom = ["middle", "none", "bottom edge"]   # not ahead; then the first sighting, the box below
    vision.same = ["no"]                          # but it is not the thing seen first
    text = use(tools.approach, target="blue box").content
    assert "the camera shows it right in front of the car" not in text and "lost it, backed up" in text


def test_not_below_either_it_backs_up_as_before(tools, car, vision):
    vision.answers = ["middle", "no", "no", "no"]
    vision.box_bottom = ["middle", "none"]
    text = use(tools.approach, target="red box").content
    assert car.moves[-1]["speed"] < 0 and "lost it, backed up" in text


def test_approach_without_the_target_in_the_picture_does_not_move(tools, car, vision):
    text = use(tools.approach, target="red box").content
    assert car.moves == []
    assert text.startswith("The red box is not in the picture straight ahead (did not move).")


def test_approach_stops_after_12_steps(tools, car, vision):
    vision.answers = ["middle"] * 12
    text = use(tools.approach, target="red box").content
    assert len(car.moves) == 12
    assert text.endswith("Not there yet: call approach again. Reached so far: nothing yet.")


def test_approach_points_the_camera_ahead_first(tools, car, vision):
    use(tools.look, pan_deg=45)
    use(tools.approach, target="red box")
    assert car.looks == [(45.0, 0.0), (0.0, 0.0)]


def test_approach_does_not_drive_to_a_reached_target(tools, car, vision):
    stop_in_front(tools, car, vision)
    use(tools.reached, target="red box")
    moves = len(car.moves)
    text = use(tools.approach, target="red box").content
    assert text.startswith("You already reached the red box.") and len(car.moves) == moves


# ---------- approach: arrived? (the rules listed in approach.py) ----------

def test_gone_from_the_middle_just_beyond_the_stop_a_close_up_of_the_first_sighting_has_arrived(
        tools, car, vision):
    # the real car: the bed's drawer filled the picture at about 0.27 m, just outside the 0.25 m
    # safety stop, and the model said "no white bed" about it (2026-11-24)
    car.range_m = 0.32                                   # within 0.25 + 0.1
    vision.answers = ["middle", "no"]                    # seen, then "gone"
    vision.name = "white bed"
    vision.box_bottom = ["upper half", "none", "upper half"]   # not located now; the first sighting is
    text = use(tools.approach, target="white bed").content
    assert "Something is 0.32 m in front of the bumper, and the camera shows it is the white bed" in text
    question = next(q for q in vision.questions if "a close-up of the same white bed?" in q)
    assert vision.pictures[vision.questions.index(question)] == 2
    assert all(m["speed"] > 0 for m in car.moves)        # did not back up
    assert use(tools.reached, target="white bed").content.startswith("Noted: white bed")


def test_a_close_up_unlike_the_first_sighting_is_not_arrival(tools, car, vision):
    car.range_m = 0.32
    vision.answers = ["middle", "no", "no"]              # seen, gone, not below either
    vision.name = "white bed"
    vision.box_bottom = ["upper half", "none", "upper half"]
    vision.same = ["no"]                                 # a plush toy in front, not the bed
    text = use(tools.approach, target="white bed").content
    assert "lost it, backed up" in text and "you have arrived" not in text


def test_gone_from_the_middle_with_nothing_close_asks_no_close_up(tools, car, vision):
    car.range_m = 1.0
    vision.answers = ["middle", "no", "no"]
    vision.name = "white bed"
    vision.box_bottom = ["upper half", "none", "upper half"]
    use(tools.approach, target="white bed")
    assert not any("a close-up of the same" in q for q in vision.questions)


def at_the_white_bed(vision, same):
    """The model sees the white bed straight ahead, its box filling the picture's width low down."""
    vision.name = "white bed"
    vision.box_x = (0.0, 1.0)
    vision.box_bottom = ["lower third"]      # down to 83 %
    vision.same = same


def test_a_target_filling_the_view_low_down_is_checked_before_driving_on(tools, car, vision):
    # the real car drove another 0.3 m into a slipper lying at the bed; close up its pictures no
    # longer looked like the bed, but at "fills the view" the model said "yes" (2026-11-24)
    car.range_m = 1.5                     # the sensor sees under the bed
    vision.answers = ["middle"]
    at_the_white_bed(vision, ["yes"])
    text = use(tools.approach, target="white bed").content
    assert car.moves == []
    assert "The camera shows the white bed right in front of the car, like when you first saw it." in text
    assert "1.50 m" not in text           # the sensor's reading under the bed would mislead
    assert use(tools.reached, target="white bed").content.startswith("Noted: white bed")


def test_filling_the_view_but_not_like_the_first_sighting_drives_on(tools, car, vision):
    car.range_m = 1.5
    car.results = [("obstacle", 0.5, 0.2)]
    vision.answers = ["middle"]
    at_the_white_bed(vision, ["no"])
    text = use(tools.approach, target="white bed").content
    assert len(car.moves) == 1 and "Safety stop" in text


def test_after_filling_the_view_left_or_right_asks_again_before_steering(tools, car, vision):
    # the real car corrected left and right six times with its nose at the bed (2026-11-24)
    car.range_m = 1.5
    vision.answers = ["middle", "left"]
    at_the_white_bed(vision, ["no", "no", "yes"])   # the picture, then its box: no; after the step: yes
    text = use(tools.approach, target="white bed").content
    assert len(car.moves) == 1 and car.moves[0]["steering_deg"] == 0.0   # no turn to the side
    assert "like when you first saw it" in text


def test_a_target_that_filled_the_view_and_vanished_is_checked_even_with_nothing_heard(tools, car, vision):
    car.range_m = 1.5
    vision.answers = ["middle", "no"]
    at_the_white_bed(vision, ["no", "no", "yes"])
    vision.box_bottom = ["lower third", "lower third", "none", "lower third"]   # gone: not located
    text = use(tools.approach, target="white bed").content
    assert "like when you first saw it" in text and "lost it" not in text


def test_with_something_lying_in_front_the_targets_box_alone_can_say_arrived(tools, car, vision):
    # the real car: a slipper lay at the bed's drawer; the whole picture never looked like the bed,
    # only the bed's box did (2026-11-24)
    car.range_m = 1.5
    vision.answers = ["middle"]
    at_the_white_bed(vision, ["no", "yes"])   # the whole picture: no; the bed's box alone: yes
    text = use(tools.approach, target="white bed").content
    assert car.moves == [] and "with something lying in front of it: this is as close as the car can get" in text
    asked = [i for i, q in enumerate(vision.questions) if q.startswith(SAME_START)]
    assert len(asked) == 2 and all(vision.pictures[i] == 2 for i in asked)
    # the whole picture: a close-up of the first sighting? Then only the box: the same as it?
    assert "a close-up of the same white bed?" in vision.questions[asked[0]]
    assert "Is the thing in the second picture the same white bed?" in vision.questions[asked[1]]
    assert use(tools.reached, target="white bed").content.startswith("Noted: white bed")


def test_a_big_target_filling_the_view_with_something_near_has_arrived_without_asking(tools, car, vision):
    # Ferenc's choice (2026-11-24): faster, though a wrong arrival is more likely
    car.range_m = 0.32
    vision.answers = ["middle"]
    at_the_white_bed(vision, ["no"])      # the close-up question would say no: it is not asked
    text = use(tools.approach, target="white bed").content
    assert car.moves == []
    assert "Something is 0.32 m in front of the bumper, and the camera shows it is the white bed, filling" in text
    assert not any("a close-up of the same" in q for q in vision.questions)
    assert use(tools.reached, target="white bed").content.startswith("Noted: white bed")


def three_short_approaches(tools, monkeypatch, target="red box"):
    """Three approach calls of one step each (none arrives); returns their texts."""
    monkeypatch.setattr(approach, "APPROACH_MAX_STEPS", 1)
    return [use(tools.approach, target=target).content for _ in range(3)]


def test_after_three_approaches_close_to_the_target_it_counts_as_arrived(tools, car, vision, monkeypatch):
    # the real car looped approach, back up, approach for four minutes at the bed (2026-11-24)
    vision.answers = ["middle"] * 3
    vision.box_bottom = ["lower third"]      # close: down in the lower third of the picture
    texts = three_short_approaches(tools, monkeypatch)
    assert "Not there yet" in texts[0] and "Not there yet" in texts[1]
    assert "After 3 tries the red box is close in front of the car" in texts[2]
    assert use(tools.reached, target="red box").content.startswith("Noted: red box")


def test_after_three_approaches_far_from_the_target_it_says_finish(tools, car, vision, monkeypatch):
    vision.answers = ["middle"] * 3
    vision.box_bottom = ["upper half"]       # still far
    texts = three_short_approaches(tools, monkeypatch)
    assert "do not call approach again. Finish with DONE" in texts[2]
    moves = len(car.moves)
    fourth = use(tools.approach, target="red box").content
    assert fourth.startswith("You have tried to approach the red box 3 times without arriving")
    assert len(car.moves) == moves        # it does not drive again


def test_the_approach_count_starts_again_for_another_target(tools, car, vision, monkeypatch):
    monkeypatch.setattr(approach, "APPROACH_MAX_STEPS", 1)
    vision.answers = ["middle"] * 3
    use(tools.approach, target="red box")
    use(tools.approach, target="red box")
    assert "Not there yet" in use(tools.approach, target="blue box").content


# ---------- reached ----------

def test_reached_needs_the_safety_stop(tools):
    text = use(tools.reached, target="red box").content
    assert text.startswith("Not reached yet: the safety system has not stopped the car")
    assert "Reached so far: nothing yet." in text


def test_reached_counts_each_target_once_and_names_the_next(tools, car, vision):
    stop_in_front(tools, car, vision)
    text = use(tools.reached, target="Red box", next_target="yellow box").content
    assert text == ("Noted: red box. Reached so far: red box. Next target: yellow box. "
                    "Back up 0.2 m from the red box (drive with distance_m -0.2), then call look_around "
                    "for the yellow box.")
    again = use(tools.reached, target="red box").content
    assert again.startswith("You already reached the red box. Do not drive to it again")


def test_reached_the_last_target(tools, car, vision):
    stop_in_front(tools, car, vision, seen="yes, blue cylinder")
    text = use(tools.reached, target="blue cylinder").content
    assert "If that was the last target, finish with DONE." in text


def test_reached_is_turned_down_once_when_the_camera_sees_something_else(tools, car, vision):
    vision.answers = ["middle"]
    use(tools.look_around, target="red box")   # the target the tools know about
    stop_in_front(tools, car, vision, seen="no, white wall")
    first = use(tools.reached, target="red box").content
    assert first.startswith('Not reached yet: the camera check says the thing in front of the car '
                            'is not the red box: "no, white wall".')
    assert use(tools.reached, target="red box").content.startswith("Noted: red box.")


def test_pictures_show_the_progress_until_the_car_moves_away(tools, car, vision):
    stop_in_front(tools, car, vision)
    use(tools.reached, target="red box", next_target="yellow box")
    text = use(tools.look).content
    assert "Reached so far: red box. Next target: yellow box." in text
    assert "You are still at the red box: back up 0.2 m (drive with distance_m -0.2) and turn away" in text
    text = use(tools.drive, distance_m=-0.3).content
    assert "still at" not in text and "Reached so far: red box." in text


def test_a_new_task_forgets_the_targets(tools, car, vision):
    stop_in_front(tools, car, vision)
    use(tools.reached, target="red box")
    tools.new_task()
    assert "Reached so far: nothing yet." in use(tools.look).content


# ---------- remember ----------

def test_remember_keeps_the_newest_20_facts_across_restarts(tools, car, vision):
    for n in range(25):
        assert use(tools.remember, fact=f"fact {n}").content == "Saved."
    restarted = make_tools(car, vision)
    assert restarted.memory == [f"fact {n}" for n in range(5, 25)]


def test_a_broken_memory_file_starts_empty(memory_file):
    with open(memory_file, "w") as f:
        f.write("{not json")
    assert picar_tools.load_memory() == []
    with open(memory_file, "w") as f:
        f.write('{"not": "a list"}')
    assert picar_tools.load_memory() == []
    picar_tools.save_memory(["a fact"])
    with open(memory_file) as f:
        assert json.load(f) == ["a fact"]


def test_memory_is_next_to_agent_py_whatever_folder_the_agent_starts_in():
    assert AGENT_MEMORY_FILE == str(SHARED / "picar_agent" / "memory.json")


def test_remember_can_use_another_memory_file(car, vision, tmp_path):
    other = tmp_path / "other.json"
    tool_list, _, memory = picar_tools.make_picar_tools(car, vision, TWIN, car.clock, str(other))
    next(t for t in tool_list if t.name == "remember").invoke({"fact": "the red box is ahead"})
    assert json.loads(other.read_text()) == ["the red box is ahead"]
