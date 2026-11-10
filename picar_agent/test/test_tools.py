"""The agent's tools (drive, turn, look, look_around, reached, remember) against a fake car.

FakeCar answers like picar_base and records every request, FakeVision gives scripted
answers to the tools' picture questions, and FakeTime makes the waits instant.
"""

import base64
import io
import json
from types import SimpleNamespace

import numpy as np
import pytest
from PIL import Image
from rai.communication.ros2 import ROS2Message

import picar_tools


class FakeTime:
    """Stands in for the time module inside picar_tools: sleep() returns at once and
    moves the clock on, so the tests are fast and the timing is exact."""

    def __init__(self):
        self.now = 1000.0
        self.slept = []

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.slept.append(seconds)
        self.now += seconds


class FakeCar:
    """Stands in for the ROS2Connector: records what the tools send to picar_base
    and answers like picar_base would.

    results: what the next /picar/move calls return, one entry per call:
        (reason, part, min_range_m)  the move ends with this reason after driving
                                     this part (0 to 1) of the asked time
        None                         picar_base does not answer
    Once the list is empty, every move drives the whole way ("done").
    """

    def __init__(self, clock):
        self.clock = clock
        self.looks = []     # (pan_deg, tilt_deg) of every /picar/look call
        self.moves = []     # every /picar/move request: speed, steering_deg, duration_s
        self.results = []
        self.calls = []     # "look" and "move", in the order they came
        self.image = SimpleNamespace(height=2, width=2, encoding="rgb8", data=bytes(12))

    def receive_message(self, topic, **kwargs):
        assert topic == "/camera/image_raw"
        return ROS2Message(payload=self.image)

    def service_call(self, message, target, msg_type, **kwargs):
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
        return ROS2Message(payload=SimpleNamespace(
            completed=reason == "done", reason=reason, min_range_m=closest))


class FakeVision:
    """Stands in for the AI model's answers to the tools' picture questions.
    answers are used in order; after them every answer is "no"."""

    def __init__(self):
        self.answers = []
        self.questions = []

    def invoke(self, messages, config=None):
        self.questions.append(messages[0].content[0]["text"])
        return SimpleNamespace(content=self.answers.pop(0) if self.answers else "no")


@pytest.fixture
def clock(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)   # memory.json is written here, not into picar_agent/
    fake = FakeTime()
    monkeypatch.setattr(picar_tools, "time", fake)
    return fake


@pytest.fixture
def car(clock):
    return FakeCar(clock)


@pytest.fixture
def vision():
    return FakeVision()


@pytest.fixture
def tools(car, vision):
    return make_tools(car, vision)


def make_tools(car, vision):
    tool_list, new_task, memory = picar_tools.make_picar_tools(car, vision)
    return SimpleNamespace(**{t.name: t for t in tool_list}, new_task=new_task, memory=memory)


def use(tool, **args):
    """Call a tool the way the agent does. Returns the tool message:
    .content is the text, .artifact the picture (for the tools that take one)."""
    return tool.invoke({"name": tool.name, "args": args, "id": "call-1", "type": "tool_call"})


def move(speed, steering_deg, duration_s):
    return {"speed": speed, "steering_deg": steering_deg, "duration_s": pytest.approx(duration_s)}


def pixel(picture):
    """The top-left pixel of a tool's picture, as (red, green, blue)."""
    png = base64.b64decode(picture["images"][0])
    return tuple(int(v) for v in np.array(Image.open(io.BytesIO(png)).convert("RGB"))[0, 0])


# ---------- drive ----------

def test_drive_asks_picar_base_for_full_speed_for_the_right_time(tools, car):
    text = use(tools.drive, distance_m=0.5, steering_deg=10).content
    assert car.moves == [move(1.0, 10.0, 2.5)]   # 0.5 m at 0.2 m/s
    assert text.startswith("Drove +0.50 m with steering +10, the heading changed about +33 degrees")


def test_drive_backward(tools, car):
    text = use(tools.drive, distance_m=-0.3).content
    assert car.moves == [move(-1.0, 0.0, 1.5)]
    assert text.startswith("Drove -0.30 m")


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
    assert len(vision.questions) == 1   # the car has not moved since, so it is the same thing


def test_the_camera_check_names_the_target_being_searched(tools, car, vision):
    vision.answers = ["middle"]
    use(tools.look_around, target="red box")
    car.results = [("obstacle", 0.5, 0.2)]
    text = use(tools.drive, distance_m=0.6).content
    assert "Is it the red box?" in vision.questions[-1]
    assert text.startswith("Drove +0.30 m") and "Safety stop: something is 0.20 m" in text
    assert "If that is the red box, you have arrived: call reached now." in text


def test_drive_stopped_early_says_why(tools, car):
    car.results = [("no_range", 0.5, 4.0)]
    text = use(tools.drive, distance_m=0.5).content
    assert text.startswith("Drove +0.25 m")
    assert "Stopped early: the distance sensor had no fresh reading, just try again." in text


def test_no_answer_from_picar_base(tools, car):
    car.results = [None]
    text = use(tools.drive, distance_m=0.5).content
    assert "picar_base did not answer." in text


# ---------- turn ----------

def test_turn_drives_at_full_steering_for_the_angle(tools, car):
    text = use(tools.turn, angle_deg=90).content
    assert car.moves == [move(1.0, 30.0, 2.25)]   # 90 deg at 200 deg/m = 0.45 m
    assert "the heading changed about +90 degrees" in text


def test_turn_right(tools, car):
    text = use(tools.turn, angle_deg=-45).content
    assert car.moves == [move(1.0, -30.0, 1.125)]
    assert "the heading changed about -45 degrees" in text


def test_turn_is_limited_to_120_degrees(tools, car):
    use(tools.turn, angle_deg=180)
    assert car.moves == [move(1.0, 30.0, 3.0)]


def test_blocked_turn_reverses_the_rest_with_the_steering_the_other_way(tools, car):
    car.results = [("obstacle", 0.25, 0.2)]   # a quarter of the 0.45 m, then blocked
    text = use(tools.turn, angle_deg=90).content
    assert car.moves == [move(1.0, 30.0, 2.25), move(-1.0, -30.0, 2.25 * 0.75)]
    assert text.startswith("Blocked in front, so I turned the rest while reversing. "
                           "The heading changed about +90 degrees")


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
    assert "about straight ahead. Drive at it." in text


def test_look_around_not_seen(tools, car, vision):
    vision.answers = ["no, it is not on the left", "none", "nothing", "not here", "no"]
    text = use(tools.look_around, target="red box").content
    assert "with the camera at pan 0, 45, -45, 90, -90: not seen" in text
    assert "turn 120 and call look_around again" in text
    assert car.looks[-1] == (0.0, 0.0)


def test_look_around_without_a_vision_model_sees_nothing(car):
    tools = make_tools(car, None)
    assert "not seen" in use(tools.look_around, target="red box").content


def test_look_around_does_not_search_for_a_reached_target(tools, car, vision):
    use(tools.reached, target="red box")
    text = use(tools.look_around, target="Red box").content
    assert text.startswith("You already reached the red box.")
    assert car.looks == [] and vision.questions == []


# ---------- reached ----------

def test_reached_counts_each_target_once_and_names_the_next(tools):
    text = use(tools.reached, target="Red box", next_target="yellow box").content
    assert text == ("Noted: red box. Reached so far: red box. Next target: yellow box. "
                    "Back up from the red box, then call look_around for the yellow box.")
    again = use(tools.reached, target="red box").content
    assert again.startswith("You already reached the red box. Do not drive to it again")


def test_reached_the_last_target(tools):
    text = use(tools.reached, target="blue cylinder").content
    assert "If that was the last target, finish with DONE." in text


def test_pictures_show_the_progress_until_the_car_moves_away(tools):
    use(tools.reached, target="red box", next_target="yellow box")
    text = use(tools.look).content
    assert "Reached so far: red box. Next target: yellow box." in text
    assert "You are still at the red box: back up and turn away from it." in text
    text = use(tools.drive, distance_m=-0.3).content
    assert "still at" not in text and "Reached so far: red box." in text


def test_a_new_task_forgets_the_targets(tools):
    use(tools.reached, target="red box")
    tools.new_task()
    assert "Reached so far: nothing yet." in use(tools.look).content


# ---------- remember ----------

def test_remember_keeps_the_newest_20_facts_across_restarts(tools, car, vision):
    for n in range(25):
        assert use(tools.remember, fact=f"fact {n}").content == "Saved."
    restarted = make_tools(car, vision)
    assert restarted.memory == [f"fact {n}" for n in range(5, 25)]


def test_a_broken_memory_file_starts_empty(clock):
    with open(picar_tools.MEMORY_FILE, "w") as f:
        f.write("{not json")
    assert picar_tools.load_memory() == []
    picar_tools.save_memory(["a fact"])
    with open(picar_tools.MEMORY_FILE) as f:
        assert json.load(f) == ["a fact"]
