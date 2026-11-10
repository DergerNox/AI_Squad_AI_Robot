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
from types import SimpleNamespace

import numpy as np
import pytest
from PIL import Image
from rai.communication.ros2 import ROS2Message

import camera
import car_config
import picar_tools

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
        if self.next_image is not None:   # the head or the car moves: the camera sees something new
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


AGENT_MEMORY_FILE = picar_tools.MEMORY_FILE   # the real one, before the fixture below


@pytest.fixture(autouse=True)
def memory_file(monkeypatch, tmp_path):
    """Every test keeps its facts in a memory.json of its own, never the agent's."""
    path = str(tmp_path / "memory.json")
    monkeypatch.setattr(picar_tools, "MEMORY_FILE", path)
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


def stop_in_front(tools, car, vision, seen="yes, red box"):
    """Drive into something: the safety stop holds the car and the camera check says `seen`."""
    car.results = [("obstacle", 0.0, 0.2)]
    vision.answers = [seen]
    use(tools.drive, distance_m=0.3)


# ---------- the car's numbers ----------

# Another car: every number differs from the twin's
OTHER = dataclasses.replace(TWIN, speed_mps=0.25, max_move_s=2.0, max_steering_deg=25.0,
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


def test_a_big_turn_goes_half_forward_half_reversing(tools, car):
    text = use(tools.turn, angle_deg=90).content
    # 0.45 m in all: half forward steering left, half reversing steering right
    assert car.moves == [move(1.0, 30.0, 1.125), move(-1.0, -30.0, 1.125)]
    assert text.startswith("Turned half forward, half reversing. The heading changed about +90 degrees")


def test_turn_is_limited_to_120_degrees(tools, car):
    use(tools.turn, angle_deg=180)
    assert car.moves == [move(1.0, 30.0, 1.5), move(-1.0, -30.0, 1.5)]


def test_blocked_turn_reverses_the_rest_with_the_steering_the_other_way(tools, car):
    car.results = [("obstacle", 0.4, 0.2)]   # 0.1 m of the 0.25 m, then blocked
    text = use(tools.turn, angle_deg=50).content
    assert car.moves == [move(1.0, 30.0, 1.25), move(-1.0, -30.0, 0.75)]
    assert text.startswith("Blocked in front, so I turned the rest while reversing. "
                           "The heading changed about +50 degrees")


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


def test_look_around_does_not_search_for_a_reached_target(tools, car, vision):
    stop_in_front(tools, car, vision)
    use(tools.reached, target="red box")
    looks, questions = len(car.looks), len(vision.questions)
    text = use(tools.look_around, target="Red box").content
    assert text.startswith("You already reached the red box.")
    assert len(car.looks) == looks and len(vision.questions) == questions


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
    text = use(tools.approach, target="red box").content
    # lost after a turn right: back up with the front swinging right (steering left)
    assert car.moves == [move(1.0, -30.0, 0.5), move(-1.0, 15.0, 1.5)]
    assert text == ("The red box is not in the picture straight ahead (turned right, lost it, "
                    "backed up 0.30 m). Call look_around to find it. Reached so far: nothing yet.")


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


# ---------- reached ----------

def test_reached_needs_the_safety_stop(tools):
    text = use(tools.reached, target="red box").content
    assert text.startswith("Not reached yet: the safety system has not stopped the car")
    assert "Reached so far: nothing yet." in text


def test_reached_counts_each_target_once_and_names_the_next(tools, car, vision):
    stop_in_front(tools, car, vision)
    text = use(tools.reached, target="Red box", next_target="yellow box").content
    assert text == ("Noted: red box. Reached so far: red box. Next target: yellow box. "
                    "Back up from the red box, then call look_around for the yellow box.")
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
    assert "You are still at the red box: back up and turn away from it." in text
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
