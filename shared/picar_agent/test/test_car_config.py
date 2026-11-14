"""car_config: the car's numbers from its settings file, and the prompt made from them."""

import dataclasses
import pathlib

import pytest
import rclpy
import yaml
from rclpy.context import Context

import car_config
from prompt import picar_prompt

SHARED = pathlib.Path(__file__).resolve().parents[2]   # shared/picar_agent/test -> shared
TWIN_FILE = SHARED / "picar_base" / "config" / "sim.yaml"
REAL_FILE = SHARED.parent / "picar" / "picar_car" / "config" / "car.yaml"
SOURCES = {"picar_base": SHARED / "picar_base", "picar_car": SHARED.parent / "picar" / "picar_car"}


def share(package):
    """The source folder stands in for the installed share folder (the same relative paths)."""
    if package not in SOURCES:
        raise car_config.CarConfigError(f"package '{package}' is not built or not sourced")
    return str(SOURCES[package])


def words(text):
    """The text with every line break and run of spaces as one space."""
    return " ".join(text.split())


# ---------- the settings files ----------

def test_the_twins_numbers():
    car = car_config.load_car(str(TWIN_FILE))
    assert car.description == "a simulated Adeept PiCar-B in a room in Gazebo"
    assert car.speed_mps == pytest.approx(0.2)          # full_speed_mps 0.5 x max_speed 0.4
    assert car.max_drive_m == pytest.approx(0.6)        # 0.2 m/s x max_move_s 3
    assert (car.max_steering_deg, car.max_pan_deg, car.max_tilt_deg) == (30.0, 90.0, 40.0)
    assert car.stop_distance_m == 0.25
    assert (car.turn_deg_per_m, car.head_deg_per_s, car.camera_fov_deg) == (200.0, 150.0, 62.0)
    assert car.floor[0] == (1.0, 0.35) and car.floor[-1] == (0.6, 1.9) and len(car.floor) == 7
    assert car.own_parts == "black wheels, green body"


def test_the_real_car_starts_with_the_twins_numbers():
    twin, real = car_config.load_car(str(TWIN_FILE)), car_config.load_car(str(REAL_FILE))
    assert real.description == "an Adeept PiCar-B, a real car in a real room"
    different = {field.name for field in dataclasses.fields(twin)
                 if getattr(twin, field.name) != getattr(real, field.name)}
    assert different == {"file", "description", "own_parts", "camera_topic"}   # until it is measured


def test_the_real_car_sends_compressed_pictures_the_twin_raw_ones():
    # raw pictures are about 74 Mbit/s: too much for the car's Wi-Fi
    assert car_config.load_car(str(TWIN_FILE)).camera_topic == "/camera/image_raw"
    assert car_config.load_car(str(REAL_FILE)).camera_topic == "/camera/image_raw/compressed"


@pytest.mark.parametrize("path", [TWIN_FILE, REAL_FILE], ids=["sim.yaml", "car.yaml"])
def test_ros_can_still_read_the_settings_file(path):
    # picar_base (and car.launch.py's nodes) read the whole file: a list ROS cannot read
    # (nested, or whole numbers mixed with decimals) would stop them from starting
    context = Context()
    try:
        rclpy.init(args=["--ros-args", "--params-file", str(path)], context=context)
    finally:
        context.try_shutdown()


def test_sim_and_car_name_the_packages_files():
    assert car_config.load_car("sim", share).file == str(TWIN_FILE)
    assert car_config.load_car("car", share).file == str(REAL_FILE)


def test_picar_config_chooses_the_file_and_sim_is_the_default(monkeypatch):
    monkeypatch.setenv("PICAR_CONFIG", "car")
    assert car_config.load_car(share=share).file == str(REAL_FILE)
    monkeypatch.setenv("PICAR_CONFIG", str(TWIN_FILE))
    assert car_config.load_car(share=share).file == str(TWIN_FILE)
    monkeypatch.delenv("PICAR_CONFIG")
    assert car_config.load_car(share=share).file == str(TWIN_FILE)


def test_a_package_that_is_not_built():
    def nothing_built(package):
        raise car_config.CarConfigError(f"package '{package}' is not built or not sourced")

    with pytest.raises(car_config.CarConfigError, match="package 'picar_car' is not built"):
        car_config.load_car("car", nothing_built)


def settings_with(tmp_path, change):
    """A copy of sim.yaml with change(settings) applied; returns its path."""
    settings = yaml.safe_load(TWIN_FILE.read_text())
    change(settings)
    path = tmp_path / "settings.yaml"
    path.write_text(yaml.safe_dump(settings))
    return str(path)


def agent(settings):
    return settings["picar_agent"]["ros__parameters"]


def base(settings):
    return settings["picar_base"]["ros__parameters"]


@pytest.mark.parametrize("change, message", [
    (lambda s: s.pop("picar_agent"), r"has no picar_agent section \(picar_agent: ros__parameters"),
    (lambda s: base(s).pop("max_speed"), "picar_base needs max_speed: a number$"),
    (lambda s: agent(s).update(turn_deg_per_m="fast"), "needs turn_deg_per_m: a number, not 'fast'"),
    (lambda s: agent(s).update(floor_rows=[1.0, 0.9]),
     "floor_rows and floor_distances_m need the same number of values"),
    (lambda s: agent(s).pop("floor_distances_m"), "needs floor_distances_m: a list of numbers"),
    (lambda s: agent(s).pop("description"), "picar_agent needs description: a text"),
    (lambda s: agent(s).update(camera_topic="camera"),
     "camera_topic must be a topic like /camera/image_raw, not 'camera'"),
])
def test_mistakes_in_the_settings_say_what_to_fix(tmp_path, change, message):
    with pytest.raises(car_config.CarConfigError, match=message):
        car_config.load_car(settings_with(tmp_path, change))


def test_a_missing_or_broken_file(tmp_path):
    with pytest.raises(car_config.CarConfigError, match="cannot read the car's settings"):
        car_config.load_car(str(tmp_path / "none.yaml"))
    broken = tmp_path / "broken.yaml"
    broken.write_text("picar_base: [unclosed")
    with pytest.raises(car_config.CarConfigError, match="is not valid YAML"):
        car_config.load_car(str(broken))


# ---------- the prompt ----------

def test_the_twins_prompt_has_its_numbers():
    prompt = words(picar_prompt(car_config.load_car(str(TWIN_FILE))))
    for part in [
        "You drive a small robot car, a simulated Adeept PiCar-B in a room in Gazebo.",
        "22 cm long and 10 cm wide",
        "It drives at 0.2 m/s, at most 0.6 m per drive call.",
        "Steering goes from -30 to +30 degrees",
        "turns about 20 degrees for every 0.1 m it drives, so a 90 degree turn takes about "
        "0.45 m and moves the car about 0.3 m forward and 0.3 m to the side.",
        "stops the car when something is 0.25 m or closer",
        "17 cm above the floor. Pan goes from -90 to 90 degrees (positive = left), tilt from -40 to 40",
        "The picture is about 62 degrees wide. Something at the left or right edge is 31 degrees",
        "halfway between the middle and the edge about 16 degrees",
        "at the bottom edge 0.35 m, 90% of the way down 0.45 m, 80% 0.6 m, 75% 0.75 m, "
        "70% 0.9 m, 65% 1.25 m, 60% 1.9 m. The floor closer than 0.35 m is below the picture",
        "parts of your own car: black wheels, green body.",
        "drive 0.6 m into open space",
    ]:
        assert part in prompt


def test_the_real_cars_prompt_does_not_mention_gazebo():
    prompt = picar_prompt(car_config.load_car(str(REAL_FILE)))
    assert "a real car in a real room" in prompt and "Gazebo" not in prompt


def test_the_prompt_follows_other_numbers():
    other = dataclasses.replace(
        car_config.load_car(str(TWIN_FILE)), speed_mps=0.25, max_move_s=2.0,
        max_steering_deg=25.0, stop_distance_m=0.3, camera_fov_deg=80.0,
        floor=((1.0, 0.5), (0.8, 1.0)))
    prompt = words(picar_prompt(other))
    assert "It drives at 0.25 m/s, at most 0.5 m per drive call." in prompt
    assert "Steering goes from -25 to +25 degrees" in prompt
    assert "when something is 0.3 m or closer" in prompt
    assert "about 80 degrees wide. Something at the left or right edge is 40 degrees" in prompt
    assert "the edge about 20 degrees" in prompt
    assert "at the bottom edge 0.5 m, 80% of the way down 1 m. The floor closer than 0.5 m" in prompt
