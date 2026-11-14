"""car_config.py: what the agent knows about the car it drives, from the car's settings file.

The agent reads the same YAML file as picar_base, so the two always agree. PICAR_CONFIG
in .env chooses it:
    sim   the Gazebo twin:   picar_base/config/sim.yaml   (the default)
    car   the real PiCar-B:  picar_car/config/car.yaml
    or the path of another settings file.

From the file's picar_base section it takes the speed, the limits and the stop distance.
Its picar_agent section holds what only the AI needs: how the car is described, its
size, how sharply it turns, the camera, and how far away the floor is in the picture.
"""

import os
from dataclasses import dataclass

import yaml

# The short names for PICAR_CONFIG: package, and the file in its share folder
NAMED_FILES = {
    "sim": ("picar_base", "config/sim.yaml"),
    "car": ("picar_car", "config/car.yaml"),
}

# Read from the picar_base section (picar_base uses the same values)
BASE_NUMBERS = ("full_speed_mps", "max_speed", "max_move_s", "max_steering_deg",
                "max_pan_deg", "max_tilt_deg", "obstacle_stop_m")
# Read from the picar_agent section
AGENT_NUMBERS = ("length_m", "width_m", "turn_deg_per_m", "head_deg_per_s",
                 "camera_height_m", "camera_fov_deg")


class CarConfigError(Exception):
    """The settings file is missing or incomplete. The message says what to fix."""


@dataclass(frozen=True)
class Car:
    file: str                 # where these settings came from
    description: str          # "a simulated Adeept PiCar-B in a room in Gazebo"
    speed_mps: float          # driving speed: full_speed_mps x max_speed
    max_move_s: float         # picar_base stops every move after this long
    max_steering_deg: float
    max_pan_deg: float
    max_tilt_deg: float
    stop_distance_m: float    # the safety stop: obstacle_stop_m
    length_m: float
    width_m: float
    turn_deg_per_m: float     # heading change per metre at full steering
    head_deg_per_s: float     # how fast the camera head turns
    camera_height_m: float
    camera_fov_deg: float     # how wide the picture is
    floor: tuple              # (row, distance_m) pairs: see floor_rows in sim.yaml
    own_parts: str            # what the camera may see of the car itself
    camera_topic: str = "/camera/image_raw"   # .../compressed: JPEGs (the real car, over Wi-Fi)

    @property
    def max_drive_m(self):
        """The longest drive in one move."""
        return self.speed_mps * self.max_move_s


def ros_share(package):
    """The package's share folder, from the sourced ROS workspaces."""
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    try:
        return get_package_share_directory(package)
    except PackageNotFoundError:
        raise CarConfigError(f"package '{package}' is not built or not sourced") from None


def config_file(spec, share=ros_share):
    """sim, car or a file path -> the settings file's path."""
    if spec in NAMED_FILES:
        package, path = NAMED_FILES[spec]
        return os.path.join(share(package), path)
    return os.path.abspath(os.path.expanduser(spec))


def load_car(spec=None, share=ros_share):
    """The car from its settings file. spec: sim, car or a path; empty: PICAR_CONFIG, else sim."""
    spec = (spec or os.environ.get("PICAR_CONFIG", "")).strip() or "sim"
    path = config_file(spec, share)
    try:
        with open(path, encoding="utf-8") as f:
            settings = yaml.safe_load(f) or {}
    except OSError as error:
        raise CarConfigError(f"cannot read the car's settings {path}: {error.strerror}") from None
    except yaml.YAMLError as error:
        raise CarConfigError(f"{path} is not valid YAML: {error}") from None

    base = section(settings, "picar_base", path)
    agent = section(settings, "picar_agent", path)
    numbers = {name: number(base, name, f"{path}: picar_base") for name in BASE_NUMBERS}
    numbers.update({name: number(agent, name, f"{path}: picar_agent") for name in AGENT_NUMBERS})

    rows = number_list(agent, "floor_rows", path)
    distances = number_list(agent, "floor_distances_m", path)
    if len(rows) != len(distances):
        raise CarConfigError(f"{path}: picar_agent's floor_rows and floor_distances_m "
                             f"need the same number of values ({len(rows)} and {len(distances)})")

    return Car(
        file=path,
        description=text(agent, "description", path),
        speed_mps=numbers["full_speed_mps"] * numbers["max_speed"],
        max_move_s=numbers["max_move_s"],
        max_steering_deg=numbers["max_steering_deg"],
        max_pan_deg=numbers["max_pan_deg"],
        max_tilt_deg=numbers["max_tilt_deg"],
        stop_distance_m=numbers["obstacle_stop_m"],
        length_m=numbers["length_m"],
        width_m=numbers["width_m"],
        turn_deg_per_m=numbers["turn_deg_per_m"],
        head_deg_per_s=numbers["head_deg_per_s"],
        camera_height_m=numbers["camera_height_m"],
        camera_fov_deg=numbers["camera_fov_deg"],
        floor=tuple(zip(rows, distances)),
        own_parts=text(agent, "own_parts", path),
        camera_topic=topic(agent, "camera_topic", path, Car.camera_topic),
    )


def section(settings, node, path):
    """A node's parameters: settings[node]["ros__parameters"]."""
    params = (settings.get(node) or {}).get("ros__parameters") if isinstance(settings, dict) else None
    if not isinstance(params, dict):
        raise CarConfigError(f"{path} has no {node} section ({node}: ros__parameters: ...)")
    return params


def number(params, name, where):
    value = params.get(name)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        found = "" if value is None else f", not {value!r}"
        raise CarConfigError(f"{where} needs {name}: a number{found}")
    return float(value)


def number_list(params, name, path):
    values = params.get(name)
    if not values or not isinstance(values, list) or not all(
            isinstance(v, (int, float)) and not isinstance(v, bool) for v in values):
        raise CarConfigError(f"{path}: picar_agent needs {name}: a list of numbers")
    return [float(v) for v in values]


def text(params, name, path):
    value = params.get(name)
    if not value or not isinstance(value, str):
        raise CarConfigError(f"{path}: picar_agent needs {name}: a text")
    return value.strip()


def topic(params, name, path, default):
    """An optional ROS topic name (default if missing)."""
    value = params.get(name, default)
    if not isinstance(value, str) or not value.startswith("/"):
        raise CarConfigError(f"{path}: picar_agent's {name} must be a topic like {default}, "
                             f"not {value!r}")
    return value.strip()
