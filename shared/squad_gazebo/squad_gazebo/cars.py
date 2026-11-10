"""The cars the twin can load (config/cars.yaml) and turning one entry into file paths.

cars.yaml has two sections:

    worlds:  name -> a file
    cars:    name -> model, world, spawn, bridges, nodes (and optionally name)
                     folder, domain, venv: the car's terminal tab (car <name>)
                     agent: the AI started after the twin (sim)

A file is either [package, path inside that package's share folder] or a plain path
("~/MiniRobotAI/minirobot_sim/bridge.yaml" or an absolute path).

This module has no launch code in it, so it can be tested without ROS.
"""
import math
import os
from dataclasses import dataclass, field

import yaml

DEFAULT_SPAWN = {'x': 0.0, 'y': 0.0, 'z': 0.01, 'yaw': 0.0}


class CarError(Exception):
    """A mistake in cars.yaml or a car that is not built yet. The message says what to fix."""


@dataclass
class BridgeSpec:
    config: str
    name: str = ''            # empty: ros_gz_bridge's default node name


@dataclass
class NodeSpec:
    package: str = ''         # a ROS node: package + executable
    executable: str = ''
    script: str = ''          # or a Python file, run like python3 <file>
    name: str = ''
    parameters: list = field(default_factory=list)   # file paths and inline dicts


@dataclass
class TabSpec:
    folder: str               # where `car <name>` takes the terminal
    domain: int               # ROS_DOMAIN_ID, different for every car
    venv: str                 # Python virtual environment activated in the tab


@dataclass
class AgentSpec:
    folder: str               # the agent runs here
    run: str                  # the command, e.g. "python3 agent.py"
    env_file: str = ''        # loaded first (set -a; source it), full path
    ask: str = ''             # if set: the agent runs in the background and each typed
                              # task is sent with: <ask> "<task>"
    before: str = ''          # run before Gazebo starts (errors ignored)
    wait_for: str = '/camera/image_raw'   # the agent starts when this topic has a message


@dataclass
class CarSetup:
    car: str                  # the name in cars.yaml (car:=...)
    gz_name: str              # the model's name in Gazebo
    model: str                # the .urdf.xacro (or .urdf) file
    world: str                # the .sdf file
    spawn: dict
    bridges: list
    nodes: list


def ros_share(package):
    """The package's share folder, from the sourced ROS workspaces."""
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    try:
        return get_package_share_directory(package)
    except PackageNotFoundError:
        raise CarError(f"package '{package}' is not built or not sourced") from None


def load(path):
    try:
        with open(path) as f:
            registry = yaml.safe_load(f) or {}
    except OSError as error:
        raise CarError(f'cannot read {path}: {error.strerror}') from None
    except yaml.YAMLError as error:
        raise CarError(f'{path} is not valid YAML: {error}') from None
    if not isinstance(registry.get('cars'), dict) or not registry['cars']:
        raise CarError(f"{path} has no 'cars:' section")
    return registry


def resolve(registry, car, world='', x='', y='', yaw='', share=ros_share):
    """Everything sim.launch.py needs to start this car, with every path checked.

    world, x, y and yaw are the launch arguments; empty means the car's default.
    yaw is in degrees.
    """
    entry = car_entry(registry, car)
    if not entry.get('model'):
        raise CarError(f"car '{car}' has no 'model:' in cars.yaml")
    model = file_in(entry['model'], f"car '{car}', model", share)
    world_file = pick_world(registry, car, entry, world, share)

    spawn = dict(DEFAULT_SPAWN)
    spawn.update(entry.get('spawn') or {})
    for key, value in (('x', x), ('y', y), ('yaw', yaw)):
        if value != '':
            spawn[key] = value
    for key, value in spawn.items():
        try:
            spawn[key] = float(value)
        except (TypeError, ValueError):
            raise CarError(f"car '{car}': spawn {key} '{value}' is not a number") from None
    spawn['yaw'] = math.radians(spawn['yaw'])

    bridges = []
    for i, item in enumerate(entry.get('bridges') or []):
        if not isinstance(item, dict) or 'config' not in item:
            raise CarError(f"car '{car}': bridge {i + 1} needs 'config:' (a file)")
        bridges.append(BridgeSpec(
            config=file_in(item['config'], f"car '{car}', bridge {i + 1}", share),
            name=item.get('name', '')))
    if not bridges:
        raise CarError(f"car '{car}' has no bridges: without one ROS gets no camera or sensors")

    nodes = [node_spec(car, i, item, share) for i, item in enumerate(entry.get('nodes') or [])]

    return CarSetup(car, str(entry.get('name') or car), model, world_file, spawn, bridges, nodes)


def car_entry(registry, car):
    cars = registry['cars']
    if car not in cars:
        raise CarError(f"unknown car '{car}'. Cars in cars.yaml: {', '.join(sorted(cars))}")
    return cars[car] or {}


def tab(registry, car):
    """The car's terminal settings: folder, ROS domain and venv."""
    entry = car_entry(registry, car)
    folder = existing_dir(entry.get('folder'), f"car '{car}', folder")
    domain = entry.get('domain')
    if not isinstance(domain, int) or isinstance(domain, bool) or not 0 <= domain <= 101:
        raise CarError(f"car '{car}' needs 'domain:', a whole number 0-101 (its ROS_DOMAIN_ID)")
    same = sorted(name for name, other in registry['cars'].items()
                  if name != car and (other or {}).get('domain') == domain)
    if same:
        raise CarError(f"car '{car}' has the same domain {domain} as {', '.join(same)}: "
                       f"give every car its own")
    venv = os.path.expanduser(str(entry.get('venv') or '~/rai_venv'))
    return TabSpec(folder, domain, venv)


def agent(registry, car):
    """The car's agent, or None if cars.yaml gives it none. CarError if it cannot start."""
    spec = car_entry(registry, car).get('agent')
    if not spec:
        return None
    what = f"car '{car}', agent"
    if not isinstance(spec, dict):
        raise CarError(f'{what}: expected folder: and run:')
    folder = existing_dir(spec.get('folder'), f'{what} folder')
    run = spec.get('run')
    if not run or not isinstance(run, str):
        raise CarError(f"{what} needs 'run:', the command that starts it")
    env_file = ''
    if spec.get('env_file'):
        env_file = os.path.join(folder, os.path.expanduser(str(spec['env_file'])))
        if not os.path.isfile(env_file):
            raise CarError(f'{what}: {env_file} is missing (copy it from .env.example?)')
    return AgentSpec(folder=folder, run=run, env_file=env_file, ask=str(spec.get('ask') or ''),
                     before=str(spec.get('before') or ''),
                     wait_for=str(spec.get('wait_for') or '/camera/image_raw'))


def existing_dir(spec, what):
    if not spec or not isinstance(spec, str):
        raise CarError(f'{what}: missing in cars.yaml (a folder like "~/MiniRobotAI")')
    folder = os.path.expanduser(spec)
    if not os.path.isabs(folder):
        raise CarError(f"{what}: '{spec}' must start with ~/ or /")
    if not os.path.isdir(folder):
        raise CarError(f'{what}: {folder} does not exist')
    return folder


def node_spec(car, i, item, share):
    if isinstance(item, dict) and 'script' in item:
        script = file_in(item['script'], f"car '{car}', node {i + 1}", share)
        name = item.get('name') or os.path.splitext(os.path.basename(script))[0]
        return NodeSpec(script=script, name=name)
    if not isinstance(item, dict) or 'package' not in item or 'executable' not in item:
        raise CarError(f"car '{car}': node {i + 1} needs 'package:' and 'executable:', "
                       f"or 'script:' (a Python file)")
    what = f"car '{car}', node {item['executable']}"
    try:
        share(item['package'])
    except CarError as error:
        raise CarError(f'{what}: {error}') from None
    parameters = [param if isinstance(param, dict) else file_in(param, what, share)
                  for param in item.get('parameters') or []]
    return NodeSpec(package=item['package'], executable=item['executable'],
                    name=item.get('name', ''), parameters=parameters)


def pick_world(registry, car, entry, world, share):
    worlds = registry.get('worlds') or {}
    if world.endswith('.sdf') or os.sep in world:
        path = os.path.expanduser(world)
        if not os.path.isfile(path):
            raise CarError(f'world file {world} does not exist')
        return os.path.abspath(path)
    name = world or entry.get('world')
    if not name:
        raise CarError(f"car '{car}' has no 'world:' in cars.yaml; give one with world:=")
    if name not in worlds:
        raise CarError(f"unknown world '{name}'. Worlds in cars.yaml: {', '.join(sorted(worlds))}")
    return file_in(worlds[name], f"world '{name}'", share)


def file_in(spec, what, share):
    """[package, path] or a plain path ("~/..." or absolute) -> the absolute path, checked."""
    if isinstance(spec, str):
        full = os.path.expanduser(spec)
        if not os.path.isabs(full):
            raise CarError(f"{what}: '{spec}' must start with ~/ or /, or be [package, path]")
        if not os.path.isfile(full):
            raise CarError(f'{what}: {full} does not exist')
        return full
    if not (isinstance(spec, (list, tuple)) and len(spec) == 2):
        raise CarError(f'{what}: expected [package, path] or a file path, got {spec!r}')
    package, path = spec
    try:
        folder = share(package)
    except CarError as error:
        raise CarError(f'{what}: {error}') from None
    full = os.path.join(folder, path)
    if not os.path.isfile(full):
        raise CarError(f'{what}: {path} is missing from package {package} ({folder})')
    return full