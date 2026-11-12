"""cars.yaml and cars.resolve, without ROS: package folders come from the repo's source tree."""
import math
import os
import pathlib

import pytest

from squad_gazebo import cars

HERE = pathlib.Path(__file__).resolve().parent
REGISTRY = HERE.parent / 'config' / 'cars.yaml'
REPO = HERE.parents[2]          # shared/squad_gazebo/test -> repo root


def source_packages():
    """package name -> source folder, for every package.xml in the repo."""
    found = {}
    for xml in REPO.rglob('package.xml'):
        text = xml.read_text()
        name = text.split('<name>', 1)[1].split('</name>', 1)[0].strip()
        found[name] = str(xml.parent)
    return found


PACKAGES = source_packages()


def share(package):
    # The installed share folders keep the same relative paths as the sources
    # (urdf/, worlds/, config/), so the source folder stands in for the share folder.
    if package not in PACKAGES:
        raise cars.CarError(f"package '{package}' is not built or not sourced")
    return PACKAGES[package]


def registry():
    return cars.load(REGISTRY)


def test_picar_entry_points_at_real_files():
    car = cars.resolve(registry(), 'picar', share=share)
    assert car.car == 'picar'
    assert car.model.endswith('picar_description/urdf/picar.urdf.xacro')
    assert car.world.endswith('picar_gazebo/worlds/picar_room.sdf')
    assert [b.config.split('picar_gazebo/')[1] for b in car.bridges] == [
        'config/camera_bridge.yaml', 'config/picar_bridge.yaml']
    assert [b.name for b in car.bridges] == ['camera', 'sim_bridge']
    assert [(n.package, n.executable, n.name) for n in car.nodes] == [
        ('picar_base', 'picar_base', ''), ('picar_gazebo', 'sim_ultrasonic.py', 'ultrasonic')]
    assert car.nodes[0].parameters[0].endswith('picar_base/config/sim.yaml')
    assert car.spawn == {'x': 0.0, 'y': 0.0, 'z': 0.01, 'yaw': 0.0}
    for path in [car.model, car.world, car.bridges[0].config, car.nodes[0].parameters[0]]:
        assert os.path.isfile(path)


def test_pccar_not_modelled_yet_names_the_missing_package():
    with pytest.raises(cars.CarError,
                       match="car 'pccar', model: package 'pccar_description' is not built"):
        cars.resolve(registry(), 'pccar', share=share)


MINI_FILES = ['minirobot.urdf.xacro', 'minirobot_room.sdf', 'bridge.yaml', 'sim_sensors.py']


def fake_home(tmp_path, monkeypatch, files=MINI_FILES, mini_env=True, picar=True):
    """A home folder like Ferenc's: ~/picar_ws with the agent, ~/MiniRobotAI with minirobot_sim."""
    home = tmp_path / 'home'
    folder = home / 'MiniRobotAI' / 'minirobot_sim'
    folder.mkdir(parents=True)
    for name in files:
        (folder / name).write_text('')
    if mini_env:
        (home / 'MiniRobotAI' / '.env').write_text('GOOGLE_API_KEY=x\n')
    if picar:
        (home / 'picar_ws' / 'src' / 'picar' / 'shared' / 'picar_agent').mkdir(parents=True)
    monkeypatch.setenv('HOME', str(home))
    return folder


def test_mini_comes_straight_from_minirobot_sim(tmp_path, monkeypatch):
    folder = fake_home(tmp_path, monkeypatch)
    car = cars.resolve(registry(), 'mini', share=share)
    assert car.car == 'mini' and car.gz_name == 'minirobot'
    assert car.model == str(folder / 'minirobot.urdf.xacro')
    assert car.world == str(folder / 'minirobot_room.sdf')
    assert [(b.config, b.name) for b in car.bridges] == [(str(folder / 'bridge.yaml'), '')]
    assert car.nodes == [cars.NodeSpec(script=str(folder / 'sim_sensors.py'), name='sim_sensors')]
    assert car.spawn == {'x': 0.0, 'y': 0.0, 'z': 0.005, 'yaw': 0.0}


def test_mini_in_the_picar_room(tmp_path, monkeypatch):
    fake_home(tmp_path, monkeypatch)
    car = cars.resolve(registry(), 'mini', world='room', share=share)
    assert car.world.endswith('picar_gazebo/worlds/picar_room.sdf')


def test_mini_folder_missing_says_which_file(tmp_path, monkeypatch):
    fake_home(tmp_path, monkeypatch, files=['minirobot.urdf.xacro', 'minirobot_room.sdf'])
    with pytest.raises(cars.CarError, match=r"car 'mini', bridge 1: .*minirobot_sim/bridge.yaml does not exist"):
        cars.resolve(registry(), 'mini', share=share)
    monkeypatch.setenv('HOME', str(tmp_path / 'nobody'))
    with pytest.raises(cars.CarError, match=r"car 'mini', model: .*minirobot.urdf.xacro does not exist"):
        cars.resolve(registry(), 'mini', share=share)


def test_picar_and_its_gazebo_name_unchanged():
    assert cars.resolve(registry(), 'picar', share=share).gz_name == 'picar'


def test_world_path_with_tilde(tmp_path, monkeypatch):
    folder = fake_home(tmp_path, monkeypatch)
    car = cars.resolve(registry(), 'picar', world='~/MiniRobotAI/minirobot_sim/minirobot_room.sdf',
                       share=share)
    assert car.world == str(folder / 'minirobot_room.sdf')


def test_relative_path_is_refused(tmp_path):
    reg = cars.load(write(tmp_path, """
cars:
  toy: {model: "MiniRobotAI/minirobot.urdf.xacro"}
"""))
    with pytest.raises(cars.CarError, match='must start with ~/ or /'):
        cars.resolve(reg, 'toy', share=share)


def test_unknown_car_lists_the_cars():
    with pytest.raises(cars.CarError, match="unknown car 'picra'. Cars in cars.yaml: mini, pccar, picar"):
        cars.resolve(registry(), 'picra', share=share)


def test_spawn_from_launch_arguments():
    car = cars.resolve(registry(), 'picar', x='-0.6', y='0.2', yaw='90', share=share)
    assert car.spawn['x'] == -0.6 and car.spawn['y'] == 0.2 and car.spawn['z'] == 0.01
    assert math.isclose(car.spawn['yaw'], math.pi / 2)


def test_bad_spawn_number():
    with pytest.raises(cars.CarError, match="spawn x 'left' is not a number"):
        cars.resolve(registry(), 'picar', x='left', share=share)


def test_world_by_name_path_and_unknown(tmp_path):
    assert cars.resolve(registry(), 'picar', world='room', share=share).world.endswith('picar_room.sdf')
    own = tmp_path / 'empty.sdf'
    own.write_text('<sdf version="1.9"/>')
    assert cars.resolve(registry(), 'picar', world=str(own), share=share).world == str(own)
    with pytest.raises(cars.CarError, match="unknown world 'garage'. Worlds in cars.yaml: minirobot_room, room"):
        cars.resolve(registry(), 'picar', world='garage', share=share)
    with pytest.raises(cars.CarError, match='does not exist'):
        cars.resolve(registry(), 'picar', world=str(tmp_path / 'nope.sdf'), share=share)


def write(tmp_path, text):
    path = tmp_path / 'cars.yaml'
    path.write_text(text)
    return path


def test_a_new_car_with_named_bridges_and_inline_parameters(tmp_path):
    pkg = tmp_path / 'toy_description'
    (pkg / 'urdf').mkdir(parents=True)
    (pkg / 'urdf' / 'toy.urdf').write_text('<robot name="toy"/>')
    (pkg / 'config').mkdir()
    for name in ('camera', 'ultrasonic'):
        (pkg / 'config' / f'{name}.yaml').write_text('[]')
    folders = {'toy_description': str(pkg), 'toy_base': str(tmp_path)}

    def toy_share(package):
        return folders.get(package) or share(package)

    reg = cars.load(write(tmp_path, """
worlds:
  room: [picar_gazebo, worlds/picar_room.sdf]
cars:
  toy:
    model: [toy_description, urdf/toy.urdf]
    world: room
    spawn: {x: 0.5, yaw: 180}
    bridges:
      - {name: camera, config: [toy_description, config/camera.yaml]}
      - {name: ultrasonic, config: [toy_description, config/ultrasonic.yaml]}
    nodes:
      - {package: toy_base, executable: toy_base, name: base, parameters: [{max_speed: 0.3}]}
"""))
    car = cars.resolve(reg, 'toy', share=toy_share)
    assert [b.name for b in car.bridges] == ['camera', 'ultrasonic']
    assert car.nodes[0].name == 'base' and car.nodes[0].parameters == [{'max_speed': 0.3}]
    assert car.spawn['x'] == 0.5 and math.isclose(car.spawn['yaw'], math.pi)


def test_missing_file_in_a_built_package(tmp_path):
    reg = cars.load(write(tmp_path, """
worlds: {room: [picar_gazebo, worlds/picar_room.sdf]}
cars:
  picar:
    model: [picar_description, urdf/picar_v2.urdf.xacro]
    world: room
    bridges: [{config: [picar_gazebo, config/picar_bridge.yaml]}]
"""))
    with pytest.raises(cars.CarError, match='urdf/picar_v2.urdf.xacro is missing from package picar_description'):
        cars.resolve(reg, 'picar', share=share)


def test_entry_mistakes(tmp_path):
    reg = cars.load(write(tmp_path, """
worlds: {room: [picar_gazebo, worlds/picar_room.sdf]}
cars:
  nobridge: {model: [picar_description, urdf/picar.urdf.xacro], world: room}
  nomodel: {world: room, bridges: [{config: [picar_gazebo, config/picar_bridge.yaml]}]}
  badpair: {model: [picar_description, urdf, picar.urdf.xacro], world: room}
  badnode:
    model: [picar_description, urdf/picar.urdf.xacro]
    world: room
    bridges: [{config: [picar_gazebo, config/picar_bridge.yaml]}]
    nodes: [{package: picar_base}]
"""))
    with pytest.raises(cars.CarError, match="car 'nobridge' has no bridges"):
        cars.resolve(reg, 'nobridge', share=share)
    with pytest.raises(cars.CarError, match="car 'nomodel' has no 'model:'"):
        cars.resolve(reg, 'nomodel', share=share)
    with pytest.raises(cars.CarError, match=r'expected \[package, path\] or a file path'):
        cars.resolve(reg, 'badpair', share=share)
    with pytest.raises(cars.CarError, match="node 1 needs 'package:' and 'executable:', or 'script:'"):
        cars.resolve(reg, 'badnode', share=share)


def test_load_errors(tmp_path):
    with pytest.raises(cars.CarError, match='cannot read'):
        cars.load(tmp_path / 'none.yaml')
    with pytest.raises(cars.CarError, match='not valid YAML'):
        cars.load(write(tmp_path, 'cars: [unclosed'))
    with pytest.raises(cars.CarError, match="no 'cars:' section"):
        cars.load(write(tmp_path, 'worlds: {}'))


def test_tabs_folders_domains_and_venv(tmp_path, monkeypatch):
    fake_home(tmp_path, monkeypatch)
    home = str(tmp_path / 'home')
    assert cars.tab(registry(), 'picar') == cars.TabSpec(f'{home}/picar_ws', 51, f'{home}/rai_venv')
    assert cars.tab(registry(), 'mini') == cars.TabSpec(f'{home}/MiniRobotAI', 53, f'{home}/rai_venv')
    assert cars.tab(registry(), 'pccar').domain == 52


def test_tab_mistakes(tmp_path, monkeypatch):
    fake_home(tmp_path, monkeypatch)
    reg = cars.load(write(tmp_path, """
cars:
  a: {folder: "~/picar_ws", domain: 7}
  b: {folder: "~/picar_ws", domain: 7}
  c: {folder: "~/picar_ws", domain: 200}
  d: {folder: "~/nowhere", domain: 9}
  e: {domain: 10}
"""))
    with pytest.raises(cars.CarError, match="car 'a' has the same domain 7 as b"):
        cars.tab(reg, 'a')
    with pytest.raises(cars.CarError, match="car 'c' needs 'domain:', a whole number 0-101"):
        cars.tab(reg, 'c')
    with pytest.raises(cars.CarError, match='nowhere does not exist'):
        cars.tab(reg, 'd')
    with pytest.raises(cars.CarError, match="car 'e', folder: missing"):
        cars.tab(reg, 'e')


def test_agents(tmp_path, monkeypatch):
    fake_home(tmp_path, monkeypatch)
    home = str(tmp_path / 'home')
    assert cars.agent(registry(), 'picar') == cars.AgentSpec(
        folder=f'{home}/picar_ws/src/picar/shared/picar_agent', run='python3 agent.py',
        before='ollama stop qwen3-vl:4b-instruct')
    assert cars.agent(registry(), 'mini') == cars.AgentSpec(
        folder=f'{home}/MiniRobotAI', run='python3 agent.py', env_file=f'{home}/MiniRobotAI/.env',
        ask='python3 minirobot_sim/ask.py')
    assert cars.agent(registry(), 'pccar') is None


def test_agent_problems(tmp_path, monkeypatch):
    fake_home(tmp_path, monkeypatch, mini_env=False, picar=False)
    with pytest.raises(cars.CarError, match=r"car 'mini', agent: .*MiniRobotAI/.env is missing"):
        cars.agent(registry(), 'mini')
    with pytest.raises(cars.CarError, match=r"car 'picar', agent folder: .*picar_agent does not exist"):
        cars.agent(registry(), 'picar')