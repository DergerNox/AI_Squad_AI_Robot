# squad_gazebo: start any car's twin and its agent with one command

`sim` asks which car and which world. It then starts the car's Gazebo twin with its
window, waits until the twin's camera works, and starts that car's agent in the same
terminal. Ctrl+C stops everything.

The cars are listed in `config/cars.yaml`: model, world, bridges, extra nodes, the
car's terminal settings (folder, ROS domain, venv) and its agent. Adding a car is an
entry there, not a new launch file or script.

## Set up (once)

Build the package, then add this one line to the end of `~/.bashrc` (after the lines
that source ROS and `~/picar_ws`):

```bash
[ -f ~/picar_ws/install/squad_gazebo/share/squad_gazebo/squad.bash ] && source ~/picar_ws/install/squad_gazebo/share/squad_gazebo/squad.bash
```

`squad.bash` comes with the package, so changes to it arrive with a rebuild.

## Use

```bash
sim                      # car and world; Enter takes the default in [brackets]
sim world:=room          # this tab's car, no questions (launch arguments work too)
car mini                 # only set this tab up for a car: its folder, ROS domain, venv
```

Rarely needed, e.g. on a machine without a screen: `sim headless:=true` (no Gazebo
window), `sim agent:=false` (the twin only).

- PiCar: its agent has its own `> ` prompt: type a task, `forget`, Ctrl+C to stop.
- MiniRobot: its agent runs in the background; the `> ` prompt sends each task with
  `minirobot_sim/ask.py`. Ctrl+C or Ctrl+D stops everything.
- A terminal stays set up for one car (the prompt shows `[picar]`). For another car,
  open a new terminal. Each car has its own ROS domain, so two cars never mix.
- Messages: the twin's go to `~/.ros/squad/<car>-twin.log`; MiniRobot's agent's to
  `~/.ros/squad/mini-agent.log`. When the twin fails, `sim` shows its last lines.

Without the shell commands: `ros2 launch squad_gazebo sim.launch.py car:=picar`
(`world:=`, `headless:=true`, `x:=`, `y:=`, `yaw:=` in degrees, `cars:=` another list).
This starts the twin only.

## Add a car

A file in `cars.yaml` is either `[package, path inside its share folder]` (a built ROS
package) or a plain path in quotes, `"~/folder/file"` or `"/full/path"` (no build needed;
MiniRobot's twin is loaded this way, straight from `~/MiniRobotAI/minirobot_sim`).

```yaml
  pccar:
    folder: "~/picar_ws"                 # where `car pccar` takes the terminal
    domain: 52                           # its own ROS_DOMAIN_ID, 0-101
    name: pccar                          # model name in Gazebo (default: the entry's name)
    model: [pccar_description, urdf/pccar.urdf.xacro]
    world: room
    spawn: {x: 0.0, y: 0.0, z: 0.01, yaw: 0.0}
    bridges:                             # one parameter_bridge node per file
      - {name: camera, config: [pccar_description, config/camera_bridge.yaml]}
    nodes:                               # ROS nodes, or Python files run as python3 <file>
      - {package: pccar_base, executable: base, parameters: [[pccar_base, config/sim.yaml]]}
      - script: "~/pccar/sim/sim_sensors.py"
    agent:
      folder: "~/pccar"                  # it runs here
      run: python3 agent.py
      env_file: .env                     # optional: loaded first
      ask: python3 sim/ask.py            # optional: the agent runs in the background and
                                         # each typed task is sent with this
      before: ollama stop qwen3-vl:4b-instruct   # optional: run before Gazebo starts
      wait_for: /camera/image_raw        # optional (the default): start the agent after
                                         # this topic has a message (needs a header)
```

Then `colcon build --packages-select squad_gazebo` and `sim`. A new world goes under
`worlds:` as a name and a file. Every car can be loaded into every world; `sim` offers
the car's own world first. Cars and worlds that can't start are listed with the reason;
a car whose agent can't start (for example its `.env` is missing) counts as not ready.
A car without an `agent:` entry starts as a twin only.

## Test

```bash
cd ~/picar_ws/src/picar/shared/squad_gazebo && python3 -m pytest test
```

The tests check `cars.yaml` against the repo's own files (MiniRobot's against a stand-in
`minirobot_sim` folder) and `sim`'s questions with typed answers. They need no ROS.