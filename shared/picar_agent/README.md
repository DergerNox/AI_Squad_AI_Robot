# PiCar agent

An AI that drives the Adeept PiCar-B from its camera: you give it a task in plain
words ("drive to the red box, then the yellow box"), and it looks, drives and
turns until it is done. It runs against the Gazebo twin for testing, and against
the real car through the same ROS 2 names.

## Files

| File | What it does |
|---|---|
| `agent.py` | The loop: the conversation, your input and its answers |
| `llm.py` | The AI model chosen in `.env` (Ollama, OpenAI or Gemini) |
| `picar_tools.py` | The tools: drive, turn, look, look_around, approach, reached, remember |
| `car_client.py` | Talks to the car: picar_base's moves and camera head, the distance sensor |
| `camera.py` | Gets the camera's newest picture (never an old one) and turns it into the model's picture |
| `car_config.py` | Reads the car's numbers from its settings file, the same one picar_base uses |
| `prompt.py` | What the AI is told: its tools, the car (with its numbers) and how to drive |
| `history.py` | Keeps the conversation short enough for the model's context |
| `.env.example` | The settings: which AI to use, its key, and which car. Copy it to `.env` |
| `test/` | The tests (`python3 -m pytest test`, in `~/rai_venv` with ROS sourced) and, in `test/twin/`, the stand-ins for the twin |

## What it needs

With the real car, the agent runs on a laptop on the car's Wi-Fi, not on the Pi (the Pi
3B+ has too little memory): the Pi only runs the car's nodes. One command at the repo root
installs everything below: `./setup_laptop.sh` on Fedora (it puts ROS and the agent in an
Ubuntu 24.04 distrobox, and installs Ollama with the model), or `./setup_car.sh laptop`
on Ubuntu 24.04.

- Ubuntu 24.04 with ROS 2 Jazzy (and Gazebo Harmonic for the twin)
- This repository cloned into `~/picar_ws/src/picar`, and the workspace built:
  `cd ~/picar_ws && colcon build --symlink-install`
- RAI in a virtual environment `~/rai_venv`, made with `--system-site-packages`
  so it sees ROS 2 (the same setup as in the MiniRobot installation guide)
- An AI model that understands pictures and supports tool calling:
  local in Ollama, or a cloud model

## Choose the AI

```bash
cp .env.example .env
nano .env
```

| Setting | Meaning |
|---|---|
| `PICAR_LLM` | `ollama` (local, the default), `openai` (OpenAI or any OpenAI-compatible server) or `google` (Gemini) |
| `PICAR_MODEL` | The model name. Empty means `qwen3-vl:4b-instruct` for Ollama |
| `PICAR_BASE_URL` | The server address, if it is not the default (Ollama, OpenAI) |
| `PICAR_TEMPERATURE` | How much the answers vary, 0 to 2. Empty means 0.2 |
| `OPENAI_API_KEY`, `GOOGLE_API_KEY` | The key for a cloud model |
| `PICAR_CONFIG` | Which car: `sim` (the Gazebo twin, the default), `car` (the real PiCar-B: the laptop setup sets it) or the path of a settings file |

When the agent starts, it prints which model it uses and which car.

One task with three targets takes 50 or more model requests, because every step
is one request and each `look_around` adds up to five. Free cloud tiers run out
fast; check your quota.

## Run

With the twin, in a new terminal (the `sim` command: see `squad_gazebo`'s README for the
one line it needs in `~/.bashrc`):

```bash
sim          # pick picar and the room: Gazebo opens, then the agent starts here
```

Wait for `PiCar agent ready`, then type a task, for example
`drive to the red box, then the yellow box, then the blue cylinder`.
Type `forget` to clear what the AI has remembered, Ctrl+C to stop the agent and the twin.

The twin runs on the PiCar-B's own ROS domain, so a second terminal sees it only after
`car picar`:

```bash
car picar
ros2 topic echo /ultrasonic/range --field range
```

With the real car, start the car's nodes on the Pi first, then the agent on the laptop:
`picar-agent` (made by `setup_laptop.sh`), or `python3 agent.py` in this folder with ROS
and `~/rai_venv` active. The laptop and the Pi must be on the same Wi-Fi.

Tasks can also come from any ROS 2 program, and the answers go out on a topic:

```bash
ros2 topic pub --once /from_human rai_interfaces/msg/HRIMessage "{text: 'drive to the red box'}"
ros2 topic echo /to_human rai_interfaces/msg/HRIMessage --field text
```

## How it works

Every step is one round of this loop:

1. The AI gives an order: a tool call such as `drive(distance_m=0.6)`.
2. The tool sends it to the car: `/picar/move` or `/picar/look`.
3. The tool reads the car: the camera picture (`/camera/image_raw`), the distance
   sensor (`/ultrasonic/range`, in approach) and the result of the move.
4. The AI gets the picture with a text: what happened, and which targets it has
   reached.

The camera, the distance sensor and `picar_base` are separate nodes, the same in the
twin and on the real car, so the agent works with both unchanged.

`picar_base` checks every move with its safety rules (stop at 0.25 m, at most 3 s
per move, speed limit), so the AI cannot drive the car into a wall.

## Change things

- Another AI: only `.env`, or `llm.py` for another kind of service (one `..._model()`
  function, added to `MAKERS`)
- The car's numbers: the settings file `PICAR_CONFIG` names. The speed, the limits and
  the stop distance come from its `picar_base` section, so the agent and picar_base
  always agree; the size, the turning, the camera and the floor distances from its
  `picar_agent` section. The twin's are in `picar_base/config/sim.yaml` (measured in the
  twin), the real car's in `picar/picar_car/config/car.yaml` (the twin's for now:
  measure them on the car).
- What the AI is told about the car and how to drive: `picar_prompt()` in `prompt.py`
- Another sensor for the AI: read its topic in `car_client.py`, like `range_ahead()` does,
  and use it in a tool in `picar_tools.py`
