# PiCar agent

An AI that drives the Adeept PiCar-B from its camera: you give it a task in plain
words ("drive to the red box, then the yellow box"), and it looks, drives and
turns until it is done. It runs against the Gazebo twin for testing, and against
the real car through the same ROS 2 names.

## Files

| File | What it does |
|---|---|
| `agent.py` | The AI connection and the loop: model, conversation, your input and its answers |
| `picar_tools.py` | The car side: the tools (drive, turn, look, look_around, approach, reached, remember) and the prompt |
| `run.sh` | Starts the Gazebo twin in the background and the agent in the terminal |
| `.env.example` | The settings: which AI to use, and its key. Copy it to `.env` |

## What it needs

- Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic
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
| `PICAR_BASE_URL` | The server address, if it is not the default |
| `OPENAI_API_KEY`, `GOOGLE_API_KEY` | The key for a cloud model |

When the agent starts, it prints which model it uses.

One task with three targets takes 50 or more model requests, because every step
is one request and each `look_around` adds up to five. Free cloud tiers run out
fast; check your quota.

## Run

```bash
~/picar_ws/src/picar/picar_agent/run.sh
```

Wait for `PiCar agent ready`, then type a task, for example
`drive to the red box, then the yellow box, then the blue cylinder`.
Type `forget` to clear what the AI has remembered, Ctrl+C to quit.

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

- Another AI: only `.env`, or `make_llm()` in `agent.py` for another kind of service
- What the AI knows about the car, and the car's numbers: `PICAR_PROMPT` and the
  constants at the top of `picar_tools.py` (measured in the twin; measure them
  again on the real car)
- Another sensor for the AI: read its topic in `picar_tools.py`, like `range_ahead()` does
