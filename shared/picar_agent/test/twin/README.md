# Twin checks with stand-ins

Scripts that drive the Gazebo twin with stand-ins instead of the real model. They test
the tools and the driving, not how well the real model sees. pytest skips this folder.

| File | What it is |
|---|---|
| `color_vision.py` | Answers the tools' picture questions by colour (red, blue, yellow) |
| `fake_ollama_obedient.py` | Ollama stand-in on 127.0.0.1:11434: its chat model does what the last tool result says |
| `test_approach_tools.py` | `approach` from six starts, with where the car really ended (from Gazebo) |
| `test_approach_agent.py` | The whole agent on one task |

`test_approach_tools.py` and `fake_ollama_obedient.py` need the `approach` tool, so they
only work with the 3 November `picar_tools.py`.

## Run

The twin must be running (`ros2 launch picar_gazebo sim.launch.py`). In a second terminal,
from the `picar_agent` folder:

```bash
source /opt/ros/jazzy/setup.bash && source ~/picar_ws/install/setup.bash
source ~/rai_venv/bin/activate

python3 test/twin/test_approach_tools.py .
```

The whole agent, with the stand-in model instead of Ollama (stop the real one first,
both use port 11434):

```bash
python3 test/twin/fake_ollama_obedient.py /tmp/requests.jsonl "red box,yellow box" &
python3 test/twin/test_approach_agent.py . "drive to the red box, then the yellow box"
kill %1
```

`/tmp/requests.jsonl` logs every step the stand-in model took.
