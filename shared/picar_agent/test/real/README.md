# Checks with the real model

`check_pictures.py` asks the real vision model the tools' own questions (`questions.py`) about
saved car pictures, and counts how often it answers as it should. Run it whenever a question's
wording, the model or its settings change; pytest does not run it (it needs the model).

```bash
source /opt/ros/humble/setup.bash && source ~/picar_ws/install/setup.bash   # (jazzy on the twin PC)
source ~/rai_venv/bin/activate
python3 test/real/check_pictures.py ~/picar_checks/cases/cases_2026-11-24.json
```

The cases file and its pictures stay on the PC (they show your room). The picture log
(`PICAR_SAVE_PICTURES`, `~/picar_runs`) is where new cases come from: copy the pictures out of it
before it wipes old runs, and write the cases as the script's docstring says.

Result on 2026-11-24 (qwen3-vl:4b-instruct, temperature 0, 45 cases from three real-car runs):
40 right. The whole-picture close-up question 11 of 11; boxes compared with the first sighting
29 of 34 (two of the bed taken for something else, three of the fluffy slipper taken for the bed).
