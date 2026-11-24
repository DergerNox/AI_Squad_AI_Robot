"""check_pictures.py: ask the real vision model the tools' questions about saved car pictures,
and count how often it answers as it should.

    python3 test/real/check_pictures.py CASES.json [--model qwen3-vl:4b-instruct]

Run it from the agent folder with ROS sourced and the model running (Ollama, or as .env says),
whenever a question's wording, the model or its settings change. pytest does not run it.

CASES.json is a list of cases, paths relative to the file (the pictures stay on your PC: they
show your room, so they are not in the repo):
    {"picture": "pics/017.jpg", "first": "pics/first.jpg", "target": "white bed",
     "question": "close_up", "expect": "yes", "note": "the bed's drawer fills the picture"}
question: same (the box cut out of picture, compared with first), close_up (the whole picture
compared with first) or stop (the stop check: the whole picture compared with first).
box: [x1, y1, x2, y2] in fractions of the picture, for "same": the part to cut out.
The model gets exactly the agent's question, at the agent's picture-question temperature.
"""

import argparse
import base64
import json
import logging
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from eyes import Eyes  # noqa: E402
from llm import VISION_TEMPERATURE, make_llm  # noqa: E402
from picture_log import PictureLog  # noqa: E402
from picture_math import both, cut_out  # noqa: E402
from questions import CLOSE_UP, IN_FRONT_FIRST, SAME, says_no, says_yes  # noqa: E402
from task_state import TaskState  # noqa: E402

QUESTIONS = {"same": SAME, "close_up": CLOSE_UP, "stop": IN_FRONT_FIRST}


def picture(path):
    with open(path, "rb") as f:
        return {"images": [base64.b64encode(f.read()).decode()], "audios": []}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("cases")
    parser.add_argument("--model", help="instead of PICAR_MODEL")
    args = parser.parse_args()
    if args.model:
        os.environ["PICAR_MODEL"] = args.model
    logging.getLogger("httpx").setLevel(logging.WARNING)   # no line for every request
    vision, description = make_llm(temperature=VISION_TEMPERATURE)
    eyes = Eyes(vision, TaskState(), PictureLog(None))
    folder = os.path.dirname(os.path.abspath(args.cases))
    with open(args.cases, encoding="utf-8") as f:
        cases = json.load(f)
    print(f"{len(cases)} cases, {description}")
    right, times = 0, []
    for case in cases:
        shown = picture(os.path.join(folder, case["picture"]))
        if case["question"] == "same" and case.get("box"):
            shown = cut_out(shown, *case["box"])
        first = picture(os.path.join(folder, case["first"]))
        start = time.monotonic()
        answer = eyes.ask(both(first, shown), QUESTIONS[case["question"]].format(target=case["target"]),
                          echo=False)
        times.append(time.monotonic() - start)
        ok = says_yes(answer) if case["expect"] == "yes" else says_no(answer)
        right += ok
        print(f"  {'ok   ' if ok else 'WRONG'} {case['question']:8s} expect {case['expect']:3s} "
              f"got {answer[:20]!r:24s} {case.get('note', case['picture'])}")
    print(f"right {right} of {len(cases)}; {sum(times) / max(1, len(times)):.1f} s per answer")


if __name__ == "__main__":
    main()
