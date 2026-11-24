"""picture_log.py: keeps the pictures of a run, to see afterwards what the AI saw.

One folder per task, e.g. ~/picar_runs/2026-11-20_15-29-56_drive-to-the-white-bed/, with every
picture the tools asked the model about (and the pictures they handed back to the AI) as JPEGs,
numbered in order, and log.txt: for each picture the tool, the question and the model's answer.

Switched on in .env:  PICAR_SAVE_PICTURES=1   (or the folder to use instead of ~/picar_runs)
When a task starts, runs older than PICAR_PICTURES_KEEP_DAYS (7) are wiped, and only the newest
20 are kept. Only folders named like a run are ever deleted.
"""

import base64
import io
import os
import re
import shutil
import time
from datetime import datetime

from PIL import Image

DEFAULT_ROOT = "~/picar_runs"
KEEP_DAYS = 7
KEEP_RUNS = 20
RUN_NAME = re.compile(r"^\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}_")   # what start() names a run folder
OFF = ("", "0", "no", "false", "off")


def slug(text, length=40):
    """A file-name-safe short form: "Drive to the white bed." -> "drive-to-the-white-bed";
    a minus sign stays: "pan -50.0" -> "pan-minus-50-0" (not the same as "pan 50.0")."""
    text = re.sub(r"(?<![a-z0-9])-(?=\d)", "minus ", text.lower())
    return "-".join(re.findall(r"[a-z0-9]+", text))[:length].strip("-") or "x"


class PictureLog:
    """root: the folder that holds the runs; None: switched off (saves nothing)."""

    def __init__(self, root=None, keep_days=KEEP_DAYS, keep_runs=KEEP_RUNS, clock=time):
        self.root = os.path.expanduser(root) if root else None
        self.keep_days, self.keep_runs, self.clock = keep_days, keep_runs, clock
        self.folder, self.count = None, 0

    @classmethod
    def from_env(cls, environ=None):
        """Switched on and configured by PICAR_SAVE_PICTURES and PICAR_PICTURES_KEEP_DAYS."""
        environ = os.environ if environ is None else environ
        value = environ.get("PICAR_SAVE_PICTURES", "").strip()
        if value.lower() in OFF:
            return cls(None)
        root = DEFAULT_ROOT if value.lower() in ("1", "yes", "true", "on") else value
        days = environ.get("PICAR_PICTURES_KEEP_DAYS", "").strip()
        return cls(root, keep_days=float(days) if days else KEEP_DAYS)

    def start(self, task):
        """A new task: wipe old runs, then start a folder for this one."""
        if not self.root:
            return
        try:
            os.makedirs(self.root, exist_ok=True)
            self.wipe_old()
            stamp = datetime.fromtimestamp(self.clock.time()).strftime("%Y-%m-%d_%H-%M-%S")
            self.folder = os.path.join(self.root, f"{stamp}_{slug(task or 'task')}")
            os.makedirs(self.folder, exist_ok=True)
            self.count = 0
            with open(os.path.join(self.folder, "log.txt"), "w", encoding="utf-8") as f:
                f.write(f"Task: {task}\n\n")
        except OSError as error:   # the log must never stop a drive
            print(f"\n[pictures not saved: {error}]")
            self.folder = None

    def save(self, picture, label, question="", answer=""):
        """One picture (a tool picture: {"images": [base64 PNG]}), with what was asked about it.
        Of several (a first sighting, then the picture now), the last one."""
        if not self.folder or not picture or not picture.get("images"):
            return
        try:
            self.count += 1
            name = f"{self.count:03d}_{slug(label, 30)}.jpg"
            data = picture["images"][-1]
            data = data.split(",", 1)[1] if data.startswith("data:") else data
            image = Image.open(io.BytesIO(base64.b64decode(data))).convert("RGB")
            image.save(os.path.join(self.folder, name), format="JPEG", quality=85)
            with open(os.path.join(self.folder, "log.txt"), "a", encoding="utf-8") as f:
                f.write(f"{name}  [{label}]\n")
                if question:
                    f.write(f"  asked:   {question}\n")
                if answer:
                    f.write(f"  answer:  {answer}\n")
                f.write("\n")
        except Exception as error:   # a bad picture or a full disk must never stop a drive
            print(f"\n[picture not saved: {error}]")

    def wipe_old(self):
        """Delete run folders older than keep_days, and the oldest beyond keep_runs - 1 (room
        for the run about to start). Nothing else in the folder is touched."""
        runs = sorted(d for d in os.listdir(self.root)
                      if RUN_NAME.match(d) and os.path.isdir(os.path.join(self.root, d)))
        cutoff = self.clock.time() - self.keep_days * 24 * 3600
        for index, name in enumerate(runs):   # oldest first (the names start with the time)
            path = os.path.join(self.root, name)
            too_many = len(runs) - index > self.keep_runs - 1
            if too_many or os.path.getmtime(path) < cutoff:
                shutil.rmtree(path, ignore_errors=True)
