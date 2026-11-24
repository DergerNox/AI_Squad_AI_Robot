"""picture_log.PictureLog: the pictures of each run, kept in a folder per task, wiped by age."""

import base64
import io
import os

from PIL import Image

from picture_log import PictureLog


def picture(colour=(255, 0, 0)):
    """A tool picture: {"images": [base64 PNG]}."""
    buffer = io.BytesIO()
    Image.new("RGB", (64, 48), colour).save(buffer, format="PNG")
    return {"images": [base64.b64encode(buffer.getvalue()).decode()], "audios": []}


class Clock:
    def __init__(self, now):
        self.now = now

    def time(self):
        return self.now


NOW = 1795000000.0   # 2026-11-18


def test_off_unless_switched_on_in_env(tmp_path):
    for value in ("", "0", "no", "off"):
        log = PictureLog.from_env({"PICAR_SAVE_PICTURES": value})
        log.start("drive to the box")
        log.save(picture(), "camera")
        assert log.root is None and log.folder is None
    assert PictureLog.from_env({}).root is None


def test_switched_on_with_1_or_a_folder(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    assert PictureLog.from_env({"PICAR_SAVE_PICTURES": "1"}).root == str(tmp_path / "picar_runs")
    log = PictureLog.from_env({"PICAR_SAVE_PICTURES": str(tmp_path / "runs"), "PICAR_PICTURES_KEEP_DAYS": "3"})
    assert (log.root, log.keep_days) == (str(tmp_path / "runs"), 3.0)


def test_a_run_keeps_its_pictures_and_what_was_asked(tmp_path):
    log = PictureLog(tmp_path, clock=Clock(NOW))
    log.start("Drive to the white bed.")
    log.save(picture(), "camera at pan 0", "Is there a white bed in this picture?", "middle")
    log.save(picture((0, 0, 255)), "approach, close look", "What is the white thing in it?", "white bed")
    (folder,) = tmp_path.iterdir()
    assert folder.name.endswith("_drive-to-the-white-bed")
    assert sorted(p.name for p in folder.iterdir()) == [
        "001_camera-at-pan-0.jpg", "002_approach-close-look.jpg", "log.txt"]
    text = (folder / "log.txt").read_text()
    assert text.startswith("Task: Drive to the white bed.")
    assert "  asked:   What is the white thing in it?\n  answer:  white bed\n" in text
    assert Image.open(folder / "001_camera-at-pan-0.jpg").size == (64, 48)


def test_old_runs_are_wiped_when_a_task_starts_and_nothing_else(tmp_path):
    old = tmp_path / "2026-11-01_10-00-00_drive-to-the-box"
    recent = tmp_path / "2026-11-17_10-00-00_look-around"
    mine = tmp_path / "my-notes"
    for folder in (old, recent, mine):
        folder.mkdir()
        os.utime(folder, (NOW - 8 * 86400, NOW - 8 * 86400) if folder != recent else (NOW - 86400, NOW - 86400))
    PictureLog(tmp_path, keep_days=7, clock=Clock(NOW)).start("next task")
    names = sorted(p.name for p in tmp_path.iterdir())
    assert old.name not in names and recent.name in names and "my-notes" in names   # never ours: kept
    assert len(names) == 3   # recent, my-notes and the new run


def test_at_most_20_runs_are_kept(tmp_path):
    for day in range(1, 26):
        (tmp_path / f"2026-11-{day:02d}_10-00-00_run").mkdir()   # all recent enough by time
    PictureLog(tmp_path, keep_days=1000, keep_runs=20, clock=Clock(NOW)).start("next task")
    runs = sorted(p.name for p in tmp_path.iterdir())
    assert len(runs) == 20 and runs[0].startswith("2026-11-07")   # the 19 newest + the new one


def test_a_bad_picture_never_stops_a_drive(tmp_path, capsys):
    log = PictureLog(tmp_path, clock=Clock(NOW))
    log.start("task")
    log.save({"images": ["not a picture"]}, "camera")
    log.save(None, "camera")
    assert "picture not saved" in capsys.readouterr().out


def test_the_minus_sign_stays_in_file_names():
    from picture_log import slug
    assert slug("camera at pan -50.0") == "camera-at-pan-minus-50-0"
    assert slug("camera at pan 50.0") == "camera-at-pan-50-0"
    assert slug("Drive to the white bed.") == "drive-to-the-white-bed"
