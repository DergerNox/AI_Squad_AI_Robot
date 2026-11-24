"""eyes.py: the tools' questions to the vision model about pictures, and the first sighting of
each target that later pictures are compared with.

Eyes asks one short question at a time, without the chat (questions.py has the texts), keeps
every picture and answer in the picture log, and decides whether a picture shows the target:
named as its kind, or like its first sighting in this task.
"""

from dataclasses import dataclass

from rai.messages import HumanMultimodalMessage

from picture_math import both, cut_out, nearest_boxes, part_of
from questions import (IN_FRONT, IN_FRONT_FIRST, LOCATE, NAME_CROP, NAME_IT, OTHER, SAME, WHERE,
                       boxes_in, names_target, place, says_yes, thing_of, vague_name)
from task_state import FirstSight


@dataclass
class Eyes:
    """vision: the chat model (None: it sees nothing); task: the TaskState; log: the PictureLog."""

    vision: object
    task: object
    log: object

    def ask(self, picture, question, label="camera", limit=60, echo=True):
        """One short question to the vision model about a picture, without the chat. The
        answer comes back in lower case, at most limit characters. echo: print it."""
        if self.vision is None:
            return ""
        message = HumanMultimodalMessage(content=question, images=picture["images"])
        try:
            # No callbacks: the answer is not streamed to the terminal or /to_human
            answer = self.vision.invoke([message], config={"callbacks": []}).content
        except Exception as error:
            print(f"\n[camera question failed: {error}]")
            return ""
        answer = " ".join(str(answer).strip(" .'\"").lower().split())[:limit]
        self.log.save(picture, label, question, answer)
        if echo:
            print(f"\n[{label}: {answer}]")
        return answer

    def locate(self, picture, target, label):
        """The bounding boxes the model gives for the target; prints them short:
        "green bag 20-50% across, down to 72%"."""
        boxes = boxes_in(self.ask(picture, LOCATE.format(target=target), label, limit=1500, echo=False))
        shown = "; ".join(f"{name or '?'} {x1:.0%}-{x2:.0%} across, down to {y2:.0%}"
                          for name, x1, _, x2, y2 in boxes)
        print(f"\n[{label}: {shown or 'none'}]")
        return boxes

    def where_is(self, picture, target, label, confirm=True, lenient=False):
        """Where the vision model sees the target in a picture: "left", "middle",
        "right", or "" if it is not there. confirm: the model must also name the thing
        it sees as the target's kind (see NAME_IT), or a part of the picture it locates must
        look like the target's first sighting; lenient: a vague name will do (VAGUE_NAMES)."""
        where = place(self.ask(picture, WHERE.format(target=target), label))
        if not where or not confirm:
            return where
        thing = thing_of(target)
        name = self.ask(picture, NAME_IT.format(thing=thing, where=where), f"{label}, named")
        if names_target(name, target) or (lenient and vague_name(name)):
            self.seen_first(picture, target, where)
            return where
        # Perhaps something else of the target's colour stands in front of it: ask where the
        # target is exactly, and whether that part of the picture shows it
        boxed = self.confirm_by_box(picture, target, label, where)
        if boxed:
            return boxed
        # Something else of the target's colour: ask once more past it
        other = place(self.ask(picture, OTHER.format(target=target, name=name), f"{label}, other"))
        if not other or other == where:
            return ""
        name = self.ask(picture, NAME_IT.format(thing=thing, where=other), f"{label}, named")
        if names_target(name, target) or (lenient and vague_name(name)):
            self.seen_first(picture, target, other)
            return other
        return ""

    def confirm_by_box(self, picture, target, label, near):
        """left, middle or right if a bounding box the model gives for the target (the box nearest
        `near` first, at most two), cut out of the picture, looks like the target's first sighting,
        or without one, shows the target's kind; else ""."""
        boxes = self.locate(picture, target, f"{label}, where exactly")
        for box in nearest_boxes(boxes, near)[:2]:
            if self.is_target(picture, box, target, label):
                return part_of(box)
        return ""

    def is_target(self, picture, box, target, label, vague=False):
        """True if a bounding box shows the target: like its first sighting, or without one, named
        as its kind in a close look (vague: a vague name will do)."""
        crop = cut_out(picture, *box[1:])
        first = self.reference(target)
        if first is not None:
            return says_yes(self.ask(both(first, crop), SAME.format(target=target),
                                     f"{label}, same as the first sighting"))
        name = self.ask(crop, NAME_CROP.format(thing=thing_of(target)), f"{label}, close look")
        if names_target(name, target) or (vague and vague_name(name)):
            self.seen_first(picture, target, part_of(box), box)
            return True
        return False

    def close_up(self, picture, target, question, label):
        """True if the model says yes to a two-picture question (the target's first sighting,
        then this picture); False without a first sighting."""
        first = self.reference(target)
        if first is None:
            return False
        return says_yes(self.ask(both(first, picture), question.format(target=target), label))

    def seen_first(self, picture, target, where, box=None):
        """Keep the target's first sighting in this task (later ones are compared with it)."""
        if not target or target in self.task.first_seen:
            return
        sight = self.task.first_seen[target] = FirstSight(picture, where)
        if box is not None:
            sight.crop, sight.tried = cut_out(picture, *box[1:]), True
            self.log.save(sight.crop, f"first sighting of the {target}")

    def reference(self, target):
        """The target's part of its first sighting, to compare with; None if there is none."""
        sight = self.task.first_seen.get(target)
        if sight is None:
            return None
        if not sight.tried:   # cut out when first needed: often the names settle everything
            sight.tried = True
            boxes = self.locate(sight.picture, target, "first sighting, where exactly")
            if boxes:
                box = nearest_boxes(boxes, sight.where)[0]
                sight.crop = cut_out(sight.picture, *box[1:])
                self.log.save(sight.crop, f"first sighting of the {target}")
        return sight.crop

    def check_in_front(self, picture):
        """Ask what the car has stopped in front of (the target? Compared with its first
        sighting, if there is one)."""
        target = self.task.target_now()
        first = self.reference(target) if target else None
        if first is not None:
            return self.ask(both(first, picture), IN_FRONT_FIRST.format(target=target))
        if target:
            question = (f"{IN_FRONT} Is it the {target}? Answer 'yes' or 'no', then say "
                        "what it is in two or three words, colour first.")
        else:
            question = (f"{IN_FRONT} What is that thing? Answer with two or three words, "
                        "colour first, like 'green ball' or 'white wall'.")
        return self.ask(picture, question)
