"""questions.py: what the tools ask the vision model about pictures, and how they read the answers.

The questions are short and ask for short answers (a word, yes or no, a bounding box), because
the small model answers those reliably. Each one notes what happened on the real car that made
it the way it is.
"""

import re

# Where is the target? Answered with one word.
WHERE = ("Is there a {target} in this picture? Answer only 'no', or where it is in the picture: "
         "'left', 'middle' or 'right'.")
# Asked again past something else of the target's colour. The model points at one thing only:
# with a blue bag in front it never pointed at the blue box (real car, 2026-11-17).
OTHER = ("Is there a {target} in this picture, other than the {name}? Answer only 'no', or where it "
         "is in the picture: 'left', 'middle' or 'right'.")

# Asked to name a sighting. The model says "yes, there" to anything of the target's colour
# (on the real car it took a blue bag for a blue box, 3 times out of 3), but names things
# right ("blue bag", "red stick"; 2026-11-14), so a sighting counts only if named right.
NAME_IT = ("This picture is from a small robot car's camera. What is the {thing} at the {where} of "
           "the picture? Answer with two or three words, colour first, like 'red ball' or 'white wall'.")
# Asked about a cut-out of the picture: the target's bounding box. With a white plastic bag in
# front of a white bed, "the white thing in the middle" was always the bag (real car, 2026-11-20).
NAME_CROP = ("This is part of a picture from a small robot car's camera. What is the {thing} in it? "
             "Answer with two or three words, colour first, like 'red ball' or 'white wall'.")

# Ferenc's idea (2026-11-20): the first time a task's target is seen and named, its part of the
# picture is kept, and later sightings that a name cannot settle are compared with it. The real
# model on cut-outs (3 tries each): the white bed from another run "yes"; the plastic bag that
# had fooled it, a slipper and a white door "no".
SAME = ("The first picture shows a {target} that a small robot car saw earlier. The second picture "
        "is from the car's camera now. Is the thing in the second picture the same {target}? "
        "Answer only 'yes' or 'no'.")
# Close to the target. The real model on run 2's pictures (2026-11-24), 2 tries each: the bed's
# drawer and top close up "yes" 6/6, a white plush toy in front "no" 6/6 (asked "part of the same
# bed?" it explained instead, and SAME said "no" to the drawer).
CLOSE_UP = ("The first picture shows a {target} that a small robot car saw earlier. The second picture "
            "is from the car's camera now, very close to something. Is the second picture a close-up "
            "of the same {target}? Answer only 'yes' or 'no'.")

# Where things are in a picture, as bounding boxes: qwen3-vl answers with numbers (0 to 1000
# across and down the picture), much more exactly than in words. On the real car the words
# failed (2026-11-17): "bottom edge" for a box 70 % down the picture (approach stopped 0.7 m
# early: "arrived by the camera") and "left" for a power strip that reached the car's path.
LOCATE = ('Locate the {target} in the image. Output its bounding box in JSON: '
          '[{{"bbox_2d": [x1, y1, x2, y2], "label": "{target}"}}], or [] if there is none.')

# Asked about the picture when the safety system has stopped the car
IN_FRONT = ("This picture is from a small robot car's camera, looking straight ahead. "
            "The car has stopped because something is very close in front of it.")
# The same, with the target's first sighting to compare with
IN_FRONT_FIRST = ("These pictures are from a small robot car's camera. The first one shows a {target} "
                  "it saw earlier. In the second one, looking straight ahead, the car has stopped "
                  "because something is very close in front of it. Is it the same {target}? Answer "
                  "'yes' or 'no', then say what it is in two or three words, colour first.")

# look_around without a real target ("look around and tell me what you see"): the model
# called look_around("unknown") and then drove at it (real car, 2026-11-14). It describes now.
NO_TARGET = {"", "unknown", "anything", "something", "everything", "nothing", "object", "objects",
             "thing", "things", "surroundings", "around", "room", "area", "environment", "scene",
             "view", "what i see", "what you see"}
DESCRIBE = ("This picture is from a small robot car's camera. Describe it in one short sentence: "
            "the main things you see and their colours.")

COLOUR_WORDS = {"red", "blue", "green", "yellow", "white", "black", "orange", "purple", "pink",
                "brown", "grey", "gray"}
KINDS = {   # what may stand for the target's kind in the model's name for it
    "box": ("box", "cube", "carton", "crate", "package", "block", "container"),
    # not blankets, duvets or pillows: things of their own, which were taken for the bed (2026-11-20)
    "bed": ("bed", "mattress", "sheet", "bedding"),
    "stick": ("stick", "pole", "rod", "bar", "pipe"),
    "cylinder": ("cylinder", "can", "tube", "pillar", "column"),
    "ball": ("ball", "sphere"),
    "shoe": ("shoe", "slipper", "sandal", "sneaker", "boot", "trainer"),
}
# What the model calls a target it has just seen and named, once the car is close: the real
# car's blue box became "blue wall" and "blue object" (2026-11-17). Accepted only right after
# the target was seen and named (TaskState.just_seen), never while searching: blue look-alikes
# get names of their own ("blue backpack", "blue books").
VAGUE_NAMES = {"object", "thing", "item", "wall", "surface", "panel", "shape", "rectangle",
               "square", "board", "cardboard", "packaging"}


def no_target(target):
    """True if a look_around target names nothing to search for."""
    return re.sub(r"^(the|a|an|any)\s+", "", target.strip().lower()) in NO_TARGET


def thing_of(target):
    """What to call the target in a question that must not give away what it is."""
    first = target.split()[0] if target.split() else ""
    return f"{first} thing" if first in COLOUR_WORDS else "object"


def place(answer):
    """left, middle, right or "" (not there) from an answer to WHERE or OTHER."""
    words = re.findall(r"[a-z]+", answer)
    if not words or words[0] in ("no", "not", "none", "nothing"):
        return ""
    where = next((w for w in ("left", "middle", "centre", "center", "right") if w in words), "")
    if where in ("centre", "center") or (not where and words[0] == "yes"):
        where = "middle"   # in the middle, or seen without saying where
    return where


def says_yes(answer):
    """True if an answer to a yes/no question starts with yes."""
    words = re.findall(r"[a-z]+", answer.lower())
    return bool(words) and words[0] == "yes"


def says_no(answer):
    """True if an answer to a yes/no question starts with no."""
    words = re.findall(r"[a-z]+", answer.lower())
    return bool(words) and words[0] in ("no", "not")


def vague_name(answer):
    """True if the model's name for a thing says only its colour and that it is a thing."""
    words = re.findall(r"[a-z]+", answer)
    return bool(words) and words[-1] in VAGUE_NAMES


def word_forms(word):
    """A word and its singular forms: boxes -> {boxes, boxe, box}."""
    forms = {word}
    if word.endswith("s") and len(word) > 3:
        forms.add(word[:-1])
    if word.endswith("es") and len(word) > 4:
        forms.add(word[:-2])
    return forms


def one_letter_off(a, b):
    """True if a and b differ by one letter: one changed, added or left out."""
    if abs(len(a) - len(b)) > 1:
        return False
    if len(a) == len(b):
        return sum(x != y for x, y in zip(a, b)) == 1
    short, long_ = sorted((a, b), key=len)
    return any(long_[:i] + long_[i + 1:] == short for i in range(len(long_)))


def same_word(a, b):
    """The same word, also as a plural, or with one letter wrong in a word of five letters or
    more ("sliper" for "slipper": the user's typo made the slipper impossible to confirm)."""
    for x in word_forms(a):
        for y in word_forms(b):
            if x == y or (min(len(x), len(y)) >= 5 and one_letter_off(x, y)):
                return True
    return False


def names_target(answer, target):
    """True if the model's name for a thing says it is the target's kind (box, stick, ...)."""
    kind = target.split()[-1] if target.split() else ""
    if not kind:
        return False
    family = next((words for words in KINDS.values() if any(same_word(kind, w) for w in words)), (kind,))
    return any(same_word(word, f) for word in re.findall(r"[a-z]+", answer) for f in family)


def boxes_in(answer):
    """The bounding boxes in the model's answer: [(label, x1, y1, x2, y2)], as fractions of
    the picture's width and height (0 is the left or top edge, 1 the right or bottom edge)."""
    boxes = []
    for item in re.finditer(r"\{[^{}]*\}", answer):
        numbers = re.search(r'"bbox_2d"\s*:\s*\[\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,'
                            r'\s*([-\d.]+)\s*\]', item.group(0))
        if not numbers:
            continue
        x1, y1, x2, y2 = (min(max(float(v) / 1000, 0.0), 1.0) for v in numbers.groups())
        label = re.search(r'"label"\s*:\s*"([^"]*)"', item.group(0))
        boxes.append(((label.group(1) if label else "").strip().lower(),
                      min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)))
    return boxes
