"""task_state.py: what the tools know about the task in progress (TaskState), and the first
sighting of each target (FirstSight).

One TaskState is shared by all the tools; agent.py starts it afresh for every task.
"""

from dataclasses import dataclass, field

# Nothing sees behind the car, and on the real car reversing bumped into things (2026-11-17):
# the tools reverse only when the way forward is blocked, and then at most this far.
BACK_UP_M = 0.2


@dataclass
class FirstSight:
    """The first time a target was seen and named in a task: the picture and where in it.
    crop: its part of the picture (cut out when first needed); tried: the model was asked
    where exactly it is in the picture (if it did not say, there is nothing to compare with)."""

    picture: dict
    where: str
    crop: dict = None
    tried: bool = False


@dataclass
class TaskState:
    """What the tools know about the task in progress."""

    reached: list = field(default_factory=list)   # the targets reached so far, in order
    next_target: str = ""     # the next target, as reached was told
    searched: str = ""        # the last target look_around or approach was asked for
    # Where the car is stopped. All of these clear when the car moves:
    blocked_m: float = None   # something this close in front stops it (None: nothing does)
    seen: str = ""            # what the camera check says that is
    standing_at: str = ""     # the target it has just reached and still stands at
    tries: int = 0            # drives forward that did not move it
    doubted: bool = False     # reached was turned down once here because of the camera check
    close_by_camera: str = ""  # the target the camera sees right in front while the sensor cannot
    no_echo: int = 0          # moves in a row that ended without a distance reading
    misses: int = 0           # look_arounds in a row that did not find the searched target
    just_seen: str = ""       # the target the last look_around or approach saw and named
    not_moving: int = 0       # finished drives in a row that did not change the camera picture
    approaches: int = 0       # approach calls for the searched target without arriving
    # The first sighting of each target ({target: FirstSight}), to compare later ones with.
    # Kept only in memory, and only until the target is reached or the task ends.
    first_seen: dict = field(default_factory=dict)

    def new_task(self):
        """Forget the last task: its targets, their first sightings and where the car stopped."""
        self.reached.clear()
        self.first_seen.clear()
        self.next_target = self.searched = self.just_seen = ""
        self.no_echo = self.not_moving = self.misses = self.approaches = 0
        self.leave_spot()

    def leave_spot(self):
        """The car has moved: whatever it was standing at is behind it now."""
        self.blocked_m, self.seen, self.standing_at, self.close_by_camera = None, "", "", ""
        self.tries, self.doubted = 0, False

    def target_now(self):
        """The target the AI is going for, as far as the tools know."""
        return self.next_target or self.searched

    def progress_note(self):
        note = f" Reached so far: {', '.join(self.reached) if self.reached else 'nothing yet'}."
        if self.next_target:
            note += f" Next target: {self.next_target}."
        return note

    def status_note(self):
        """Added to every picture: progress, and what to do if the car is stuck."""
        note = self.progress_note()
        if self.standing_at:
            note += (f" You are still at the {self.standing_at}: back up {BACK_UP_M:g} m (drive with"
                     f" distance_m {-BACK_UP_M:g}) and turn away from it.")
        elif self.close_by_camera:
            note += (f" The camera shows the {self.close_by_camera} right in front of the car: you have"
                     f" arrived, call reached now.")
        elif self.blocked_m is not None:
            target = f"the {self.target_now()}" if self.target_now() else "your target"
            note += " The car is stopped right in front of something and cannot drive forward."
            if self.seen:
                note += f' The camera check says: "{self.seen}".'
            note += f" If that is {target}, you have arrived: call reached now. If not, back up or turn."
        return note
