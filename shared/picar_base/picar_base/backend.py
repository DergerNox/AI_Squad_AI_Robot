"""backend.py: what picar_base needs from a car's hardware, or from the twin.

picar_base keeps the SafetyGate, the services and the timing; a backend only moves things.
Each car has its own, in its own package, named by picar_base's `backend` parameter:

    sim     picar_base.sim_backend:SimBackend      the Gazebo twin
    hw      picar_car.hw_backend:HwBackend         the real PiCar-B (not written yet)
    esp32   pccar_car.esp32_backend:Esp32Backend   the PC car (not written yet)

or any "package.module:ClassName". A new backend subclasses Backend:

    class HwBackend(Backend):
        def __init__(self, node):
            super().__init__(node)
            ...                                    # open the motor driver and the servos
        def drive(self, power, steering_deg): ...
        def look(self, pan_deg, tilt_deg): ...
        def close(self): ...
"""

import importlib

NAMED_BACKENDS = {
    "sim": "picar_base.sim_backend:SimBackend",
    "hw": "picar_car.hw_backend:HwBackend",
    "esp32": "pccar_car.esp32_backend:Esp32Backend",
}


class BackendError(Exception):
    """The backend cannot be loaded. The message says why."""


class Backend:
    """Moves the car. picar_base calls these from its threads, never two at once.

    node: the picar_base node, for its parameters (node.get_parameter), publishers and logger.
    """

    def __init__(self, node):
        self.node = node

    def drive(self, power, steering_deg):
        """Motor power from -1 to 1 (already through the SafetyGate; positive is forward)
        and the front wheels' angle in degrees (positive steers left). Called 20 times a
        second, also while the car stands still (power 0)."""
        raise NotImplementedError

    def look(self, pan_deg, tilt_deg):
        """Point the camera head, in degrees (already within the limits). Positive pan is
        left, positive tilt is up."""
        raise NotImplementedError

    def close(self):
        """The last call: stop the motors and let go of the hardware."""
        self.drive(0.0, 0.0)


def load_backend(name, node):
    """The backend called `name` (see NAMED_BACKENDS, or "package.module:ClassName")."""
    spec = NAMED_BACKENDS.get(name, name)
    module_name, _, class_name = spec.partition(":")
    if not module_name or not class_name:
        raise BackendError(f"backend '{name}': use one of {', '.join(NAMED_BACKENDS)} "
                           f"or package.module:ClassName")
    try:
        module = importlib.import_module(module_name)
    except ModuleNotFoundError as error:
        if module_name.startswith(error.name or "\0"):
            raise BackendError(f"backend '{name}': {module_name} is not there (not written yet, "
                               f"or its package is not built or not sourced)") from None
        raise
    backend_class = getattr(module, class_name, None)
    if not (isinstance(backend_class, type) and issubclass(backend_class, Backend)):
        raise BackendError(f"backend '{name}': {module_name} has no Backend class {class_name}")
    return backend_class(node)
