# picar_car: the real PiCar-B

The nodes that run on the Raspberry Pi. They are the same as in the twin, with the
real hardware instead of Gazebo:

| Node | What it does | Twin | Real car |
|---|---|---|---|
| `camera` | publishes `/camera/image_raw` | Gazebo bridge | `camera_ros` |
| `ultrasonic` | publishes `/ultrasonic/range` | `picar_gazebo`'s `sim_ultrasonic.py` (Gazebo's reading, with noise and missing echoes) | this package |
| `picar_base` | motors, steering, camera head, safety stop | sim backend | hw backend (this package's `hw_backend.py`) |
| agent | the AI (`shared/picar_agent/agent.py`) | same | same |
| `command_listener` | prints the drive commands (`picar_listener`) | same | same |

## Install (once)

The repository's `setup_car.sh picar` does all of this. On the car (Ubuntu 22.04, ROS 2
Humble) our code has its own workspace, `~/squad_ws`: the team's `~/picar_ws` stays theirs.

```bash
sudo apt install python3-gpiozero python3-lgpio python3-rpi.gpio python3-smbus \
    ros-$ROS_DISTRO-camera-ros ros-$ROS_DISTRO-compressed-image-transport
cd ~/squad_ws
colcon build --symlink-install --packages-select picar_interfaces picar_base picar_listener picar_car
```

On Ubuntu the distance sensor's pins (GPIO 8 and 11) also belong to SPI, which Ubuntu
turns on. If the node says the pins are busy, put `dtparam=spi=off` in
`/boot/firmware/config.txt` (instead of `dtparam=spi=on`) and reboot.

If the camera is not found, the camera library in the ROS package may not support your
camera module: see the camera_ros README (github.com/christianrauch/camera_ros) for
building the Raspberry Pi version of libcamera.

## Run

```bash
ros2 launch picar_car car.launch.py                              # camera, distance sensor, picar_base
ros2 launch picar_car car.launch.py camera:=false base:=false    # only the distance sensor
```

Until `camera_ros` is installed, add `camera:=false`, otherwise the launch stops with
"package 'camera_ros' not found".

**picar_base moves the car as soon as it starts:** it centres the steering and the camera,
and then drives whatever `/cmd_vel` or `/picar/move` asks for. The first time, put the car
on a box so the wheels are off the ground.

**Never run the team's `picar_control` nodes at the same time** (`motor_steering_node`,
`ultrasonic_node`, `picar_control.launch.py`): they use the same pins and the same servo
board, so the two programs would fight over the car.

Checks, in a second terminal:

```bash
ros2 node list                                 # camera, ultrasonic (and picar_base)
ros2 topic echo /ultrasonic/range --field range   # metres; .inf = nothing in range, .nan = no echo
ros2 topic hz /camera/image_raw                # about 10 pictures a second
```

## Settings

All in `config/car.yaml`: the camera's picture size and mode, the distance sensor's
pins, and `picar_base`'s speed and safety values.

## The hw backend (motors, steering, camera head)

`picar_car/hw_backend.py` is the part of `picar_base` that moves the real car, on
Adeept's board:

| Part | How | Default (car.yaml, `hw:`) |
|---|---|---|
| rear motors | the Pi's own pins, RPi.GPIO PWM at 1000 Hz, both motors together | A: enable 4, forward 14, backward 15; B: 17, 27, 18 |
| steering | servo on the PCA9685 board (I2C bus 1, 0x40, 50 Hz) | channel 2, centre 300, full left 242, full right 350 |
| camera head | two servos on the same board | pan channel 6 (centre 311), tilt channel 10 (centre 340) |

The numbers are the team's, from `picar_control/motor_steering_node.py`, which drives this
car. Servo values are the board's steps (1/4096 of 20 ms): lower turns left and tilts up.

- **Speed:** the motors stall below about 60 % duty, so power 0..1 becomes 60..100 %.
  picar_base's safety limit (`max_speed: 0.4`) makes that 76 % at most.
- **Steering:** `max_steering_deg` (30) is full lock; left and right have their own
  full-lock values. Reversing keeps the wheels where they point.
- **Camera head:** 2.2 steps per degree (a guess: measure it), and never outside the
  team's safe ranges (`pan_limits`, `tilt_limits`), whatever angle is asked for.
- **Watchdog:** picar_base sends a command 20 times a second. If none comes for 0.5 s,
  the motors stop. On exit the motor pins stay driven low.

Tests, on fake pins and a fake servo board (no Pi needed): `python3 -m pytest test`.
