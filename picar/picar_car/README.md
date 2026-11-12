# picar_car: the real PiCar-B

The nodes that run on the Raspberry Pi. They are the same as in the twin, with the
real hardware instead of Gazebo:

| Node | What it does | Twin | Real car |
|---|---|---|---|
| `camera` | publishes `/camera/image_raw` | Gazebo bridge | `camera_ros` |
| `ultrasonic` | publishes `/ultrasonic/range` | `picar_gazebo`'s `sim_ultrasonic.py` (Gazebo's reading, with noise and missing echoes) | this package |
| `picar_base` | motors, steering, camera head, safety stop | sim backend | hw backend, **not written yet** |
| agent | the AI (`shared/picar_agent/agent.py`) | same | same |
| `command_listener` | prints the drive commands (`picar_listener`) | same | same |

## Install (once)

```bash
sudo apt install python3-gpiozero python3-lgpio ros-jazzy-camera-ros
cd ~/picar_ws
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

Until the hardware part of `picar_base` is written, start with `base:=false`. Until
`camera_ros` is installed, add `camera:=false`, otherwise the launch stops with
"package 'camera_ros' not found".

Checks, in a second terminal:

```bash
ros2 node list                                 # camera, ultrasonic (and picar_base)
ros2 topic echo /ultrasonic/range --field range   # metres; .inf = nothing in range, .nan = no echo
ros2 topic hz /camera/image_raw                # about 10 pictures a second
```

## Settings

All in `config/car.yaml`: the camera's picture size and mode, the distance sensor's
pins, and `picar_base`'s speed and safety values.
