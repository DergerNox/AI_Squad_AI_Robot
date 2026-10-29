# PiCar-B Gazebo twin

Two ROS 2 Jazzy packages for Gazebo Harmonic:

- `picar_description`: the car model in `urdf/picar.urdf.xacro`, with rear-wheel drive, Ackermann front steering, a pan/tilt camera head and an ultrasonic sensor.
- `picar_gazebo`: a 3 m × 3 m test room, the ROS ↔ Gazebo bridge config and `sim.launch.py`.

## Build

```bash
cd ~/picar_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch picar_gazebo sim.launch.py                  # with the Gazebo window
ros2 launch picar_gazebo sim.launch.py headless:=true   # no window
```

Drive it from a second terminal:

```bash
source ~/picar_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/sim/cmd_vel
```

Keys: `i` forward, `,` reverse, `u` / `o` forward while steering left / right, `k` stop.
`j` / `l` only turn the front wheels, because a car can't turn on the spot.
The twin keeps driving at the last command until you press `k`.

Teleop goes straight to the twin for now. Once the `picar_base` sim backend exists,
teleop and the agent publish `/cmd_vel` and the SafetyGate forwards it to `/sim/cmd_vel`.

## Topics

| Topic | Type | Notes |
|---|---|---|
| `/sim/cmd_vel` | geometry_msgs/Twist | In. `linear.x` in m/s (max 0.8), `angular.z` yaw rate in rad/s |
| `/sim/head_pan`, `/sim/head_tilt` | std_msgs/Float64 | In. Radians; pan + = left, tilt + = up |
| `/camera/image_raw` | sensor_msgs/Image | Out. 640×480 at 10 Hz, frame `camera_optical_link` |
| `/camera/camera_info` | sensor_msgs/CameraInfo | Out |
| `/ultrasonic/range` | sensor_msgs/Range | Out. 10 Hz, 0.02–4 m; above 4 m means nothing in range |
| `/sim/odom` | nav_msgs/Odometry | Out. Wheel odometry |
| `/joint_states`, `/tf` | | Out. For RViz |
| `/clock` | rosgraph_msgs/Clock | Out. Run your own nodes with `use_sim_time:=true` |

## Quick checks

```bash
ros2 topic echo /ultrasonic/range --field range            # about 0.71 m at start: red box ahead
ros2 run rqt_image_view rqt_image_view /camera/image_raw
ros2 topic pub --once /sim/head_pan std_msgs/msg/Float64 "{data: 0.5}"   # look left
```

## TODO

The dimensions at the top of `picar.urdf.xacro` are estimates. Measure the wheel radius,
wheelbase, track width and steering limit on the real car and update them, so the twin
turns and stops like the real one.
