# PiCar-B Gazebo twin

The real car's twin, for testing code on a PC. Four ROS 2 Jazzy packages for Gazebo Harmonic:

- `picar_interfaces`: the `Move` and `Look` services.
- `picar_base`: drives the car through the SafetyGate (speed limit, obstacle stop, timeouts). In the twin it drives Gazebo.
- `picar_description`: the car model in `urdf/picar.urdf.xacro`, with rear-wheel drive, Ackermann front steering, a pan/tilt camera head and an ultrasonic sensor.
- `picar_gazebo`: a 3 m × 3 m test room, the bridge settings and `sim.launch.py`.

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

## Nodes

The twin runs the same nodes as the real car (`picar_car/launch/car.launch.py`), with
Gazebo standing in for the hardware:

| Node | Twin | Real car |
|---|---|---|
| `camera` | bridge node, `config/camera_bridge.yaml` | `camera_ros` |
| `ultrasonic` | bridge node, `config/ultrasonic_bridge.yaml` | `ultrasonic` in `picar_car` |
| `picar_base` | sim backend, `picar_base/config/sim.yaml` | hw backend |
| `sim_bridge` | carries picar_base's motor and servo commands to Gazebo, `config/picar_bridge.yaml` | not needed |
| `robot_state_publisher` | the car model, for RViz | |

Drive it from a second terminal:

```bash
source ~/picar_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Teleop publishes `/cmd_vel`, and `picar_base` passes it on through the SafetyGate. Keys:
`i` forward, `,` reverse, `u` / `o` forward while steering left / right. Hold the key
down: the car stops half a second after the last command. `j` / `l` do nothing, because
a car can't turn on the spot.

## Topics

| Topic | Type | From | Notes |
|---|---|---|---|
| `/cmd_vel` | geometry_msgs/Twist | teleop, your nodes | In. Through the SafetyGate |
| `/camera/image_raw` | sensor_msgs/Image | `camera` | 640×480 at 10 Hz, frame `camera_optical_link` |
| `/camera/camera_info` | sensor_msgs/CameraInfo | `camera` | |
| `/ultrasonic/range` | sensor_msgs/Range | `ultrasonic` | 10 Hz, 0.02–4 m; above 4 m means nothing in range |
| `/picar/status` | std_msgs/String | `picar_base` | 2 Hz: ok, timeout, obstacle, no_range or estop |
| `/sim/cmd_vel`, `/sim/head_pan`, `/sim/head_tilt` | | `picar_base` | Twin only, to Gazebo |
| `/sim/odom` | nav_msgs/Odometry | `sim_bridge` | Twin only. Wheel odometry |
| `/joint_states`, `/tf` | | `sim_bridge` | For RViz |
| `/clock` | rosgraph_msgs/Clock | `sim_bridge` | Run your own nodes with `use_sim_time:=true` |

Services, all from `picar_base`: `/picar/move`, `/picar/look`, `/picar/stop`, `/picar/estop`.

## Quick checks

```bash
ros2 node list                                       # camera, ultrasonic, picar_base, sim_bridge, ...
ros2 topic echo /ultrasonic/range --field range      # about 0.71 m at start: red box ahead
ros2 run rqt_image_view rqt_image_view /camera/image_raw
ros2 service call /picar/look picar_interfaces/srv/Look "{pan_deg: 30.0, tilt_deg: 0.0}"   # look left
```

## TODO

The dimensions at the top of `picar.urdf.xacro` are estimates. Measure the wheel radius,
wheelbase, track width and steering limit on the real car and update them, so the twin
turns and stops like the real one.
