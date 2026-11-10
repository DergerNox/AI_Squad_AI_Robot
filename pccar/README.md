# PC car (planned)

A second car next to the PiCar-B, not a replacement. Nothing is built yet: this folder
holds the plan. It uses the same agent, interfaces and safety rules as the PiCar-B
(`shared/`), with other hardware.

## Hardware (given, can't be changed)

| Part | Job |
|---|---|
| Panasonic Toughbook CF-20 mk2 (i5/m5-7Y57, 8 GB RAM, Intel graphics) | Lubuntu 24.04, ROS 2 Jazzy, the agent |
| ESP32-S3, programmed with the Arduino IDE | motors, servos, 3 ultrasonic sensors |
| 16-channel servo controller | written down as "16/4"; the meaning is unclear |
| Samsung Galaxy A12 on USB | the camera |

## How it fits together (agreed in principle)

| Node | Job | On the PiCar-B |
|---|---|---|
| `camera` (`v4l2_camera`) | DroidCam over USB → `/dev/videoN` → `/camera/image_raw`, 640×480 | `camera_ros` |
| `esp32_bridge` | owns the serial port: publishes the ranges, passes picar_base's motor and servo commands to the ESP32 | `sim_bridge` has this role in the twin |
| `picar_base`, `esp32` backend | SafetyGate, motors, steering | `hw` backend |
| agent, `command_listener` | the same | the same |

- **Lubuntu 24.04, not 26.04.** ROS 2 Jazzy is Tier 1 only on 24.04 (26.04 goes with
  Lyrical), and everything else is on Jazzy and Gazebo Harmonic.
- **Camera.** The A12's last update is Android 12, and Android's own USB webcam mode
  needs Android 14 QPR1. So: DroidCam over USB (adb + v4l2loopback).
- **ESP32 link.** A plain text protocol over USB serial, no micro-ROS: the precompiled
  micro_ros_arduino for Jazzy is a problem in the Arduino IDE.
- **Safety on the ESP32.** The firmware stops the motors by itself when no command has
  arrived for 0.5 s.
- **Sensors.** `/ultrasonic/range` is the centre sensor, the same topic as on the PiCar-B,
  plus `/ultrasonic/left` and `/ultrasonic/right`. The SafetyGate uses whichever sensors
  a car has.
- **Car numbers.** Speed, size, steering and camera FOV move from `picar_tools.py` to a
  per-car config, and picar_base loads its backend by module name (`esp32` in
  `pccar/pccar_car`).

## Notes for building it

- The HC-SR04's echo pin gives 5 V and the ESP32 takes 3.3 V: put a voltage divider on
  each echo line.
- Fire the 3 sensors about 60 ms apart, so one does not hear another's echo.
- Motors and servos on their own battery, with a common ground with the ESP32.

## Planned packages

| Package | What |
|---|---|
| `pccar_description` | the Gazebo model (needs the sizes below) |
| `pccar_gazebo` | its bridge settings, so `sim` can load it (`cars.yaml` already has a `pccar` entry waiting for these two) |
| `pccar_car` | `esp32_bridge` and picar_base's `esp32` backend |
| `esp32_firmware` | the Arduino sketch and the serial protocol |

## Open questions

- Drive: an Ackermann steering servo, or tank-style?
- What does the servo controller drive, and what does "16/4" mean?
- Where do the 3 sensors point?
- Sizes: wheel diameter, track width, body, camera height.
