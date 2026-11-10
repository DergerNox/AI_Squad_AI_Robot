"""Start the real PiCar-B's nodes on the Raspberry Pi.

    ros2 launch picar_car car.launch.py                              # all three
    ros2 launch picar_car car.launch.py camera:=false base:=false    # only the distance sensor

    camera       the camera (camera_ros)                        -> /camera/image_raw
    ultrasonic   the distance sensor (this package)             -> /ultrasonic/range
    picar_base   motors, steering, camera head, safety stop    <- /picar/move, /picar/look, /cmd_vel

The settings of all three are in config/car.yaml. The agent and the command listener
are started separately. The twin runs the same nodes: picar_gazebo/launch/sim.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    settings = os.path.join(get_package_share_directory('picar_car'), 'config', 'car.yaml')

    def node(switch, package, executable, name):
        """A node from car.yaml, started unless its switch is set to false."""
        return Node(package=package, executable=executable, name=name,
                    parameters=[settings], output='screen',
                    condition=IfCondition(LaunchConfiguration(switch)))

    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='true', description='start the camera'),
        DeclareLaunchArgument('ultrasonic', default_value='true',
                              description='start the distance sensor'),
        DeclareLaunchArgument('base', default_value='true',
                              description='start picar_base (motors, steering, camera head)'),
        node('camera', 'camera_ros', 'camera_node', 'camera'),
        node('ultrasonic', 'picar_car', 'ultrasonic', 'ultrasonic'),
        node('base', 'picar_base', 'picar_base', 'picar_base'),
    ])
