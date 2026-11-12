"""Start the PiCar-B twin: Gazebo, the car model, the bridge nodes, picar_base and the
distance sensor node.

    ros2 launch picar_gazebo sim.launch.py                 # with the Gazebo window
    ros2 launch picar_gazebo sim.launch.py headless:=true  # no window (remote / server)
    ros2 launch picar_gazebo sim.launch.py x:=-0.6 y:=0.2 yaw:=90   # another start

This is squad_gazebo's launcher with car:=picar (`sim` starts the same), so there is one
place for the twin's nodes: squad_gazebo/config/cars.yaml. They are the same as on the
real car, with Gazebo standing in for the hardware:
    camera       the camera            (real car: camera_ros)
    ultrasonic   the distance sensor   (real car: the ultrasonic node in picar_car)
    picar_base   motors, steering, camera head and the safety stop
    sim_bridge   twin only: picar_base's commands to Gazebo, the sensor's raw reading,
                 Gazebo's odometry and the clock
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    squad_launch = os.path.join(
        get_package_share_directory('squad_gazebo'), 'launch', 'sim.launch.py')
    passed = ('headless', 'x', 'y', 'yaw')
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='true = no Gazebo window (sensors still render offscreen)'),
        DeclareLaunchArgument('x', default_value='', description='start x in metres'),
        DeclareLaunchArgument('y', default_value='', description='start y in metres'),
        DeclareLaunchArgument('yaw', default_value='', description='start heading in degrees'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(squad_launch),
            launch_arguments=[('car', 'picar')] + [(name, LaunchConfiguration(name)) for name in passed],
        ),
    ])
