"""Start the PiCar-B twin: Gazebo, the car model, and the ROS <-> Gazebo bridge.

    ros2 launch picar_gazebo sim.launch.py                 # with the Gazebo window
    ros2 launch picar_gazebo sim.launch.py headless:=true  # no window (remote / server)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo_share = get_package_share_directory('picar_gazebo')
    description_share = get_package_share_directory('picar_description')

    world = os.path.join(gazebo_share, 'worlds', 'picar_room.sdf')
    bridge_config = os.path.join(gazebo_share, 'config', 'picar_bridge.yaml')
    xacro_file = os.path.join(description_share, 'urdf', 'picar.urdf.xacro')
    base_config = os.path.join(
    get_package_share_directory('picar_base'), 'config', 'sim.yaml')
    gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    headless = LaunchConfiguration('headless')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # Gazebo with the GUI, or server only with offscreen rendering for sensors
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': f'-r {world}', 'on_exit_shutdown': 'true'}.items(),
        condition=UnlessCondition(headless),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': f'-r -s --headless-rendering {world}',
                          'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(headless),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    # Spawn the URDF published by robot_state_publisher as model "picar"
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'picar', '-z', '0.01'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )
    
    picar_base = Node(
        package='picar_base',
        executable='picar_base',
        parameters=[base_config],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true = no Gazebo window (sensors still render offscreen)'),
        gazebo_gui,
        gazebo_headless,
        robot_state_publisher,
        spawn,
        bridge,
        picar_base,
    ])
