"""Start the Gazebo twin for any of the squad's cars.

    ros2 launch squad_gazebo sim.launch.py car:=picar
    ros2 launch squad_gazebo sim.launch.py car:=picar headless:=true
    ros2 launch squad_gazebo sim.launch.py car:=picar x:=-0.6 y:=0.2 yaw:=90
    ros2 launch squad_gazebo sim.launch.py car:=mini
    ros2 launch squad_gazebo sim.launch.py car:=mini world:=room

Which model, world, bridge settings and nodes belong to each car is in config/cars.yaml.
cars:=/path/to/cars.yaml uses another list without rebuilding.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from squad_gazebo import cars


def start_car(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    registry_file = arg('cars') or os.path.join(
        get_package_share_directory('squad_gazebo'), 'config', 'cars.yaml')
    try:
        car = cars.resolve(cars.load(registry_file), arg('car'), world=arg('world'),
                           x=arg('x'), y=arg('y'), yaw=arg('yaw'))
    except cars.CarError as error:
        raise RuntimeError(f'cannot start the twin: {error}') from None

    headless = arg('headless').lower() in ('true', '1', 'yes')
    gz_args = f'-r -s --headless-rendering {car.world}' if headless else f'-r {car.world}'
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', car.model]), value_type=str),
            'use_sim_time': True,
        }],
        output='screen',
    )

    spawn = car.spawn
    spawner = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', car.gz_name,
                   '-x', str(spawn['x']), '-y', str(spawn['y']), '-z', str(spawn['z']),
                   '-Y', str(spawn['yaw'])],
        output='screen',
    )

    bridges = [
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': bridge.config}], output='screen',
             **({'name': bridge.name} if bridge.name else {}))
        for bridge in car.bridges
    ]

    nodes = [
        ExecuteProcess(cmd=[sys.executable, node.script], name=node.name, output='screen')
        if node.script else
        Node(package=node.package, executable=node.executable, parameters=node.parameters,
             output='screen', **({'name': node.name} if node.name else {}))
        for node in car.nodes
    ]

    return [
        LogInfo(msg=f"Twin: car '{car.car}' as model '{car.gz_name}' ({car.model}) in {car.world}"),
        gazebo, robot_state_publisher, spawner, *bridges, *nodes,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('car', default_value='picar',
                              description='which car from cars.yaml to load'),
        DeclareLaunchArgument('world', default_value='',
                              description="a world name from cars.yaml or an .sdf path "
                                          "(empty: the car's own world)"),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true = no Gazebo window (sensors still render offscreen)'),
        DeclareLaunchArgument('x', default_value='', description='start x in metres'),
        DeclareLaunchArgument('y', default_value='', description='start y in metres'),
        DeclareLaunchArgument('yaw', default_value='', description='start heading in degrees'),
        DeclareLaunchArgument('cars', default_value='',
                              description='another cars.yaml (empty: the installed one)'),
        OpaqueFunction(function=start_car),
    ])