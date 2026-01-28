#!/usr/bin/python3

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_robot = get_package_share_directory('biorobotics_tutorial')

    xacro_file = os.path.join(pkg_robot, 'models', 'two_link_1.xacro')
    assert os.path.exists(xacro_file), f"File not found: {xacro_file}"

    robot_desc = xacro.process_file(xacro_file).toxml()

    # Start Gazebo (gz) Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        # gz_args controls gz sim flags; '-r' means run immediately.
        # If you want "paused", omit '-r'. Add a world file if you have one.
        launch_arguments={
            'gz_args': 'empty.sdf -r'
            # Example with a world:
            # 'gz_args': f'-r {os.path.join(pkg_robot, "worlds", "empty.sdf")}',
        }.items()
    )

    # Publish robot_description
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc,
        }],
        output='screen',
    )

    # Spawn robot into gz from the /robot_description topic
    spawn_gz = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_bot_gz',
        arguments=[
            '-name', 'two_link',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.5',
        ],
        output='screen',
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen'
    )
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        clock_bridge,
        rsp,
        spawn_gz,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_gz,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
    ])
