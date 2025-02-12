#!/usr/bin/python3

# Import necessary libraries
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import xacro


# Method to generate launch description
def generate_launch_description():

    # Create parameters to be used later in the script.
    # Parameter to synchronise the ROS time with that of the Gazebo.
    use_sim_time = LaunchConfiguration('use_sim_time', default='true') 

    # Parameter with the path of gazebo_ros package.
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Parameter with the path of our package.
    pkg_robot = get_package_share_directory('biorobotics_tutorial')

    # Complete file name of the xacro file.
    xacro_file = os.path.join(pkg_robot, 'models', 'four_links.xacro')
    assert os.path.exists(xacro_file), "File not found: "+str(xacro_file)

    # Process the xacro file to generate URDF and extract the robot description.
    robot_desc_config = xacro.process_file(xacro_file)
    robot_desc = robot_desc_config.toxml()


    # Description for launching the gazebo environment with an empty world.
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
                ),
            launch_arguments={'pause': 'true'}.items()
            )
    spawn_entity = Node(
            package='biorobotics_tutorial',
            executable='spawn_bot',
            name='robot_spawner',
            arguments=[robot_desc, '0.0', '0.0', '0.5'],
            output='screen'
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

    load_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
                'position_controller'],
        output='screen'
    )

    # Return the launch descriptions, where the nodes are added.
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_position_controller],
            )
        ),

        # Add the gazebo node.
        gazebo,
        # Add the spawning node with the object defined by the robot description.
        spawn_entity,
        # Add the robot state publisher node.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {
                    "use_sim_time": use_sim_time, "robot_description": robot_desc
                }],
                output='screen'
            ),
        ])
