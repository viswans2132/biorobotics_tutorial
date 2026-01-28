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

    # # Complete file name of the xacro file.
    # xacro_file = os.path.join(pkg_robot, 'models', 'four_links.xacro')
    # assert os.path.exists(xacro_file), "File not found: "+str(xacro_file)

    # # Process the xacro file to generate URDF and extract the robot description.
    # robot_desc_config = xacro.process_file(xacro_file)
    # robot_desc = robot_desc_config.toxml()
    world_file = os.path.join(pkg_robot, 'worlds', 'cylinder_world.world')


    # Description for launching the gazebo environment with an empty world.
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
                ),
            launch_arguments={'pause': 'false'}.items()
            )
    spawn_entity = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
            get_package_share_directory('go1_gazebo'), 'launch', 'spawn_go1.launch.py'),
                ),
            launch_arguments={'world_file_name': world_file}.items()
            )

    # Return the launch descriptions, where the nodes are added.
    return LaunchDescription([
        # Add the spawning node with the object defined by the robot description.
        spawn_entity,
        # Add the robot state publisher node.
        ])
