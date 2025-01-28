#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import xacro


def generate_launch_description():
    namespace = 'robot'
    box_name = 'ground'
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot = get_package_share_directory('gazebo_urdf_tutorial')
    xacro_file = os.path.join(pkg_robot, 'models', 'quadruped.xacro')
    assert os.path.exists(xacro_file), "The file path seems to be wrong: "+str(xacro_file)
    box_file = os.path.join(pkg_robot, 'models', 'box.xacro')
    assert os.path.exists(box_file), "The file path seems to be wrong: "+str(box_file)
    # world_file = os.path.join(pkg_gazebo_ros, 'worlds', 'empty.world')
    # assert os.path.exists(world_file), "The file path seems to be wrong: "+str(world_file)

    robot_desc_config = xacro.process_file(xacro_file,
        mappings={'namespace':namespace})
    robot_desc = robot_desc_config.toxml()



    box_desc_config = xacro.process_file(box_file,
        mappings={'box_name':box_name, 'mass': '100.0'})
    box_desc = box_desc_config.toxml()






    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
                ),
            launch_arguments={'pause': 'false'}.items()
            )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_gazebo_ros, 'worlds', 'empty.world'), ''],
            description='SDF world file'),
        gazebo,
        Node(
            package='gazebo_urdf_tutorial',
            executable='spawn_bot',
            name='robot_spawner',
            arguments=[robot_desc, '0.0', '0.0', '0.5'],
            output='screen'
        ),
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
        # Node(
        #     package='gazebo_urdf_tutorial',
        #     executable='quadruped_controller',
        #     name='quadruped_controller',
        #     arguments=[namespace,'-0.5', '-1.0', '0.0', '0.0'],
        #     output='screen'
        # ),        
        ])
