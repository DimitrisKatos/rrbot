from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument ,IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
import xacro


def generate_launch_description():

    pkg_name = 'rrbot'
    file_subpath = 'urdf/rrbot5.xacro'


    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw =xacro.process_file(xacro_file).toxml()
    gazebo_pkg = get_package_share_directory('gazebo_ros')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output="screen",
        parameters=[{'robot_description':robot_description_raw,
        'use_sim_time':True}]
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            gazebo_pkg, 'launch'), '/gazebo.launch.py'
            ])
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "my_robot_description"],
        output="screen",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    base_mid_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_base_mid_position_controller", "--controller-manager", "/controller_manager"],
    )
    mid_top_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_mid_top_position_controller", "--controller-manager", "/controller_manager"],
    )
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_gripper_controller", "--controller-manager", "/controller_manager"],
    )
    return LaunchDescription([ 
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        rviz_node,
        joint_state_broadcaster_spawner,
        base_mid_controller_spawner,
        mid_top_controller_spawner,
        gripper_controller_spawner])
