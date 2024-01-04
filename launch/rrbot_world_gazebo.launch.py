import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    # Define some packages and some valuable paths.
    pkg_gazebo_ros = get_package_share_path('gazebo_ros')   
    pkg_share = get_package_share_path('rrbot')
    default_model_path = pkg_share /'urdf/rrbot4.xacro'
    default_world_path = pkg_share / 'worlds/rrbot.world'

    
    # Launch configuration variables specific to simulation
    headless = LaunchConfiguration('headless')
    urdf_model = LaunchConfiguration('urdf_model')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')
    
    # Declare the launch arguments         
    simulator = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient')

    model_arg = DeclareLaunchArgument(
        name='urdf_model', 
        default_value=str(default_model_path), 
        description='Absolute path to robot urdf file')
        
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    use_sim_arg = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=str(default_world_path),
        description='Full path to the world model file to load')
    
    # Define the Nodes which will start    
    # Start robot state publisher.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_model])}])

    # Start Gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world}.items())

    # Start Gazebo client    
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

    # spawn the robot
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'rrbot', 
                    '-topic', 'robot_description',],
                        output='screen')
     # also you can define the position where the robot will spawn

    return LaunchDescription([
        simulator,
        model_arg,
        use_sim_time_arg, use_sim_arg,
        world_arg,
        gazebo_server,gazebo_client,
        spawn_entity, robot_state_publisher
    ])