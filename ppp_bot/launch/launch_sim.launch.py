import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node
import xacro




def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='ppp_bot'

    # Check if we're told to use sim time

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('ppp_bot'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    urdf_file = os.path.join(pkg_path, 'description', 'robot.urdf')
    world_file = os.path.join(pkg_path, 'worlds', 'empty_ppp.sdf')
    robot_description_config = xacro.process_file(xacro_file)
    with open(urdf_file, 'w') as f:
        f.write(robot_description_config.toxml())
    
    # Create a robot_state_publisher node
    node_robot_state_publisher_params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': True}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[node_robot_state_publisher_params]
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )


    node_ignition_launcher_params = {'world_file': world_file}
    node_ignition_launcher = Node(
        package=package_name,
        executable='ignition_launcher',
        parameters=[node_ignition_launcher_params]
    )

    node_urdf_spawner_params = {'service': '/world/empty_ppp/create',
                                'sdf_filename': urdf_file,
                                'name': 'ppp_robot'}
    node_urdf_spawner = Node(
        package=package_name,
        executable='urdf_spawner',
        parameters=[node_urdf_spawner_params]
    )

    # Launch!
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher,
        node_ignition_launcher,
        RegisterEventHandler(
            OnProcessStart(
                target_action=node_ignition_launcher,
                on_start=[
                    LogInfo(msg='Simulation launched, spawning urdf...'),
                    node_urdf_spawner
                ]
            )
        )
    ])
