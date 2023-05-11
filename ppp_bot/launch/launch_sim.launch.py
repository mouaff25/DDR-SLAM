import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, LogInfo, ExecuteProcess, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnExecutionComplete, OnProcessExit
from launch.substitutions import EnvironmentVariable
from launch.events import Shutdown

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


    ignition_launcher_params = {'world_file': world_file}

    ignition_launcher = ExecuteProcess(
        cmd=[['ign gazebo ',
            ignition_launcher_params['world_file']]],
        shell=True
    )

    urdf_spawner_params = {'service': '/world/empty_ppp/create',
                                'sdf_filename': urdf_file,
                                'name': 'ppp_robot'}

    urdf_spawner = ExecuteProcess(
        cmd=[['ign service -s ',
           urdf_spawner_params['service'],
            ' --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean ',
            '--timeout 1000 --req \'sdf_filename: "',
            urdf_spawner_params['sdf_filename'],
            '", name: "',
            urdf_spawner_params['name'],
            '", pose: {position: {z: 1.0}}\'']],
        shell=True
    )


    node_parameter_bridge_args = ['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                                '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                                '/world/empty_ppp/model/ppp_robot/link/base_link/sensor/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                                '/world/empty_ppp/model/ppp_robot/link/base_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo']

    node_parameter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=node_parameter_bridge_args,
        output='screen'
    )


    node_teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        prefix='xterm -e',
        output='screen',
    )

    # Launch!
    return LaunchDescription([

        node_robot_state_publisher,

        node_joint_state_publisher,

        ignition_launcher,

        RegisterEventHandler(
            OnProcessStart(
                target_action=ignition_launcher,
                on_start=[
                    LogInfo(msg='Simulation launched, spawning urdf...'),
                    urdf_spawner
                ]
            )
        ),

        RegisterEventHandler(
            OnExecutionComplete(
                target_action=urdf_spawner,
                on_completion=[
                    LogInfo(msg='Spawn finished, establishing parameter_bridge'),
                    node_parameter_bridge
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessStart(
                target_action=node_parameter_bridge,
                on_start=[
                    LogInfo(msg='Running teleop_twist_keyboard...'),
                    node_teleop
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=ignition_launcher,
                on_exit=[
                    LogInfo(msg=(EnvironmentVariable(name='USER'),
                            ' closed the ignition window')),
                    EmitEvent(event=Shutdown(
                        reason='Window closed'))
                ]
            )
        ),
    ])
