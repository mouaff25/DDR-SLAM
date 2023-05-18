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



    ignition_launcher_params = {'world_file': world_file}


    ignition_launcher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py')]),
            launch_arguments=[('gz_args', [f' -r -v 4 {ignition_launcher_params["world_file"]}'])])

    ignition_spawn_entity_params = {'service': '/world/empty_ppp/create',
                                'sdf_filename': urdf_file,
                                'name': 'ppp_robot'}

    ignition_spawn_entity = ExecuteProcess(
        cmd=[['ign service -s ',
           ignition_spawn_entity_params['service'],
            ' --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean ',
            '--timeout 1000 --req \'sdf_filename: "',
            ignition_spawn_entity_params['sdf_filename'],
            '", name: "',
            ignition_spawn_entity_params['name'],
            '", pose: {position: {z: 1.0}}\'']],
        shell=True
    )


    node_parameter_bridge_args = [
                                # '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                                '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                                '/world/empty_ppp/model/ppp_robot/link/base_link/sensor/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                                '/world/empty_ppp/model/ppp_robot/link/base_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']

    node_parameter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=node_parameter_bridge_args,
        output='screen'
    )


    node_teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        remappings=[('/cmd_vel', 'diff_drive_base_controller/cmd_vel_unstamped')],
        prefix='xterm -e',
        output='screen',
    )



    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_base_controller'],
        output='screen'
    )

    # Launch!
    return LaunchDescription([

        RegisterEventHandler(
            OnExecutionComplete(
                target_action=ignition_spawn_entity,
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
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        ignition_launcher,

        node_robot_state_publisher,
        ignition_spawn_entity
        
    ])