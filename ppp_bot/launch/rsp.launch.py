import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():

    nav2_bringup_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('nav2_bringup'),
                              'launch', 'navigation_launch.py')]),
            launch_arguments=[('use_sim_time', 'true')])


    # Launch!
    return LaunchDescription([
        nav2_bringup_node
    ])