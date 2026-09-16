"""Start RViz2 with the LodeStar odometry configuration.

ROS 2 replacement for the ROS 1 `vis.launch`. The `eval` argument is kept for
compatibility; it selects `odom_eval.rviz` when set to true.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('lodestar_odometry')
    eval_cfg = os.path.join(share, 'rviz', 'odom_eval.rviz')
    odom_cfg = os.path.join(share, 'rviz', 'odom.rviz')

    eval_arg = DeclareLaunchArgument(
        'eval',
        default_value='false',
        description='Use the evaluation RViz configuration (odom_eval.rviz)',
    )

    rviz_eval = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', eval_cfg],
        output='screen',
        condition=IfCondition(LaunchConfiguration('eval')),
    )

    rviz_odom = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', odom_cfg],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('eval')),
    )

    return LaunchDescription([eval_arg, rviz_eval, rviz_odom])
