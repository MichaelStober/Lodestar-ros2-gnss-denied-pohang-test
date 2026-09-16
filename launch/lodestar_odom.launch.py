"""Run the offline LodeStar odometry on a ROS 2 bag, with RViz2.

Example:

    ros2 launch lodestar_odometry lodestar_odom.launch.py \
        bag_path:=$HOME/ros2_data/odom_test/test_sequence3 \
        est_directory:=$HOME/ros2_data/odom_test/test_sequence3/eval/ \
        sequence:=test_sequence3

Note that `est_directory` must end with a trailing slash: the node writes its
parameter dump to `<est_directory>../pars.txt`, exactly as the ROS 1 version
did.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Defaults mirror scripts/run_lodestar_odom.sh
_ARGUMENTS = [
    ('bag_path', '', 'ROS 2 bag directory (containing metadata.yaml) or .mcap/.db3 file'),
    ('est_directory', '', 'Output directory for the estimated trajectory (trailing slash)'),
    ('sequence', 'test_sequence3', 'Sequence name, used for the output file naming'),
    ('radar_topic', '/radar_image_inrange', 'Cartesian radar image topic inside the bag'),
    ('range_resolution', '0.05', 'Range resolution [m/pixel]'),
    ('cost_type', 'P2P', 'Registration cost: P2P, P2L or P2D'),
    ('submap_scan_size', '1', 'Number of keyframes in the submap'),
    ('registered_min_keyframe_dist', '1.5', 'Minimum keyframe distance [m]'),
    ('res', '3', 'Point-normal resolution'),
    ('zmin', '60', 'Minimum intensity (expected noise level)'),
    ('weight_option', '4', 'Residual weighting option'),
    ('weight_intensity', 'true', 'Weight residuals by intensity'),
    ('soft_constraint', 'false', 'Use soft velocity constraint'),
    ('disable_compensate', 'true', 'Disable motion compensation'),
    ('contour_threshold', '214', 'Contour extraction threshold'),
    ('k_nearest', '20', 'k nearest points from the radar centre'),
    ('dataset', 'marine', 'Dataset name'),
    ('job_nr', '1', 'Job number'),
]


def generate_launch_description():
    share = get_package_share_directory('lodestar_odometry')

    declared = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, desc in _ARGUMENTS
    ]

    def cfg(name):
        return LaunchConfiguration(name)

    odom_node = Node(
        package='lodestar_odometry',
        executable='lodestar_odom',
        name='lodestar_odom_node',
        output='screen',
        arguments=[
            '--bag_path', cfg('bag_path'),
            '--est_directory', cfg('est_directory'),
            '--sequence', cfg('sequence'),
            '--radar_topic', cfg('radar_topic'),
            '--range-res', cfg('range_resolution'),
            '--cost_type', cfg('cost_type'),
            '--submap_scan_size', cfg('submap_scan_size'),
            '--registered_min_keyframe_dist', cfg('registered_min_keyframe_dist'),
            '--res', cfg('res'),
            '--z-min', cfg('zmin'),
            '--weight_option', cfg('weight_option'),
            '--weight_intensity', cfg('weight_intensity'),
            '--soft_constraint', cfg('soft_constraint'),
            '--disable_compensate', cfg('disable_compensate'),
            '--contour_threshold', cfg('contour_threshold'),
            '--k_nearest', cfg('k_nearest'),
            '--dataset', cfg('dataset'),
            '--job_nr', cfg('job_nr'),
        ],
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'vis.launch.py'))
    )

    return LaunchDescription(declared + [rviz, odom_node])
