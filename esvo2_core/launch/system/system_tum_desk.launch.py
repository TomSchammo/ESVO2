#!/usr/bin/env python3
"""
ROS2 launch file for ESVO2 system with TUM Desk dataset configuration.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    esvo2_core_share = get_package_share_directory('esvo2_core')
    image_representation_share = get_package_share_directory('image_representation')

    # Launch arguments
    calib_dir_arg = DeclareLaunchArgument(
        'calib_dir',
        default_value=os.path.join(esvo2_core_share, 'calib', 'tum_desk'),
        description='Calibration directory path'
    )

    # Image representation parameters
    ir_params_file = os.path.join(image_representation_share, 'cfg', 'image_representation_fast.yaml')
    ir_params_file_r = os.path.join(image_representation_share, 'cfg', 'image_representation_fast_r.yaml')

    # Image Representation - Left
    image_representation_left = Node(
        package='image_representation',
        executable='image_representation',
        name='image_representation_left',
        remappings=[
            ('events', '/davis/left/events'),
            ('imu', '/davis/left/imu'),
            ('image', '/davis/left/image_raw'),
            ('camera_info', '/davis/left/camera_info'),
            ('image_representation_TS_', 'image_representation_TS_l'),
            ('image_representation_negative_TS_', 'image_representation_negative_TS_l'),
            ('image_representation_AA_frequency_', 'AA_left'),
            ('image_representation_AA_mat_', 'AA_map'),
            ('dx_image_pub_', 'dx_image_pub_l'),
            ('dy_image_pub_', 'dy_image_pub_l'),
        ],
        parameters=[
            ir_params_file,
            {
                'use_sim_time': True,
                'calibInfoDir': LaunchConfiguration('calib_dir'),
            }
        ],
        output='screen'
    )

    # Image Representation - Right
    image_representation_right = Node(
        package='image_representation',
        executable='image_representation',
        name='image_representation_right',
        remappings=[
            ('events', '/davis/right/events'),
            ('imu', '/davis/right/imu'),
            ('image', '/davis/right/image_raw'),
            ('camera_info', '/davis/right/camera_info'),
            ('image_representation_TS_', 'image_representation_TS_r'),
        ],
        parameters=[
            ir_params_file_r,
            {
                'use_sim_time': True,
                'calibInfoDir': LaunchConfiguration('calib_dir'),
            }
        ],
        output='screen'
    )

    # Mapping node
    mapping_params_file = os.path.join(esvo2_core_share, 'cfg', 'mapping', 'mapping_tum_AA.yaml')
    esvo2_mapping = Node(
        package='esvo2_core',
        executable='esvo2_Mapping',
        name='esvo2_Mapping',
        remappings=[
            ('time_surface_left', 'image_representation_TS_l'),
            ('time_surface_right', 'image_representation_TS_r'),
            ('time_surface_negative', 'image_representation_negative_TS_l'),
            ('time_surface_negative_dx', 'dx_image_pub_l'),
            ('time_surface_negative_dy', 'dy_image_pub_l'),
            ('stamped_pose', '/esvo2_tracking/pose_pub'),
            ('events_left', '/davis/left/events'),
            ('events_right', '/davis/right/events'),
        ],
        parameters=[
            mapping_params_file,
            {
                'use_sim_time': True,
                'dvs_frame_id': 'dvs',
                'world_frame_id': 'map',
                'calibInfoDir': LaunchConfiguration('calib_dir'),
            }
        ],
        output='screen'
    )

    # Tracking node
    tracking_params_file = os.path.join(esvo2_core_share, 'cfg', 'tracking', 'tracking_tum_AA.yaml')
    esvo2_tracking = Node(
        package='esvo2_core',
        executable='esvo2_Tracking',
        name='esvo2_Tracking',
        remappings=[
            ('time_surface_left', 'image_representation_TS_l'),
            ('time_surface_right', 'image_representation_TS_r'),
            ('time_surface_negative', 'image_representation_negative_TS_l'),
            ('time_surface_dx', 'dx_image_pub_l'),
            ('time_surface_dy', 'dy_image_pub_l'),
            ('stamped_pose', '/esvo2_tracking/pose_pub'),
            ('gt_pose', '/optitrack/davis_stereo'),
            ('events_left', '/davis/left/events'),
            ('pointcloud', '/esvo2_mapping/pointcloud_local2'),
        ],
        parameters=[
            tracking_params_file,
            {
                'use_sim_time': True,
                'dvs_frame_id': 'dvs',
                'world_frame_id': 'map',
                'calibInfoDir': LaunchConfiguration('calib_dir'),
            }
        ],
        output='screen'
    )

    # RViz
    rviz_config = os.path.join(esvo2_core_share, 'esvo2_system.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        calib_dir_arg,
        # Nodes
        image_representation_left,
        image_representation_right,
        esvo2_mapping,
        esvo2_tracking,
        rviz_node,
    ])
