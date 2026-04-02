#!/usr/bin/env python3
"""
ROS2 launch file for ESVO2 system with DVXplorer tracking configuration.
Use dvx_7.11 for hnu.bag and dvx_7.12 for taozihu.bag.
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
        default_value=os.path.join(esvo2_core_share, 'calib', 'dvx_7.11'),
        description='Calibration directory path (use dvx_7.11 for hnu.bag, dvx_7.12 for taozihu.bag)'
    )

    # Image representation parameters (40Hz variant for tracking)
    ir_params_file = os.path.join(image_representation_share, 'cfg', 'image_representation_fast_40hz.yaml')
    ir_params_file_r = os.path.join(image_representation_share, 'cfg', 'image_representation_fast_r_40hz.yaml')

    # Image Representation - Left
    image_representation_left = Node(
        package='image_representation',
        executable='image_representation',
        name='image_representation_left',
        remappings=[
            ('events', '/dvxplorer_left/events'),
            ('imu', '/dvxplorer_left/imu'),
            ('image', '/dvxplorer_left/image_raw'),
            ('camera_info', '/dvxplorer_left/camera_info'),
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
            ('events', '/dvxplorer_right/events'),
            ('imu', '/dvxplorer_right/imu'),
            ('image', '/dvxplorer_right/image_raw'),
            ('camera_info', '/dvxplorer_right/camera_info'),
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
    mapping_params_file = os.path.join(esvo2_core_share, 'cfg', 'mapping', 'mapping_dvx_AA_tracking.yaml')
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
            ('events_left', '/dvxplorer_left/events'),
            ('events_right', '/dvxplorer_right/events'),
            ('/imu/data', '/dvxplorer_left/imu'),
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
    tracking_params_file = os.path.join(esvo2_core_share, 'cfg', 'tracking', 'tracking_dvx_AA_tracking.yaml')
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
            ('events_left', '/dvxplorer_left/events'),
            ('pointcloud', '/esvo2_mapping/pointcloud_local2'),
            ('/imu/data', '/dvxplorer_left/imu'),
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
    rviz_config = os.path.join(esvo2_core_share, 'esvo2_system_DSEC.rviz')
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
