#!/usr/bin/env python3
"""Launch file for complete sheep following simulation.

Launches Gazebo with sheep, LeoRover robot, Nav2 navigation,
YOLO detection, and sheep follower node.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for complete sheep following simulation."""

    # Get package directories
    sheep_follower_dir = get_package_share_directory('sheep_follower')
    gps_wpf_dir = get_package_share_directory('nav2_gps_waypoint_follower_demo')
    yolo_bringup_dir = get_package_share_directory('yolo_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # ==================== Launch Arguments ====================
    
    # Sheep follower arguments
    sheep_id_arg = DeclareLaunchArgument(
        'sheep_id',
        default_value='any',
        description='ID of the sheep to follow (or "any" for any sheep)'
    )
    
    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='map',
        description='Target coordinate frame for navigation'
    )
    
    lost_timeout_arg = DeclareLaunchArgument(
        'lost_timeout',
        default_value='2.0',
        description='Timeout before considering sheep lost (seconds)'
    )
    
    goal_tolerance_arg = DeclareLaunchArgument(
        'goal_tolerance',
        default_value='0.5',
        description='Minimum distance between goals (meters)'
    )

    # YOLO arguments
    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value='yolov8m.pt',
        description='YOLO model to use'
    )
    
    yolo_device_arg = DeclareLaunchArgument(
        'yolo_device',
        default_value='cuda:0',
        description='Device for YOLO inference (cuda:0 or cpu)'
    )
    
    yolo_threshold_arg = DeclareLaunchArgument(
        'yolo_threshold',
        default_value='0.5',
        description='YOLO detection threshold'
    )

    # Visualization arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    # Simulation time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Camera topic (from simulation)
    input_image_topic_arg = DeclareLaunchArgument(
        'input_image_topic',
        default_value='/leo/realsense_d455/image',
        description='Input image topic for YOLO'
    )
    
    input_depth_topic_arg = DeclareLaunchArgument(
        'input_depth_topic',
        default_value='/leo/realsense_d455/depth_image',
        description='Input depth topic for 3D detection'
    )
    
    depth_info_topic_arg = DeclareLaunchArgument(
        'depth_info_topic',
        default_value='/leo/realsense_d455/camera_info',
        description='Depth camera info topic'
    )

    # ==================== Launch Gazebo Simulation ====================
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gps_wpf_dir, 'launch', 'leo_ign_gazebo_gps_world.launch.py')
        )
    )

    # ==================== Launch Robot Localization ====================
    
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gps_wpf_dir, 'launch', 'leo_ign_dual_ekf_navsat.launch.py')
        )
    )

    # ==================== Launch Nav2 Navigation ====================
    
    nav2_params = os.path.join(gps_wpf_dir, 'config', 'leo_nav2_no_map_params.yaml')
    
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params,
            'autostart': 'true',
        }.items()
    )

    # ==================== Launch YOLO Detection ====================
    
    # Delay YOLO to allow Gazebo to start
    yolo_launch = TimerAction(
        period=10.0,  # Wait 10 seconds for Gazebo
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(yolo_bringup_dir, 'launch', 'yolo.launch.py')
                ),
                launch_arguments={
                    'model': LaunchConfiguration('yolo_model'),
                    'device': LaunchConfiguration('yolo_device'),
                    'threshold': LaunchConfiguration('yolo_threshold'),
                    'input_image_topic': LaunchConfiguration('input_image_topic'),
                    'input_depth_topic': LaunchConfiguration('input_depth_topic'),
                    'input_depth_info_topic': LaunchConfiguration('depth_info_topic'),
                    'use_3d': 'True',
                    'use_tracking': 'True',
                    'use_debug': 'True',
                }.items()
            )
        ]
    )

    # ==================== Launch Sheep Follower ====================
    
    # Delay sheep follower to allow YOLO to start
    sheep_follower_node = TimerAction(
        period=15.0,  # Wait 15 seconds
        actions=[
            Node(
                package='sheep_follower',
                executable='nav2_sheep_follower',
                name='nav2_sheep_follower',
                output='screen',
                arguments=['--id', LaunchConfiguration('sheep_id')],
                parameters=[{
                    'target_frame': LaunchConfiguration('target_frame'),
                    'lost_timeout': LaunchConfiguration('lost_timeout'),
                    'goal_tolerance': LaunchConfiguration('goal_tolerance'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }]
            )
        ]
    )

    # ==================== Launch RViz ====================
    
    rviz_config = os.path.join(sheep_follower_dir, 'config', 'sheep_follower.rviz')
    
    # Use ExecuteProcess to clean snap environment variables that break RViz
    # Also pass --ros-args -p use_sim_time:=true for simulation
    rviz_node = ExecuteProcess(
        cmd=['bash', '-c', 
             f'unset GTK_PATH GIO_MODULE_DIR LOCPATH GTK_IM_MODULE_FILE && '
             f'rviz2 -d {rviz_config} --ros-args -p use_sim_time:=true'],
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )

    # ==================== Build Launch Description ====================
    
    return LaunchDescription([
        # Arguments
        sheep_id_arg,
        target_frame_arg,
        lost_timeout_arg,
        goal_tolerance_arg,
        yolo_model_arg,
        yolo_device_arg,
        yolo_threshold_arg,
        use_rviz_arg,
        use_sim_time_arg,
        input_image_topic_arg,
        input_depth_topic_arg,
        depth_info_topic_arg,
        
        # Launches
        gazebo_launch,
        localization_launch,
        navigation_launch,
        yolo_launch,
        sheep_follower_node,
        rviz_node,
    ])
