#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Nav2 sheep follower."""
    
    # Declare launch arguments
    sheep_id_arg = DeclareLaunchArgument(
        'sheep_id',
        default_value='3',
        description='ID of the sheep to follow (or "sheep" for any sheep)'
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
        description='Minimum distance between goals to send new command (meters)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Nav2 sheep follower node
    nav2_sheep_follower_node = Node(
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
    
    return LaunchDescription([
        sheep_id_arg,
        target_frame_arg,
        lost_timeout_arg,
        goal_tolerance_arg,
        use_sim_time_arg,
        nav2_sheep_follower_node,
    ])
