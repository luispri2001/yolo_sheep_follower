#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for direct sheep follower."""
    
    # Declare launch arguments
    sheep_id_arg = DeclareLaunchArgument(
        'sheep_id',
        default_value='3',
        description='ID of the sheep to follow (or "sheep" for any sheep)'
    )
    
    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='0.3',
        description='Maximum linear speed (m/s)'
    )
    
    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='0.6',
        description='Maximum angular speed (rad/s)'
    )
    
    min_follow_distance_arg = DeclareLaunchArgument(
        'min_follow_distance',
        default_value='2.5',
        description='Minimum following distance (meters)'
    )
    
    max_follow_distance_arg = DeclareLaunchArgument(
        'max_follow_distance',
        default_value='5.0',
        description='Maximum following distance (meters)'
    )
    
    # Direct sheep follower node
    direct_sheep_follower_node = Node(
        package='sheep_follower',
        executable='direct_sheep_follower',
        name='direct_sheep_follower',
        output='screen',
        arguments=['--id', LaunchConfiguration('sheep_id')],
        parameters=[{
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
            'min_follow_distance': LaunchConfiguration('min_follow_distance'),
            'max_follow_distance': LaunchConfiguration('max_follow_distance'),
        }]
    )
    
    return LaunchDescription([
        sheep_id_arg,
        max_linear_speed_arg,
        max_angular_speed_arg,
        min_follow_distance_arg,
        max_follow_distance_arg,
        direct_sheep_follower_node,
    ])
