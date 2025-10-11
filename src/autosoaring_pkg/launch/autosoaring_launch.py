#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('autosoaring_pkg')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'thermal_config.yaml'),
        description='Path to the thermal configuration file'
    )
    
    # Get launch configuration
    config_file = LaunchConfiguration('config_file')
    
    # Define nodes
    thermal_generator_node = Node(
        package='autosoaring_pkg',
        executable='thermal_generator_node',
        name='thermal_generator_node',
        arguments=[config_file],
        output='screen'
    )
    
    thermal_detection_node = Node(
        package='autosoaring_pkg',
        executable='thermal_detection_node',
        name='thermal_detection_node',
        output='screen'
    )
    
    thermal_mapping_node = Node(
        package='autosoaring_pkg',
        executable='thermal_mapping_node',
        name='thermal_mapping_node',
        output='screen'
    )
    
    battery_manager_node = Node(
        package='autosoaring_pkg',
        executable='battery_manager_node',
        name='battery_manager_node',
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        thermal_generator_node,
        thermal_detection_node,
        thermal_mapping_node,
        battery_manager_node,
    ])
