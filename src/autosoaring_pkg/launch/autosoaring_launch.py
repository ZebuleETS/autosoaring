#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _launch_setup(context, *args, **kwargs):
    """Resolve mode at runtime and return the right set of nodes."""
    pkg_dir = get_package_share_directory('autosoaring_pkg')
    config_file = LaunchConfiguration('config_file').perform(context)
    mode = LaunchConfiguration('mode').perform(context)

    # Le générateur tourne toujours
    thermal_generator_node = Node(
        package='autosoaring_pkg',
        executable='thermal_generator_node',
        name='thermal_generator_node',
        arguments=[config_file],
        output='screen'
    )

    nodes = [thermal_generator_node]

    if mode == 'full':
        nodes.append(Node(
            package='autosoaring_pkg',
            executable='thermal_detection_node',
            name='thermal_detection_node',
            arguments=[config_file],
            output='screen'
        ))
        nodes.append(Node(
            package='autosoaring_pkg',
            executable='thermal_mapping_node',
            name='thermal_mapping_node',
            output='screen'
        ))
        nodes.append(Node(
            package='autosoaring_pkg',
            executable='battery_manager_node',
            name='battery_manager_node',
            output='screen'
        ))

    return nodes


def generate_launch_description():
    pkg_dir = get_package_share_directory('autosoaring_pkg')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'thermal_config.yaml'),
        description='Path to the thermal configuration file'
    )

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='full',
        description="Launch mode: 'full' (all nodes) or 'generator' (generator only for PX4 integration)"
    )

    return LaunchDescription([
        config_file_arg,
        mode_arg,
        OpaqueFunction(function=_launch_setup),
    ])





