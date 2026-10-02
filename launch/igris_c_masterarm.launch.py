#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    node = Node(
        package="igris_c_masterarm",
        executable="igris_c_masterarm_node",
        name="igris_c_masterarm_node",
        output="screen",
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': ParameterValue(LaunchConfiguration('baud'), value_type=int),
            'domain_id': ParameterValue(LaunchConfiguration('domain_id'), value_type=int),
            'namespace': LaunchConfiguration('namespace'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='MasterArm serial device path, for example /dev/ttyUSB0',
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='1000000',
            description='MasterArm baud rate as a positive integer, for example 1000000 or 2000000',
        ),
        DeclareLaunchArgument(
            'domain_id',
            default_value='0',
            description='CycloneDDS domain id as a non-negative integer',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Optional DDS namespace prefix. Leave empty to use no namespace.',
        ),
        node,
    ])
