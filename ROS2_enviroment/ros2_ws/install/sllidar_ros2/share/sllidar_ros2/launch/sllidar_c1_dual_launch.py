#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    rviz_config_dir = os.path.join(
            get_package_share_directory('sllidar_ros2'),
            'rviz',
            'sllidar_ros2.rviz')
    
    front_lidar = Node(
        package = 'sllidar_ros2',
        executable = 'sllidar_node',
        name = 'sllidar_node',
        parameters = [{
            'channel_type':'serial',
            'serial_port': '/dev/ttyRPLIDAR_front', 
            'serial_baudrate': 460800, 
            'frame_id': 'laser_front',
            'inverted': False, 
            'angle_compensate': True, 
            'scan_mode': 'Standard'}],
        remappings = [
            ('scan','scan_front')
        ],
        output = 'screen'
    )

    back_lidar = Node(
        package = 'sllidar_ros2',
        executable = 'sllidar_node',
        name = 'sllidar_node',
        parameters = [{
            'channel_type':'serial',
            'serial_port': '/dev/ttyRPLIDAR_back', 
            'serial_baudrate': 460800, 
            'frame_id': 'laser_back',
            'inverted': False, 
            'angle_compensate': True, 
            'scan_mode': 'Standard'}],
        remappings = [
            ('scan','scan_back')
        ],
        output = 'screen'
    )

    Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

    return LaunchDescription([
        front_lidar,
        back_lidar
    ])


