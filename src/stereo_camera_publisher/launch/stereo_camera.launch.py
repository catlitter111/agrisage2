#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双目相机启动文件，用于同时启动发布节点和显示节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 声明启动参数
    camera_id_arg = DeclareLaunchArgument(
        'camera_id', default_value='1',
        description='相机ID'
    )
    
    image_width_arg = DeclareLaunchArgument(
        'image_width', default_value='1280',
        description='图像宽度'
    )
    
    image_height_arg = DeclareLaunchArgument(
        'image_height', default_value='480',
        description='图像高度'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate', default_value='10.0',
        description='发布频率（Hz）'
    )
    
    camera_params_file_arg = DeclareLaunchArgument(
        'camera_params_file', default_value='',
        description='相机参数文件路径（可选）'
    )
    
    # 创建发布节点
    publisher_node = Node(
        package='stereo_camera_publisher',
        executable='publisher_node',
        name='stereo_camera_publisher',
        parameters=[{
            'camera_id': LaunchConfiguration('camera_id'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'camera_params_file': LaunchConfiguration('camera_params_file')
        }],
        output='screen'
    )
    
    # 创建显示节点
    viewer_node = Node(
        package='stereo_camera_viewer',
        executable='viewer_node',
        name='stereo_camera_viewer',
        parameters=[{
            'show_original': True,
            'show_rectified': True,
            'show_disparity': True
        }],
        output='screen'
    )
    
    # 返回启动描述
    return LaunchDescription([
        camera_id_arg,
        image_width_arg,
        image_height_arg,
        publish_rate_arg,
        camera_params_file_arg,
        publisher_node,
        viewer_node
    ])