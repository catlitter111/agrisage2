#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双目相机显示节点
订阅图像和距离信息，并显示在窗口中
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class StereoCameraViewer(Node):
    def __init__(self):
        super().__init__('stereo_camera_viewer')
        
        # 声明参数
        self.declare_parameter('show_original', True)
        self.declare_parameter('show_rectified', True)
        self.declare_parameter('show_disparity', True)
        
        # 获取参数
        self.show_original = self.get_parameter('show_original').value
        self.show_rectified = self.get_parameter('show_rectified').value
        self.show_disparity = self.get_parameter('show_disparity').value
        
        # 创建CvBridge
        self.bridge = CvBridge()
        
        # 存储最新的距离信息
        self.current_distance = None
        
        # 创建订阅者
        self.left_image_sub = self.create_subscription(
            Image,
            'stereo_camera/left_image',
            self.left_image_callback,
            10)
        
        self.right_image_sub = self.create_subscription(
            Image,
            'stereo_camera/right_image',
            self.right_image_callback,
            10)
        
        self.left_rect_sub = self.create_subscription(
            Image,
            'stereo_camera/left_rectified',
            self.left_rect_callback,
            10)
        
        self.right_rect_sub = self.create_subscription(
            Image,
            'stereo_camera/right_rectified',
            self.right_rect_callback,
            10)
        
        self.disparity_sub = self.create_subscription(
            Image,
            'stereo_camera/disparity',
            self.disparity_callback,
            10)
        
        self.distance_sub = self.create_subscription(
            Float32,
            'stereo_camera/center_distance',
            self.distance_callback,
            10)
        
        # 初始化图像变量
        self.left_image = None
        self.right_image = None
        self.left_rect = None
        self.right_rect = None
        self.disparity = None
        
        self.get_logger().info('双目相机显示节点已启动')
    
    def left_image_callback(self, msg):
        """处理左相机原始图像"""
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.show_images()
        except CvBridgeError as e:
            self.get_logger().error(f'左相机图像转换错误: {str(e)}')
    
    def right_image_callback(self, msg):
        """处理右相机原始图像"""
        try:
            self.right_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.show_images()
        except CvBridgeError as e:
            self.get_logger().error(f'右相机图像转换错误: {str(e)}')
    
    def left_rect_callback(self, msg):
        """处理左相机校正图像"""
        try:
            self.left_rect = self.bridge.imgmsg_to_cv2(msg, "mono8")
            # 转换为彩色图用于显示
            self.left_rect = cv2.cvtColor(self.left_rect, cv2.COLOR_GRAY2BGR)
            self.show_images()
        except CvBridgeError as e:
            self.get_logger().error(f'左相机校正图像转换错误: {str(e)}')
    
    def right_rect_callback(self, msg):
        """处理右相机校正图像"""
        try:
            self.right_rect = self.bridge.imgmsg_to_cv2(msg, "mono8")
            # 转换为彩色图用于显示
            self.right_rect = cv2.cvtColor(self.right_rect, cv2.COLOR_GRAY2BGR)
            self.show_images()
        except CvBridgeError as e:
            self.get_logger().error(f'右相机校正图像转换错误: {str(e)}')
    
    def disparity_callback(self, msg):
        """处理视差图"""
        try:
            self.disparity = self.bridge.imgmsg_to_cv2(msg, "mono8")
            # 转换为伪彩色图用于显示
            self.disparity_color = cv2.applyColorMap(self.disparity, cv2.COLORMAP_JET)
            self.show_images()
        except CvBridgeError as e:
            self.get_logger().error(f'视差图转换错误: {str(e)}')
    
    def distance_callback(self, msg):
        """处理距离信息"""
        self.current_distance = msg.data
        self.get_logger().info(f'接收到距离信息: {self.current_distance:.2f}m')
    
    def show_images(self):
        """显示所有图像"""
        # 显示原始图像
        if self.show_original and self.left_image is not None and self.right_image is not None:
            original_combined = np.hstack((self.left_image, self.right_image))
            cv2.putText(original_combined, "原始左右图像", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("原始双目图像", original_combined)
        
        # 显示校正后的图像
        if self.show_rectified and self.left_rect is not None and self.right_rect is not None:
            rectified_combined = np.hstack((self.left_rect, self.right_rect))
            cv2.putText(rectified_combined, "校正后左右图像", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("校正后双目图像", rectified_combined)
        
        # 显示视差图
        if self.show_disparity and hasattr(self, 'disparity_color') and self.disparity_color is not None:
            # 添加距离信息到视差图
            disparity_with_info = self.disparity_color.copy()
            if self.current_distance is not None:
                cv2.putText(disparity_with_info, f"距离: {self.current_distance:.2f}m", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("视差图", disparity_with_info)
        
        cv2.waitKey(1)
    
    def destroy_node(self):
        """关闭节点时释放资源"""
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = StereoCameraViewer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()