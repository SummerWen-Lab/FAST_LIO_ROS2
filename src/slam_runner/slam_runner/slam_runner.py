#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
import time

class SlamRunner(Node):
    def __init__(self):
        super().__init__('slam_runner')
        
        # 创建发布器 - 直接使用 FAST-LIO 的输入话题
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        self.imu_pub = self.create_publisher(Imu, '/livox/imu', qos_profile)
        self.cloud_pub = self.create_publisher(PointCloud2, '/livox/lidar', qos_profile)
        
        self.get_logger().info("SlamRunner is ready. Waiting for bag to be played...")
        
        # 订阅数据包发布的话题
        self.imu_sub = self.create_subscription(
            Imu,
            '/livox/imu',  # 数据包中实际的IMU话题
            self.imu_callback,
            qos_profile
        )
        
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/livox/lidar',  # 数据包中实际的点云话题
            self.cloud_callback,
            qos_profile
        )
        
        # 可选：添加简单的数据处理
        self.declare_parameter('enable_downsampling', False)
        self.enable_downsampling = self.get_parameter('enable_downsampling').value
        self.get_logger().info(f"Downsampling enabled: {self.enable_downsampling}")
    
    def imu_callback(self, msg):
        """处理IMU数据回调"""
        # 可以在这里添加IMU数据处理（如滤波等）
        self.imu_pub.publish(msg)
    
    def cloud_callback(self, msg):
        """处理点云数据回调"""
        # 简单的降采样处理（可选）
        if self.enable_downsampling:
            # 这里可以添加实际的降采样算法
            # 为简化，我们只模拟处理
            msg.width = int(msg.width / 2)  # 模拟降采样
            self.get_logger().debug(f"Downsampled cloud: original width {msg.width * 2} -> {msg.width}", 
                                    throttle_duration_sec=5)
        
        self.cloud_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SlamRunner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("SlamRunner stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()