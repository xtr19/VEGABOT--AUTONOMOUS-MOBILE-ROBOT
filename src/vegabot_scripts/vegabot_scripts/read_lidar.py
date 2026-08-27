#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarReader(Node):
    def __init__(self):
        super().__init__('lidar_reader')
        self.subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
    
    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        
        # Front: combine 340-360 and 0-20 (0°)
        # front = np.concatenate([ranges[-20:], ranges[:20]]).mean()
        front = np.concatenate([ranges[-1:], ranges[:1]]).mean()
        
        # Left: 70-110 (90°)
        # left = ranges[70:110].mean()

        # Back: 160-200 (180°)
        # back = ranges[160:200].mean()
        
        # Right: 250-290 (270°)
        # right = ranges[250:290].mean()
        
        # print(f"Front: {front:.2f}m | Left: {left:.2f}m | Right: {right:.2f}m | Back: {back:.2f}m")
        print(front)

def main(args=None):
    rclpy.init(args=args)
    node = LidarReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()