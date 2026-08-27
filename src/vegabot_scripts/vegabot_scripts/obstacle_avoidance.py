#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        # Distance variables
        self.front = 10.0
        self.left = 10.0
        self.right = 10.0
        
        # Subscribe and publish
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.1, self.move)

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        
        # Get distances
        self.front = np.concatenate([ranges[-20:], ranges[:20]]).mean()
        self.left = ranges[30:60].mean()
        self.right = ranges[300:330].mean()

    def move(self):
        vel = Twist()
        
        # If obstacle in front
        if self.front < 0.5:
            # Backup and Turn
            vel.linear.x = -0.25
            vel.angular.z = 1.5
            
            # Turn to side with more space
            if self.left > self.right:
                vel.angular.z = 0.6  # Turn left
            else:
                vel.angular.z = -0.6  # Turn right
        
        # If no obstacle
        else:
            # Go forward
            vel.linear.x = 0.2
            vel.angular.z = 0.0
        
        self.vel_pub.publish(vel)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()