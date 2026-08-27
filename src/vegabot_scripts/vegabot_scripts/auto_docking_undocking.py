#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Imu, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf_transformations import euler_from_quaternion
import time

class SimpleDockingNode(Node):
    def __init__(self):
        super().__init__("simple_docking_node")
        
        # CV Bridge for camera
        self.bridge = CvBridge()
        self.qr_decoder = cv2.QRCodeDetector()
        
        # Publishers and Subscribers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # self.cam_sub = self.create_subscription(Image, "/camera/image_raw", self.camera_callback, 10) #SIM
        self.cam_sub = self.create_subscription(Image, "/image_raw", self.camera_callback, 10) #REAL
        self.imu_sub = self.create_subscription(Imu, '/imu/out', self.imu_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        
        # State variables
        self.frame = None
        self.yaw = 0.0
        self.front_distance = 100.0
        self.qr_center_x = None
        self.qr_detected = False
        self.image_width = 640
        
        # Docking threshold
        self.docking_threshold = 0.15
        
        self.get_logger().info("Simple Docking Node Initialized!")
    
    def camera_callback(self, msg):
        """Detect QR code"""
        self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.qr_detected = False
        
        if self.frame is not None:
            data, points, _ = self.qr_decoder.detectAndDecode(self.frame)
            
            if points is not None:
                points = points[0].astype(int)
                center_x = int(points[:, 0].mean())
                center_y = int(points[:, 1].mean())
                
                self.qr_center_x = center_x
                self.qr_detected = True
                
                # Draw visualization
                cv2.polylines(self.frame, [points], True, (0, 255, 0), 2)
                cv2.circle(self.frame, (center_x, center_y), 7, (255, 0, 0), 2) #frame circle
                cv2.circle(self.frame, (320, center_y), 5, (255, 255, 255), -1) #qr circle
                
                if data:
                    cv2.putText(self.frame, f'QR: {data}', (50, 50), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                self.qr_center_x = None
                
            cv2.imshow('QR Detection', self.frame)
            cv2.waitKey(1)
    
    def imu_callback(self, msg):
        """Get current yaw"""
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w
        
        roll, pitch, self.yaw = euler_from_quaternion([x, y, z, w])
    
    def lidar_callback(self, msg):
        """Get front distance"""
        ranges = np.array(msg.ranges)
        # Front: combine 358-360 and 0-2 degrees
        front = np.concatenate([ranges[-2:], ranges[:2]])
        front = front[~np.isnan(front)]  # Remove NaN
        if len(front) > 0:
            self.front_distance = np.mean(front)
    
    def velocity_publisher(self, x, z):
        """Publish velocity"""
        vel = Twist()
        vel.linear.x = x
        vel.angular.z = z
        self.vel_pub.publish(vel)
    
    def move_distance(self, distance, speed=0.2):
        """Move forward/backward for a distance"""
        self.get_logger().info(f"Moving {distance}m at speed {speed}m/s...")
        
        duration = abs(distance / speed)
        start_time = time.time()
        
        while (time.time() - start_time) < duration:
            self.velocity_publisher(speed if distance > 0 else -speed, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        self.velocity_publisher(0.0, 0.0)
        self.get_logger().info("Movement complete!")
    
    def rotate_to_angle(self, target_angle, tolerance=0.02):
        """Rotate to a target angle (in radians)"""
        self.get_logger().info(f"Rotating to angle: {np.degrees(target_angle):.1f}°...")
        
        while True:
            rclpy.spin_once(self, timeout_sec=0.01)
            
            # Calculate angle difference
            angle_diff = target_angle - self.yaw
            
            # Normalize to -pi to pi
            angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
            
            if abs(angle_diff) < tolerance:
                break
            
            # Proportional control
            angular_speed = np.clip(angle_diff * 0.8, -0.6, 0.6)
            self.velocity_publisher(0.0, angular_speed)
            
            self.get_logger().info(f"Current: {np.degrees(self.yaw):.1f}°, Target: {np.degrees(target_angle):.1f}°")
        
        self.velocity_publisher(0.0, 0.0)
        self.get_logger().info("Rotation complete!")
    
    def align_with_qr(self):
        """Align with QR code"""
        self.get_logger().info("Aligning with QR code...")
        
        while True:
            rclpy.spin_once(self, timeout_sec=0.01)
            
            if not self.qr_detected or self.qr_center_x is None:
                self.get_logger().warn("QR not detected! Waiting...")
                time.sleep(0.1)
                continue
            
            # Calculate error from center
            error = self.qr_center_x - (self.image_width / 2)
            
            # Check if aligned (within 5 pixels)
            if abs(error) < 5:
                break
            
            # Proportional control
            angular_speed = -error * 0.003
            self.velocity_publisher(0.0, angular_speed)
            
            self.get_logger().info(f"Aligning... Error: {error:.0f} pixels")
        
        self.velocity_publisher(0.0, 0.0)
        self.get_logger().info("Aligned with QR!")
    
    def dock_forward(self):
        """Move forward until docked (distance < 0.15m)"""
        self.get_logger().info("Docking forward...")
        
        while True:
            rclpy.spin_once(self, timeout_sec=0.01)
            
            self.get_logger().info(f"Distance: {self.front_distance:.3f}m")
            
            # Stop if docked
            if self.front_distance < self.docking_threshold:
                break
            
            # Move forward slowly with alignment correction
            if self.qr_detected and self.qr_center_x is not None:
                error = self.qr_center_x - (self.image_width / 2)
                angular_correction = -error * 0.002
            else:
                angular_correction = 0.0
            
            self.velocity_publisher(0.1, angular_correction)
        
        self.velocity_publisher(0.0, 0.0)
        self.get_logger().info("DOCKED SUCCESSFULLY!")
    
    def run_sequence(self):
        """Run complete undocking and docking sequence"""
        
        self.get_logger().info("===== STARTING SEQUENCE =====")
        time.sleep(2)
        
        # Step 1: Undock - Move backward 0.2 meters
        self.get_logger().info("\n[STEP 1] Undocking - Moving backward 0.2m")
        self.move_distance(-0.2, speed=0.2)
        time.sleep(1)
        
        # Step 2: Rotate 180 degrees
        self.get_logger().info("\n[STEP 2] Rotating 180 degrees")
        initial_yaw = self.yaw
        target_yaw = initial_yaw + np.pi  # Add 180 degrees
        target_yaw = np.arctan2(np.sin(target_yaw), np.cos(target_yaw))  # Normalize
        self.rotate_to_angle(target_yaw)
        time.sleep(1)
        
        # Step 3: Rotate back to 0 degrees (reference)
        self.get_logger().info("\n[STEP 3] Rotating back to 0 degrees (reference)")
        self.rotate_to_angle(0.0)
        time.sleep(1)
        
        # Step 4: Detect and align with QR code
        self.get_logger().info("\n[STEP 4] Detecting and aligning with QR code")
        
        # Wait for QR detection
        self.get_logger().info("Waiting for QR detection...")
        while not self.qr_detected:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        self.get_logger().info("QR detected! Aligning...")
        self.align_with_qr()
        time.sleep(1)
        
        # Step 5: Dock forward until LiDAR < 0.15m
        self.get_logger().info("\n[STEP 5] Docking forward until distance < 0.15m")
        self.dock_forward()
        
        self.get_logger().info("\n===== SEQUENCE COMPLETE =====")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleDockingNode()
    
    try:
        # Run the sequence
        node.run_sequence()
        
        # Keep node alive
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()