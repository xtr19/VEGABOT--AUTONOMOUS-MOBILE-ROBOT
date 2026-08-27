#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image, Imu, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from nav2_simple_commander.robot_navigator import BasicNavigator
import time
from std_msgs.msg import Int32
import threading

class SimpleDockingNode(Node):
    def __init__(self):
        super().__init__("simple_docking_node")
        
        # CV Bridge for camera
        self.bridge = CvBridge()
        self.qr_decoder = cv2.QRCodeDetector()
        
        # Nav2 Navigator
        self.nav = BasicNavigator()
        
        # Publishers and Subscribers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cam_sub = self.create_subscription(Image, "/camera/image_raw", self.camera_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/out', self.imu_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.battery_sub = self.create_subscription(Int32, '/battery_status', self.battery_callback, 10)
        
        # State variables
        self.frame = None
        self.display_frame = None
        self.yaw = 0.0
        self.front_distance = 100.0
        self.qr_center_x = None
        self.qr_detected = False
        self.image_width = 640
        
        # Docking threshold
        self.docking_threshold = 0.15  # 15cm
        
        # Store initial position for return
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0

        #Battery Status
        self.battery = 100
        self.battery_lock = threading.Lock()  # Add lock for thread safety
        self.frame_lock = threading.Lock()  # Lock for frame access
        
        self.get_logger().info("Simple Docking Node Initialized!")

    def battery_callback(self, msg):
        """Get battery status"""
        with self.battery_lock:
            self.battery = msg.data
        self.get_logger().info(f"Battery updated: {self.battery}%")
    
    def get_battery(self):
        """Thread-safe battery getter"""
        with self.battery_lock:
            return self.battery
    
    def camera_callback(self, msg):
        """Detect QR code"""
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.qr_detected = False
        
        if frame is not None:
            data, points, _ = self.qr_decoder.detectAndDecode(frame)
            
            if points is not None:
                points = points[0].astype(int)
                center_x = int(points[:, 0].mean())
                center_y = int(points[:, 1].mean())
                
                self.qr_center_x = center_x
                self.qr_detected = True
                
                # Draw visualization
                cv2.polylines(frame, [points], True, (0, 255, 0), 2)
                cv2.circle(frame, (center_x, center_y), 7, (255, 0, 0), 2)
                cv2.circle(frame, (320, center_y), 5, (255, 255, 255), -1)
                
                if data:
                    cv2.putText(frame, f'QR: {data}', (50, 50), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                self.qr_center_x = None
            
            # Store frame for display in main thread
            with self.frame_lock:
                self.display_frame = frame.copy()
    
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
    
    def update_display(self):
        """Update CV display - call this from main thread"""
        with self.frame_lock:
            if self.display_frame is not None:
                cv2.imshow('QR Detection', self.display_frame)
                cv2.waitKey(1)
    
    def move_distance(self, distance, speed=0.2):
        """Move forward/backward for a distance"""
        self.get_logger().info(f"Moving {distance}m at speed {speed}m/s...")
        
        duration = abs(distance / speed)
        start_time = time.time()
        
        while (time.time() - start_time) < duration:
            self.velocity_publisher(speed if distance > 0 else -speed, 0.0)
            self.update_display()
            time.sleep(0.01)
        
        self.velocity_publisher(0.0, 0.0)
        self.get_logger().info("Movement complete!")
    
    def rotate_to_angle(self, target_angle, tolerance=0.02):
        """Rotate to a target angle (in radians)"""
        self.get_logger().info(f"Rotating to angle: {np.degrees(target_angle):.1f}°...")
        
        while True:
            self.update_display()
            time.sleep(0.01)
            
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
            self.update_display()
            time.sleep(0.01)
            
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
        """Move forward until docked (distance < threshold)"""
        self.get_logger().info("Docking forward...")
        
        while True:
            self.update_display()
            time.sleep(0.01)
            
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
    
    
    def create_pose_stamped(self, x, y, yaw):
        """Create a PoseStamped message"""
        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, yaw)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w
        return pose
    
    def set_initial_pose(self, x, y, yaw):
        """Set initial pose for Nav2"""
        self.get_logger().info(f"Setting initial pose: x={x}, y={y}, yaw={yaw}")
        initial_pose = self.create_pose_stamped(x, y, yaw)
        self.nav.setInitialPose(initial_pose)
        self.nav.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active!")
    
    def go_to_waypoint(self, x, y, yaw):
        """Navigate to a single waypoint"""
        self.get_logger().info(f"Going to waypoint: x={x}, y={y}, yaw={yaw}")
        
        goal_pose = self.create_pose_stamped(x, y, yaw)
        self.nav.goToPose(goal_pose)
        
        while not self.nav.isTaskComplete():
            self.update_display()
            time.sleep(0.1)
        
        result = self.nav.getResult()
        self.get_logger().info(f"Navigation result: {result}")
        self.get_logger().info("Waypoint reached!")
    
    def follow_waypoints(self, waypoints):
        """Follow multiple waypoints"""
        self.get_logger().info(f"Following {len(waypoints)} waypoints...")
        
        waypoint_poses = []
        for wp in waypoints:
            pose = self.create_pose_stamped(wp[0], wp[1], wp[2])
            waypoint_poses.append(pose)
        
        self.nav.followWaypoints(waypoint_poses)
        
        while not self.nav.isTaskComplete():
            current_battery = self.get_battery()
            
            if current_battery <= 5:
                self.get_logger().warn(f"Battery critically low ({current_battery}%)! Canceling navigation...")
                self.nav.cancelTask()
                time.sleep(1.0)  # Wait for cancellation to process
                return False  # Return False to indicate cancellation
            
            self.update_display()
            time.sleep(0.1)
        
        result = self.nav.getResult()
        self.get_logger().info(f"Waypoints result: {result}")
        self.get_logger().info("All waypoints completed!")
        return True  # Return True to indicate success
    
    def run_sequence(self):
        """Run complete undocking, navigation, and docking sequence"""
        
        self.get_logger().info("===== STARTING SEQUENCE =====")
        time.sleep(2)
        
        # Set initial pose (robot starts in dock)
        self.get_logger().info("\n[SETUP] Setting initial pose")
        self.initial_x = 1.5
        self.initial_y = 5.18
        self.initial_yaw = 1.57
        self.set_initial_pose(self.initial_x, self.initial_y, self.initial_yaw)
        time.sleep(2)
        
        # Step 1: Undock - Move backward 0.6 meters
        self.get_logger().info("\n[STEP 1] Undocking - Moving backward 0.2m")
        self.move_distance(-0.6, speed=0.2)
        time.sleep(1)
        
        # Step 2: Rotate 180 degrees
        self.get_logger().info("\n[STEP 2] Rotating 180 degrees")
        initial_yaw = self.yaw
        target_yaw = initial_yaw + np.pi  # Add 180 degrees
        target_yaw = np.arctan2(np.sin(target_yaw), np.cos(target_yaw))  # Normalize
        self.rotate_to_angle(target_yaw)
        time.sleep(1)
        
        # Step 3: Navigate to waypoints
        self.get_logger().info("\n[STEP 3] Navigating to waypoints")
        
        # Define your waypoints here (x, y, yaw)
        waypoints = [
            (5.0, 1.0, 0.0),      # Waypoint 1
            (0.0, 3.0, 0.0),      # Waypoint 2
        ]

        navigation_success = self.follow_waypoints(waypoints)
        
        if not navigation_success:
            self.get_logger().warn("Navigation canceled due to low battery!")
        
        time.sleep(2)
        
        # Step 4: Return to pre-dock position (0.2m away from dock)
        self.get_logger().info("\n[STEP 4] Returning to pre-dock position")
        pre_dock_x = self.initial_x
        pre_dock_y = self.initial_y - 0.5  
        pre_dock_yaw = self.initial_yaw
        
        self.go_to_waypoint(pre_dock_x, pre_dock_y, pre_dock_yaw)
        time.sleep(2)
        
        # Step 5: Detect and align with QR code
        self.get_logger().info("\n[STEP 5] Detecting and aligning with QR code")
        
        # Wait for QR detection
        self.get_logger().info("Waiting for QR detection...")
        while not self.qr_detected:
            self.update_display()
            time.sleep(0.1)
        
        self.get_logger().info("QR detected! Aligning...")
        self.align_with_qr()
        time.sleep(1)
        
        # Step 6: Dock forward until LiDAR < threshold
        self.get_logger().info("\n[STEP 6] Docking forward")
        self.dock_forward()
        
        self.get_logger().info("\n===== SEQUENCE COMPLETE =====")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleDockingNode()
    
    # Create multi-threaded executor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    # Run executor in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        # Run the sequence in the main thread (for CV display)
        node.run_sequence()
        
        # Keep spinning
        while rclpy.ok():
            time.sleep(0.1)
        
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()