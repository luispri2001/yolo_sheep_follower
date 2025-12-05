#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection3DArray
from yolo_msgs.msg import DetectionArray
import numpy as np
import math
import argparse


class YoloObjectFollower(Node):
    """ROS2 node for following a sheep using YOLO detections and cmd_vel commands."""
    
    def __init__(self, target_id="any"):
        """Initialize the YOLO object follower node.
        
        Args:
            target_id: ID of the sheep to follow or "any" for first detected sheep.
        """
        super().__init__("yolo_object_follower")
        
        self.target_id = target_id
        self.current_sheep_id = None
        self.lost_object_timeout = 2.0
        self.last_detection_time = None
        self.found_target = False
        
        self.max_linear_speed = 0.3
        self.max_angular_speed = 0.6
        self.min_follow_distance = 2.5
        self.max_follow_distance = 5.0
        self.center_threshold = 30
        
        self.detection_sub = self.create_subscription(
            DetectionArray,
            "/detections_3d",
            self.detection_callback_yolo,
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.target_position = None
        self.target_bbox = None
        
        self.get_logger().info(f"YOLO Sheep Follower started - Tracking ID: {self.target_id}")

    def detection_callback_yolo(self, msg):
        """Process YOLO detections to find target sheep.
        
        Args:
            msg: YOLO detection message.
        """
        best_sheep = None
        
        for detection in msg.detections:
            # Accept both sheep and cow (YOLO often confuses them)
            if detection.class_name not in ["sheep", "cow"]:
                continue
            
            detection_id = str(detection.id)
            
            if self.target_id == "any":
                if self.current_sheep_id is None or detection_id == self.current_sheep_id:
                    best_sheep = detection
                    if detection_id == self.current_sheep_id:
                        break
                elif best_sheep is None:
                    best_sheep = detection
            elif detection_id == self.target_id:
                best_sheep = detection
                break
        
        if best_sheep:
            self.found_target = True
            self.last_detection_time = self.get_clock().now()
            self.current_sheep_id = str(best_sheep.id)
            
            self.target_position = best_sheep.bbox3d.center.position
            self.target_bbox = best_sheep.bbox
            
            self.get_logger().info(f"Following sheep ID {self.current_sheep_id} at: "
                                   f"x={self.target_position.x:.2f}, "
                                   f"y={self.target_position.y:.2f}")
        elif self.found_target:
            self.get_logger().debug("Sheep not found in current detections")

    def control_loop(self):
        """Main control loop for following behavior."""
        cmd = Twist()
        current_time = self.get_clock().now()
        
        if not self.found_target:
            self.get_logger().info(f"Waiting for sheep ID {self.target_id}...")
            self.publish_stop_command()
            return
            
        if self.last_detection_time is None or \
           (current_time - self.last_detection_time).nanoseconds / 1e9 > self.lost_object_timeout:
            self.get_logger().warn("Sheep lost! Stopping robot.")
            self.publish_stop_command()
            return
        
        if self.target_position is None or self.target_bbox is None:
            self.get_logger().warn("Invalid target data. Stopping robot.")
            self.publish_stop_command()
            return
        
        linear_x = self.calculate_linear_velocity()
        angular_z = self.calculate_angular_velocity()
        
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        
        self.get_logger().info(f"Sending cmd_vel: linear={linear_x:.2f}, angular={angular_z:.2f}")
        
        self.cmd_vel_pub.publish(cmd)
        
    def calculate_linear_velocity(self):
        """Calculate linear velocity based on distance to target.
        
        Returns:
            float: Linear velocity command.
        """
        distance = self.target_position.x
        
        if distance < self.min_follow_distance:
            return -0.2 * self.max_linear_speed
        elif distance > self.max_follow_distance:
            return self.max_linear_speed
        else:
            speed_factor = (distance - self.min_follow_distance) / (self.max_follow_distance - self.min_follow_distance)
            return 0.5 * speed_factor * self.max_linear_speed
    
    def calculate_angular_velocity(self):
        """Calculate angular velocity to center target in view.
        
        Returns:
            float: Angular velocity command.
        """
        lateral_offset = self.target_position.y
        
        image_width = 640
        target_center_x = self.target_bbox.center.position.x
        image_center_x = image_width / 2
        
        pixel_offset = target_center_x - image_center_x
        
        if abs(pixel_offset) < self.center_threshold:
            return 0.0
        
        pixel_factor = pixel_offset / (image_width / 2)
        combined_factor = -0.8 * lateral_offset + 0.2 * pixel_factor
        
        angular_z = math.copysign(min(abs(combined_factor), 1.0) * self.max_angular_speed, combined_factor)
        
        return angular_z
    
    def publish_stop_command(self):
        """Publish zero velocity command to stop the robot."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    """Main function to initialize and run the sheep follower node.
    
    Args:
        args: Command line arguments.
    """
    parser = argparse.ArgumentParser(description="YOLO Sheep Follower")
    parser.add_argument("--id", type=str, default="any", 
                       help="ID of the sheep to follow (or 'any' for first detected)")
    
    rclpy.init(args=args)
    
    parsed_args, remaining_args = parser.parse_known_args()
    
    follower = YoloObjectFollower(target_id=parsed_args.id)
    
    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        pass
    finally:
        follower.publish_stop_command()
        follower.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
