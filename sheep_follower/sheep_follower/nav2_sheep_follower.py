#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.logging import LoggingSeverity
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from yolo_msgs.msg import DetectionArray
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs
import math
import time
import argparse


class Nav2SheepFollower(Node):
    """ROS2 node for following a sheep using Nav2 based on YOLO detections."""
    
    def __init__(self, target_id="3"):
        """Initialize the sheep follower node.
        
        Args:
            target_id (str): ID of the sheep to follow or "sheep" for any sheep.
        """
        super().__init__("nav2_sheep_follower")
        self.get_logger().set_level(LoggingSeverity.DEBUG)
        
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("lost_timeout", 2.0)
        self.declare_parameter("goal_tolerance", 0.5)
        
        self.target_id = target_id
        self.target_frame = self.get_parameter("target_frame").value
        self.lost_timeout = self.get_parameter("lost_timeout").value
        self.goal_tolerance = self.get_parameter("goal_tolerance").value
        self.goal_offset = 1.0
        
        self.get_logger().info(f"Sheep Follower - ID: {self.target_id}, Frame: {self.target_frame}")
        
        self.last_detection_time = time.time()
        self.found_target = False
        self.last_goal = None
        self.current_goal_handle = None
        self.target_position = None
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.detection_sub = self.create_subscription(
            DetectionArray,
            "/yolo/detections_3d",
            self.detection_callback,
            10
        )
        
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.timer = self.create_timer(1.0, self.check_sheep_timeout)
        
        self.get_logger().info("Nav2 Sheep Follower initialized")

    def detection_callback(self, msg):
        """Process YOLO detections to find target sheep.
        
        Args:
            msg (DetectionArray): YOLO detection message.
        """
        sheep_found = False
        
        for detection in msg.detections:
            if str(detection.id) == self.target_id or (
                detection.class_name == "sheep" and self.target_id == "sheep"):
                
                sheep_found = True
                self.found_target = True
                self.last_detection_time = time.time()
                
                self.target_position = detection.bbox3d.center.position
                
                self.get_logger().debug(f"Sheep {self.target_id} detected at: "
                                       f"x={self.target_position.x:.2f}, "
                                       f"y={self.target_position.y:.2f}, "
                                       f"z={self.target_position.z:.2f}")
                
                self.follow_sheep()
                return
        
        if not sheep_found and self.found_target:
            self.get_logger().debug(f"Sheep {self.target_id} not found in current detections")

    def follow_sheep(self):
        """Calculate and send navigation goal based on sheep position."""
        if not self.target_position:
            return
            
        try:
            sheep_pose = PoseStamped()
            sheep_pose.header.frame_id = "base_link"
            sheep_pose.header.stamp = Time().to_msg()
            sheep_pose.pose.position = self.target_position
            sheep_pose.pose.orientation.w = 1.0
            
            sheep_pose_map = self.tf_buffer.transform(
                sheep_pose, 
                self.target_frame, 
                timeout=Duration(seconds=1.0)
            )
            
            robot_tf = self.tf_buffer.lookup_transform(
                self.target_frame, 
                "base_link", 
                Time(), 
                timeout=Duration(seconds=1.0)
            )
            
            sheep_x = sheep_pose_map.pose.position.x
            sheep_y = sheep_pose_map.pose.position.y
            robot_x = robot_tf.transform.translation.x
            robot_y = robot_tf.transform.translation.y
            
        except Exception as e:
            self.get_logger().warn(f"TF Error: {e}")
            return
            
        dx = sheep_x - robot_x
        dy = sheep_y - robot_y
        angle = math.atan2(dy, dx)
        
        goal_x = sheep_x - self.goal_offset * math.cos(angle)
        goal_y = sheep_y - self.goal_offset * math.sin(angle)
        
        if self.last_goal:
            goal_distance = math.sqrt((goal_x - self.last_goal[0])**2 + 
                                    (goal_y - self.last_goal[1])**2)
            if goal_distance < self.goal_tolerance:
                self.get_logger().debug("New goal too similar to previous. Not sending.")
                return
                
        self.send_navigation_goal(goal_x, goal_y, angle)
        self.last_goal = (goal_x, goal_y)

    def send_navigation_goal(self, x, y, angle):
        """Send navigation goal to Nav2.
        
        Args:
            x (float): Goal x coordinate in target frame.
            y (float): Goal y coordinate in target frame.
            angle (float): Goal orientation angle.
        """
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.target_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.z = math.sin(angle / 2.0)
        goal_pose.pose.orientation.w = math.cos(angle / 2.0)
        
        self.get_logger().info(f"Sending goal: x={x:.2f}, y={y:.2f}")
        
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Cannot connect to navigate_to_pose server")
            return
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response from Nav2.
        
        Args:
            future: Goal response future.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2")
            self.current_goal_handle = None
            return
            
        self.get_logger().debug("Goal accepted. Waiting for result...")
        self.current_goal_handle = goal_handle
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Handle goal result from Nav2.
        
        Args:
            future: Goal result future.
        """
        result = future.result()
        if result.status == 4:
            self.get_logger().info("Goal reached successfully!")
        else:
            self.get_logger().warn(f"Goal failed with status: {result.status}")
        self.current_goal_handle = None

    def check_sheep_timeout(self):
        """Check if sheep has been lost for too long."""
        if not self.found_target:
            self.get_logger().debug(f"Waiting for sheep ID {self.target_id}...")
            return
            
        elapsed = time.time() - self.last_detection_time
        if elapsed > self.lost_timeout:
            self.get_logger().warn(f"Sheep lost for {elapsed:.1f}s. Continuing to last goal.")
            self.last_goal = None
            self.found_target = False


def main(args=None):
    """Main function to initialize and run the sheep follower node.
    
    Args:
        args: Command line arguments.
    """
    parser = argparse.ArgumentParser(description="Nav2 Sheep Follower")
    parser.add_argument("--id", type=str, default="3",
                       help="ID of the sheep to follow (or 'sheep' to follow any sheep)")

    rclpy.init(args=args)
    
    parsed_args, _ = parser.parse_known_args()
    
    follower = Nav2SheepFollower(target_id=parsed_args.id)
    
    follower.set_parameters([
        rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
    ])
    
    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        pass
    finally:
        follower.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
