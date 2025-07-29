#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray
from std_msgs.msg import Float32
import math

class WolfDistanceDetector(Node):
    """ROS2 node that detects wolves and publishes their distance."""

    def __init__(self):
        super().__init__("wolf_distance_detector")
        self.declare_parameter("wolf_id", "wolf")
        self.declare_parameter("distance_topic", "/wolf/distance")
        self.declare_parameter("detection_topic", "/yolo/detections_3d")

        self.wolf_id = self.get_parameter("wolf_id").value
        self.distance_pub = self.create_publisher(Float32, self.get_parameter("distance_topic").value, 10)
        self.detection_sub = self.create_subscription(
            DetectionArray,
            self.get_parameter("detection_topic").value,
            self.detection_callback,
            10
        )
        self.get_logger().info("Wolf Distance Detector initialized.")

    def detection_callback(self, msg):
        for detection in msg.detections:
            if detection.class_name == "wolf":
                pos = detection.bbox3d.center.position
                distance = math.sqrt(pos.x**2 + pos.y**2 + pos.z**2)
                self.get_logger().info(f"Wolf detected at distance: {distance:.2f} m")
                msg_out = Float32()
                msg_out.data = distance
                self.distance_pub.publish(msg_out)
                return  # Only report the first wolf detected

def main(args=None):
    rclpy.init(args=args)
    node = WolfDistanceDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()