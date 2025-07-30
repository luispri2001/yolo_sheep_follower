#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
import math
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class WolfDistanceDetector(Node):
    """ROS2 node that detects wolves and publica su distancia y visualización."""

    def __init__(self):
        super().__init__("wolf_distance_detector")
        self.declare_parameter("wolf_id", "wolf")
        self.declare_parameter("distance_topic", "/wolf/distance")
        self.declare_parameter("detection_topic", "/yolo/detections_3d")
        self.declare_parameter("visualization_topic", "/wolf/visualization")

        self.wolf_id = self.get_parameter("wolf_id").value
        self.distance_pub = self.create_publisher(Float32, self.get_parameter("distance_topic").value, 10)
        self.detection_sub = self.create_subscription(
            DetectionArray,
            self.get_parameter("detection_topic").value,
            self.detection_callback,
            10
        )
        self.vis_pub = self.create_publisher(MarkerArray, self.get_parameter("visualization_topic").value, 10)
        self.bridge = CvBridge()
        self.last_image = None
        self.image_sub = self.create_subscription(
            Image,
            "/leo/realsense_d455/image",
            self.image_callback,
            10
        )
        self.image_pub = self.create_publisher(Image, "/wolf/image_detection", 10)
        self.get_logger().info("Wolf Distance Detector initialized.")

    def image_callback(self, msg):
        try:
            self.last_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_image_header = msg.header
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def detection_callback(self, msg):
        marker_array = MarkerArray()
        found = False

        for i, detection in enumerate(msg.detections):
            if detection.class_name == self.wolf_id:
                found = True
                pos = detection.bbox3d.center.position
                size = detection.bbox3d.size
                distance = math.sqrt(pos.x**2 + pos.y**2 + pos.z**2)
                self.get_logger().info(f"Wolf detected at distance: {distance:.2f} m")
                msg_out = Float32()
                msg_out.data = distance
                self.distance_pub.publish(msg_out)

                # Bounding box marker (RViz)
                bbox_marker = Marker()
                bbox_marker.header = msg.header
                bbox_marker.ns = "wolf_bbox"
                bbox_marker.id = i
                bbox_marker.type = Marker.CUBE
                bbox_marker.action = Marker.ADD
                bbox_marker.pose.position = pos
                bbox_marker.pose.orientation = detection.bbox3d.center.orientation
                bbox_marker.scale.x = size.x
                bbox_marker.scale.y = size.y
                bbox_marker.scale.z = size.z
                bbox_marker.color.r = 1.0
                bbox_marker.color.g = 0.0
                bbox_marker.color.b = 0.0
                bbox_marker.color.a = 0.5
                bbox_marker.lifetime.sec = 1
                marker_array.markers.append(bbox_marker)

                # Text marker (RViz)
                text_marker = Marker()
                text_marker.header = msg.header
                text_marker.ns = "wolf_text"
                text_marker.id = 1000 + i
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = pos.x
                text_marker.pose.position.y = pos.y
                text_marker.pose.position.z = pos.z + size.z / 2 + 0.2
                text_marker.scale.z = 0.3
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                text_marker.text = f"Wolf\n{distance:.2f} m"
                text_marker.lifetime.sec = 1
                marker_array.markers.append(text_marker)

                # --- Imagen con bounding box 2D y distancia ---
                if self.last_image is not None and hasattr(detection, "bbox"):
                    img = self.last_image.copy()
                    # Extrae centro y tamaño de la bounding box 2D
                    cx = detection.bbox.center.position.x
                    cy = detection.bbox.center.position.y
                    w = detection.bbox.size.x
                    h = detection.bbox.size.y
                    # Calcula esquinas
                    x1 = int(cx - w / 2)
                    y1 = int(cy - h / 2)
                    x2 = int(cx + w / 2)
                    y2 = int(cy + h / 2)
                    # Dibuja la bounding box
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    # Escribe la distancia
                    label = f"Wolf: {distance:.2f} m"
                    cv2.putText(img, label, (x1, max(y1-10, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
                    # Publica la imagen anotada
                    img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
                    img_msg.header = self.last_image_header
                    self.image_pub.publish(img_msg)


        if found:
            self.vis_pub.publish(marker_array)
 

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