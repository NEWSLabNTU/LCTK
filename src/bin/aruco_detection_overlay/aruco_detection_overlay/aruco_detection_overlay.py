#!/usr/bin/env python3

"""
ArUco detection overlay node for RViz visualization.
Subscribes to camera images and Detection2DArray messages,
draws bounding boxes on the images, and publishes annotated images.
"""

import threading
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray


class DetectionOverlayNode(Node):
    def __init__(self):
        super().__init__("aruco_detection_overlay")

        # Create CV bridge
        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter("image_topic", "/sensing/camera/front_center/image_raw")
        self.declare_parameter(
            "detection_topic", "/calibration/aruco_locator/aruco_detections"
        )
        self.declare_parameter(
            "output_topic", "/calibration/visualization/image_with_detections"
        )
        self.declare_parameter("cache_size", 10)

        image_topic = self.get_parameter("image_topic").value
        detection_topic = self.get_parameter("detection_topic").value
        output_topic = self.get_parameter("output_topic").value
        cache_size = self.get_parameter("cache_size").value

        # Publisher for annotated images
        self.image_pub = self.create_publisher(Image, output_topic, 10)

        # Cache for recent detections
        self.detection_cache = deque(maxlen=cache_size)
        self.cache_lock = threading.Lock()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )
        self.detection_sub = self.create_subscription(
            Detection2DArray, detection_topic, self.detection_callback, 10
        )

        self.get_logger().info(f"ArUco detection overlay node started")
        self.get_logger().info(f"Subscribing to image: {image_topic}")
        self.get_logger().info(f"Subscribing to detections: {detection_topic}")
        self.get_logger().info(f"Publishing annotated images to: {output_topic}")

        # Color map for different marker IDs
        self.colors = {
            "696": (255, 0, 0),  # Red for ID 696
            "64": (0, 255, 0),  # Green for ID 64
            "306": (0, 0, 255),  # Blue for ID 306
            "195": (255, 255, 0),  # Cyan for ID 195
        }
        self.default_color = (255, 0, 255)  # Magenta for unknown IDs

        # Stats
        self.image_count = 0
        self.detection_count = 0

    def detection_callback(self, msg):
        """Cache detection messages"""
        with self.cache_lock:
            self.detection_cache.append(msg)
            self.detection_count += 1
            if self.detection_count % 10 == 0:
                self.get_logger().debug(
                    f"Received {self.detection_count} detection messages"
                )

    def find_matching_detection(self, image_stamp):
        """Find detection message closest in time to the image"""
        with self.cache_lock:
            if not self.detection_cache:
                return None

            # Convert stamps to nanoseconds for comparison
            image_time = image_stamp.sec * 1e9 + image_stamp.nanosec

            best_match = None
            min_diff = float("inf")

            for detection in self.detection_cache:
                detection_time = (
                    detection.header.stamp.sec * 1e9 + detection.header.stamp.nanosec
                )
                diff = abs(image_time - detection_time)

                # Accept detections within 100ms (100 million nanoseconds)
                if diff < 1e8 and diff < min_diff:
                    min_diff = diff
                    best_match = detection

            return best_match

    def image_callback(self, image_msg):
        """Process image and overlay detections"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")

            # Find matching detection
            detection_msg = self.find_matching_detection(image_msg.header.stamp)

            if detection_msg and detection_msg.detections:
                # Draw detections
                for i, detection in enumerate(detection_msg.detections):
                    # Get bounding box
                    bbox = detection.bbox
                    center_x = int(bbox.center.position.x)
                    center_y = int(bbox.center.position.y)
                    width = int(bbox.size_x)
                    height = int(bbox.size_y)

                    # Calculate corners
                    x1 = int(center_x - width / 2)
                    y1 = int(center_y - height / 2)
                    x2 = int(center_x + width / 2)
                    y2 = int(center_y + height / 2)

                    # Get marker ID if available
                    marker_id = "Unknown"
                    if detection.results:
                        marker_id = detection.results[0].hypothesis.class_id

                    # Choose color based on marker ID
                    color = self.colors.get(marker_id, self.default_color)

                    # Draw bounding box
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 3)

                    # Draw marker ID label with background
                    label = f"ArUco {marker_id}"
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 0.8
                    thickness = 2
                    (text_width, text_height), baseline = cv2.getTextSize(
                        label, font, font_scale, thickness
                    )

                    # Position label above the box if possible
                    label_y = (
                        y1 - 10 if y1 - 10 > text_height else y2 + text_height + 10
                    )

                    # Draw label background
                    cv2.rectangle(
                        cv_image,
                        (x1, label_y - text_height - 5),
                        (x1 + text_width + 10, label_y + 5),
                        color,
                        -1,
                    )

                    # Draw label text
                    cv2.putText(
                        cv_image,
                        label,
                        (x1 + 5, label_y),
                        font,
                        font_scale,
                        (255, 255, 255),
                        thickness,
                    )

                    # Draw corner points
                    corner_radius = 6
                    cv2.circle(cv_image, (x1, y1), corner_radius, color, -1)
                    cv2.circle(cv_image, (x2, y1), corner_radius, color, -1)
                    cv2.circle(cv_image, (x1, y2), corner_radius, color, -1)
                    cv2.circle(cv_image, (x2, y2), corner_radius, color, -1)

                    # Draw center point
                    cv2.circle(cv_image, (center_x, center_y), 4, (255, 255, 255), -1)
                    cv2.circle(cv_image, (center_x, center_y), 6, color, 2)

                # Add detection count overlay
                detection_text = f"ArUco Markers: {len(detection_msg.detections)}"
                cv2.putText(
                    cv_image,
                    detection_text,
                    (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.2,
                    (0, 255, 0),
                    3,
                )
            else:
                # No detections overlay
                cv2.putText(
                    cv_image,
                    "No ArUco markers detected",
                    (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.2,
                    (0, 0, 255),
                    3,
                )

            # Convert back to ROS image and publish
            output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            output_msg.header = image_msg.header
            self.image_pub.publish(output_msg)

            self.image_count += 1
            if self.image_count % 30 == 0:
                self.get_logger().debug(
                    f"Published {self.image_count} annotated images"
                )

        except Exception as e:
            self.get_logger().error(f"Error in image callback: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = DetectionOverlayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down detection overlay node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
