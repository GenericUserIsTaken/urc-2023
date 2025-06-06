import math
import sys
from typing import Optional

import cv2  # pylint: disable=no-member
import numpy as np
import rclpy
from cv2 import aruco
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32MultiArray

# Install: pip install "ultralytics>=8.1.0" "torch>=1.8"
from ultralytics import YOLO


class SensorProcessingNode(Node):
    """
    A ROS 2 node for processing sensor data with:
      - ArUco marker detection
      - Depth-based obstacle ignoring wheels/ground
      - LIDAR min-distance
      - Custom object detection for "Hammer" + "Bottle" using YOLO World
    """

    def __init__(self) -> None:
        super().__init__("sensor_processing_node")

        self.get_logger().info("Initializing sensor_processing_node with YOLO World detection...")

        self.bridge = CvBridge()
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None

        # ----------------------------------------------------------------------
        # Load a YOLO World model for custom object detection
        self.model = YOLO("yolov8l-world.pt")  # Use YOLO World model
        self.model.set_classes(["hammer", "bottle"])  # Define custom objects

        # ----------------------------------------------------------------------
        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, "/zed/zed_node/rgb/image_rect_color", self.arucoMarkerDetection, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/zed/zed_node/rgb/camera_info", self.processCameraInfo, 10
        )

        self.depth_sub = self.create_subscription(
            Image, "/zed/zed_node/depth/depth_registered", self.depthCallBack, 10
        )

        # # Object Detection with YOLO World
        # self.yolo_sub = self.create_subscription(
        #     Image, "/zed/zed_node/rgb/image_rect_color", self.yoloDetectionCallback, 10
        # )

        self.get_logger().info("sensor_processing_node is up and running with YOLO World.")

    # --------------------------------------------------------------------------
    #   processCameraInfo
    # --------------------------------------------------------------------------
    def processCameraInfo(self, msg: CameraInfo) -> None:
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    # --------------------------------------------------------------------------
    #   YOLO World Object Detection
    # --------------------------------------------------------------------------
    # def yoloDetectionCallback(self, msg: Image) -> None:
    #     """
    #     Runs YOLO World detection on the camera feed.
    #     """
    #     try:
    #         frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    #     except Exception as e:
    #         self.get_logger().error(f"Failed to convert image for YOLO World detection: {e}")
    #         return

    #     # Predict with YOLO World
    #     results = self.model(frame, conf=0.3)[0]  # Adjust confidence threshold if needed

    #     # Get frame dimensions
    #     frame_h, frame_w = frame.shape[:2]

    #     # Filter results for "hammer" and "bottle"
    #     for det in results.boxes.data:
    #         x1, y1, x2, y2, confidence, class_idx = det.tolist()
    #         label = self.model.names[int(class_idx)]  # Get detected object name
    #         confidence = float(confidence)

    #         if confidence < 0.3:  # Adjust confidence threshold if needed
    #             continue

    #         # Pick color
    #         color = (255, 255, 255) if label.lower() == "bottle" else (0, 165, 255)

    #         # Draw bounding box
    #         cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
    #         text = f"{label} {confidence:.2f}"
    #         cv2.putText(
    #             frame, text, (int(x1), int(y1) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2
    #         )

    #     # Display result
    #     disp = cv2.resize(frame, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_LINEAR)
    #     cv2.imshow("YOLO World Detection", disp)
    #     cv2.waitKey(1)

    # --------------------------------------------------------------------------
    #   Depth Processing
    # --------------------------------------------------------------------------
    def depthCallBack(self, msg: Image) -> None:
        """
        Processes depth data to ignore obstacles like wheels and ground.
        """
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            height, width = depth_image.shape
            valid_depths = depth_image[depth_image > 0]  # Filter out invalid depth values

            if len(valid_depths) > 0:
                self.get_logger().warning("No valid depth values found in the image.")
                min_depth = np.min(valid_depths)
                max_depth = np.max(valid_depths)
                mean_depth = np.mean(valid_depths)

                # Get depth at center of image
                center_depth = depth_image[height // 2, width // 2]

                # Print depth information
                self.get_logger().info(
                    f"Depth Stats - Min: {min_depth:.2f}m, Max: {max_depth:.2f}m, "
                    f"Mean: {mean_depth:.2f}m, Center: {center_depth:.2f}m"
                )

                # Print a small sample of depth values around center
                sample_region = depth_image[
                    height // 2 - 2 : height // 2 + 3, width // 2 - 2 : width // 2 + 3
                ]
                self.get_logger().info(f"5x5 Center Sample (meters):\n{sample_region}")
            else:
                self.get_logger().warning("No valid depth values found in the image.")
                return
        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {e}")
            return

        # Convert depth image to meters
        depth_image_meters = depth_image * 0.001

    # --------------------------------------------------------------------------
    #   ArUco Marker Detection
    # --------------------------------------------------------------------------
    def arucoMarkerDetection(self, msg: Image) -> None:
        """
        Basic ArUco marker detection.
        """
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warning("Camera intrinsics not received yet. Skipping frame.")
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters()
        aruco_detector = aruco.ArucoDetector(aruco_dict, parameters)

        corners, ids, _ = aruco_detector.detectMarkers(gray_image)

        if ids is not None and len(ids) > 0:
            self.get_logger().info(f"Detected ArUco markers: {ids.flatten()}")

    # --------------------------------------------------------------------------
    #   ROS 2 Node Main
    # --------------------------------------------------------------------------


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    sensor_processing_node = None
    try:
        sensor_processing_node = SensorProcessingNode()
        rclpy.spin(sensor_processing_node)
    except KeyboardInterrupt:
        if sensor_processing_node is not None:
            sensor_processing_node.get_logger().info(
                "Keyboard interrupt received. Shutting down..."
            )
    finally:
        if sensor_processing_node is not None:
            cv2.destroyAllWindows()
            sensor_processing_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
