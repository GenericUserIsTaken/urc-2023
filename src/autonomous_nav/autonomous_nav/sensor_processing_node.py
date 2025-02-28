"""
Role: This script defines the sensor processing node, responsible for processing all sensor data:
  - ArUco detection
  - LIDAR min-distance
  - Depth-based obstacle detection
  - Pretrained detection for "Hammer" + "Bottle" (from Open Images V6),
    using FiftyOne's Model Zoo (no custom training required).
"""

import sys
import math
import numpy as np
import cv2  # pylint: disable=no-member
from cv2 import aruco
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Imu, LaserScan, Image, CameraInfo
from std_msgs.msg import Float32, Float32MultiArray, Bool

from cv_bridge import CvBridge

# Make sure you have: pip install fiftyone[all]
import fiftyone.zoo as foz
import fiftyone as fo

from lib.color_codes import ColorCodes, colorStr


class SensorProcessingNode(Node):
    """
    A ROS 2 node for processing sensor data with:
      - ArUco detection
      - Depth-based obstacle ignoring wheels/ground
      - LIDAR min-distance
      - Pretrained detection (Open Images V6) for "Hammer" and "Bottle"
        => Treat "Hammer" as your "mallet", "Bottle" as the water bottle
    """

    def __init__(self) -> None:
        super().__init__("sensor_processing_node")

        self.get_logger().info(
            "Initializing sensor_processing_node with OpenImages-based detection..."
        )

        self.bridge = CvBridge()
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None

        # ----------------------------------------------------------------------
        # Load an Open Images V6 detection model from the FiftyOne Model Zoo
        # restricting classes to ["Hammer", "Bottle"]
        # If you want to lower or raise confidence_thresh, do so below
        self.model = foz.load_zoo_model(
            "open-images-v6-det",  # open-images detection model from FiftyOne
            classes=["Hammer", "Bottle"],
            confidence_thresh=0.2,
        )

        # ----------------------------------------------------------------------
        #  Subscriptions
        # ----------------------------------------------------------------------
        # 1) ArUco detection
        self.image_sub = self.create_subscription(
            Image, "/zed/zed_node/rgb/image_rect_color", self.arucoMarkerDetection, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/zed/zed_node/rgb/camera_info", self.processCameraInfo, 10
        )

        # 2) OpenImages-based detection
        self.oid_sub = self.create_subscription(
            Image, "/zed/zed_node/rgb/image_rect_color", self.openImagesDetectionCallback, 10
        )

        # 3) LIDAR
        self.lidar_sub = self.create_subscription(LaserScan, "/lidar_scan", self.processLidar, 10)
        self.processed_pub = self.create_publisher(Float32, "/processed_data", 10)

        # 4) Depth-based obstacle detection
        self.depth_sub = self.create_subscription(
            Image, "/zed/zed_node/depth/depth_registered", self.obstacleDetection, 10
        )
        self.obstacle_detected_pub = self.create_publisher(Bool, "/obstacle_detected", 10)
        self.obstacle_info_pub = self.create_publisher(Float32MultiArray, "/obstacle_info", 10)

        # ArUco publisher
        self.aruco_pub = self.create_publisher(Float32MultiArray, "/aruco_marker_data", 10)

        self.get_logger().info(
            "sensor_processing_node is up and running with Open Images V6 'Hammer' + 'Bottle' detection."
        )

    # --------------------------------------------------------------------------
    #   processCameraInfo
    # --------------------------------------------------------------------------
    def processCameraInfo(self, msg: CameraInfo) -> None:
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    # --------------------------------------------------------------------------
    #   openImagesDetectionCallback
    # --------------------------------------------------------------------------
    def openImagesDetectionCallback(self, msg: Image) -> None:
        """
        Runs an Open Images V6 detection model from FiftyOne on the camera feed.
        We specifically loaded "Hammer" + "Bottle" classes. We interpret "Hammer"
        as your "mallet," "Bottle" for water bottles, no custom training needed.
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image for OpenImages detection: {e}")
            return

        # Predict with FiftyOne's open-images-v6-det model
        predictions = self.model.predict(frame)  # returns fiftyone.core.labels.Detections

        for det in predictions.detections:
            label = det.label  # "Hammer" or "Bottle"
            confidence = det.confidence
            x, y, w, h = det.bounding_box  # normalized [0..1]
            frame_h, frame_w = frame.shape[:2]

            # Convert to pixel coords
            x1 = int(x * frame_w)
            y1 = int(y * frame_h)
            x2 = int((x + w) * frame_w)
            y2 = int((y + h) * frame_h)

            # pick color
            if label.lower() == "bottle":
                color = (255, 255, 255)  # white
            else:
                color = (0, 165, 255)  # orange-ish for "hammer"

            # draw box
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            text = f"{label} {confidence:.2f}"
            cv2.putText(frame, text, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        # Display
        scale_factor = 2.0
        disp = cv2.resize(
            frame, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_LINEAR
        )
        cv2.namedWindow("OpenImages Detection", cv2.WINDOW_NORMAL)
        cv2.imshow("OpenImages Detection", disp)
        cv2.waitKey(1)

    # --------------------------------------------------------------------------
    #   ArUco Marker Detection
    # --------------------------------------------------------------------------
    def arucoMarkerDetection(self, msg: Image) -> None:
        """
        Basic ArUco detection (unchanged).
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
            # If you do pose estimation, keep that logic here
        # Could draw markers or debug display

    # --------------------------------------------------------------------------
    #   processLidar
    # --------------------------------------------------------------------------
    def processLidar(self, msg: LaserScan) -> None:
        self.get_logger().debug("processLidar callback triggered.")
        if not msg.ranges:
            self.get_logger().warning("Lidar ranges data is empty.")
            return

        min_distance = min(msg.ranges)
        self.get_logger().info(f"Processed lidar data. Min distance: {min_distance:.2f}m")
        self.processed_pub.publish(Float32(data=min_distance))

    # --------------------------------------------------------------------------
    #   obstacleDetection (Depth)
    # --------------------------------------------------------------------------
    def obstacleDetection(self, depth_msg: Image) -> None:
        if depth_msg.encoding not in ["32FC1", "16UC1"]:
            self.get_logger().warning(
                f"Unsupported depth image encoding: {depth_msg.encoding}. Expected 32FC1 or 16UC1."
            )
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, depth_msg.encoding)
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")
            return

        height, width = depth_image.shape
        if height == 0 or width == 0:
            self.get_logger().warning("Depth image has zero size.")
            return

        roi_top = 0
        roi_bottom = int(0.75 * height)
        roi_left = int(0.25 * width)
        roi_right = int(0.75 * width)
        roi = depth_image[roi_top:roi_bottom, roi_left:roi_right]
        roi_h, roi_w = roi.shape

        close_thresh = 1.5
        mask_close = (roi > 0.0) & (roi < close_thresh)

        mask_uint8 = np.where(mask_close, 255, 0).astype(np.uint8)
        kernel = np.ones((9, 9), np.uint8)
        opened = cv2.morphologyEx(mask_uint8, cv2.MORPH_OPEN, kernel)
        filtered_mask = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)

        num_close_pixels = np.count_nonzero(filtered_mask)
        total_pixels_roi = roi_h * roi_w
        threshold_percentage = 0.01
        self_detected = (total_pixels_roi > 0) and (
            (num_close_pixels / total_pixels_roi) > threshold_percentage
        )

        self.obstacle_detected_pub.publish(Bool(data=self_detected))

        obstacle_data = Float32MultiArray()
        if self_detected:
            close_pixels = roi[filtered_mask == 255]
            avg_close_distance = float(np.mean(close_pixels)) if close_pixels.size > 0 else 0.0
            obstacle_data.data = [1.0, avg_close_distance, float(num_close_pixels)]
            self.get_logger().info(
                f"Obstacle in ROI. ~{avg_close_distance:.2f}m average distance. ROI: {roi_w}x{roi_h}"
            )
        else:
            obstacle_data.data = [0.0, 0.0, 0.0]

        self.obstacle_info_pub.publish(obstacle_data)


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
    except ExternalShutdownException:
        if sensor_processing_node is not None:
            sensor_processing_node.get_logger().info(
                colorStr(
                    "External shutdown requested. Shutting down sensor_processing_node",
                    ColorCodes.BLUE_OK,
                )
            )
    finally:
        if sensor_processing_node is not None:
            cv2.destroyAllWindows()
            sensor_processing_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
