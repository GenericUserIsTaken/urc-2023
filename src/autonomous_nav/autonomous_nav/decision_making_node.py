"""
Role: This script defines the autonomous node that acts as the high-level controller.

Functionality:
    - Receives inputs from Navigation (/navigation_feedback, /navigation_status),
      Sensor Processing (/obstacle_detected, /obstacle_info), and (optionally) Localization
      to make decisions on how to drive the rover to its waypoint.
    - Publishes commands to the drivebase to move or stop.



    - Receives goals from navigation node to determine the next waypoint
    - Uses DWA to determine the best path to the waypoint
    - Publishes commands to the drivebase to move or stop.
    - Publishes the current waypoint to the navigation node.
"""

import math
import sys
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

from lib.color_codes import ColorCodes, colorStr


class Obstacle:
    def __init__(self, position: list, radius: float):
        """
        Initialize an obstacle with a position and radius.

        Args:
            position: [x, y] coordinates of the obstacle
            radius: Radius of the obstacle
        """
        self.position = position
        self.radius = radius


class DecisionMakingNode(Node):
    """
    A ROS 2 node that handles decision-making for a robot.

    Subscriptions:
      - /navigation_feedback (Pose2D): (dx, dy, dtheta) from the NavigationNode.
      - /navigation_status (String): e.g. "En route...", "No waypoint...", "Reached waypoint"
      - /obstacle_detected (Bool): True if a large obstacle is present.
      - /obstacle_info (Float32MultiArray): Additional obstacle data (e.g. average distance).

    Publications:
      - /move_left_drivebase_side_message (Float32)
      - /move_right_drivebase_side_message (Float32)

    Basic finite-state logic:
      1. If no waypoint or it's reached, stop.
      2. If obstacle is present, pivot left until clear, then drive forward for 3s
         to ensure we bypass the obstacle.
      3. Otherwise, steer toward the waypoint using heading error.
    """

    def __init__(self) -> None:
        super().__init__("decision_making_node")

        # ---- State Variables ----
        self.obstacle_detected: bool = False
        self.obstacle_pointcloud: list[float] = []  # Mypy: typed list of floats
        self.raw_obstacle_pointcloud: list[float] = (
            []
        )  # Mypy: typed list of floats directly from sensor

        self.navigation_status: str = "No waypoint provided; Navigation Stopped."
        self.nav_feedback = Pose2D()

        # Finite-state variable for obstacle avoidance
        # Mypy fix: allow str or None
        self.avoid_state: Optional[str] = None
        self.avoid_start_time = self.get_clock().now()

        # ---- Subscribers ----
        self.create_subscription(Bool, "/obstacle_detected", self.obstacleCallback, 10)
        self.create_subscription(PointCloud2, "/obstacle_goal_pointcloud", self.goalPointCloud, 10)
        self.create_subscription(PointCloud2, "/raw_pointcloud", self.rawPointCloud, 10)
        self.create_subscription(String, "/navigation_status", self.navStatusCallback, 10)
        self.create_subscription(Pose2D, "/navigation_feedback", self.navFeedbackCallback, 10)

        # ---- Publishers (to Drivebase) ----
        self.left_drive_pub = self.create_publisher(Float32, "move_left_drivebase_side_message", 10)
        self.right_drive_pub = self.create_publisher(
            Float32, "move_right_drivebase_side_message", 10
        )

        # Timer to run decision logic at ~10Hz
        self.timer = self.create_timer(0.1, self.updateDecision)

        self.get_logger().info(colorStr("DecisionMakingNode started.", ColorCodes.BLUE_OK))

    # --------------------------------------------------------------------------
    #   Subscription Callbacks
    # --------------------------------------------------------------------------
    def obstacleCallback(self, msg: Bool) -> None:
        self.obstacle_detected = msg.data

    def obstacleGoalPointCloud(self, msg: Float32MultiArray) -> None:
        self.obstacle_pointcloud = list(msg.data)

    def rawPointCloud(self, msg: Float32MultiArray) -> None:
        """
        Receives raw point cloud data from the sensor.
        This is not used in the current logic but can be extended for more complex decision-making.
        """
        self.raw_obstacle_pointcloud = list(msg.data)

    def navStatusCallback(self, msg: String) -> None:
        self.navigation_status = msg.data
        # self.get_logger().info(
        #     colorStr(f"Navigation Status: {self.navigation_status}", ColorCodes.YELLOW_WARN)
        # )

    def navFeedbackCallback(self, msg: Pose2D) -> None:
        self.nav_feedback = msg

    # --------------------------------------------------------------------------
    #   Decision Logic
    # --------------------------------------------------------------------------

    """
    Receives a 2D map of the obstacles and navigates to a goal point.
    Point Cloud is represented by a numpy array of a list of floats.
    Maps Point Cloud to a list of Obstacle objects and inputs to dwa_planner
    to navigate around them.
    """


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DecisionMakingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        node.get_logger().info(colorStr("Shutting down decision making node", ColorCodes.BLUE_OK))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
