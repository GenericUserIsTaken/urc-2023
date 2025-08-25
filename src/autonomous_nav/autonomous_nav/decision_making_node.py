"""
Role: This script defines the autonomous node that acts as the high-level controller.

Functionality:
    - Receives inputs from Navigation (/navigation_feedback, /navigation_status),
      Sensor Processing (/obstacle_detected, /obstacle_info), and (optionally) Localization
      to make decisions on how to drive the rover to its waypoint.
    - Publishes commands to the drivebase to move or stop.
"""

import math
import sys
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose2D
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

from lib.color_codes import ColorCodes, colorStr

from .dwa_planner import DWAPlanner


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
        # Change obstacle_info to be Nav2 Costmap 2D data.
        self.obstacle_info: list[float] = []

        self.navigation_status: str = "No waypoint provided; Navigation Stopped."
        self.nav_feedback = Pose2D()

        # Finite-state variable for obstacle avoidance
        # Mypy fix: allow str or None
        self.avoid_state: Optional[str] = None
        self.avoid_start_time = self.get_clock().now()

        # ---- Subscribers ----
        self.create_subscription(Bool, "/obstacle_detected", self.obstacleCallback, 10)
        self.create_subscription(OccupancyGrid, "/costmap", self.obstacleInfoCallback, 10)
        self.create_subscription(String, "/navigation_status", self.navStatusCallback, 10)
        # gives us the angle and position of the rover
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

    def obstacleInfoCallback(self, msg: OccupancyGrid) -> None:
        self.obstacle_info = PyCostmap2D(msg)

    def navStatusCallback(self, msg: String) -> None:
        self.navigation_status = msg.data
        # self.get_logger().info(
        #     colorStr(f"Navigation Status: {self.navigation_status}", ColorCodes.YELLOW_WARN)
        # )

    def navFeedbackCallback(self, msg: Pose2D) -> None:
        self.nav_feedback = msg

    # --------------------------------------------------------------------------
    #   Main Decision Logic
    # --------------------------------------------------------------------------
    def updateDecision(self) -> None:
        """
        Periodically checks obstacles, waypoint status, and decides how to drive.
        """
        # If no waypoint or reached
        if (
            "No waypoint provided" in self.navigation_status
            or "Successfully reached" in self.navigation_status
        ):
            self.stopRover()
            self.avoid_state = None  # Reset
            return

        # Obstacle logic
        if self.obstacle_detected or self.avoid_state is not None:
            self.handleObstacleAvoidance()
            return

        # Normal waypoint driving
        self.driveTowardWaypoint()

    def handleObstacleAvoidance(self) -> None:
        if self.obstacle_info is not None:
            # Extract numpy array from PyCostmap2D for DWA
            costmap_array = self.obstacle_info.costmap

            # Create DWA Planner with costmap
            dwa_planner = DWAPlanner(
                costmap=costmap_array,
                robot_radius=0.3,  # Your robot radius
                current_velocity=self.current_wheel_vel,
                current_position=self.current_pos,
                time_delta=0.1,
                goal=self.goal,
                theta=self.current_theta,
            )

            # Get optimal wheel velocities
            left_vel, right_vel = dwa_planner.plan()
            self.publishDriveCommands(left_vel, right_vel)

    # --------------------------------------------------------------------------
    #   Helper Functions
    # --------------------------------------------------------------------------
    def stopRover(self) -> None:
        """Publish zero velocity."""
        self.publishDriveCommands(0.0, 0.0)

    def publishDriveCommands(self, left_speed: float, right_speed: float) -> None:
        """
        Publishes Float32 speeds to the drivebase.
        Positive => forward, negative => reverse.
        """
        self.left_drive_pub.publish(Float32(data=left_speed))
        self.right_drive_pub.publish(Float32(data=right_speed))

    @staticmethod
    def sign(value: float) -> float:
        """
        Returns +1.0 if value >= 0.0, else -1.0.
        """
        return 1.0 if value >= 0.0 else -1.0


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    decision_making_node = None
    try:
        decision_making_node = DecisionMakingNode()
        rclpy.spin(decision_making_node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        if decision_making_node is not None:
            decision_making_node.get_logger().info(
                colorStr("Shutting down decision_making_node", ColorCodes.BLUE_OK)
            )
    finally:
        if decision_making_node is not None:
            decision_making_node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == "__main__":
    main()
