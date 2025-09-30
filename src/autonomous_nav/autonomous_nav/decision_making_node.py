"""
Role: This script defines the autonomous node that acts as the high-level controller.

Functionality:
    - Receives inputs from Navigation (/navigation_feedback, /navigation_status),
      Sensor Processing (/obstacle_detected, /obstacle_info), and (optionally) Localization
      to make decisions on how to drive the rover to its waypoint.
    - Publishes commands to the drivebase to move or stop.
"""

import sys
from queue import Queue
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose2D
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav_msgs.msg import OccupancyGrid

# import numpy as np
from numpy import int8, ndarray, zeros
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

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
        # self.obstacle_detected: bool = False
        # Change obstacle_info to be Nav2 Costmap 2D data.
        self.obstacle_info: Optional[PyCostmap2D] = None

        self.navigation_status: str = "No waypoint provided; Navigation Stopped."
        self.nav_feedback = Pose2D()

        # Finite-state variable for obstacle avoidance
        # Mypy fix: allow str or None
        self.avoid_state: Optional[str] = None
        self.avoid_start_time = self.get_clock().now()

        # ---- Subscribers ----
        # self.create_subscription(Bool, "/obstacle_detected", self.obstacle_callback, 10)
        self.create_subscription(OccupancyGrid, "/costmap", self.obstacle_info_callback, 10)
        self.create_subscription(String, "/navigation_status", self.nav_status_callback, 10)
        # gives us the angle and position of the rover
        self.create_subscription(Pose2D, "/navigation_feedback", self.nav_feedback_callback, 10)
        self.create_subscription(Tuple[float, float], "/waypoint", self.waypoint_callback, 10)
        self.create_subscription(Queue[Tuple[float, float]], "/path", 10)

        # ---- Publishers (to Drivebase) ----
        self.left_drive_pub = self.create_publisher(Float32, "move_left_drivebase_side_message", 10)
        self.right_drive_pub = self.create_publisher(
            Float32, "move_right_drivebase_side_message", 10
        )

        # Timer to run decision logic at ~10Hz
        self.timer = self.create_timer(0.1, self.update_decision)

        self.get_logger().info(colorStr("DecisionMakingNode started.", ColorCodes.BLUE_OK))

    # --------------------------------------------------------------------------
    #   Subscription Callbacks
    # --------------------------------------------------------------------------
    def obstacle_callback(self, msg: Bool) -> None:
        self.obstacle_detected = msg.data

    def obstacle_info_callback(self, msg: OccupancyGrid) -> None:
        self.obstacle_info = PyCostmap2D(msg)

    def nav_status_callback(self, msg: String) -> None:
        self.navigation_status = msg.data
        # self.get_logger().info(
        #     colorStr(f"Navigation Status: {self.navigation_status}", ColorCodes.YELLOW_WARN)
        # )

    def nav_feedback_callback(self, msg: Pose2D) -> None:
        self.nav_feedback = msg

    def waypoint_callback(self, msg: Tuple[float, float]) -> None:
        self.goal = msg
        self.current_pos = (self.nav_feedback.x, self.nav_feedback.y)
        self.current_theta = self.nav_feedback.theta
        # For simplicity, assume current wheel velocities are zero.
        self.current_wheel_vel = (0.0, 0.0)

    # --------------------------------------------------------------------------
    #   Main Decision Logic
    # --------------------------------------------------------------------------
    def update_decision(self) -> None:
        """
        Periodically checks obstacles, waypoint status, and decides how to drive.
        """
        # If no waypoint or reached
        if (
            "No waypoint provided" in self.navigation_status
            or "Successfully reached" in self.navigation_status
        ):
            self.stop_rover()
            self.avoid_state = None  # Reset
            return

        else:
            self.handle_obstacle_avoidance()

    def handle_obstacle_avoidance(self) -> None:
        if self.obstacle_info is not None:
            # Create DWA Planner with costmap

            obstacles = self.convert_costmap_to_grid(self.obstacle_info)

            dwa_planner = DWAPlanner(
                costmap=self.obstacle_info,
                robot_radius=0.3,
                current_velocity=self.current_wheel_vel,
                current_position=self.current_pos,
                time_delta=0.1,
                goal=self.goal,
                theta=self.current_theta,
            )

            # Get optimal wheel velocities
            left_vel, right_vel = dwa_planner.plan()
            self.publish_drive_commands(left_vel, right_vel)
        else:
            self.stop_rover()

    # --------------------------------------------------------------------------
    #   Helper Functions
    # --------------------------------------------------------------------------
    def stop_rover(self) -> None:
        """Publish zero velocity."""
        self.publish_drive_commands(0.0, 0.0)

    def publish_drive_commands(self, left_speed: float, right_speed: float) -> None:
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

    @staticmethod
    def convert_costmap_to_grid(costmap: PyCostmap2D) -> ndarray:
        """
        Converts a PyCostmap2D to a 2D grid representation.
        """
        width = costmap.getSizeInCellsX()
        height = costmap.getSizeInCellsY()

        grid: ndarray = zeros((width, height), dtype=int8)

        # grid = [[0 for _ in range(width)] for _ in range(height)]

        for y in range(height):
            for x in range(width):
                grid[x][y] = costmap.getCostXY(x, y)

        return grid


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    decision_making_node = None
    try:
        decision_making_node = DecisionMakingNode()
        rclpy.spin(decision_making_node)

        # Entry point for business logic
        decision_making_node.update_decision()

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
