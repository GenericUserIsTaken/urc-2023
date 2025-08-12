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

import sys
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

from autonomous_nav.dwa_planner import DWAPlanner
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
        self.occupancy_grid: list[Tuple[float, float]] = []
        self.goal: Tuple[float, float]
        self.current_yaw = 0.0

        self.navigation_status: str = "No waypoint provided; Navigation Stopped."
        self.nav_feedback = Pose2D()

        # Finite-state variable for obstacle avoidance
        # Mypy fix: allow str or None
        self.avoid_state: Optional[str] = None
        self.avoid_start_time = self.get_clock().now()

        # ---- Subscribers ----
        self.create_subscription(Bool, "/obstacle_detected", self.obstacleCallback, 10)
        self.create_subscription(PointCloud2, "/obstacle_goal", self.goal, 10)
        self.create_subscription(String, "/navigation_status", self.navStatusCallback, 10)
        self.create_subscription(Pose2D, "/navigation_feedback", self.navFeedbackCallback, 10)
        self.create_subscription(float, "/navigation_heading", self.navHeadingCallback, 10)

        # Subscribe to get data about current velocity and position
        # self.create_subscription(Pose2D, "/current_velocity", self.currentVelocityCallback, 10)

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
    def obstacleCallback(self, msg: Bool) -> None:
        self.obstacle_detected = msg.data

    def obstacleOccupancyGridCallback(self, msg: Float32MultiArray) -> None:
        self.occupancy_grid = list(msg.data)

    def navStatusCallback(self, msg: String) -> None:
        self.navigation_status = msg.data
        # self.get_logger().info(
        #     colorStr(f"Navigation Status: {self.navigation_status}", ColorCodes.YELLOW_WARN)
        # )

    def navFeedbackCallback(self, msg: Pose2D) -> None:
        self.nav_feedback = msg

    def navHeadingCallback(self, msg: float) -> None:
        self.current_yaw = msg

    # --------------------------------------------------------------------------
    #   Decision Logic
    # --------------------------------------------------------------------------

    """
    Receives a 2D map of the obstacles and navigates to a goal point.
    Point Cloud is represented by a numpy array of a list of floats.
    Maps Point Cloud to a list of Obstacle objects and inputs to dwa_planner
    to navigate around them.
    """

    def update_decision(self) -> None:
        velocity_command: Tuple[float, float] = self.get_vel_command()
        self.publish_drive_commands(velocity_command[0], velocity_command[1])

    def get_vel_command(self) -> Tuple[float, float]:
        """
        Use the dwa_planner class to calculate a velocity command based on the 2d map of obstacles.

        Args:
            obstacles: List of detected obstacles.

        Returns:
            Tuple of left and right wheel velocities.
        """

        current_state: list[float] = []
        goal: Tuple[float, float] = 100.0, 100.0

        planner: DWAPlanner = DWAPlanner(
            occupancy_grid=self.occupancy_grid,
            robot_radius=0.5,
            current_velocity=(0.0, 0.0),
            current_position=(50.0, 0.0),
            time_delta=0.1,
            goal=self.goal,
            heading=self.current_yaw,
        )

        velocity_command: Tuple[float, float] = planner.plan()

        print("Velocity commands: {velocity_command[0]}, {velocity_command[1]}")

        return velocity_command

    def publish_drive_commands(self, left_speed: float, right_speed: float) -> None:
        """
        Publishes Float32 speeds to the drivebase.
        Positive => forward, negative => reverse.
        """
        self.left_drive_pub.publish(Float32(data=left_speed))
        self.right_drive_pub.publish(Float32(data=right_speed))


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
