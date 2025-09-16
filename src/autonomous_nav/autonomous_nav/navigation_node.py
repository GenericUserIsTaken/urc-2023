import math
import sys
from queue import Queue
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32MultiArray, Float64MultiArray, String

from lib.color_codes import ColorCodes, colorStr


class NavigationNode(Node):
    """
    A ROS 2 node for handling robot navigation using latitude/longitude waypoints
    relative to a dynamically received anchor lat/lon.

    This node:
        - Subscribes to /anchor_position (Float64MultiArray).
        - Subscribes to /goal_latlon (NavSatFix) for a new lat/lon waypoint.
        - Subscribes to /odometry/filtered (Odometry) for current pose.
        - Publishes /navigation_status (String) and /navigation_feedback (Pose2D).

    TODO:
        -Create a funcion that points the rover at a given gps coordinate

    """

    def __init__(self) -> None:
        super().__init__("navigation_node")

        # ---- Configuration / Parameters ----
        self.reached_threshold = 1.0  # meters
        self.earth_radius = 6371000.0  # Approx Earth radius in meters

        # ---- Anchor State (from /anchor_position) ----
        self.anchor_received = False
        self.ref_lat = 0.0
        self.ref_lon = 0.0
        self.ref_alt = 0.0
        self.start_lat = 0.0
        self.start_lon = 0.0
        self.start_alt = 0.0

        # ---- Internal State ----
        self.active_waypoint: Optional[Tuple[float, float]] = None
        self.current_position = (0.0, 0.0)  # x, y
        self.current_yaw = 0.0

        # ---- Subscribers ----
        # latitude, longitude, altitude
        self.anchor_sub = self.create_subscription(
            Float64MultiArray, "/anchor_position", self.anchorCallback, 10
        )

        self.latlon_sub = self.create_subscription(
            NavSatFix, "/goal_latlon", self.processLatLonGoal, 10
        )

        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odomCallback, 10
        )

        # TODO subscribe to the cost map right here

        # point ckoud data from the data processing node
        self.could_sub = self.create_subscription(Float32MultiArray, "/processed_cloud", 10)
        # ---- Publishers ----
        self.status_pub = self.create_publisher(String, "/navigation_status", 10)
        self.feedback_pub = self.create_publisher(Pose2D, "/navigation_feedback", 10)

        # ---- Timers ----
        self.timer = self.create_timer(0.1, self.updateNavigation)  # 10 Hz

        self.get_logger().info(
            colorStr("NavigationNode (dynamic anchor) initialized", ColorCodes.BLUE_OK)
        )

    # ----------------------
    #   Subscriptions
    # ----------------------
    def anchorCallback(self, msg: Float64MultiArray) -> None:
        """
        Receives the anchor position [lat, lon, alt].
        """
        if len(msg.data) < 2:
            self.get_logger().warn("Received anchor message with insufficient data.")
            return

        if not self.anchor_received:
            self.ref_lat = msg.data[0]
            self.ref_lon = msg.data[1]
            if len(msg.data) >= 3:
                self.ref_alt = msg.data[2]
            self.anchor_received = True
            self.get_logger().info(
                colorStr(
                    f"Anchor received. ref_lat={self.ref_lat:.6f}, "
                    f"ref_lon={self.ref_lon:.6f}, ref_alt={self.ref_alt:.2f}",
                    ColorCodes.BLUE_OK,
                )
            )

    def processLatLonGoal(self, msg: NavSatFix) -> None:
        """
        Converts lat/lon from the NavSatFix message into local x,y coordinates
        only if anchor is known. Otherwise, do nothing.
        """
        if not self.anchor_received:
            self.get_logger().warn("Received /goal_latlon but anchor not set yet.")
            return

        lat = msg.latitude
        lon = msg.longitude

        x, y = self.convertLatLonToXY(lat, lon)
        self.active_waypoint = (x, y)
        self.get_logger().info(
            colorStr(
                f"New lat/lon goal received: lat={lat:.6f}, lon={lon:.6f} => (x={x:.2f}, y={y:.2f})",
                ColorCodes.BLUE_OK,
            )
        )

    def odomCallback(self, msg: Odometry) -> None:
        """
        Updates self.current_position and self.current_yaw from odometry.
        """
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    # ----------------------
    #   Main Navigation Logic
    # ----------------------
    def updateNavigation(self) -> None:
        # If anchor not received, publish status but do not navigate
        if not self.anchor_received:
            self.publishStatus("No anchor received; Navigation Stopped.")
            return

        # If no active waypoint
        if self.active_waypoint is None:
            # plan a set of waypoints using a queue
            self.publishStatus("No waypoint provided; Navigation Stopped.")
            return

        # Compute distance to the waypoint
        goal_x, goal_y = self.active_waypoint
        dist_to_goal = self.distance_2d(
            self.current_position[0], self.current_position[1], goal_x, goal_y
        )

        if dist_to_goal < self.reached_threshold:
            # Reached => Publish success, clear waypoint
            self.publishStatus(f"Successfully reached waypoint ({goal_x:.2f}, {goal_y:.2f})")
            self.active_waypoint = None
            return

        self.publishStatus(f"En route to waypoint ({goal_x:.2f}, {goal_y:.2f})")
        self.publishFeedback(goal_x, goal_y)

    def planPath(
        self,
        goal_location: Tuple[float, float],
        path: Queue,
        grid: OccupancyGrid,
    ) -> None:
        path_radius = 5
        grid_height = grid.info.height
        grid_width = grid.info.width
        grid_origin = grid.info.origin  # global coordinates of origin
        self.ref_lat  # x
        self.ref_lon  # y

        # cycle through a costmap and assign each point within a 5 meter radius a certain value (heuristic)
        # add the point with the lowest value to the path queue
        # heuristic  (cartesian distance to the point) - costmap value
        #  expensive:  run through whole aarray
        # cheap : only look at point that get you closer to the goal location and are within 5 meters
        # task: make an algorithm that filters out all points farther than 5 meters and thet

        # localize the rover within the map to draw a boundary
        # TODO
        # attain position within costmap
        current_index = self.localize_rover(grid, self.current_position)
        # gather points within a certain radius
        target_area = self.collect_radius(grid, current_index)
        # choose point with the lowest value
        # base value on the distance to the goal location and the cost on the map

    def collect_radius(self, grid: OccupancyGrid, current_index: int) -> list[int]:
        radius = 5
        points_in_radius = []
        row_width = grid.info.width
        num_rows = int(radius / grid.info.resolution)
        starting_index = int(
            current_index - ((grid.info.width * (radius / 2) / grid.info.resolution))
        )
        ending_index = int(
            current_index + ((grid.info.width * (radius / 2) / grid.info.resolution))
        )
        for i in range(0, num_rows):
            for j in range(0, row_width):
                index = starting_index + (i * row_width) + j
                points_in_radius.append({grid.data[index], index})
        return points_in_radius

    def localize_rover(self, grid: OccupancyGrid, current_position: Tuple[float, float]) -> int:
        starting_position = 0, 0  # big assumptionß
        column = current_position[0] / grid.info.resolution  # x index position
        row = current_position[1] / grid.info.resolution  # y index position
        current_index = int((row * grid.info.height / grid.info.resolution) + column) % 1
        return current_index

    def find_index_location(
        self, grid: OccupancyGrid, current_position: Tuple[float, float], target_index: int
    ) -> Tuple[float, float]:
        rover_index = self.localize_rover(grid, current_position)
        # find row position of both items
        rover_row = int(rover_index / grid.info.width)
        target_row = int(target_index / grid.info.width)
        # find column position of both items
        rover_column = grid.info.width - (rover_index % grid.info.width)
        target_column = grid.info.width - (target_index % grid.info.width)
        # use the differences in those numbers to determine the position
        col_dif = abs(target_column - rover_column)
        row_diff = abs(rover_row - target_row)
        if target_column >= rover_column:
            x_position = float(current_position[0] + (col_dif * grid.info.resolution))
        elif target_column < rover_column:
            x_position = float(current_position[0] - (col_dif * grid.info.resolution))
        if target_row >= rover_row:
            y_position = float(current_position[1] - (row_diff * grid.info.resolution))
        elif target_row < rover_row:
            y_position = float(current_position[1] + (row_diff * grid.info.resolution))

        return (x_position, y_position)

    def turnTowardGoal(self, goal_Location: Tuple[float, float]) -> None:
        a = self.distance_2d(
            self.current_position[0], self.current_position[1], goal_Location[0], goal_Location[1]
        )
        b = self.distance_2d(goal_Location[0], goal_Location[1], self.start_lat, self.start_lon)
        c = self.distance_2d(goal_Location[0], goal_Location[1], self.start_lat, self.start_lon)
        temp = (a**2 - c**2 - b**2) / (-2 * b * c)
        turn_angle = math.degrees(math.acos(temp))
        # read from current pose and anchor position
        # do the same calculation using measurmentts relative to the anchor
        # turn left or right that number of degrees

    # ----------------------
    #   Publishing Helpers
    # ----------------------
    def publishStatus(self, msg: str) -> None:
        self.status_pub.publish(String(data=msg))
        # self.get_logger().info(colorStr(msg, ColorCodes.GREEN_OK))

    def publishFeedback(self, gx: float, gy: float) -> None:
        dx = gx - self.current_position[0]
        dy = gy - self.current_position[1]
        desired_yaw = math.atan2(dy, dx)
        heading_error = self.normalize_angle(desired_yaw - self.current_yaw)

        feedback = Pose2D()
        feedback.x = dx
        feedback.y = dy
        feedback.theta = heading_error
        self.feedback_pub.publish(feedback)

    # ----------------------
    #   Coordinate Conversion
    # ----------------------
    def convertLatLonToXY(self, lat: float, lon: float) -> Tuple[float, float]:
        lat_r = math.radians(lat)
        lon_r = math.radians(lon)
        ref_lat_r = math.radians(self.ref_lat)
        ref_lon_r = math.radians(self.ref_lon)

        x = (lon_r - ref_lon_r) * math.cos((lat_r + ref_lat_r) / 2.0) * self.earth_radius
        y = (lat_r - ref_lat_r) * self.earth_radius
        return (x, y)

    # ----------------------
    #   Utilities
    # ----------------------
    @staticmethod
    def distance_2d(x1: float, y1: float, x2: float, y2: float) -> float:
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    @staticmethod
    def quaternion_to_yaw(x_q: float, y_q: float, z_q: float, w_q: float) -> float:
        siny_cosp = 2.0 * (w_q * z_q + x_q * y_q)
        cosy_cosp = 1.0 - 2.0 * (y_q * y_q + z_q * z_q)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle <= -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = NavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        node.get_logger().info(colorStr("External shutdown request received", ColorCodes.BLUE_OK))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
