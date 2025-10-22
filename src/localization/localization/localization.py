import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from lib.color_codes import ColorCodes, colorStr


class Localization(Node):

    def __init__(self) -> None:
        super().__init__("localization_node")
        self.get_logger().info(colorStr("Launching localization_node", ColorCodes.BLUE_OK))
        # setup subscription to odometry and gps
        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odomCallback, 10
        )
        # self.latlon_sub = self.create_subscription(
        #    NavSatFix, "/goal_latlon", self.processLatLonGoal, 10
        # )
        # setup publisher to publish localization position


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        localization = Localization()
        rclpy.spin(localization)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        localization.get_logger().info(
            colorStr("Shutting down localization_node", ColorCodes.BLUE_OK)
        )
        localization.destroy_node()
        sys.exit(0)


# def odomCallback(self, msg: Odometry) -> None:
#    """
#    Updates self.current_position and self.current_yaw from odometry.
#    """
#    pass
