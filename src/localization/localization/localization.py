import sys

import rclpy
from geometry_msgs.msg import Quaternion, Vector3
from nav_msgs.msg import Odometry  # todo, incorporate odometry to make position more accurate

# todo, subscribe to imu to get rotation information
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64MultiArray

from lib.color_codes import ColorCodes, colorStr


#'zed/zed_node/imu/data'
class Localization(Node):

    def __init__(self) -> None:
        super().__init__("localization_node")
        self.init_pos = Float64MultiArray()  # first gps coords we recieve
        self.init_pos.data = [0, 0, 0]
        self._first_gps_recieved = False
        # self.init_pos.data = [self.anchor_lat, self.anchor_lon, self.anchor_alt]
        self.local_pos = Float64MultiArray()  # calculated offset from first gps coords
        self.local_pos.data = [0, 0, 0]
        # self.local_pos.data = [self.anchor_lat, self.anchor_lon, self.anchor_alt]
        self.global_pos = Float64MultiArray()  # calculated offset from first gps coords
        self.global_pos.data = [0, 0, 0]
        # self.global_pos.data = [self.anchor_lat, self.anchor_lon, self.anchor_alt]
        self.get_logger().info(colorStr("Launching localization_node", ColorCodes.BLUE_OK))
        # supposedly 10 in the subscription refers to the queue size, but who knows what that means
        self.gps_sub = self.create_subscription(NavSatFix, "/anchor_position", self.processGPS, 10)
        self.gps_sub = self.create_subscription(Imu, "/zed/zed_node/imu/data", self.processIMU, 10)
        # setup publisher to publish localization position
        self.local_position_pub = self.create_publisher(Vector3, "localization_local_position", 10)
        self.global_position_pub = self.create_publisher(
            Vector3, "localization_global_position", 10
        )
        self.orientation_pub = self.create_publisher(Quaternion, "localization_orientation", 10)
        # self.rotation_pub = self.create_publisher(Quaternion, "localization_rotation", 10)

    def processGPS(self, msg: Float64MultiArray) -> None:
        """
        Updates self.local_position and self.init_position from gps.
        """
        # [self.anchor_lat, self.anchor_lon, self.anchor_alt]
        if self._first_gps_recieved:
            self.local_pos.data = [
                msg.data[0] - self.init_pos.data[0],
                msg.data[1] - self.init_pos.data[1],
                msg.data[2] - self.init_pos.data[2],
            ]
        else:
            self.init_pos.data = [msg.data[0], msg.data[1], msg.data[2]]
            self._first_gps_recieved = True

        self.global_pos.data = [
            msg.data[0],
            msg.data[1],
            msg.data[2],
        ]
        self.publishPosition()

    def processIMU(self, msg: Imu) -> None:
        content = "quaternion from imu" + msg.orientation
        self.get_logger().info(colorStr(content, ColorCodes.GREEN_OK))
        self.orientation_pub.publish(msg.orientation)
        # could also use this info:
        # msg.linear_acceleration
        # msg.angular_velocity

    def publishLocalPosition(self) -> None:
        contents = self.local_pos
        msg = Vector3()  # Create the message
        msg.x = contents[1]  # x = lon
        msg.y = contents[0]  # y = lat
        msg.z = contents[2]  # z = alt
        self.local_position_pub.publish(msg)

    def publishGlobalPosition(self) -> None:
        contents = self.global_pos
        msg = Vector3()  # Create the message
        msg.x = contents[1]  # x = lon
        msg.y = contents[0]  # y = lat
        msg.z = contents[2]  # z = alt
        self.global_position_pub.publish(msg)

    def publishPosition(self) -> None:
        self.publishLocalPosition()
        self.publishGlobalPosition()

    # def global_gps_loc_to_local_offset(self):


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
