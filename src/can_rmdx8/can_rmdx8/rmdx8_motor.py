# Write the code for the node, implement everything that you can until
# the other RMD task is finished. Make sure to mimic the behaviour of
# the current Moteus node, but not necessarily copy the code.
import sys

import myactuator_rmd_py as rmd
import rclpy
import std_msgs.msg

# from moteus.moteus import Result
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from std_msgs.msg import String

from lib.configs import RMDx8MotorConfig
from lib.motor_state.rmd_motor_state import RMDX8MotorState, RMDX8RunSettings


class RMDx8Motor(Node):
    """
    A wrapper class to interact with the RMDx8 actuator and the ROS nodes.
    """

    def __init__(self, config: RMDx8MotorConfig) -> None:
        super().__init__("can_rmdx8_node")

        self.config = config
        self.driver = rmd.CanDriver("can1")
        self.my_actuator = rmd.ActuatorInterface(self.driver, config.can_id)
        self._subscriber = self._createSubscriber()
        self._publisher = self._createPublisher()
        self.run_settings: RMDX8RunSettings = RMDX8RunSettings()

    # Main function
    def main(self, args: list[str] | None = None) -> None:
        """
        The entry point of the node.
        """

        rclpy.init(args=args)

    # create a subscriber
    def _createSubscriber(self) -> Subscription:
        topic_name = self.config.getInterfaceTopicName()
        subscriber = self.create_subscription(
            std_msgs.msg.String,
            topic_name,
            self.dataInCallback,
            1,
        )
        return subscriber

    # create a publisher
    def _createPublisher(self) -> Publisher:
        """
        The publisher to send data to.
        """
        topic_name = self.config.getCanTopicName()
        # Size of queue is 1. All additional ones are dropped
        publisher = self.create_publisher(std_msgs.msg.String, topic_name, 1)
        self.get_logger().info("Hello World!")
        return publisher

    def dataInCallback(self, msg: String) -> None:
        """
        Updates the RMDx8 motor state
        """
        self.run_settings = RMDX8RunSettings.fromJsonMsg(msg)

    def publishData(self) -> None:
        """
        Publishes data from the rmdx8 controller
        """
        # TODO Replace None with the Result that Saharsh creates
        state = RMDX8MotorState.fromRMDX8Data(
            self.config.can_id,
            self.my_actuator.getMotorStatus1(),
            self.my_actuator.getMotorStatus2(),
            self.my_actuator.getMotorPower(),
            self.my_actuator.getAcceleration(),
        )
        self._publisher.publish(state.toMsg())
