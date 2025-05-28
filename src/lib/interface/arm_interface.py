"""
This module just contains the ArmInterface class. It provides utility functions to interact with
motors on the arm.
"""

import math

from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String

from lib.configs import MoteusMotorConfig, MotorConfigs
from lib.moteus_motor_state import MoteusRunSettings

from .robot_info import RobotInfo
from .robot_interface import RobotInterface

REVS_TO_RADIANS = math.pi * 2
RADIANS_TO_REVS = 1 / REVS_TO_RADIANS


class ArmInterface:
    """
    A class that provides methods to operate the motors on the arm using feedforward.
    """

    def __init__(self, ros_node: Node, info: RobotInfo, interface: RobotInterface) -> None:
        self._ros_node = ros_node
        self._publishers: dict[int, Publisher] = {}

        # moteus motor classes
        self._info = info
        self._interface = interface

        self.shoulder_current_pos = 0.0
        self.elbow_current_pos = 0.0

        # feedforward values
        self.shoulder_feedforward = 0.0

        # variables for running to position
        self.shoulder_position = False  # using position mode (rather than velocity)
        # so motor won't be moved with position when it's far from target and moving with velocity

        self.shoulder_moving_to_position = False  # shoulder is actively moving to position
        self.shoulder_target_position = 0.0

        ros_node.create_timer(0.05, self.runShoulderToPosition)

    def update_motor_positions(self) -> None:
        """
        Updates the current shoulder and elbow positions.
        """
        self.shoulder_current_pos = (
            self._info.getMotorState(MotorConfigs.ARM_SHOULDER_MOTOR).position * -REVS_TO_RADIANS
        )

        self.elbow_current_pos = (
            self._info.getMotorState(MotorConfigs.ARM_ELBOW_MOTOR).position * -REVS_TO_RADIANS
        )

    def calc_shoulder_feedforward(self) -> None:
        """
        Calculates and updates the shoulder feedforward value based on the positions of the
        shoulder and elbow

        """
        self.update_motor_positions()

        if self.elbow_current_pos is not None and self.shoulder_current_pos is not None:

            if (
                self.shoulder_current_pos > math.pi
                and self.shoulder_current_pos - self.elbow_current_pos > math.pi
            ):
                self.shoulder_feedforward = (
                    19.53 * (math.cos(self.shoulder_current_pos))
                ) + 0.15 * (math.cos(self.shoulder_current_pos - self.elbow_current_pos))

            else:
                self.shoulder_feedforward = (
                    19.53 * (math.cos(self.shoulder_current_pos))
                ) - 0.15 * (math.cos(self.shoulder_current_pos - self.elbow_current_pos))

            self._ros_node.get_logger().info("feedforward value: " + str(self.shoulder_feedforward))
            self._ros_node.get_logger().info(
                "shoulder current (radians): " + str(self.shoulder_current_pos)
            )
            self._ros_node.get_logger().info(
                "elbow current (radians): " + str(self.elbow_current_pos)
            )
        else:
            # if one or both motors don't have a valid position, give 0 to feedforward
            self.shoulder_feedforward = 0.0

    def stopShoulder(self) -> None:
        """
        Runs shoulder motor at velocity 0 with the current feedforward value.
        """
        self.calc_shoulder_feedforward()

        self._interface.runMotor(
            MotorConfigs.ARM_SHOULDER_MOTOR,
            MoteusRunSettings(
                velocity=0.0,
                feedforward_torque=-self.shoulder_feedforward,
                set_stop=False,
            ),
        )

    def runElbowVelocity(self, target_velocity: float) -> None:
        """
        Runs the specified motor to reach the specified position.

        Parameters
        -------
        target_velocity: float
            The target velocity in revolutions per second.
        """
        self.shoulder_position = False

        self._interface.runMotor(
            MotorConfigs.ARM_ELBOW_MOTOR,
            MoteusRunSettings(
                velocity=target_velocity,
                feedforward_torque=0.0,
                set_stop=False,
            ),
        )

    def runShoulderVelocity(self, target_velocity: float) -> None:
        """
        Runs the specified motor to reach the specified position.

        Parameters
        -------
        target_velocity: float
            The target velocity in revolutions per second.
        """

        self._interface.runMotor(
            MotorConfigs.ARM_SHOULDER_MOTOR,
            MoteusRunSettings(
                velocity=target_velocity,
                feedforward_torque=-self.shoulder_feedforward,
                set_stop=False,
            ),
        )

    def setShoulderTargetPosition(self, target_position: float) -> None:
        self.shoulder_target_position = target_position
        self.shoulder_position = True
        self.shoulder_moving_to_position = True

    def runShoulderToPosition(self) -> None:
        """
        This gets called on a timer. Checks if we're trying to move to a position and then
        runs shoulder motor towards position with updated feedforward value
        """
        if self.shoulder_moving_to_position and self.shoulder_position:
            self.calc_shoulder_feedforward()

            if math.abs(self.shoulder_current_pos - self.shoulder_target_position) > 0.05:
                self._interface.runMotor(
                    MotorConfigs.ARM_SHOULDER_MOTOR,
                    MoteusRunSettings(
                        position=self.shoulder_target_position,
                        feedforward_torque=self.shoulder_feedforward,
                        set_stop=False,
                    ),
                )
            else:
                self.stopShoulder()
                self.shoulder_moving_to_position = False
