import time

from rclpy.node import Node
from std_msgs.msg import Float32

from lib.configs import MotorConfigs
from lib.interface.arm_interface import ArmInterface
from lib.interface.robot_interface import RobotInterface


class IndividualControlVel:
    WRIST_VEL = 0.3
    VEL = 0.2
    SHOULDER_VEL = 0.3
    TURNTABLE_VEL = 0.2

    def __init__(self, ros_node: Node, interface: RobotInterface, arm_interface: ArmInterface):
        self._ros_node = ros_node
        self.bot_interface = interface
        self.arm_interface = arm_interface
        self.arm_interface.update_motor_positions()

        self.shoulder_vel = 0.0

        self.elbow_vel = 0.0

        self.can_send = False

        self.last_shoulder_sub_time = 0.0
        self.last_elbow_sub_time = 0.0

        # Left wrist motor
        self.left_wrist_motor_sub = ros_node.create_subscription(
            Float32, "left_wrist_cw", self.leftWristCW, 10
        )
        self.left_wrist_motor_sub = ros_node.create_subscription(
            Float32, "left_wrist_ccw", self.leftWristCCW, 10
        )

        # Right wrist motor
        self.left_wrist_motor_sub = ros_node.create_subscription(
            Float32, "right_wrist_cw", self.rightWristCW, 10
        )
        self.left_wrist_motor_sub = ros_node.create_subscription(
            Float32, "right_wrist_ccw", self.rightWristCCW, 10
        )

        # Elbow motor
        self.elbow_up_sub = ros_node.create_subscription(Float32, "elbow_up", self.elbowUp, 10)

        self.elbow_down_sub = ros_node.create_subscription(
            Float32, "elbow_down", self.elbowDown, 10
        )

        # Shoulder motor
        self.shoulder_up_sub = ros_node.create_subscription(
            Float32, "shoulder_up", self.shoulderUp, 10
        )

        self.elboshoulder_down_sub = ros_node.create_subscription(
            Float32, "shoulder_down", self.shoulderDown, 10
        )

        # Turn table
        self.turntable_clockwise_sub = ros_node.create_subscription(
            Float32, "turntable_cw", self.turntableCW, 10
        )

        self.turntable_counter_clockwise_sub = ros_node.create_subscription(
            Float32, "turntable_ccw", self.turntableCCW, 10
        )

        # create timer for calling shoulder stationary feedforward method
        ros_node.create_timer(0.1, self.handle_shoulder)

        ros_node.create_timer(0.1, self.handle_elbow)

        ros_node.create_timer(0.1, self.shoulder_input)

        ros_node.create_timer(0.1, self.elbow_input)

    def handle_shoulder(self) -> None:
        if self.can_send:
            self._ros_node.get_logger().info("-------------handle_shoulder----------------")
            self.arm_interface.runShoulderVelocity(self.shoulder_vel)
            self._ros_node.get_logger().info(
                "running shoulder @ velocity " + str(self.shoulder_vel)
            )

    def handle_elbow(self) -> None:
        if self.can_send:
            self._ros_node.get_logger().info("-------------handle_elbow----------------")
            self.bot_interface.runMotorSpeed(MotorConfigs.ARM_ELBOW_MOTOR, self.elbow_vel)
            self._ros_node.get_logger().info("running elbow @ velocity " + str(self.elbow_vel))

    def shoulder_input(self) -> None:
        self._ros_node.get_logger().info("-------------shoulder_input----------------")
        if time.time() - self.last_shoulder_sub_time > 0.1:
            self.shoulder_vel = 0.0
            self._ros_node.get_logger().info("setting shoulder vel to 0")

    def elbow_input(self) -> None:
        self._ros_node.get_logger().info("-------------elbow_input----------------")
        if time.time() - self.last_elbow_sub_time > 0.1:
            self.elbow_vel = 0.0
            self._ros_node.get_logger().info("setting elbow vel to 0")

    def leftWristCW(self, msg: Float32) -> None:
        if not self.can_send:
            return

        joystick = msg.data

        if joystick > 0:
            self._ros_node.get_logger().info("Left Wrist CW" + str(joystick))
            self.bot_interface.runMotorSpeed(MotorConfigs.ARM_LEFT_WRIST_MOTOR, self.WRIST_VEL)

        else:
            self._ros_node.get_logger().info("Left Wrist STOP")
            self.bot_interface.stopMotor(MotorConfigs.ARM_LEFT_WRIST_MOTOR)

    def leftWristCCW(self, msg: Float32) -> None:
        if not self.can_send:
            return

        joystick = msg.data

        if joystick > 0:
            self._ros_node.get_logger().info("Left Wrist CCW" + str(joystick))
            self.bot_interface.runMotorSpeed(MotorConfigs.ARM_LEFT_WRIST_MOTOR, -self.WRIST_VEL)

        else:
            self._ros_node.get_logger().info("Left Wrist STOP")
            self.bot_interface.stopMotor(MotorConfigs.ARM_LEFT_WRIST_MOTOR)

    def rightWristCW(self, msg: Float32) -> None:
        if not self.can_send:
            return

        joystick = msg.data

        if joystick > 0:
            self._ros_node.get_logger().info("Right Wrist CW" + str(joystick))
            self.bot_interface.runMotorSpeed(MotorConfigs.ARM_RIGHT_WRIST_MOTOR, -self.WRIST_VEL)

        else:
            self._ros_node.get_logger().info("Right Wrist STOP")
            self.bot_interface.stopMotor(MotorConfigs.ARM_RIGHT_WRIST_MOTOR)

    def rightWristCCW(self, msg: Float32) -> None:
        if not self.can_send:
            return

        joystick = msg.data

        if joystick > 0:
            self._ros_node.get_logger().info("Right Wrist CCW" + str(joystick))
            self.bot_interface.runMotorSpeed(MotorConfigs.ARM_RIGHT_WRIST_MOTOR, -self.WRIST_VEL)

        else:
            self._ros_node.get_logger().info("Right Wrist STOP")
            self.bot_interface.stopMotor(MotorConfigs.ARM_RIGHT_WRIST_MOTOR)

    def elbowUp(self, msg: Float32) -> None:
        if not self.can_send:
            return

        data = msg.data

        self._ros_node.get_logger().info("-------------elbowUp----------------")

        if abs(data) != 0:
            self._ros_node.get_logger().info("Elbow velocity = " + str(data * self.VEL))
            self.elbow_vel = data * self.VEL

        else:
            self._ros_node.get_logger().info("Controller input 0")
            self.elbow_vel = 0.0

    def elbowDown(self, msg: Float32) -> None:
        """if not self.can_send:
            return

        data = msg.data

        if data > 0:
            self._ros_node.get_logger().info("Elbow down" + str(data))
            self.bot_interface.runMotorSpeed(MotorConfigs.ARM_ELBOW_MOTOR, self.VEL)

        else:
            self._ros_node.get_logger().info("Elbow down STOP")
            self.bot_interface.stopMotor(MotorConfigs.ARM_ELBOW_MOTOR)"""
        return

    def shoulderUp(self, msg: Float32) -> None:
        if not self.can_send:
            return

        data = msg.data
        self.last_shoulder_sub_time = time.time()
        self._ros_node.get_logger().info("-------------shoulderUp----------------")
        if data != 0:
            self._ros_node.get_logger().info("Shoulder vel = " + str(-data * self.SHOULDER_VEL))
            self.shoulder_vel = -self.SHOULDER_VEL * data

        else:
            self._ros_node.get_logger().info("Controller input 0")
            self.shoulder_vel = 0.0

    def shoulderDown(self, msg: Float32) -> None:
        """if not self.can_send:
            return

        data = msg.data

        if data > 0:
            self._ros_node.get_logger().info("Shoulder down" + str(data))
            self.arm_interface.runArmShoulderMotorVelocity(
                MotorConfigs.ARM_SHOULDER_MOTOR, self.VEL
            )

        else:
            self._ros_node.get_logger().info("Shoulder STOP")
            self.bot_interface.stopMotor(MotorConfigs.ARM_SHOULDER_MOTOR)"""
        return

    def turntableCW(self, msg: Float32) -> None:
        if not self.can_send:
            return

        data = msg.data

        if data != 0:
            self._ros_node.get_logger().info("Turntable" + str(data * self.TURNTABLE_VEL))
            self.bot_interface.runMotorSpeed(
                MotorConfigs.ARM_TURNTABLE_MOTOR, self.TURNTABLE_VEL * data
            )

        else:
            self._ros_node.get_logger().info("Turntable STOP")
            self.bot_interface.stopMotor(MotorConfigs.ARM_TURNTABLE_MOTOR)

    def turntableCCW(self, msg: Float32) -> None:

        return

        """if not self.can_send:
            return

        data = msg.data

        if data > 0:
            self._ros_node.get_logger().info("Turntable counter clock wise")
            self.bot_interface.runMotorSpeed(MotorConfigs.ARM_TURNTABLE_MOTOR, -self.VEL)

        else:
            self._ros_node.get_logger().info("Turntable STOP")
            self.bot_interface.stopMotor(MotorConfigs.ARM_TURNTABLE_MOTOR)"""
