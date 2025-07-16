import os
import cv2  # OpenCV library
import rclpy  # Python Client Library for ROS 2
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage  # Image is the message type

ZED_VENDOR_ID = "2b03"
ZED_PRODUCT_ID = "f880"


def isZedCamera(video_index: int) -> bool:
    """
    Checks if the /dev/video{video_index} is a ZED camera by reading Vendor/Product IDs in sysfs.
    Returns True if it's the ZED, False otherwise.
    """
    # Path to the sysfs directory for this video device
    dev_path = f"/sys/class/video4linux/video{video_index}/device"

    if not os.path.exists(dev_path):
        return False

    # We walk upwards to find "usb" device info
    # e.g. /sys/class/video4linux/video{X}/device/../..
    # Might have multiple symlinks so we climb up until we find "idVendor" or we run out of path.
    path = dev_path
    for _ in range(5):  # limit climbing to 5 levels
        # Check if idVendor/idProduct exist here
        vendor_file = os.path.join(path, "idVendor")
        product_file = os.path.join(path, "idProduct")

        if os.path.isfile(vendor_file) and os.path.isfile(product_file):
            with open(vendor_file, "r") as vf:
                vendor_id = vf.read().strip()
            with open(product_file, "r") as pf:
                product_id = pf.read().strip()

            if vendor_id.lower() == ZED_VENDOR_ID and product_id.lower() == ZED_PRODUCT_ID:
                return True

        # Move one directory up
        path = os.path.join(path, "..")

    return False


def getCameras() -> list[int]:
    """
    Returns a list of working camera IDs for all cameras EXCEPT the ZED 2i.
    """

    non_working_ports = 0
    dev_port = 0
    working_ports = []

    max_checks = 10  # total number of /dev/video ports we'll probe

    while non_working_ports < 6 and dev_port < max_checks:
        # Check if this is a ZED
        if isZedCamera(dev_port):
            print(f"Skipping ZED device at /dev/video{dev_port}")
            dev_port += 1
            continue

        camera = cv2.VideoCapture(dev_port)
        if not camera.isOpened():
            non_working_ports += 1
        else:
            # Try reading a frame to confirm it’s valid
            is_reading, _ = camera.read()
            _ = camera.get(3)
            _ = camera.get(4)
            if is_reading:
                working_ports.append(dev_port)

        camera.release()
        dev_port += 1

    return working_ports


class RosCamera(Node):

    def __init__(self, topicName: str, camera: int):
        super().__init__("ros_camera")
        self.get_logger().info("Launching ros_camera node")

        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        self._publisher = self.create_publisher(CompressedImage, topicName, 10)
        self.get_logger().info("Created Publisher " + topicName)

        # We will publish a message every 0.1 seconds
        # timer_period = 0.1  # seconds
        timer_period = 0.1  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.publishCameraFrame)

        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        self.cap = cv2.VideoCapture(camera)
        self.get_logger().info(
            "Using video ID: " + str(camera) + ", ON: " + str(self.cap.isOpened())
        )

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Declare parameters for each 'ros_camera' node thread
        self.declare_parameter("cameraNumber", camera)

    def publishCameraFrame(self) -> None:
        """
        Callback function publishes a frame captured from a camera to /video_framesX (X is specific
        camera ID) every 0.1 seconds
        """

        # Capture frame-by-frame
        # This method returns True/False as well as the video frame.
        ret, frame = self.cap.read()

        if ret:
            # Publish the image.
            self._publisher.publish(self.br.cv2_to_compressed_imgmsg(frame))


def main(args: list[str] | None = None) -> None:
    """
    The entry point of the node.
    """

    rclpy.init(args=args)
    try:
        # We need an executor because running .spin() is a blocking function.
        # using the MultiThreadedExecutor, we can control multiple nodes
        executor = MultiThreadedExecutor()
        nodes = []
        camera_num = 0

        for camera_id in getCameras():
            node = RosCamera("video_frames" + str(camera_num), camera_id)
            nodes.append(node)
            executor.add_node(node)
            camera_num += 1

        try:
            executor.spin()
        finally:
            executor.shutdown()
            for node in nodes:
                node.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
