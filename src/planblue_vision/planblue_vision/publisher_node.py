import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from datetime import datetime


class ImagePublisher(Node):
    def __init__(
        self,
        buffer_size: int,
        topic_name: str,
        fps: int,
        width: int,
        height: int,
    ) -> None:
        super().__init__("image_publisher")
        self.width: int = width
        self.height: int = height
        self.publisher_ = self.create_publisher(Image, topic_name, buffer_size)
        self.bridge: CvBridge = CvBridge()
        self.frame_id: int = 0
        self.fps: int = fps  # Target frames per second

        self.start_camera_feed()

        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # Function to start camera feed and check if it is working or not
    def start_camera_feed(self) -> None:
        self.cap = cv2.VideoCapture(0)
        if self.cap.isOpened():
            self.source: str = "webcam"
            self.get_logger().info("Camera feed started successfully")
        else:
            self.cap.release()
            self.cap = None
            self.source = "synthetic"
            self.get_logger().warn(
                "Failed to start camera feed, switching to synthetic source"
            )

    # Function to generate synthetic frames with a rainbow effect
    def generate_synthetic_frame(self, width: int, height: int) -> np.ndarray:
        hue_shift: int = (self.frame_id * 2) % 180
        hsv: np.ndarray = np.zeros((height, width, 3), dtype=np.uint8)
        hsv[:, :, 0] = hue_shift
        hsv[:, :, 1] = 255
        hsv[:, :, 2] = np.linspace(80, 255, width).astype(np.uint8)
        return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    # function to get frames
    def get_frame(self, timestamp: str) -> np.ndarray:
        if self.source == "webcam" and self.cap is not None:
            ret, frame = self.cap.read()  # type: ignore[union-attr]
            if ret:
                return frame
            else:
                self.log_warning(
                    self.frame_id,
                    timestamp,
                    "Webcam read failed, switching to synthetic",
                )
                self.source = "synthetic"
        return self.generate_synthetic_frame(self.width, self.height)

    # Function to publish the frames and log the status of the publisher
    def publish_frame(self, frame: np.ndarray, timestamp: str) -> None:
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
            self.log_info(self.frame_id, timestamp, self.source, "RUNNING")
        except Exception as e:
            self.log_error(self.frame_id, timestamp, str(e))

    def timer_callback(self) -> None:
        self.frame_id += 1
        current_timestamp: str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

        frame: np.ndarray = self.get_frame(current_timestamp)

        self.publish_frame(frame, current_timestamp)

    # Function to release the camera feed when the node is destroyed
    def destroy_node(self) -> None:
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()

    # function to write info log
    def log_info(self, frame_id: int, timestamp: str, source: str, status: str) -> None:
        self.get_logger().info(
            f"Frame Id: {frame_id} | Time: {timestamp} | Source: {source} | Status: {status}"
        )

    # function to write error log
    def log_error(self, frame_id: int, timestamp: str, error_message: str) -> None:
        self.get_logger().error(
            f"Frame Id: {frame_id} | Time: {timestamp} | Error: {error_message}"
        )

    # function to write warning log
    def log_warning(self, frame_id: int, timestamp: str, warning_message: str) -> None:
        self.get_logger().warn(
            f"Frame Id: {frame_id} | Time: {timestamp} | Warning: {warning_message}"
        )


def main(args=None):
    rclpy.init()
    image_publisher = ImagePublisher(10, "camera/image_raw", 10, 640, 480)
    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        image_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
