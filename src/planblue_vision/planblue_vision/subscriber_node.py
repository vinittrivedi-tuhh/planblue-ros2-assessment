import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import json
from datetime import datetime
import numpy as np
from typing import List, Tuple


class ImageSubscriber(Node):
    def __init__(self, buffer_size: int, topic_name: str) -> None:
        super().__init__("image_subscriber")

        self.subscription = self.create_subscription(
            Image, topic_name, self.listener_callback, buffer_size
        )
        self.bridge: CvBridge = CvBridge()
        self.frame_count: int = 0
        self.metadata_list: List[dict] = []
        self.dropped_frames: int = 0  # Tracking frames we couldn't save

        # Using relative paths to make it generic
        self.output_dir = os.path.join(os.getcwd(), "output")
        self.images_dir = os.path.join(self.output_dir, "images")
        self.metadata_file = os.path.join(self.output_dir, "metadata.json")

        os.makedirs(self.images_dir, exist_ok=True)
        self.get_logger().info("Subscriber started: waiting for image frames")

    def listener_callback(self, data: Image) -> None:
        self.frame_count += 1
        current_time: str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

        try:
            received_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            received_image = self.put_text_on_image(
                received_image, current_time, 0.7, (255, 255, 255)
            )
            filename: str = self.save_image(received_image, current_time)
            self.save_metadata(self.frame_count, filename, current_time)
            self.log_info(
                self.frame_count, current_time, "received", "saved successfully"
            )
        except Exception as e:
            self.dropped_frames += 1
            self.log_error(self.frame_count, current_time, str(e), self.dropped_frames)

    # function to write the required text on the image
    def put_text_on_image(
        self,
        image: np.ndarray,
        timestamp: str,
        font_size: float,
        color: Tuple[int, int, int],
    ) -> np.ndarray:
        cv2.putText(
            image,
            f"Timestamp: {timestamp}",
            (20, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_size,
            color,
            2,
        )
        return image

    # function to save file with unique name
    def save_image(self, image: np.ndarray, timestamp: str) -> str:
        file_time: str = timestamp.replace(" ", "_").replace(":", "-")
        filename: str = f"frame_{self.frame_count:04d}_{file_time}.jpg"
        filepath: str = os.path.join(self.images_dir, filename)

        # Checking if file is saved or not
        success: bool = cv2.imwrite(filepath, image)

        if not success:
            raise RuntimeError(f"OpenCV failed to write image to disk: {filepath}")

        return filename

    # def to save meta data in list
    def save_metadata(self, frame_id: int, filename: str, timestamp: str) -> None:
        meta_entry: dict = {
            "frame_id": frame_id,
            "saved_file": filename,
            "timestamp": timestamp,
        }
        self.metadata_list.append(meta_entry)

    # Function to save meta data in json file
    def write_metadata_to_json(self) -> None:
        with open(self.metadata_file, "w") as f:
            json.dump(self.metadata_list, f, indent=4)

    def destroy_node(self):
        self.write_metadata_to_json()
        super().destroy_node()

    # function to write info log
    def log_info(self, frame_id: int, timestamp: str, source: str, status: str) -> None:
        self.get_logger().info(
            f"Frame Id: {frame_id} | Time: {timestamp} | Source: {source} | Status: {status}"
        )

    # function to write error log
    def log_error(
        self, frame_id: int, timestamp: str, error_message: str, dropped_frames: int
    ) -> None:
        self.get_logger().error(
            f"Frame Id: {frame_id} | Time: {timestamp} | Error: {error_message} , Total Dropped Frames: {dropped_frames}"
        )

    # function to write warning log
    def log_warning(self, frame_id: int, timestamp: str, warning_message: str) -> None:
        self.get_logger().warn(
            f"Frame Id: {frame_id} | Time: {timestamp} | Warning: {warning_message}"
        )


def main(args=None) -> None:
    rclpy.init()
    image_subscriber = ImageSubscriber(10, "camera/image_raw")
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        image_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
