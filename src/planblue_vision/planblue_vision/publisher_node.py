import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from datetime import datetime

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        self.buffer_size = 10
        self.topic_name = 'camera/image_raw'
        self.publisher_ = self.create_publisher(Image, self.topic_name,self.buffer_size)
        self.bridge = CvBridge()
        self.frame_id = 0
        self.fps = 10  # Target frames per second

        # Trying to use webcam first, if not we will genrate synthetic frames
        self.cap = cv2.VideoCapture(0)
        if self.cap.isOpened():
            self.source = 'webcam'
            self.get_logger().info('Publisher started - Source: Webcam')
        else:
            self.cap.release()
            self.cap = None
            self.source = 'synthetic'
            self.get_logger().info('Publisher started - Source: Synthetic (webcam not available)')

        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

    #Function to generate synthetic frames with a rainbow effect
    def generate_synthetic_frame(self):
        hue_shift = (self.frame_id * 2) % 180
        hsv = np.zeros((480, 640, 3), dtype=np.uint8)
        hsv[:, :, 0] = hue_shift
        hsv[:, :, 1] = 255
        hsv[:, :, 2] = np.linspace(80, 255, 640).astype(np.uint8)
        return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    def timer_callback(self):
        self.frame_id += 1
        current_timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

        # Trying to Get Feed from Webcam
        if self.source == 'webcam':
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn(f'Frame Id: {self.frame_id} | Time: {current_timestamp} | Webcam read failed, skipping')
                return
        # Using self genrated images as webcam is not available
        else:
            frame = self.generate_synthetic_frame()

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
            self.get_logger().info(f'Frame Id: {self.frame_id} | Time: {current_timestamp} | Source: {self.source} | Status: RUNNING')
        except Exception as e:
            self.get_logger().error(f'Frame Id: {self.frame_id} | Time: {current_timestamp} | Error: {str(e)}')

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()
def main(args=None):
    rclpy.init()
    image_publisher = ImagePublisher()

    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        image_publisher.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()