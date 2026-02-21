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
        
        # Create the publisher. Queue size of 10 is standard for video.
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        
        # Set publish rate to 10 FPS (0.1 seconds per frame)
        timer_period = 0.1 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.bridge = CvBridge()
        self.frame_id = 0
        
        # Variables for our synthetic moving image (a bouncing ball)
        self.width, self.height = 640, 480
        self.x, self.y = self.width // 2, self.height // 2
        self.dx, self.dy = 15, 15

        # Requirement: Log when publishing starts
        self.get_logger().info('Publishing started: Node A is active.')

    def timer_callback(self):
        # 1. Generate a synthetic image (black background, green circle)
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        cv2.circle(img, (self.x, self.y), 40, (0, 255, 0), -1)

        # Update circle position for the next frame
        self.x += self.dx
        self.y += self.dy
        if self.x <= 40 or self.x >= self.width - 40: self.dx *= -1
        if self.y <= 40 or self.y >= self.height - 40: self.dy *= -1

        # 2. Gather logging metrics
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        self.frame_id += 1

        try:
            # 3. Convert OpenCV image to ROS 2 Image message
            msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            
            # 4. Publish the message
            self.publisher_.publish(msg)
            
            # Requirement: Log frame number, timestamp, and success status
            self.get_logger().info(f"Frame: {self.frame_id} | Time: {current_time} | Status: Sent successfully")
            
        except Exception as e:
            # Requirement: Log any errors
            self.get_logger().error(f"Frame: {self.frame_id} | Time: {current_time} | Status: Failed to send ({str(e)})")

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()