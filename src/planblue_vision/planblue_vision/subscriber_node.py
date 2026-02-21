import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import json
from datetime import datetime

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # 1. Create the subscription to Node A's topic [cite: 286]
        self.subscription = self.create_subscription(
            Image,
            'video_frames',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.bridge = CvBridge()
        self.frame_count = 0
        self.metadata_list = []

        # 2. Dynamic Path Management (Crucial for the assessment)
        # This creates paths relative to where the user runs the node [cite: 305, 306, 307]
        self.output_dir = os.path.join(os.getcwd(), 'output')
        self.images_dir = os.path.join(self.output_dir, 'images')
        self.metadata_file = os.path.join(self.output_dir, 'metadata.json')

        # Ensure directories exist so cv2.imwrite doesn't fail
        os.makedirs(self.images_dir, exist_ok=True)

        # Requirement: Log when subscribing starts [cite: 296]
        self.get_logger().info('Subscribing started: Node B is active and waiting for frames.')

    def listener_callback(self, data):
        self.frame_count += 1
        
        # Get timestamps for display and filenames [cite: 288, 298]
        display_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        file_time = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]

        try:
            # 1. Convert ROS Image to OpenCV Image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # 2. Overlay the timestamp as text onto the image [cite: 288]
            # cv2.putText(image, text, position, font, font_scale, color, thickness)
            cv2.putText(cv_image, f"Timestamp: {display_time}", (20, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

            # 3. Save the updated image to disk [cite: 289]
            filename = f"frame_{self.frame_count:04d}_{file_time}.jpg"
            filepath = os.path.join(self.images_dir, filename)
            cv2.imwrite(filepath, cv_image)

            # 4. Save metadata [cite: 290, 291, 292, 293, 294]
            meta_entry = {
                "frame_id": self.frame_count,
                "saved_file": filename,
                "timestamp": display_time
            }
            self.metadata_list.append(meta_entry)

            # Write the updated JSON list to disk
            with open(self.metadata_file, 'w') as f:
                json.dump(self.metadata_list, f, indent=4)

            # 5. Log Success [cite: 297, 298, 299]
            self.get_logger().info(f"Frame: {self.frame_count} | Time: {display_time} | Saved to: {filepath}")

        except Exception as e:
            # Requirement: Log any errors 
            self.get_logger().error(f"Frame: {self.frame_count} | Error processing frame: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()