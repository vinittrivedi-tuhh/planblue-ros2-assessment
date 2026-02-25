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

        self.buffer_size = 10
        self.topic_name = 'camera/image_raw'
        self.subscription = self.create_subscription(Image,self.topic_name,self.listener_callback,self.buffer_size)
        self.bridge = CvBridge()
        self.frame_count = 0
        self.metadata_list = []
        self.dropped_frames = 0  # Tracking frames we couldn't save

        # Using relative paths to make it generic
        self.output_dir = os.path.join(os.getcwd(), 'output')
        self.images_dir = os.path.join(self.output_dir, 'images')
        self.metadata_file = os.path.join(self.output_dir, 'metadata.json')

        os.makedirs(self.images_dir, exist_ok=True)
        self.get_logger().info('Subscriber started: waiting for image frames')

    def listener_callback(self, data):
        self.frame_count += 1
        current_time = datetime.now()
        # Variables we will use for overwriting the image received
        display_time = current_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        file_time = current_time.strftime("%Y%m%d_%H%M%S_%f")[:-3]

        try:
            received_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # Writing the required text on the image
            cv2.putText(received_image, f"Timestamp: {display_time}", (20, 50),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            #Saving the image with unique name
            filename = f"frame_{self.frame_count:04d}_{file_time}.jpg"
            filepath = os.path.join(self.images_dir, filename)
            cv2.imwrite(filepath, received_image)

            # Saving the metadata as required
            meta_entry = {
                "frame_id": self.frame_count,
                "saved_file": filename,
                "timestamp": display_time
            }
            self.metadata_list.append(meta_entry)

            #Creating JSON file to save our metadata
            with open(self.metadata_file, 'w') as f:
                json.dump(self.metadata_list, f, indent=4)

            self.get_logger().info(f"Frame Id: {self.frame_count} | TimeStamp: {display_time} | Path: {filepath}")

        except Exception as e:
            self.dropped_frames += 1
            self.get_logger().error(f"Frame Id: {self.frame_count} | TimeStamp: {display_time} | Error: {str(e)} | Total Dropped Frames: {self.dropped_frames}")
def main(args=None):
    rclpy.init()
    image_subscriber = ImageSubscriber()
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        image_subscriber.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()



