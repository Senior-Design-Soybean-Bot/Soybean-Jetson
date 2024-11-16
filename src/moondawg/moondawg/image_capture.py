import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from cv_bridge import CvBridge
import cv2
from collections import deque
from moondawg.msg import LastCapturedImages

class ImageCapture(Node):
    def __init__(self):
        super().__init__('image_capture_node')
        self.cv_bridge = CvBridge()
        self.gps_data = None
        self.latest_image = None
        self.last_images = deque(maxlen=4)
        self.last_filenames = deque(maxlen=4)

        self.create_subscription(Image, '/image', self.image_callback, 10)
        self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)

        self.timer = self.create_timer(5.0, self.save_image)  # Save image every 5 seconds

    def image_callback(self, msg):
        self.latest_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

    def gps_callback(self, msg):
        self.gps_data = msg

    def save_image(self):
        if self.latest_image is not None:
            timestamp = self.get_clock().now().to_msg()
            if self.gps_data:
                filename = f"image_{timestamp.sec}_lat{self.gps_data.latitude:.6f}_lon{self.gps_data.longitude:.6f}.jpg"
            else:
                filename = f"image_{timestamp.sec}_no_gps.jpg"

            cv2.imwrite(filename, self.latest_image)
            self.get_logger().info(f'Saved image: {filename}')

            self.last_images.appendLeft(self.latest_image)
            self.last_filenames.appendleft(filename)

            self.publish_last_images()

    def publish_last_images(self):
        msg = LastCapturedImages()
        msg.images = []
        msg.filenames = list(self.last_filenames)

        for img in self.last_images:
            img_msg = self.cv_bridge.cv2_to_imgmsg(img, encoding='bgr8')
            msg.images.append(img_msg)

        self.last_images_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageCapture()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
