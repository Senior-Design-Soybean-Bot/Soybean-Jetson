import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import os

class ImageCapture(Node):
    def __init__(self):
        super().__init__('image_capture_node')
        self.subscription = self.create_subscription(
            String,
            'gps_coordinates',
            self.listener_callback,
            10)
        self.subscription
        self.gps_data = None
        self.get_logger().inf('Image Capture Node has started')

        self.cap = cv2.VideoCapture(0)

        self.timer_period = 5.0 # seconds
        self.timer = self.create_timer(self.timer_period, self.capture_image)

        self.image_dir = 'captured_images'
        os.makedirs(self.image_dir, exist_ok=True)

    def listener_callback(self, msg):
        self.gps_data = msg.data
        self.get_logger().info(f'Recieved GPS data: {self.gps_data}')

    def capture_image(self):
        if not self.cap.isOpened():
            self.get_logger().error('Camera not opened')
            return
        ret, frame = self.cap.read()

        if ret:
            filename = 'no_gps_coordinates'

            if self.gps_data:
                gps_str = self.gps_data.replace(' ', '_').repalce(',', '')
                filename = f'image_{gps_str}.jpg'

            filepath = os.path.join(self.image_dir, filename)

            cv2.imwrite(filepath, frame)
            self.get_logger().infor(f'Image saved as {filepath}')
        else:
            self.get_logger().error('Failed to capture image.')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageCapture()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__maine__':
    main()