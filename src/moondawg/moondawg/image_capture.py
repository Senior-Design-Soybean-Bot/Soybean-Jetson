import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import cv2
import os
import datetime

class ImageCapture(Node):
    def __init__(self):
        super().__init__('image_capture_node')
        self.subscription = self.create_subscription(
            NavSatFix,
            'gps/fix',
            self.listener_callback,
            10)
        self.gps_data = None
        self.get_logger().info('Image Capture Node has started')

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera')
            raise RuntimeError('Failed to open camera')

        self.timer_period = 5.0 # seconds
        self.timer = self.create_timer(self.timer_period, self.capture_image)

        self.image_dir = 'captured_images'
        os.makedirs(self.image_dir, exist_ok=True)

    def listener_callback(self, msg):
        self.gps_data = msg.data
        self.get_logger().info(f'Recieved GPS data: Lat {msg.latitude}, Lon {msg.longitude}')

    def capture_image(self):
        ret, frame = self.cap.read()

        if ret:
            current_date = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f'no_gps_coordinates_{current_date}.jpg'

            if self.gps_data:
                gps_str = f'lat_{self.gps_data.latitude: .6f}_lon_{self.gps_data.longitude: .6f}'
                filename = f'image_{gps_str}_{current_date}.jpg'

            filepath = os.path.join(self.image_dir, filename)

            cv2.imwrite(filepath, frame)
            self.get_logger().info(f'Image saved as {filepath}')
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

if __name__ == '__main__':
    main()