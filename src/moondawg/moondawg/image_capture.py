import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, NavSatFix
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class ImageCapture(Node):
    def __init__(self):
        super().__init__('image_capture_node')
        self.cv_bridge = CvBridge()
        self.gps_data = None

        self.save_dir = '/home/soybean/Pictures'
        os.makedirs(self.save_dir, exist_ok=True)

        # Create publisher for compressed images
#        self.image_publisher = self.create_publisher(
#            CompressedImage,
#            '/camera/image/compressed',
#            10
#        )

        self.camera_indices = [0, 2, 4, 6]
        self.cameras = self.initialize_cameras()

        # Configure GPS subscription with reliable QoS
        gps_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # Changed from BEST_EFFORT
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10  # Increased buffer size
        )

        # Create GPS subscription with debug logging
        self.create_subscription(
            NavSatFix,
            '/fix',  # Verify this topic matches your GPS node's output
            self.gps_callback,
            qos_profile=gps_qos
        )
        self.get_logger().info('GPS subscription created')

        self.timer = self.create_timer(5.0, self.capture_and_publish)
        self.get_logger().info('Image Capture Node initialized')

    def initialize_cameras(self):
        """Initialize cameras with v4l2 backend"""
        cameras = {}
        for i in self.camera_indices:
            try:
                # Use v4l2 backend explicitly
                cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
                if cap.isOpened():
                    # Set camera properties
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                    cap.set(cv2.CAP_PROP_FPS, 30)

                    # Verify settings were applied
                    if cap.get(cv2.CAP_PROP_FRAME_WIDTH) == 1280 and \
                       cap.get(cv2.CAP_PROP_FRAME_HEIGHT) == 720:
                        cameras[i] = cap
                        self.get_logger().info(f'Initialized camera {i}')
                    else:
                        self.get_logger().warn(f'Failed to set properties for camera {i}')
                        cap.release()
                else:
                    self.get_logger().warn(f'Failed to open camera {i}')
                    if cap:
                        cap.release()
            except Exception as e:
                self.get_logger().error(f'Error initializing camera {i}: {str(e)}')

        if not cameras:
            self.get_logger().error('No cameras initialized')
        return cameras

    def capture_and_publish(self):
        """Capture and publish images from all cameras"""
        timestamp = self.get_clock().now().to_msg()

        for camera_id, cap in self.cameras.items():
            try:
                if not cap.isOpened():
                    # Try to reinitialize camera
                    cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
                    if not cap.isOpened():
                        self.get_logger().warn(f'Camera {camera_id} is not opened')
                        continue

                ret, frame = cap.read()
                if not ret or frame is None:
                    self.get_logger().warn(f'Failed to capture image from camera {camera_id}')
                    continue

                # Create compressed image message
#                msg = CompressedImage()
#                msg.header.stamp = timestamp
#                msg.format = 'jpeg'

                # Encode image with JPEG compression
#                _, jpeg_data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
#                msg.data = np.array(jpeg_data).tobytes()

                # Publish compressed image
#                self.image_publisher.publish(msg)

                # Save image
                self.save_image(frame, camera_id, timestamp)

            except Exception as e:
                self.get_logger().error(f'Error processing camera {camera_id}: {str(e)}')

    def save_image(self, image, camera_id, timestamp):
        """Save image with GPS data if available"""
        try:
            valid_gps = self.gps_data and (
                self.gps_data.latitude != 0.0 or 
                self.gps_data.longitude != 0.0
            )

            if valid_gps:
                filename = (f"cam{camera_id}_image_{timestamp.sec}_"
                          f"lat{self.gps_data.latitude:.6f}_"
                          f"lon{self.gps_data.longitude:.6f}.jpg")
            else:
                filename = f"cam{camera_id}_image_{timestamp.sec}_no_gps.jpg"

            full_path = os.path.join(self.save_dir, filename)
            cv2.imwrite(full_path, image)
            self.get_logger().debug(f'Saved image: {filename}')

        except Exception as e:
            self.get_logger().error(f'Error saving image: {str(e)}')

    def gps_callback(self, msg):
        """Handle incoming GPS messages"""
        try:
            # Log all GPS messages for debugging
#            self.get_logger().info(
#                f'GPS message received: lat={msg.latitude:.6f}, '
#                f'lon={msg.longitude:.6f}, alt={msg.altitude:.2f}'
#            )

            # Store GPS data if valid
            if msg.latitude != 0.0 or msg.longitude != 0.0:
                self.gps_data = msg
#                self.get_logger().info(
#                    f'Valid GPS data stored: lat={msg.latitude:.6f}, '
#                    f'lon={msg.longitude:.6f}'
#                )
#            else:
#                self.get_logger().warn('Invalid GPS data received (0,0)')

        except Exception as e:
            self.get_logger().error(f'Error in GPS callback: {str(e)}')

    def __del__(self):
        """Cleanup camera resources"""
        if hasattr(self, 'cameras'):
            for cap in self.cameras.values():
                if cap and cap.isOpened():
                    cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = ImageCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'cameras'):
            for cap in node.cameras.values():
                if cap and cap.isOpened():
                    cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

