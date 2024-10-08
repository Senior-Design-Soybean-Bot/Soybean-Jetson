import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import pynmea2

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps_coordinates', 10)

        # Connect to the USB GPS device
        # You may need to change the port and baud rate to match your device
        self.declare_parameter('port', '/dev/ttyACM0')
        port = self.get_parameter('port').value
        self.serial_port = serial.Serial(port, 9600, timeout=1)

        # Timer to publish GPS data periodically
        timer_period = 1.0 # seconds
        self.timer = self.create_timer(timer_period, self.publish_gps_coordinates)
        self.get_logger().info('GPS Publisher Node using GPSd has started.')

    def publish_gps_coordinates(self):
        try:
            line = self.serial_port.readline().decode('ascii', errors='replace').strip()
            if line.startswith('$GPGGA'):
                msg = pynmea2.parse(line)
                if msg.lattitude and msg.longitude:
                    gps_msg = NavSatFix()
                    gps_msg.latitude = msg.latitude
                    gps_msg.longitude = msg.longitude

                    self.publisher_.publish(gps_msg)
                    self.get_logger().info(f'Published GPS: Lat {msg.latitude}, Lon {msg.longitude}')
                else:
                    self.get_logger().warn('No valid GPS fix yet.')

        except serial.SerialException as e:
            self.get_logger().error(f'Serial port error: {e}')

        except pynmea2.ParseError as e:
            self.get_logger().warn(f'NMEA parse error: {e}')

        except Exception as e:
            self.get_logger().error(f'Error reading GPS data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GPSPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()