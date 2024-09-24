import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import gps

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(String, 'gps_coordinates', 10)

        # Connect to local gpsd daemon
        self.gpsd = gps.gps(mode=gps.WATCH_ENABLE)
        self.gpsd.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)

        # Timer to publish GPS data periodically
        timer_period = 1.0 # seconds
        self.timer = self.create_timer(timer_period, self.publish_gps_coordinates)
        self.get_logger().info('GPS Publisher Node using GPSd has started.')

    def publish_gps_coordinates(self):
        try:
            report = self.gpsd.next()
            if report['class'] == 'TPV':
                latitude = getattr(report, 'lat', 'nan')
                longitude = getattr(report, 'lon', 'nan')
                altitude = getattr(report, 'alt', 'nan')

                if latitude != 'nan' and longitude != 'nan':
                    gps_data = f'Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude} meters'
                    self.publisher_.publish(String(data=gps_data))
                    self.get_logger().info(f'Published GPS: {gps_data}')
                else:
                    self.get_logger().warn('No valid GPS fix yet.')

        except KeyError:
            # If the report doesn't have the expected fields
            self.get_logger().warn('Incomplete GPS data.')

        except StopIteration:
            self.get_logger().warn('GPSD has terminated the stream.')

        except Exception as e:
            self.get_logger().error(f'Error reading GPS data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GPSPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()