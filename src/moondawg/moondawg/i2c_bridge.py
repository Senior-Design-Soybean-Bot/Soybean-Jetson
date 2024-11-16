import rclpy.logging
from smbus2 import SMBus
import rclpy
from rclpy.lifecycle import Node
from std_msgs.msg import String, Header
from diagnostic_msgs.msg import DiagnosticStatus

hardware_id = 1

class I2CBridge(Node):
    def __init__(self):
        super().__init__(node_name='i2c_bridge')

        heartbeat_interval = 1
        self.diag = DiagnosticStatus(name=self.get_name(), level=DiagnosticStatus.OK,
                                     hardware_id=str(hardware_id), message="Initilizing...")
        self.diag_topic = self.create_publisher(DiagnosticStatus, 'i2c_bridge_diag', 10)
        self.heartbeat_timer = self.create_timer(heartbeat_interval, self.heartbeat)

        self.declare_parameter('bus_numer', 1)
        self.declare_parameter('address1', 0x08)

        self.bus_number = self.get_parameter('bus_number').get_parameter_value().integer_value
        self.address1 = self.get_parameter('address1').get_parameter_value().integer_value

        try:
            self.i2c_bus = SMBus(self.bus_number)
            self.diag.level = DiagnosticStatus.OK
            self.diag.message = "I2C connected"
            self.get_logger().info("I2C bus initialized")
        except Exception as e:
            self.diag.level = DiagnosticStatus.ERROR
            self.diag.message = f"Failed to initialized I2C: {str(e)}"
            self.get_logger().error(self.diag.message)
            self.i2c_bus = None
        
        self.subscription1 = self.create_subscription(
            String, 'i2c_topic_1',
            lambda msg: self.i2c_callback(msg, self.address1),
            10
        )

        self.publisher1 = self.create_publisher(String, 'i2c_received_1', 10)

    def i2c_callback(self, message, address):
        if self.i2c_bus is None:
            self.get_logger().warn("failed to send: I2C bus not initialized")
            return
        
        try:
            data = message.data + '\n'
            self.i2c_bus.write_i2c_block_data(address, 0, list(data.encode()))
            if self.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
                self.get_logger().debug(f"Wrote to address {hex(address)}: {message.data}")
        except Exception as e:
            self.get_logger().error(f"Error sending data to address {hex(address)}: {str(e)}")
            self.diag.level = DiagnosticStatus.ERROR
            self.diag.message = f"I2C write error: {str(e)}"

    def read_i2c(self):
        if self.i2c_bus is None:
            return
        
        try:
            data1 = self.read_from.device(self.address1)
            if data1:
                msg = String()
                msg.data = data1
                self.publisher1.publish(msg)
            

        except Exception as e:
            self.get_logger().error(f"Error reading from I2C: {str(e)}")
            self.diag.level = DiagnosticStatus.ERROR
            self.diag.message = f"I2C read error: {str(e)}"

    def read_from_device(self, address):
        try:
            data = self.i2c_bus.read_i2c_block_data(address, 0, 32)
            return bytes(data).decode().rstrip('\x00')
        except Exception:
            return None
        
    def stop(self):
        if self.i2c_bus is not None:
            self.i2c_bus.close()

    def heartbeat(self):
        self.diag_topic.publish(self.diag)
        if self.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
            self.get_logger().debug(
                f"Heartbeat published with diagnostic status: Level: {self.diag.level},"
                f"Message: {self.diag.message}"
            )

def main(args=None):
    rclpy.init(args=args)
    i2c_bridge = I2CBridge()

    try:
        rclpy.spin(i2c_bridge)
    except KeyboardInterrupt:
        i2c_bridge.get_logger().warning('Ctrl-C pressed, shutting down...')
    finally:
        i2c_bridge.stop()
        rclpy.shutdown()
        exit(130)

if __name__ == '__main__':
    main()