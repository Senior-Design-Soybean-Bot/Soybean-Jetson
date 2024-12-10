import rclpy
from std_msgs.msg import Int8MultiArray, String
from rclpy.lifecycle import Node
from sys import exit
from .string_gen import StringGen

hardware_id = 0

# handle the controller data and send a parsable string to the serial node
class XboxTranslator(Node):

    def init_vars(self):
        self.camera_arm = 90
        self.left_speed = 90
        self.right_speed = 90
        self.connection_time = 0
        self.connected = 0

        # button states
        self.dpad_up = 0
        self.dpad_down = 0
        self.dpad_left = 0
        self.dpad_right = 0
        self.left_speed = 0
        self.right_speed = 0
        self.button_x = 0
        self.button_a = 0
        self.button_b = 0
        self.button_y = 0
        self.lbutton = 0
        self.rbutton = 0
        self.ltrigger = 0
        self.rtrigger = 0
        self.select = 0
        self.menu = 0
        self.lstickbutton = 0
        self.rstickbutton = 0


    def __init__(self):
        super().__init__(node_name='xbox_translator')

        self.init_vars()
        self.declare_parameters(
            namespace='',
            parameters=[
                ('wheel_full_speed', 180),
                ('wheel_full_stopped', 90),
            ])
        
        self.axis_subscription = self.create_subscription(Int8MultiArray, 'gamepad_axis', self.axis_callback, 10)
        self.button_subscription = self.create_subscription(Int8MultiArray, 'gamepad_button', self.button_callback, 10)
        self.serial_publisher = self.create_publisher(String, '/serial_node/serial', 10)
        self.get_logger().info('xbox node init')

    # called by website, tracks last time connected
    def connection_callback(self, message):
        self.connection_time = datetime.datetime.now()
        self.connected = 1

    # parse controller data from website to be easily referenced
    def parse_axis(self, data):
        return {
            "lstick_x": data[0],
            "lstick_y": data[1]
        }    
        
    def parse_buttons(self, data):
        return {
            "dpad_left": data[14]/100, 
            "dpad_right": data[15]/100,
        }

    # This function is called when the gamepad axis data is received
    def axis_callback(self, request):
        try:
            # Parse the data from the request
            axis = self.parse_axis(request.data)
#            self.get_logger().info(f'Movement - X: {axis["lstick_x"]}, Y: {axis["lstick_y"]}')
            self.movement_stick_handler(axis)

        except Exception as e:
            self.get_logger().error("Exception in gamepad axis callback:" + str(e))

    def movement_stick_handler(self, axis):
        left_speed, right_speed = self.calculate_speed(axis['lstick_x'], axis['lstick_y'])
      
        if left_speed == self.left_speed and right_speed == self.right_speed:
            return
        
        self.left_speed = left_speed
        self.right_speed = right_speed
        
        movement_msg = StringGen.movement_string(left_speed, right_speed)
#        self.get_logger().info(f'Sending movement: {movement_msg.data}')
        self.serial_publisher.publish(movement_msg)
    
    def calculate_speed(self, x_axis, y_axis):
        full_forward = self.get_parameter('wheel_full_speed').value
        stopped = self.get_parameter('wheel_full_stopped').value
        full_reverse = stopped - (full_forward - stopped)

        if abs(x_axis) < 15 and abs(y_axis) < 15:
            x_axis = 0
            y_axis = 0

        speed = (-y_axis) * ((full_forward-full_reverse)/200) + stopped
        direction = (x_axis) * ((full_forward-full_reverse)/200) * 0.75

        left_speed = round(max(full_reverse, min(speed - direction, full_forward)))
        right_speed = round(max(full_reverse, min(speed + direction, full_forward)))

        return left_speed, right_speed

    # called when website sends controller data
    def button_callback(self, request):
        try:
            buttons = self.parse_buttons(request.data)
            
            if buttons['dpad_left']:
                self.camera_arm = max(0, self.camera_arm - 5)
                self.serial_publisher.publish(StringGen.camera_arm_string(self.camera_arm))
            elif buttons['dpad_right']:
                self.camera_arm = max(180, self.camera_arm + 5)
                self.serial_publisher.publish(StringGen.camera_arm_string(self.camera_arm))

        except Exception as e:
            self.get_logger().error("Exception in gamepad button callback:" + str(e))

    # called when everything must stop!
    def stop_all(self):
        self.serial_publisher.publish(StringGen.movement_string(90,90))


# start node
def main(args=None):
    rclpy.init(args=args)

    xbox_translator = XboxTranslator()

    try:
        rclpy.spin(xbox_translator)
    except KeyboardInterrupt:
        xbox_translator.get_logger().warning('Ctrl-C pressed, shutting down...')
    finally:
        xbox_translator.stop_all()
        rclpy.shutdown()
        exit(130)

if __name__ == '__main__':
    main()
