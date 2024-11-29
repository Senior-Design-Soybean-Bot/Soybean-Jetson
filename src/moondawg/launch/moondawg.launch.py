import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='websocket',
            output='screen'
        ),
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='ublox_gps',
            parameters=[{
                'device': '/dev/ttyACM0',
                'frame_id': 'gps',
                'uart1.baudrate': 9600,
                'nav_rate': 10,
                'rate': 10.0,
                'gnss.enable': True,
                'gnss.enable_gps': True,
                'gnss.enable_glonass': False,
                'config_on_startup': True,
                'debug': 1
            }],
            output='screen'
        ),
        Node(
            package='moondawg',
            executable='xbox_translator',
            name='xbox_translator',
            output='screen'
        ),
        Node(
            package='moondawg',
            executable='i2c_bridge',
            name='i2c_bridge',
            parameters=[{
                'bus_number': 1,
                'address1': 0x08
            }],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='moondawg',
            executable='image_capture',
            name='image_capture',
            output='screen',
            parameters=[{'camera_indices': [0, 2, 4, 6], 'image_dir': '/Pictures'}]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()

