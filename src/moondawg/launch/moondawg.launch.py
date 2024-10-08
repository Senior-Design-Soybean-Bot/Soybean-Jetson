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
            package='image_tools',
            executable='cam2image',
            name='cam2image',
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
            }]
        ),
        Node(
            package='moondawg',
            executable='xbox_translator',
            name='xbox_translator',
            output='screen'
        ),
        Node(
            package='moondawg',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen'
        ),
        Node(
            package='moondawg',
            executable='diagnostics',
            name='diagnostics',
            output='screen'
        ),
        Node(
            package='moondawg',
            executable='image_capture',
            name='image_capture',
            output='screen',
            parameters=[{'image_dir': '/Pictures'},
                        {'include_date': True}]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
