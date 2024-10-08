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
        # Node(
        #     package='image_tools',
        #     executable='cam2image',
        #     name='cam2image',
        #     output='screen'
        # ),
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
            executable='gps_publisher',
            name='gps_publisher',
            output='screen',
            parameters=[{'port': '/dev/ttyACM0', 'baud': 9600}]
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
