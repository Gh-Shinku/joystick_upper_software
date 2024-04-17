from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明并获取参数
    ip_arg = DeclareLaunchArgument(
        'ip', default_value='192.168.4.1',
        description='IP address for the joystick publisher')

    port_arg = DeclareLaunchArgument(
        'port', default_value='3456',
        description='Port for the joystick publisher')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Serial port for the joystick publisher')

    # 创建节点配置
    joystick_publisher_node = Node(
        package='joystick_upper_software',
        executable='joystick_publisher',
        name='joystick_publisher',
        output='screen',
        parameters=[{
            'ip': LaunchConfiguration('ip'),
            'port': LaunchConfiguration('port'),
            'serial_port': LaunchConfiguration('serial_port')
        }]
    )

    # 创建并返回launch描述
    return LaunchDescription([
        ip_arg,
        port_arg,
        serial_port_arg,
        joystick_publisher_node
    ])
