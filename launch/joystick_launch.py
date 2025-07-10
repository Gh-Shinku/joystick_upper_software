from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joystick_topic = "/joystick"
    joystick_serial_port_name = "/dev/joystick_usb2ttl"
    R1_service_name = "R1_srv"
    R2_service_name = "R2_srv"
    go_service_name = "go_srv"
    joy_service_name = "joyvel_srv"

    return LaunchDescription(
        [
            Node(
                package="joystick_upper_software",
                executable="joystick_publisher",
                name="joystick_publisher",
                output="log",
                parameters=[{"serial_port": joystick_serial_port_name}],
            ),
            Node(
                package="joystick_upper_software",
                executable="joystick_handle_controller",
                name="joystick_handle_controller",
                output="log",
                parameters=[{}],
            ),
            Node(
                package="joystick_upper_software",
                executable="joystick_button_controller",
                name="joystick_button_controller",
                output="log",
                parameters=[
                    {
                        "joystick_topic": joystick_topic,
                        "R1_service": R1_service_name,
                        "R2_service": R2_service_name,
                        "go_service": go_service_name,
                        "joy_service": joy_service_name,
                    }
                ],
            ),
        ]
    )
