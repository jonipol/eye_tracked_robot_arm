from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_list = []

    emulate_tty = LaunchConfiguration('emulate_tty')
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_declare = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )

    emulate_tty_declare = DeclareLaunchArgument(
        'emulate_tty',
        default_value='True',
        description='Emulate tty to show python prints for Nodes and to show log colors correctly'
    )

    tobii_camera_node = Node(
        package='tobii_driver',
        executable='tobii_camera',
        namespace='',
        emulate_tty=emulate_tty,
        parameters=[{
            "camera_id": 0,
            "flip_code": 1,
        }]
    )

    tobii_gaze_node = Node(
        package='tobii_driver',
        executable='tobii_gaze_node',
        namespace='',
        emulate_tty=emulate_tty,
    )

    launch_list += [
        use_sim_time_declare,
        emulate_tty_declare,
        tobii_camera_node,
        tobii_gaze_node,
    ]

    return LaunchDescription(launch_list)
