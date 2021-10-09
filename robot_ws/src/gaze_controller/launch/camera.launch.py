from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


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

    camera_calibration_file = DeclareLaunchArgument(
        'camera_calibration_file',
        default_value='file://' + get_package_share_directory('usb_camera_driver') + '/config/camera.yaml')

    camera_node = Node(
        package='usb_camera_driver',
        executable='usb_camera_driver_node',
        namespace='/camera',
        parameters=[
            {"camera_calibration_file": LaunchConfiguration('camera_calibration_file')}
        ]
    )

    launch_list += [
        use_sim_time_declare,
        emulate_tty_declare,
        camera_calibration_file,
        camera_node,
    ]

    return LaunchDescription(launch_list)
