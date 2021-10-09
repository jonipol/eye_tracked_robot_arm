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

    camera_calibrator = Node(
        package='camera_calibration',
        executable='cameracalibrator',
        name='camera_calibration',
        namespace='',
        output='screen',
        emulate_tty=emulate_tty,
        use_sim_time=use_sim_time,
        arguments=[{
            '--camera_name': 'camera',
            '--pattern': 'chessboard',
            '--size': ['7x7'],
            '--square': [2.0],
            '--approximate': 0.0,
            '--no-service-check': True,
            '--fix_principal-point': False,
            '--fix-aspect-ratio': False,
            '--zero-tangent-dist': False,
            '--k-coefficients': 2,
            '--disable_calib_cb_fast_check': False,

        }]
    )

    launch_list += [
        use_sim_time_declare,
        emulate_tty_declare,
        camera_calibrator,
    ]
    return LaunchDescription(launch_list)
