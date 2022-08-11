from launch.exit_handler import ignore_exit_handler, restart_exit_handler
from ros2run.api import get_executable_path


def launch(launch_descriptor, argv):
    ld = launch_descriptor
    package = 'ydlidar'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='ydlidar_node')],
        name='ydlidar_node',
        exit_handler=restart_exit_handler,
    )
    package = 'tf2_ros'
    ld.add_process(
        # The XYZ/Quat numbers for base_link -> laser_frame are taken from the
        # turtlebot URDF in
        # https://github.com/turtlebot/turtlebot/blob/931d045/turtlebot_description/urdf/sensors/astra.urdf.xacro
        cmd=[
            get_executable_path(
                package_name=package, executable_name='static_transform_publisher'),
            '0', '0', '0.02',
            '0', '0', '0', '1',
            'base_link',
            'laser_frame'
        ],
        name='static_tf_pub_laser',
        exit_handler=restart_exit_handler,
    )
    return ld
