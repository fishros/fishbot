import os
import launch
import launch_ros


def generate_launch_description():
    package_name = 'fishbot_navigation2'

    ld =  launch.LaunchDescription()
    pkg_share = launch_ros.substitutions.FindPackageShare(package=package_name).find(package_name) 

    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')}]
    )

    
    ld.add_action(launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                                description='Flag to enable use_sim_time'))
    ld.add_action(robot_localization_node)                                           
    return ld