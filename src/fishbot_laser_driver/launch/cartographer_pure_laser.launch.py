
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 定位到功能包的地址
    pkg_share = FindPackageShare(
        package='fishbot_laser_driver').find('fishbot_laser_driver')

    # =====================运行节点需要的配置=======================================================================
    # 是否使用仿真时间，我们用gazebo，这里设置成true
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # 地图的分辨率
    resolution = LaunchConfiguration('resolution', default='0.05')
    # 地图的发布周期
    publish_period_sec = LaunchConfiguration(
        'publish_period_sec', default='1.0')
    # 配置文件夹路径
    configuration_directory = LaunchConfiguration(
        'configuration_directory', default=os.path.join(pkg_share, 'config'))
    # 配置文件
    configuration_basename = LaunchConfiguration(
        'configuration_basename', default='fishbot_laser_2d.lua')
    rviz_config_dir = os.path.join(pkg_share, "config", 'map.rviz')

    # =====================声明节点，cartographer/occupancy_grid_node/rviz_node=================================
    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    arguments=['0.0', '0.0', '0.0', '0', '0',
                               '0', 'base_link', 'laser_frame'],
                    )
    # ros2 run tf2_ros static_transform_publisher x y z r p y base_link laser_frame

    laser_node = Node(package='fishbot_laser_driver',
                      executable='laser_x2',
                      name='laser_x2',
                      )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename])

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    # ===============================================定义启动文件========================================================
    ld = LaunchDescription()
    ld.add_action(laser_node)
    ld.add_action(tf2_node)
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)
    ld.add_action(rviz_node)

    return ld
