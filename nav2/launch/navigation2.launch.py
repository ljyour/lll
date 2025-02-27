import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    nav2_dir = get_package_share_directory('nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 对于实体机器人，将 use_sim_time 设置为 false
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')  # 实体机器人使用真实时间
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(nav2_dir, 'maps', 'bigroom.yaml'))  # 修改为你的地图文件路径
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(nav2_dir, 'param', 'fishbot.yaml'))  # 修改为你的导航参数文件路径

    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('map', default_value=map_yaml_path, description='Full path to map file to load'),
        DeclareLaunchArgument('params_file', default_value=nav2_param_path, description='Full path to param file to load'),

        # 启动 nav2_bringup 包中的启动文件
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('params_file')
            }.items(),
        ),
        # 启动 RViz2 用于可视化导航过程
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
