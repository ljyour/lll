import os
import launch
import launch_ros

def generate_launch_description():
    package_name = 'nav2'
    ld = launch.LaunchDescription()

    pkg_share = launch_ros.substitutions.FindPackageShare(package=package_name).find(package_name)

    # 在实体机器人上，use_sim_time 应该设置为 false
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')  # 修改为 false，实体机器人使用真实时间

    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml'), {'use_sim_time': use_sim_time}]
    )

    # 修改为 use_sim_time 默认值为 'false'，适合实体机器人
    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='use_sim_time', default_value='false',  # 修改默认值为 'false'
        description='Flag to enable use_sim_time'))

    ld.add_action(robot_localization_node)
    return ld
