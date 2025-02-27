from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    motor_dvice_launch_dir = os.path.join(
        get_package_share_directory('motor_drive'),
        'launch'
    )
    base_launch_path = os.path.join(motor_dvice_launch_dir,'joy_launch.py')
    include_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_path)
    ) 

    battery_launch_dir = os.path.join(
        get_package_share_directory('battery_topic'),
        'launch'
    )
    battery_launch_path = os.path.join(battery_launch_dir,'battery_launch.py')
    include_battery_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(battery_launch_path)
    )

    imu_launch_dir = os.path.join(
        get_package_share_directory('hipnuc_imu'),
        'launch'
    )
    imu_launch_path = os.path.join(imu_launch_dir,'imu_spec_msg.launch.py')
    include_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(imu_launch_path)
    )    

    rplidar_launch_dir = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch'
    )
    rplidar_launch_path = os.path.join(rplidar_launch_dir,'rplidar_t1_launch.py')
    rplidar_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch_path)
    )
    
    #  # 添加模型启动部分
    robot_model_launch_dir = os.path.join(
        get_package_share_directory('fishbot_description'),
        'launch'
    )
    robot_model_launch_path = os.path.join(robot_model_launch_dir, 'gazebo.launch.py')
    include_robot_model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_model_launch_path)
    )
   

    ld.add_action(include_base_launch)
    ld.add_action(include_battery_launch)
    ld.add_action(include_imu_launch)
    ld.add_action(rplidar_imu_launch)
    ld.add_action(include_robot_model_launch)
    return ld

