#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <std_msgs/UInt8.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool emergency_stop = false; // 急停变量
bool error_detected = false; // 异常、故障变量
bool is_idle = true; // 待机状态变量
bool is_moving = false; // 移动状态变量
bool is_teleop = false; // 是否正在进行遥控移动
bool is_navigation_active = false; // 是否正在进行导航移动

geometry_msgs::Twist current_velocity; // 当前速度
ros::Time last_velocity_time; // 上一次速度消息的时间戳

// 定义速度发布器和灯光控制发布器
ros::Publisher cmd_vel_pub;
ros::Publisher light_control_pub;

// 灯光状态常量
const uint8_t LIGHT_EMERGENCY_STOP = 0x53; // 急停  83
const uint8_t LIGHT_ERROR = 0x45; // 存在异常、故障   69
const uint8_t LIGHT_IDLE = 0x4E; // 待机            78
const uint8_t LIGHT_MOVING = 0x57; // 移动中         87

// 速度回调函数
void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    current_velocity = *msg;
    last_velocity_time = ros::Time::now(); // 更新速度消息的时间戳
}

// 判断机器人是否在移动的函数
bool isRobotMoving(tf::TransformListener& tf_listener, bool move_base_status) {
    // 静态变量存储上次变换信息
    static tf::StampedTransform last_transform;
    static bool last_transform_set = false;

    // 获取当前时间
    ros::Time now = ros::Time::now();

    // 如果上次变换未设置，或者时间超出缓冲区范围，重新获取变换
    if (!last_transform_set || (now - last_transform.stamp_).toSec() > 1.0) {
        try {
            // 等待并获取变换信息
            tf_listener.waitForTransform("map", "base_link", now, ros::Duration(1.0)); // 等待变换
            tf_listener.lookupTransform("map", "base_link", now, last_transform);
            last_transform_set = true;
        } catch (tf::TransformException& ex) {
            // 输出 TF 错误信息并返回 false
            ROS_ERROR("%s", ex.what());
            return false;
        }
    }

    // 检查当前速度消息是否过期
    if ((now - last_velocity_time).toSec() > 1.0) {
        ROS_WARN("Velocity information is outdated. Skipping movement detection.");
        return false;
    }

    // 计算机器人当前位置与上次位置的距离
    double distance = (now - last_velocity_time).toSec() * current_velocity.linear.x;

    // 判断机器人是否在移动
    bool moving = (distance > 0.01 || current_velocity.angular.z != 0.0 || move_base_status);

    // 如果机器人移动了，更新上次变换
    if (moving) {
        last_transform.stamp_ = now; // 更新上次变换的时间戳
    }

    return moving;
}

// 移动基地状态回调函数
void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& status) {
    if (!status->status_list.empty()) {
        int latest_status = status->status_list.back().status;
        if (latest_status == actionlib_msgs::GoalStatus::ACTIVE) {
            is_navigation_active = true;
            is_idle = false;
            is_moving = true;
        } else {
            is_navigation_active = false;
            is_moving = false;
        }
    } else {
        is_navigation_active = false;
        is_moving = false;
    }
}

// 异常消息回调函数
void errorMessageCallback(const std_msgs::String::ConstPtr& msg) {
    if (msg->data.empty()) {
        ROS_INFO("No error detected.");
        error_detected = false;
    } else {
        ROS_ERROR("Error detected: %s", msg->data.c_str());
        error_detected = true;
    }
}

// 急停回调函数
void emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg) {
    emergency_stop = msg->data;
}

// 手柄控制回调函数
void joystickControlCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
    if (!emergency_stop && !error_detected) {
        // 假设轴1控制线速度，轴3控制角速度
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = joy_msg->axes[1]; // 线速度
        cmd_vel.angular.z = joy_msg->axes[3]; // 角速度
        // 发布命令速度以控制机器人
        cmd_vel_pub.publish(cmd_vel);
        is_teleop = true; // 正在进行遥控移动
    } else {
        ROS_WARN("Robot is in emergency stop or error state. Ignoring joystick commands."); // 机器人处于急停或错误状态，忽略手柄命令
        is_teleop = false; // 遥控移动被中断
    }
}

// 更新灯光状态函数
void updateLightStatus() {
    std_msgs::UInt8 light_msg;
    if (error_detected) {
        light_msg.data = LIGHT_ERROR; // 优先显示异常状态
    } else if (emergency_stop) {
        light_msg.data = LIGHT_EMERGENCY_STOP; // 其次显示急停状态
    } else if (is_moving) {
        light_msg.data = LIGHT_MOVING; // 然后显示移动状态
    } else  {
        light_msg.data = LIGHT_IDLE; // 最后显示待机状态
    }
    light_control_pub.publish(light_msg);
}

void printRobotStatus() {
    if (error_detected) {
        ROS_INFO("Robot has detected an error."); // 机器人检测到错误
    } else if (emergency_stop) {
        ROS_INFO("Robot is in emergency stop state."); // 机器人处于急停状态
    } else if (is_teleop) {
        ROS_INFO("Robot is moving by teleop."); // 机器人正在进行遥控移动
    } else if (is_navigation_active) {
        ROS_INFO("Robot is moving by navigation."); // 机器人正在进行导航移动
    } else {
        ROS_INFO("Robot is in idle state."); // 机器人处于待机状态
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_movement_checker");
    ros::NodeHandle nh;

    tf::TransformListener tf_listener;

    // 订阅速度话题，假设话题名称为 "/cmd_vel"
    ros::Subscriber velocity_sub = nh.subscribe("/cmd_vel", 10, velocityCallback);
    // 订阅移动基地状态话题
    ros::Subscriber move_base_status_sub = nh.subscribe("/move_base/status", 10, moveBaseStatusCallback);
    // 订阅错误消息话题
    ros::Subscriber error_msg_sub = nh.subscribe("/error_messages", 10, errorMessageCallback);
    // 订阅急停话题
    ros::Subscriber emergency_stop_sub = nh.subscribe("stop", 10, emergencyStopCallback);
    // 订阅手柄控制话题
    ros::Subscriber joystick_sub = nh.subscribe("/joy", 10, joystickControlCallback);
    // 定义速度发布器
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // 定义灯光控制发布器
    light_control_pub = nh.advertise<std_msgs::UInt8>("/light_control", 10);

    ros::Rate rate(10); // 10 Hz
    while (ros::ok()) {
        // 获取机器人是否在移动的状态
        bool moving = isRobotMoving(tf_listener, !emergency_stop && !error_detected);
        if (moving) {
            is_idle = false;
            is_moving = true;
        } else {
            is_moving = false;
        }

        // 如果处于急停状态，发布停止命令
        if (emergency_stop) {
            geometry_msgs::Twist stop_vel;
            stop_vel.linear.x = 0.0;
            stop_vel.angular.z = 0.0;
            cmd_vel_pub.publish(stop_vel);
        }

        // 更新灯光状态
        updateLightStatus();

        // 打印机器人状态
        printRobotStatus();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
