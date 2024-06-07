#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <std_msgs/UInt8.h>

// 串口对象
serial::Serial sp;

// 定义灯功能ID枚举
enum class LightFunctionID : unsigned char
{
    EmergencyStop = 0x53,
    Fault = 0x45,
    RobotImmobile = 0x50,
    Standby = 0x4E,
    Moving = 0x57,
    Charging = 0x43,
    LowBattery = 0x44,
    ChargeComplete = 0x64,
    VoiceWakeup = 0x48
};

// 定义常量
const unsigned char FRAME_HEADER = 0xA5;   // 帧头
const unsigned char MASTER_ADDRESS = 0x5A; // 主机地址

// 发送指令帧的函数
void sendCommandFrame(serial::Serial &sp, LightFunctionID functionID)
{
    std::vector<unsigned char> frame;
    frame.push_back(FRAME_HEADER);                           // 帧头
    frame.push_back(MASTER_ADDRESS);                         // 地址
    frame.push_back(static_cast<unsigned char>(functionID)); // 功能ID

    // 发送帧
    try
    {
        sp.write(frame); // Changed to use vector directly
        std::cout << "Command frame sent for function: " << static_cast<int>(functionID) << std::endl;
    }
    catch (serial::SerialException &e)
    {
        std::cerr << "Serial error: " << e.what() << std::endl;
    }
}

// 接收数据的回调函数
void receiveData()
{
    // 读取串口数据
    if (sp.available() > 0)
    {
        std::vector<unsigned char> data;
        sp.read(data, sp.available());

        // 打印原始数据
        std::stringstream ss;
        for (unsigned char b : data)
        {
            ss << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
        }
        ROS_INFO("Received raw data: %s", ss.str().c_str());
        // 处理接收到的数据
        // Example: Check if received data matches a specific pattern
        if (data.size() >= 3 && data[0] == FRAME_HEADER && data[1] == 0x43 && data[2] == 0x43)
        {
            ROS_INFO("Received confirmation: The lamp switch is successful");
        }
    }
}

// 命令回调函数
void commandCallback(const std_msgs::String::ConstPtr &msg)
{
    LightFunctionID functionID;

    // 使用字符串映射到功能ID
    if (msg->data == "EmergencyStop")
    {
        functionID = LightFunctionID::EmergencyStop;
    }
    else if (msg->data == "Fault")
    {
        functionID = LightFunctionID::Fault;
    }
    else if (msg->data == "RobotImmobile")
    {
        functionID = LightFunctionID::RobotImmobile;
    }
    else if (msg->data == "Standby")
    {
        functionID = LightFunctionID::Standby;
    }
    else if (msg->data == "Moving")
    {
        functionID = LightFunctionID::Moving;
    }
    else if (msg->data == "Charging")
    {
        functionID = LightFunctionID::Charging;
    }
    else if (msg->data == "LowBattery")
    {
        functionID = LightFunctionID::LowBattery;
    }
    else if (msg->data == "ChargeComplete")
    {
        functionID = LightFunctionID::ChargeComplete;
    }
    else if (msg->data == "VoiceWakeup")
    {
        functionID = LightFunctionID::VoiceWakeup;
    }
    else
    {
        ROS_WARN("Unknown command received: %s", msg->data.c_str());
        return;
    }

    // 发送指令帧
    sendCommandFrame(sp, functionID); // Changed to use functionID variable
}

// 回调函数，当接收到 /light_control 主题的消息时调用
void lightControlCallback(const std_msgs::UInt8::ConstPtr& msg) {

    LightFunctionID functionID;
    switch (msg->data) {
        case 0x53:
            functionID = LightFunctionID::EmergencyStop; // 急停状态
            break;
        case 0x45:
            functionID = LightFunctionID::Fault;// 错误状态
            break;
        case 0x4E:
            functionID = LightFunctionID::Standby; // 待机状态
            break;
        case 0x57:
            functionID = LightFunctionID::Moving; // 移动状态
            break;
        default:
            ROS_WARN("Unknown light status: %d", msg->data); // 未知状态
            break;
    }

    // 发送指令帧
    sendCommandFrame(sp, functionID);
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "light_control_node");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 设置串口名称和超时
    std::string serial_port = "/dev/ttyUSB0"; // 串口设备名
    int baud_rate = 115200; // 波特率
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000); // 超时时间

    // 打开串口
    try
    {
        sp.setPort(serial_port);
        sp.setBaudrate(baud_rate);
        sp.setTimeout(timeout);
        sp.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open serial port: " << e.what());
        return -1;
    }

    // 检查串口是否已经打开
    if (!sp.isOpen())
    {
        ROS_ERROR("Serial port failed to open!");
        return -1;
    }

    ROS_INFO_STREAM("Serial port initialized.");

    // 发布者
    ros::Publisher pub = nh.advertise<std_msgs::UInt8>("light_control", 10);

    // 订阅者
    ros::Subscriber sub = nh.subscribe("light_control_commands", 10, commandCallback);

    // 订阅 /light_control 主题
    ros::Subscriber light_control_sub = nh.subscribe<std_msgs::UInt8>("/light_control", 10, lightControlCallback);

    // ROS循环
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        // 接收串口数据
        receiveData();

        ros::spinOnce();
        loop_rate.sleep();
    }

    // 关闭串口
    sp.close();

    return 0;
}

