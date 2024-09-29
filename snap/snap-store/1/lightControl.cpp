#include <iostream>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <serial/serial.h>
#include <battery_pkg/Battery.h>
#include <vector>
#include <iomanip>
#include <sstream>

#define FRAME_HEADER 0xA5   // 帧头
#define MASTER_ADDRESS 0x5A // 主机地址

#define EMERGENCY_STOP 0x53
#define FAULT 0x45
#define ROBOT_IMMOBILE 0x50
#define STANDBY 0x4E
#define NAVING 0x57
#define JOYDRIVING 0x58
#define CHARGING 0x43
#define LOW_BATTERY 0x44
#define CHARGE_COMPLETE 0x64
#define VOICE_WAKEUP 0x48
#define LOWLINE 20
#define PORT "/dev/ttyUSB0"

std::mutex mtx;
serial::Serial sp;
static uint8_t cmdHistory = 0;

// 定义灯功能ID枚举
bool emergencyStatus = false;
// 大故障动不了，小故障能动
bool bigError = false;
bool littleError = false;
double battPercentage = 50;
bool voiceStatus = false;
bool chargeStatus = false;

ros::Time ErrorStartTime; // 记录故障开始时间

int InitSerial(ros::NodeHandle &nh)
{
    std::string serial_port;
    int32_t baud_rate;

    // 从参数服务器获取参数
    if (!nh.getParam("port", serial_port))
    {
        ROS_ERROR("Failed to get param 'serial_port'");
        return -1;
    }

    if (!nh.getParam("baud", baud_rate))
    {
        ROS_ERROR("Failed to get param 'baud_rate'");
        return -1;
    }

    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);

    try
    {
        sp.setPort(serial_port);
        sp.setBaudrate(baud_rate);
        sp.setTimeout(timeout);
        sp.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR("Serial Error");
        return -1;
    }

    if (!sp.isOpen())
    {
        ROS_ERROR("Serial Error");
        return -1;
    }

    ROS_INFO("Serial working");
    return 0;
}
// 发送指令帧的函数
void sendCommandFrame(serial::Serial &sp, uint8_t status)
{
    std::unique_lock<std::mutex> lock(mtx);
    std::vector<unsigned char> frame;
    {
        frame.push_back(FRAME_HEADER);   // 帧头
        frame.push_back(MASTER_ADDRESS); // 地址
        frame.push_back(status);         // 传入状态
    }

    // 发送帧
    try
    {
        {
            sp.write(frame);
        }
        ROS_INFO("Command sent: %hx", status);
    }
    catch (serial::SerialException &e)
    {
        ROS_ERROR_STREAM("Serial error: " << e.what());
    }
}

// 接收数据的回调函数
void receiveData()
{
    // 检查串口缓冲区中是否有可用数据
    if (sp.available() > 0)
    {
        // 创建一个vector用于存储接收到的数据
        std::vector<unsigned char> data;

        // 从串口读取所有可用的数据并存储到data向量中
        sp.read(data, sp.available());

        if (data[2] != 0x43)
            ROS_ERROR("Failed to switch");
    }
}

// 命令回调函数
void MoveStatusSubCb(const std_msgs::UInt8::ConstPtr &msg)
{
    uint8_t ledCmd = STANDBY;

    ros::Duration errorDuration = ros::Time::now() - ErrorStartTime; // 计算错误持续时间

    if (emergencyStatus)
    {
        ledCmd = EMERGENCY_STOP;
    }
    else if (bigError) // 检查大故障持续时间
    {
        ledCmd = ROBOT_IMMOBILE;
        // if (errorDuration.toSec() > 5.0)
        //     bigError = false;
    }
    else if (littleError) // 检查小故障持续时间
    {
        ledCmd = FAULT;
        if (errorDuration.toSec() > 5.0)
            littleError = false;
    }
    else if (msg->data == 1)
    {
        ledCmd = NAVING;
    }
    else if (msg->data == 2)
    {
        ledCmd = JOYDRIVING;
    }
    else if (voiceStatus)
    {
        ledCmd = VOICE_WAKEUP;
    }
    else if (chargeStatus && battPercentage == 100.0)
    {
        ledCmd = CHARGE_COMPLETE;
    }
    else if (chargeStatus)
    {
        ledCmd = CHARGING;
    }
    else if (battPercentage < LOWLINE)
    {
        ledCmd = LOW_BATTERY;
    }

    if (ledCmd != cmdHistory)
    {
        cmdHistory = ledCmd;
        sendCommandFrame(sp, cmdHistory);
    }
}

void EmergencyStatusSubCb(const std_msgs::Bool::ConstPtr &msg)
{
    emergencyStatus = msg->data;
}
void ErrorStatusSubCb(const std_msgs::String::ConstPtr &msg)
{

    bigError = msg->data.size() == 0 ? false : true;
    if (littleError)
    {
        ErrorStartTime = ros::Time::now(); // 重置开始时间
    }
}
void BattSubCb(const battery_pkg::Battery::ConstPtr &msg)
{
    battPercentage = msg->electricity;
    chargeStatus = msg->status;
}
void VoiceStatusSubCb(const std_msgs::Bool::ConstPtr &msg)
{
    voiceStatus = msg->data;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "light_control_node");
    ros::NodeHandle nh("~");

    InitSerial(nh);
    ros::Subscriber moveStatusSub = nh.subscribe("/lightcon/movestatus", 10, MoveStatusSubCb);
    ros::Subscriber emergencyStatusSub = nh.subscribe("/stop", 10, EmergencyStatusSubCb);
    ros::Subscriber errorStatusSub = nh.subscribe("/YHS_CIR02/Error", 10, ErrorStatusSubCb);
    ros::Subscriber BattSub = nh.subscribe("/battery_pub", 10, BattSubCb);
    ros::Subscriber voiceStatusSub = nh.subscribe("/aikit/wakenUp", 10, VoiceStatusSubCb);

    ros::Rate loop_rate(10); // 设置循环频率为10Hz
    while (ros::ok())
    {
        ros::spinOnce();

        // 检查故障状态
        if (littleError && (ros::Time::now() - ErrorStartTime).toSec() >= 5.0)
        {
            // 5秒后恢复待机状态
            //bigError = false;
            littleError = false;
            ErrorStartTime = ros::Time();
            sendCommandFrame(sp, STANDBY); // 发送待机状态
        }

        receiveData(); // 调用接收数据的函数

        loop_rate.sleep();
    }

    sp.close();
    return 0;
}
