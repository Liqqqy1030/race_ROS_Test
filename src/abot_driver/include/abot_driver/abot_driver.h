#ifndef ABOT_DRIVER_HPP
#define ABOT_DRIVER_HPP

#include <chrono>
#include <string>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <serial/serial.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "abot_driver/Servo.h"
#include "abot_driver/Akm.h"

#define BATTERY_RATE 1
#define M_PI 3.14159265358979323846

#define head1 0xAB
#define head2 0xCD
#define sendType_velocity    0x11
#define servoType_angle      0x12

// IMU加速度计转换系数
#define ACC_RATIO (2*9.8/32768.0)
// IMU陀螺仪转换系数
#define GYRO_RATIO ((500*M_PI/180.0)/32768.0)

// IMU数据结构体
typedef struct {
    double acc_x;
    double acc_y;
    double acc_z;
    
    double gyro_x;
    double gyro_y;
    double gyro_z;
} Imu_Data;

// 机器人速度数据结构体
typedef struct {
    double linear_x;
    double linear_y;
    double angular_z;
} Velocity;

class AbotDriver {
public:
    AbotDriver();
    ~AbotDriver();

private:
    // ROS 节点
    ros::NodeHandle nh_;
    
    // ROS 发布者
    ros::Publisher odom_pub_;
    ros::Publisher battery_pub_;
    ros::Publisher sonar_pub_;
    ros::Publisher infrared_pub_;
    ros::Publisher imu_pub_;
    
    // ROS 订阅者
    ros::Subscriber velocity_sub_;
    ros::Subscriber servo_sub_;
    ros::Subscriber akm_sub_;
    
    // 定时器
    ros::Timer topic_timer_;
    tf::TransformBroadcaster tf_broadcaster_;
    
    // 串口对象
    serial::Serial Robot_Serial;
    
    // 回调函数
    void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& twist_msg);
    void servo_callback(const abot_driver::Servo::ConstPtr& servo_msg);
    void akm_vel_callback(const abot_driver::Akm::ConstPtr& akm_msg);
    void RosNodeTopicCallback(const ros::TimerEvent& event);
    
    // 数据处理函数
    bool ReadFromSerial();
    void Check_sum(const uint8_t* data, size_t len, uint8_t& dest);
    
    // 数据发布函数
    void publisherOdom();
    void publisherImuSensor();
    void publisherBattery();
    void publisherSonarSensor();
    void publisherInfrared();
    
    // 控制函数
    void SetVelocity(double x, double y, double yaw, double diff);
    void SetServoAngle(uint8_t id, uint16_t angle, uint16_t time);
    
    // 参数变量
    std::string port_name_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string imu_frame_;
    bool publish_tf_;
    int baud_rate_;
    
    // 数据缓冲区
    std::string rxdata;
    uint8_t *rxbuf;
    
    // 状态变量
    ros::Time last_time_;
    ros::Time prev_vel_time_;
    double x_, y_, th_;
    Velocity vel_data_;
    Velocity prev_vel_;
    
    // 消息对象
    nav_msgs::Odometry odom_msgs_;
    std_msgs::Float32 bat_msgs_;
    std_msgs::Float32 sonar_msgs_;
    std_msgs::UInt8MultiArray infrared_msgs_;
    sensor_msgs::Imu imu_msgs_;
    
    // IMU数据
    Imu_Data imu_data_;
};

#endif // ABOT_DRIVER_HPP
