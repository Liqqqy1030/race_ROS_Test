#include "abot_driver/abot_driver.h"
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <sensor_msgs/Imu.h>

using namespace std::chrono;

AbotDriver::AbotDriver() : 
  nh_(),
  x_(0.0), y_(0.0), th_(0.0),
  vel_data_({0,0,0})
{
  // 使用全局参数服务器获取参数
  ros::param::param<std::string>("~port_name", port_name_, "/dev/abotbase");
  ros::param::param<int>("~baud_rate", baud_rate_, 115200);
  ros::param::param<std::string>("~odom_frame", odom_frame_, "odom");
  ros::param::param<std::string>("~base_frame", base_frame_, "base_link");
  ros::param::param<std::string>("~imu_frame", imu_frame_, "imu_link");
  ros::param::param<bool>("~publish_tf", publish_tf_, true);

  // 发布者 - 全局命名空间
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("raw_odom", 50);
  battery_pub_ = nh_.advertise<std_msgs::Float32>("battery", 1);
  sonar_pub_ = nh_.advertise<std_msgs::Float32>("sonar", 10);
  infrared_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("infrared", 10);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 50);

  // 订阅者 - 全局命名空间
  velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 50, &AbotDriver::cmd_vel_callback, this);
  servo_sub_ = nh_.subscribe<abot_driver::Servo>("servo", 10, &AbotDriver::servo_callback, this);
  akm_sub_ = nh_.subscribe<abot_driver::Akm>("akm_vel", 50, &AbotDriver::akm_vel_callback, this);
  // 定时器
  topic_timer_ = nh_.createTimer(ros::Duration(0.01), &AbotDriver::RosNodeTopicCallback, this);

  // 初始化变量
  infrared_msgs_.data.resize(4);
  last_time_ = ros::Time::now();
  prev_vel_time_ = ros::Time::now();

  // 串口初始化
  try {
    Robot_Serial.setPort(port_name_);
    Robot_Serial.setBaudrate(baud_rate_);
    serial::Timeout to = serial::Timeout::simpleTimeout(2000);
    Robot_Serial.setTimeout(to);
    Robot_Serial.open();
    if (Robot_Serial.isOpen()) {
      ROS_INFO("Abot Serial Port opened: %s @ %d", port_name_.c_str(), baud_rate_);
    } else {
      ROS_ERROR("Failed to open serial port");
    }
  } catch (const serial::IOException& e) {
    ROS_FATAL("Abot Serial IOException: %s", e.what());
  } catch (const std::exception& e) {
    ROS_FATAL("Serial port exception: %s", e.what());
  }
}

AbotDriver::~AbotDriver()
{
  SetVelocity(0, 0, 0, 0);
  if (Robot_Serial.isOpen()) {
    Robot_Serial.close();
  }
}

void AbotDriver::Check_sum(const uint8_t* data, size_t len, uint8_t& dest)
{
  dest = 0x00;
  for (size_t i = 0; i < len; i++) {
    dest += *(data + i);
  }
}

void AbotDriver::RosNodeTopicCallback(const ros::TimerEvent& event)
{
  if (ReadFromSerial()) {
    publisherOdom();
    publisherImuSensor();
    publisherBattery();
    publisherSonarSensor();
    publisherInfrared();
  }
}

bool AbotDriver::ReadFromSerial()
{
  static std::vector<uint8_t> buffer;
  static const size_t MAX_BUFFER_SIZE = 512;
  
  // 读取串口数据到缓冲区
  if (Robot_Serial.available()) {
    size_t available_bytes = Robot_Serial.available();
    std::vector<uint8_t> new_data(available_bytes);
    size_t bytes_read = Robot_Serial.read(new_data.data(), available_bytes);
    new_data.resize(bytes_read);
    
    buffer.insert(buffer.end(), new_data.begin(), new_data.end());
    
    // 防止缓冲区过大
    if (buffer.size() > MAX_BUFFER_SIZE) {
      buffer.erase(buffer.begin(), buffer.begin() + (buffer.size() - MAX_BUFFER_SIZE));
      ROS_WARN("Serial buffer overflow, truncating");
    }
  }

  // 在缓冲区中查找完整的数据帧
  while (buffer.size() >= 2) {
    // 查找帧头
    if (buffer[0] == head1 && buffer[1] == head2) {
      if (buffer.size() < 3) return false;
      
      uint8_t frame_length = buffer[2];
      if (buffer.size() < static_cast<size_t>(frame_length)) {
        return false;
      }
      
      // 提取完整帧
      std::vector<uint8_t> frame(buffer.begin(), buffer.begin() + frame_length);
      buffer.erase(buffer.begin(), buffer.begin() + frame_length);
      
      // 校验和检查
      uint8_t calculated_checksum;
      Check_sum(frame.data(), frame_length - 1, calculated_checksum);
      if (calculated_checksum == frame[frame_length - 1]) {
        rxdata = std::string(frame.begin(), frame.end());
        rxbuf = (uint8_t *)rxdata.c_str();
        return true;
      } else {
        ROS_WARN("Checksum error, expected: %02X, got: %02X", 
                 frame[frame_length - 1], calculated_checksum);
      }
    } else {
      buffer.erase(buffer.begin());
    }
  }
  return false;
}

void AbotDriver::publisherOdom()
{
  ros::Time current_time = ros::Time::now();
  double dt = (current_time - last_time_).toSec();
  last_time_ = current_time;

  // 获取速度数据
  vel_data_.linear_x = static_cast<double>(static_cast<int16_t>(rxbuf[16] << 8 | rxbuf[17])) / 1000.0;
  vel_data_.linear_y = static_cast<double>(static_cast<int16_t>(rxbuf[18] << 8 | rxbuf[19])) / 1000.0;
  vel_data_.angular_z = static_cast<double>(static_cast<int16_t>(rxbuf[20] << 8 | rxbuf[21])) / 1000.0;

  // 使用数值积分方法更新位置
  if (dt > 0.001 && dt < 1.0) {
    double avg_linear_x = (prev_vel_.linear_x + vel_data_.linear_x) / 2.0;
    double avg_linear_y = (prev_vel_.linear_y + vel_data_.linear_y) / 2.0;
    double avg_angular_z = (prev_vel_.angular_z + vel_data_.angular_z) / 2.0;
    
    th_ += avg_angular_z * dt;
    
    if (th_ > M_PI) th_ -= 2 * M_PI;
    else if (th_ < -M_PI) th_ += 2 * M_PI;
    
    double delta_x = (avg_linear_x * cos(th_) - avg_linear_y * sin(th_)) * dt;
    double delta_y = (avg_linear_x * sin(th_) + avg_linear_y * cos(th_)) * dt;
    
    x_ += delta_x;
    y_ += delta_y;
    
    prev_vel_ = vel_data_;
  }

  // 创建四元数
  tf::Quaternion q;
  q.setRPY(0, 0, th_);
  q.normalize();

  // 填充里程计消息
  odom_msgs_.header.stamp = current_time;
  odom_msgs_.header.frame_id = odom_frame_;
  odom_msgs_.child_frame_id = base_frame_;
  
  odom_msgs_.pose.pose.position.x = x_;
  odom_msgs_.pose.pose.position.y = y_;
  odom_msgs_.pose.pose.position.z = 0.0;
  
  odom_msgs_.pose.pose.orientation.x = q.x();
  odom_msgs_.pose.pose.orientation.y = q.y();
  odom_msgs_.pose.pose.orientation.z = q.z();
  odom_msgs_.pose.pose.orientation.w = q.w();
  
  // 设置协方差
  const double ODOM_POSE_COVARIANCE[36] = {
    1e-3, 0, 0, 0, 0, 0,
    0, 1e-3, 0, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e3
  };
  std::copy(ODOM_POSE_COVARIANCE, ODOM_POSE_COVARIANCE+36, 
            odom_msgs_.pose.covariance.begin());
  
  odom_msgs_.twist.twist.linear.x = vel_data_.linear_x;
  odom_msgs_.twist.twist.linear.y = vel_data_.linear_y;
  odom_msgs_.twist.twist.angular.z = vel_data_.angular_z;
  
  // 设置协方差
  const double ODOM_TWIST_COVARIANCE[36] = {
    1e-3, 0, 0, 0, 0, 0,
    0, 1e-3, 0, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e3
  };
  std::copy(ODOM_TWIST_COVARIANCE, ODOM_TWIST_COVARIANCE+36, 
            odom_msgs_.twist.covariance.begin());
  
  // 发布里程计
  odom_pub_.publish(odom_msgs_);

  // 发布TF变换
  if(publish_tf_){
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x_, y_, 0.0));
    transform.setRotation(q);
    
    tf::StampedTransform stamped_tf(
      transform, current_time, odom_frame_, base_frame_);
    
    tf_broadcaster_.sendTransform(stamped_tf);
  }
}

void AbotDriver::publisherImuSensor()
{
  // 解析IMU数据
  imu_data_.gyro_x = static_cast<double>(static_cast<int16_t>(rxbuf[4] << 8 | rxbuf[5])) * GYRO_RATIO;
  imu_data_.gyro_y = static_cast<double>(static_cast<int16_t>(rxbuf[6] << 8 | rxbuf[7])) * GYRO_RATIO;
  imu_data_.gyro_z = static_cast<double>(static_cast<int16_t>(rxbuf[8] << 8 | rxbuf[9])) * GYRO_RATIO;
  
  imu_data_.acc_x = static_cast<double>(static_cast<int16_t>(rxbuf[10] << 8 | rxbuf[11])) * ACC_RATIO;
  imu_data_.acc_y = static_cast<double>(static_cast<int16_t>(rxbuf[12] << 8 | rxbuf[13])) * ACC_RATIO;
  imu_data_.acc_z = static_cast<double>(static_cast<int16_t>(rxbuf[14] << 8 | rxbuf[15])) * ACC_RATIO;

  // 填充IMU消息
  imu_msgs_.header.stamp = ros::Time::now();
  imu_msgs_.header.frame_id = imu_frame_;
  
  imu_msgs_.angular_velocity.x = imu_data_.gyro_x;
  imu_msgs_.angular_velocity.y = imu_data_.gyro_y;
  imu_msgs_.angular_velocity.z = imu_data_.gyro_z;

  if (imu_msgs_.angular_velocity.x > -0.01 && imu_msgs_.angular_velocity.x < 0.01 ){
    imu_msgs_.angular_velocity.x = 0;
  }

  if (imu_msgs_.angular_velocity.y > -0.01 && imu_msgs_.angular_velocity.y < 0.01 ){
    imu_msgs_.angular_velocity.y = 0;
  }

  if (imu_msgs_.angular_velocity.z > -0.01 && imu_msgs_.angular_velocity.z < 0.01 ){
    imu_msgs_.angular_velocity.z = 0;
  }
  
  imu_msgs_.linear_acceleration.x = imu_data_.acc_x;
  imu_msgs_.linear_acceleration.y = imu_data_.acc_y;
  imu_msgs_.linear_acceleration.z = imu_data_.acc_z;
  
  // 设置协方差
  constexpr double GYRO_COVARIANCE = 0.02;
  constexpr double ACCEL_COVARIANCE = 0.04;
  
  imu_msgs_.angular_velocity_covariance[0] = GYRO_COVARIANCE;
  imu_msgs_.angular_velocity_covariance[4] = GYRO_COVARIANCE;
  imu_msgs_.angular_velocity_covariance[8] = GYRO_COVARIANCE;
  
  imu_msgs_.linear_acceleration_covariance[0] = ACCEL_COVARIANCE;
  imu_msgs_.linear_acceleration_covariance[4] = ACCEL_COVARIANCE;
  imu_msgs_.linear_acceleration_covariance[8] = ACCEL_COVARIANCE;
  
  // 方向四元数暂不提供
  imu_msgs_.orientation_covariance[0] = -1;
  
  // 发布IMU数据
  imu_pub_.publish(imu_msgs_);
}

void AbotDriver::publisherBattery()
{
  static ros::Time last_battery_time(0);
  ros::Time current_time = ros::Time::now();
  
  if ((current_time - last_battery_time).toSec() >= 1.0/BATTERY_RATE) {
    bat_msgs_.data = static_cast<double>(static_cast<int16_t>(rxbuf[22] << 8 | rxbuf[23])) / 100.0;
    if (bat_msgs_.data > 0.1) {
      battery_pub_.publish(bat_msgs_);
      last_battery_time = current_time;
    }
  }
}

void AbotDriver::publisherSonarSensor()
{
  sonar_msgs_.data = static_cast<double>(static_cast<int16_t>(rxbuf[24] << 8 | rxbuf[25])) / 10.0;
  sonar_pub_.publish(sonar_msgs_);
}

void AbotDriver::publisherInfrared()
{
  infrared_msgs_.data[0] = rxbuf[26];
  infrared_msgs_.data[1] = rxbuf[27];
  infrared_msgs_.data[2] = rxbuf[28];
  infrared_msgs_.data[3] = rxbuf[29];
  infrared_pub_.publish(infrared_msgs_);
}

void AbotDriver::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& twist_msg)
{
  SetVelocity(twist_msg->linear.x, twist_msg->linear.y, twist_msg->angular.z, 0);
}

void AbotDriver::akm_vel_callback(const abot_driver::Akm::ConstPtr& akm_msg)
{
  SetVelocity(akm_msg->linear, 0, akm_msg->angular, akm_msg->diff);
}

void AbotDriver::servo_callback(const abot_driver::Servo::ConstPtr& servo_msg)
{
  SetServoAngle(servo_msg->id, servo_msg->angle, servo_msg->time);
}


void AbotDriver::SetVelocity(double x, double y, double yaw, double diff)
{
  uint8_t vel_data[13];
  vel_data[0] = head1;
  vel_data[1] = head2;
  vel_data[2] = 0x0d;
  vel_data[3] = sendType_velocity;

  if(abs(diff) > abs(x))
    diff = x;
  
  int16_t x_val = static_cast<int16_t>(x * 1000);
  int16_t y_val = static_cast<int16_t>(y * 1000);
  int16_t yaw_val = static_cast<int16_t>(yaw * 1000);
  int16_t diff_val = static_cast<int16_t>(diff * 1000);
  
  vel_data[4] = static_cast<uint8_t>((x_val >> 8) & 0xFF);
  vel_data[5] = static_cast<uint8_t>(x_val & 0xFF);
  vel_data[6] = static_cast<uint8_t>((y_val >> 8) & 0xFF);
  vel_data[7] = static_cast<uint8_t>(y_val & 0xFF);
  vel_data[8] = static_cast<uint8_t>((yaw_val >> 8) & 0xFF);
  vel_data[9] = static_cast<uint8_t>(yaw_val & 0xFF);
  vel_data[10] = static_cast<uint8_t>((diff_val >> 8) & 0xFF);
  vel_data[11] = static_cast<uint8_t>(diff_val & 0xFF);
  
  Check_sum(vel_data, 12, vel_data[12]);

  try {
    if (Robot_Serial.isOpen()) {
      Robot_Serial.write(vel_data, sizeof(vel_data));
    }
  } catch (const serial::IOException& e) {
    ROS_ERROR("Serial write error: %s", e.what());
  }
}

void AbotDriver::SetServoAngle(uint8_t id, uint16_t angle, uint16_t time)
{
  uint8_t angle_data[9];
  angle_data[0] = head1;
  angle_data[1] = head2;
  angle_data[2] = 0x0a;
  angle_data[3] = servoType_angle;
  angle_data[4] = id & 0xFF;
  angle_data[5] = static_cast<uint8_t>((angle >> 8) & 0xFF);
  angle_data[6] = static_cast<uint8_t>(angle & 0xFF);
  angle_data[7] = static_cast<uint8_t>((time >> 8) & 0xFF);
  angle_data[8] = static_cast<uint8_t>(time & 0xFF);
  
  Check_sum(angle_data, 9, angle_data[9]);

  try {
    if (Robot_Serial.isOpen()) {
      Robot_Serial.write(angle_data, sizeof(angle_data));
    }
  } catch (const serial::IOException& e) {
    ROS_ERROR("Serial write error: %s", e.what());
  }
}


int main(int argc, char * argv[])
{
  ros::init(argc, argv, "abot_driver");
  AbotDriver driver;
  ros::spin();
  return 0;
}
