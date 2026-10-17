#ifndef RIKI_BASE_H
#define RIKI_BASE_H

#include <ros/ros.h>
#include <riki_msgs/Velocities.h>
#include <riki_msgs/Battery.h>
#include <tf/transform_broadcaster.h>

class RikiBase
{
public:
    RikiBase();
    void velCallback(const riki_msgs::Velocities& vel);
    void batteryCallback(const riki_msgs::Battery& bat);


private:
    ros::NodeHandle nh_;
    ros::Publisher odom_publisher_;
    ros::Subscriber velocity_subscriber_;
    ros::Subscriber battery_sub_;
    tf::TransformBroadcaster odom_broadcaster_;

    float linear_scale_;
    float low_battery_;
    float steering_angle_;
    float linear_velocity_x_;
    float linear_velocity_y_;
    float angular_velocity_z_;
    ros::Time last_vel_time_;
    float vel_dt_;
    float x_pos_;
    float y_pos_;
    float heading_;
    std::string frame_id_;
    std::string child_frame_id_;

};

#endif
