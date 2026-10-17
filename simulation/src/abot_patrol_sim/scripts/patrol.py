#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class PatrolController(object):
    def __init__(self):
        self.side = rospy.get_param("~side_length", 1.5)
        self.max_linear = rospy.get_param("~max_linear_speed", 0.28)
        self.max_angular = rospy.get_param("~max_angular_speed", 0.70)
        self.position_tolerance = rospy.get_param("~position_tolerance", 0.10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_odom = False
        self.waypoints = []
        self.target_index = 0

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=1)
        rospy.on_shutdown(self.stop)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        if not self.have_odom:
            self.have_odom = True
            self.waypoints = [
                (self.x + self.side, self.y),
                (self.x + self.side, self.y + self.side),
                (self.x, self.y + self.side),
                (self.x, self.y),
            ]
            rospy.loginfo("Patrol started with %.2f m square route.", self.side)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if not self.have_odom:
                rospy.loginfo_throttle(3.0, "Waiting for /odom ...")
                rate.sleep()
                continue

            target_x, target_y = self.waypoints[self.target_index]
            dx = target_x - self.x
            dy = target_y - self.y
            distance = math.hypot(dx, dy)

            if distance < self.position_tolerance:
                self.stop()
                rospy.sleep(0.5)
                self.target_index = (self.target_index + 1) % len(self.waypoints)
                rospy.loginfo("Next patrol point: %d", self.target_index + 1)
                rate.sleep()
                continue

            desired_yaw = math.atan2(dy, dx)
            yaw_error = normalize_angle(desired_yaw - self.yaw)
            cmd = Twist()

            cmd.angular.z = clamp(1.8 * yaw_error, -self.max_angular, self.max_angular)
            if abs(yaw_error) < 0.75:
                alignment = max(0.18, math.cos(yaw_error))
                cmd.linear.x = min(self.max_linear, 0.65 * distance) * alignment

            self.cmd_pub.publish(cmd)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("abot_patrol")
    PatrolController().run()
