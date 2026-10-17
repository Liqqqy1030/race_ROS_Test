#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第一人：纯红色区域检测
- 只检测红色（黄色不算），HSV 双区间
- 面积阈值 + 多帧确认：连续 N 帧命中才报 detected=true；目标移开后连续 M 帧无才报 false
- 发布 /fire_detector/detected (std_msgs/Bool)
- 发布 /fire_detector/result  (std_msgs/String, JSON: time/detected/center_x/center_y/area/stream_lost/error)
- 每帧都发结果（未检出也发）；相机断流时报告 stream_lost + error
- 只报告颜色，不直接控制底盘，不向 TTS 连续发 "fire"
"""
import json

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge, CvBridgeError


class FireDetector(object):
    def __init__(self):
        self.bridge = CvBridge()

        # ---- 参数 ----
        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
        self.min_area = float(rospy.get_param("~min_area", 500))
        self.confirm_frames = int(rospy.get_param("~confirm_frames", 3))
        self.clear_frames = int(rospy.get_param("~clear_frames", 5))
        self.stream_timeout = float(rospy.get_param("~stream_timeout", 2.0))

        # HSV 红色双区间（已去掉黄色区间）
        self.red1_low = np.array(rospy.get_param("~red1_low", [0, 100, 50]), np.uint8)
        self.red1_high = np.array(rospy.get_param("~red1_high", [10, 255, 255]), np.uint8)
        self.red2_low = np.array(rospy.get_param("~red2_low", [160, 100, 50]), np.uint8)
        self.red2_high = np.array(rospy.get_param("~red2_high", [180, 255, 255]), np.uint8)

        # ---- 状态 ----
        self.hit_streak = 0
        self.miss_streak = 0
        self.confirmed = False
        self.last_frame_time = None

        # ---- 话题 ----
        rospy.Subscriber(self.image_topic, Image, self._img_cb, queue_size=1)
        self.det_pub = rospy.Publisher("/fire_detector/detected", Bool,
                                       queue_size=10)
        self.res_pub = rospy.Publisher("/fire_detector/result", String,
                                       queue_size=10)

        # 断流看门狗
        rospy.Timer(rospy.Duration(0.2), self._watchdog)

        rospy.loginfo("[fire] ready on %s, min_area=%.0f",
                      self.image_topic, self.min_area)

    def _img_cb(self, msg):
        self.last_frame_time = rospy.Time.now()
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn("[fire] cv_bridge err: %s", e)
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.red1_low, self.red1_high)
        mask = cv2.bitwise_or(
            mask, cv2.inRange(hsv, self.red2_low, self.red2_high))

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        area = int(cv2.countNonZero(mask))
        cx, cy = -1, -1
        if area > 0:
            M = cv2.moments(mask)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

        self._update_state(area >= self.min_area)
        self._publish(stream_lost=False, cx=cx, cy=cy, area=area, err="")

    def _update_state(self, hit):
        if hit:
            self.hit_streak += 1
            self.miss_streak = 0
            if self.hit_streak >= self.confirm_frames and not self.confirmed:
                self.confirmed = True
                rospy.loginfo("[fire] red CONFIRMED")
        else:
            self.miss_streak += 1
            self.hit_streak = 0
            if self.miss_streak >= self.clear_frames and self.confirmed:
                self.confirmed = False
                rospy.loginfo("[fire] red cleared")

    def _publish(self, stream_lost, cx, cy, area, err):
        self.det_pub.publish(Bool(data=self.confirmed))
        out = {
            "time": rospy.Time.now().to_sec(),
            "detected": self.confirmed,
            "center_x": cx,
            "center_y": cy,
            "area": area,
            "stream_lost": stream_lost,
            "error": err,
        }
        self.res_pub.publish(json.dumps(out, ensure_ascii=False))

    def _watchdog(self, _evt):
        if self.last_frame_time is None:
            self._publish(True, -1, -1, 0, "no_image_yet")
            return
        if (rospy.Time.now() - self.last_frame_time).to_sec() > self.stream_timeout:
            self._publish(True, -1, -1, 0, "stream_lost")


if __name__ == "__main__":
    rospy.init_node("fire_detector")
    FireDetector()
    rospy.spin()
