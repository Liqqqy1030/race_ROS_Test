#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
第一人：视觉结果解析节点
- 新接口：订阅 /patrol/vision/result (JSON)，优先取 target_id，否则从 answer 提取数字
- 旧兼容：订阅 /vision_result 文本，正则提取第一个数字（不会同时命中 1 和 2）
- 统一发布 /vlm_id_topic (std_msgs/Int32)
- 已去掉直接 mplayer 播放 MP3
"""
import re
import json

import rospy
from std_msgs.msg import String, Int32


class VisionResultParser(object):
    def __init__(self):
        self.pub = rospy.Publisher("vlm_id_topic", Int32, queue_size=10)

        # 新接口（正式巡航）
        rospy.Subscriber("/patrol/vision/result", String,
                         self._new_cb, queue_size=4)
        # 旧接口（兼容调试）
        rospy.Subscriber("vision_result", String,
                         self._legacy_cb, queue_size=10)

        rospy.loginfo("vision_result_subscriber_node started")

    @staticmethod
    def _extract_id(text):
        """从任意文本里提取第一个 1~8 的整数；没有就返回 None。"""
        if text is None:
            return None
        m = re.search(r"[1-8]", str(text))
        return int(m.group()) if m else None

    def _new_cb(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception as e:
            rospy.logwarn("bad result json: %s", e)
            return
        if not d.get("success", False):
            rospy.logwarn("vision result failed: %s", d.get("error", ""))
            return
        tid = d.get("target_id", None)
        if isinstance(tid, int) and 1 <= tid <= 8:
            vlm_id = tid
        else:
            vlm_id = self._extract_id(d.get("answer", ""))
        if vlm_id is None:
            rospy.logwarn("cannot parse id from result: %s", msg.data)
            return
        rospy.loginfo("[parser] new result -> vlm_id=%d", vlm_id)
        self.pub.publish(vlm_id)

    def _legacy_cb(self, msg):
        vlm_id = self._extract_id(msg.data)
        if vlm_id is None:
            return
        rospy.loginfo("[parser] legacy -> vlm_id=%d", vlm_id)
        self.pub.publish(vlm_id)


def main():
    rospy.init_node("vision_result_subscriber_node")
    VisionResultParser()
    rospy.spin()


if __name__ == "__main__":
    main()
