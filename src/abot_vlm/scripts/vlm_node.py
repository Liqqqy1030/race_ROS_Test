#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第一人：视觉识别节点
- 沿用相机订阅 /usb_cam/image_raw
- 按请求识别：订阅 /patrol/vision/request (std_msgs/String, JSON)
- 发布       /patrol/vision/result (std_msgs/String, JSON)
- 兼容旧接口：参数 /top_view_shot_node/im_flag=1 触发，发布 /vision_result
- 超时 / 重试 / 错误返回全部参数化；密钥不写进 yaml，从 API_KEY.py 或环境变量读
"""
import os
import sys
import json
import base64
import threading

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String
from openai import OpenAI

# 本地密钥（不入库、不写 yaml）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from API_KEY import YI_KEY
except Exception:
    YI_KEY = os.environ.get("VLM_API_KEY", "")


def imgmsg_to_cv2(img_msg):
    dtype = np.dtype("uint8")
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(
        shape=(img_msg.height, img_msg.width, 3),
        dtype=dtype, buffer=img_msg.data)
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    if img_msg.encoding == "rgb8":
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_RGB2BGR)
    elif img_msg.encoding == "mono8":
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_GRAY2BGR)
    elif img_msg.encoding != "bgr8":
        rospy.logerr("Unsupported encoding: %s", img_msg.encoding)
        return None
    return image_opencv


class VlmNode(object):
    def __init__(self):
        # ---- 参数（由 vision.yaml 加载，均有默认值）----
        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
        self.tmp_img_path = rospy.get_param(
            "~tmp_img_path",
            "/home/abot/abot_ws/src/abot_vlm/temp/vl_now.jpg")
        self.base_url = rospy.get_param(
            "~base_url", "https://api.lingyiwanwu.com/v1")
        self.model = rospy.get_param("~model", "yi-vision")
        self.api_key = rospy.get_param("~api_key", YI_KEY)
        self.timeout_s = float(rospy.get_param("~timeout_s", 20.0))
        self.max_retry = int(rospy.get_param("~max_retry", 2))
        self.prompt = rospy.get_param(
            "~prompt",
            "请识别图片中的题目：先原样描述图中完整题目信息(text)，"
            "再给出你计算或判断得到的最终答案(answer)。"
            "只输出 JSON：{\"text\":\"...\",\"answer\":\"...\"}，不要多余解释。")

        self.client = OpenAI(
            api_key=self.api_key,
            base_url=self.base_url,
            timeout=self.timeout_s,
            max_retries=self.max_retry,
        )

        # ---- 状态 ----
        self.latest_frame = None
        self.busy = False          # 旧 im_flag 与新请求互斥，不得同时启动同一识别
        self.lock = threading.Lock()

        # ---- 话题 ----
        rospy.Subscriber(self.image_topic, ROSImage, self._img_cb, queue_size=1)
        rospy.Subscriber("/patrol/vision/request", String,
                         self._req_cb, queue_size=4)
        self.res_pub = rospy.Publisher("/patrol/vision/result", String,
                                       queue_size=4)
        self.legacy_pub = rospy.Publisher("vision_result", String,
                                          queue_size=10)

        # 旧接口 im_flag 轮询
        rospy.Timer(rospy.Duration(0.05), self._legacy_poll)

        rospy.loginfo("[vlm] ready. image=%s model=%s",
                      self.image_topic, self.model)

    # ---------- 相机 ----------
    def _img_cb(self, msg):
        img = imgmsg_to_cv2(msg)
        if img is not None:
            self.latest_frame = img

    # ---------- 新请求接口 ----------
    def _req_cb(self, msg):
        try:
            req = json.loads(msg.data)
        except Exception as e:
            rospy.logwarn("[vlm] bad request json: %s", e)
            return
        run_id = req.get("run_id", "")
        request_id = req.get("request_id", "")
        task_id = req.get("task_id", 0)

        with self.lock:
            if self.busy:
                rospy.logwarn("[vlm] busy, drop req %s", request_id)
                self._publish_result(run_id, request_id, task_id,
                                     False, "", "", None, "busy")
                return
            if self.latest_frame is None:
                self._publish_result(run_id, request_id, task_id,
                                     False, "", "", None, "no_image")
                return
            self.busy = True
            frame = self.latest_frame.copy()

        # 后台线程跑识别，避免阻塞图像回调
        threading.Thread(
            target=self._do_recognize,
            args=(run_id, request_id, task_id, frame, True),
            daemon=True).start()

    def _do_recognize(self, run_id, request_id, task_id, frame, is_new):
        text, answer, err = "", "", ""
        try:
            tmp_dir = os.path.dirname(self.tmp_img_path)
            if tmp_dir and not os.path.exists(tmp_dir):
                os.makedirs(tmp_dir)
            cv2.imwrite(self.tmp_img_path, frame)

            with open(self.tmp_img_path, "rb") as f:
                b64 = base64.b64encode(f.read()).decode("utf-8")
            data_url = "data:image/jpeg;base64," + b64

            completion = self.client.chat.completions.create(
                model=self.model,
                messages=[{
                    "role": "user",
                    "content": [
                        {"type": "text", "text": self.prompt},
                        {"type": "image_url",
                         "image_url": {"url": data_url}},
                    ],
                }],
            )
            raw = completion.choices[0].message.content.strip()
            text, answer = self._parse_vlm_output(raw)
            rospy.loginfo("[vlm] ok: text=%s answer=%s", text, answer)
        except Exception as e:
            err = str(e)
            rospy.logwarn("[vlm] fail: %s", err)
        finally:
            with self.lock:
                self.busy = False

        if is_new:
            self._publish_result(run_id, request_id, task_id,
                                 err == "", text, answer, None, err)
        else:
            # 旧接口兼容：只发文本
            self.legacy_pub.publish(answer if answer else text)

    @staticmethod
    def _parse_vlm_output(raw):
        """从大模型输出里尽力取出 text/answer；失败就整段当 text。"""
        text, answer = raw, ""
        s = raw.strip()
        if s.startswith("```"):
            s = s.split("\n", 1)[-1]
            if s.endswith("```"):
                s = s[:-3].strip()
        if s.startswith("{") and s.endswith("}"):
            try:
                d = json.loads(s)
                text = str(d.get("text", raw))
                answer = str(d.get("answer", ""))
            except Exception:
                pass
        return text, answer

    def _publish_result(self, run_id, request_id, task_id,
                        success, text, answer, target_id, error):
        out = {
            "run_id": run_id,
            "request_id": request_id,
            "task_id": task_id,
            "success": bool(success),
            "text": text,
            "answer": answer,
            "target_id": target_id,
            "error": error,
        }
        self.res_pub.publish(json.dumps(out, ensure_ascii=False))

    # ---------- 旧兼容：im_flag ----------
    def _legacy_poll(self, _evt):
        flag = rospy.get_param("/top_view_shot_node/im_flag", 255)
        if flag != 1:
            return
        with self.lock:
            if self.busy or self.latest_frame is None:
                return
            self.busy = True
            frame = self.latest_frame.copy()
            rospy.set_param("/top_view_shot_node/im_flag", 255)
        threading.Thread(
            target=self._do_recognize,
            args=("", "", 0, frame, False),
            daemon=True).start()


def main():
    rospy.init_node("top_view_shot_node")   # 固定节点名，不再 anonymous
    VlmNode()
    rospy.spin()


if __name__ == "__main__":
    main()
