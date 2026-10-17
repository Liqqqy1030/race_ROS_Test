#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS-native Python bridge, compatible with Python 2.7 and Python 3."""
from __future__ import print_function, unicode_literals
import base64
import json
import threading
import time
try:
    from urllib.request import Request, urlopen
    from urllib.error import HTTPError, URLError
except ImportError:
    from urllib2 import Request, urlopen, HTTPError, URLError

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from TTS_audio.srv import StringService, StringServiceResponse


def post_json(url, value, timeout):
    body = json.dumps(value).encode("utf-8")
    request = Request(url, data=body, headers={"Content-Type": "application/json"})
    try:
        response = urlopen(request, timeout=timeout)
        raw = response.read()
        return json.loads(raw.decode("utf-8"))
    except HTTPError as exc:
        detail = exc.read().decode("utf-8", "replace")
        raise RuntimeError("backend HTTP {}: {}".format(exc.code, detail))
    except URLError as exc:
        raise RuntimeError("backend unavailable: {}".format(exc))


def get_json(url, timeout):
    try:
        response = urlopen(url, timeout=timeout)
        return json.loads(response.read().decode("utf-8"))
    except HTTPError as exc:
        detail = exc.read().decode("utf-8", "replace")
        raise RuntimeError("backend HTTP {}: {}".format(exc.code, detail))
    except URLError as exc:
        raise RuntimeError("backend unavailable: {}".format(exc))


class Bridge(object):
    def __init__(self):
        self.backend = rospy.get_param("~backend_url", "http://127.0.0.1:18080").rstrip("/")
        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
        self.lock = threading.Lock()
        self.frame = None
        self.vision_busy = False
        self.red_busy = False
        self.hit_streak = 0
        self.miss_streak = 0
        self.red_confirmed = False
        self.voice_armed = False
        self.voice_baseline = 0
        self.run_id = ""

        self.vision_pub = rospy.Publisher("/patrol/vision/result", String, queue_size=4)
        self.red_pub = rospy.Publisher("/fire_detector/detected", Bool, queue_size=10)
        self.red_detail_pub = rospy.Publisher("/fire_detector/result", String, queue_size=10)
        self.start_pub = rospy.Publisher("/start_mission", String, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)
        rospy.Subscriber("/patrol/vision/request", String, self.vision_cb, queue_size=4)
        rospy.Subscriber("/patrol/stage", String, self.stage_cb, queue_size=10)
        rospy.Service("/tts_service", StringService, self.tts_cb)
        rate = float(rospy.get_param("~red_rate", 2.0))
        if rate > 0:
            rospy.Timer(rospy.Duration(1.0 / rate), self.red_timer)
        rospy.Timer(rospy.Duration(0.25), self.voice_timer)
        rospy.loginfo("AI bridge ready: %s; image=%s", self.backend, self.image_topic)

    def stage_cb(self, msg):
        try:
            value = json.loads(msg.data)
            stage = value.get("stage", "")
            run_id = value.get("run_id", "")
        except (ValueError, TypeError, AttributeError):
            return
        if stage == "WAIT_START" and not self.voice_armed:
            try:
                state = get_json(self.backend + "/voice-event", 2)
                self.voice_baseline = int(state.get("event", 0))
                self.run_id = run_id
                self.voice_armed = True
                rospy.loginfo("Offline start phrase armed")
            except Exception as exc:
                rospy.logwarn("cannot arm offline voice: %s", exc)
        elif stage != "WAIT_START":
            self.voice_armed = False

    def voice_timer(self, event):
        if not self.voice_armed:
            return
        try:
            state = get_json(self.backend + "/voice-event", 2)
            current = int(state.get("event", 0))
            if current > self.voice_baseline:
                command = {"command": "start", "run_id": self.run_id}
                self.start_pub.publish(String(data=json.dumps(command)))
                self.voice_baseline = current
                self.voice_armed = False
                rospy.loginfo("Offline phrase accepted; /start_mission published")
        except Exception as exc:
            rospy.logwarn_throttle(5, "offline voice poll failed: %s", exc)

    @staticmethod
    def pack_image(msg):
        return {"width": msg.width, "height": msg.height, "step": msg.step,
                "encoding": msg.encoding,
                "data": base64.b64encode(bytes(bytearray(msg.data))).decode("ascii")}

    def image_cb(self, msg):
        with self.lock:
            self.frame = msg

    def vision_cb(self, msg):
        try:
            request = json.loads(msg.data)
        except (ValueError, TypeError):
            rospy.logwarn("invalid vision request JSON")
            return
        with self.lock:
            if self.vision_busy:
                self.publish_vision(request, False, "", "", "bridge_busy")
                return
            if self.frame is None:
                self.publish_vision(request, False, "", "", "no_image")
                return
            self.vision_busy = True
            frame = self.frame
        worker = threading.Thread(target=self.do_vision, args=(request, frame))
        worker.daemon = True
        worker.start()

    def do_vision(self, request, frame):
        try:
            result = post_json(self.backend + "/vision",
                               {"image": self.pack_image(frame)}, 75)
            self.publish_vision(request, result.get("success") is True,
                                result.get("text", ""), result.get("answer", ""),
                                result.get("error", ""))
        except Exception as exc:
            self.publish_vision(request, False, "", "", str(exc))
        finally:
            with self.lock:
                self.vision_busy = False

    def publish_vision(self, request, success, text, answer, error):
        value = {"run_id": request.get("run_id", ""),
                 "request_id": request.get("request_id", ""),
                 "task_id": request.get("task_id", 0),
                 "success": bool(success), "text": text,
                 "answer": answer, "error": error}
        self.vision_pub.publish(String(data=json.dumps(value, ensure_ascii=True)))

    def tts_cb(self, request):
        try:
            result = post_json(self.backend + "/tts", {"text": request.data}, 65)
            if result.get("success") is True:
                return StringServiceResponse("ok")
            return StringServiceResponse("error:" + str(result.get("error", "unknown")))
        except Exception as exc:
            return StringServiceResponse("error:" + str(exc))

    def red_timer(self, event):
        with self.lock:
            if self.red_busy or self.frame is None:
                return
            self.red_busy = True
            frame = self.frame
        worker = threading.Thread(target=self.do_red, args=(frame,))
        worker.daemon = True
        worker.start()

    def do_red(self, frame):
        try:
            result = post_json(self.backend + "/red",
                               {"image": self.pack_image(frame)}, 5)
            hit = result.get("detected") is True
            if hit:
                self.hit_streak += 1
                self.miss_streak = 0
                if self.hit_streak >= 3:
                    self.red_confirmed = True
            else:
                self.miss_streak += 1
                self.hit_streak = 0
                if self.miss_streak >= 5:
                    self.red_confirmed = False
            result["detected"] = self.red_confirmed
            result["time"] = rospy.Time.now().to_sec()
            self.red_pub.publish(Bool(data=self.red_confirmed))
            self.red_detail_pub.publish(String(data=json.dumps(result)))
        except Exception as exc:
            rospy.logwarn_throttle(5, "red detection backend failed: %s", exc)
        finally:
            with self.lock:
                self.red_busy = False


def wait_backend(url):
    deadline = time.time() + 45
    while not rospy.is_shutdown() and time.time() < deadline:
        try:
            response = urlopen(url.rstrip("/") + "/health", timeout=2)
            if json.loads(response.read().decode("utf-8")).get("success") is True:
                return
        except Exception:
            pass
        time.sleep(0.5)
    raise RuntimeError("AI backend did not become ready")


if __name__ == "__main__":
    rospy.init_node("competition_ai_bridge")
    backend_url = rospy.get_param("~backend_url", "http://127.0.0.1:18080")
    wait_backend(backend_url)
    Bridge()
    rospy.spin()
