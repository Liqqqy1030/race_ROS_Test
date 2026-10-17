#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Independent Python 3 backend: Doubao image analysis, red detection and Volcano TTS.

This process deliberately imports no ROS modules, so it can use a private Python 3
virtual environment while a Melodic workspace continues using Python 2.
"""
import base64
import gzip
import json
import os
import re
import shutil
import subprocess
import tempfile
import threading
import uuid
import sys
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

import cv2
import numpy as np
import requests
import websocket


HOST = "127.0.0.1"
PORT = int(os.environ.get("COMPETITION_AI_PORT", "18080"))
MAX_BODY = 16 * 1024 * 1024
VALID_ANSWERS = ("31", "32", "33", "40", "41", "42", "49", "50", "51")
VISION_LOCK = threading.Lock()
TTS_LOCK = threading.Lock()
VOICE_LOCK = threading.Lock()
VOICE_EVENT = 0


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def voice_detected():
    global VOICE_EVENT
    with VOICE_LOCK:
        VOICE_EVENT += 1
        value = VOICE_EVENT
    print("[ai_backend] offline keyword detected; event={}".format(value), flush=True)


def start_offline_voice():
    root = required_env("SNOWBOY_ROOT")
    scripts = os.path.join(root, "scripts")
    resources = os.path.join(root, "resources")
    model = os.environ.get("SNOWBOY_MODEL",
                           os.path.join(resources, "models", "startGame.pmdl"))
    resource = os.path.join(resources, "common.res")
    extension = os.path.join(resources, "_snowboydetect.so")
    for path in (scripts, model, resource, extension):
        if not os.path.exists(path):
            raise RuntimeError("missing offline voice file: " + path)
    # The original robot library is a Python 3 extension stored beside its resources.
    sys.path.insert(0, resources)
    sys.path.insert(0, scripts)
    import decoder
    detector = decoder.HotwordDetector(model, resource=resource, sensitivity=0.60)

    def run():
        try:
            detector.start(detected_callback=voice_detected,
                           interrupt_check=lambda: False, sleep_time=0.03)
        finally:
            detector.terminate()

    worker = threading.Thread(target=run, name="offline_voice")
    worker.daemon = True
    worker.start()
    return worker


def required_env(name):
    value = os.environ.get(name, "").strip()
    if not value:
        raise RuntimeError("missing environment variable " + name)
    return value


def decode_ros_image(value):
    width = int(value["width"])
    height = int(value["height"])
    step = int(value["step"])
    encoding = str(value["encoding"]).lower()
    raw = base64.b64decode(value["data"])
    if len(raw) < height * step:
        raise ValueError("incomplete ROS image")
    rows = np.frombuffer(raw[:height * step], dtype=np.uint8).reshape(height, step)
    if encoding in ("rgb8", "bgr8"):
        frame = rows[:, :width * 3].reshape(height, width, 3)
        if encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    elif encoding == "mono8":
        mono = rows[:, :width].reshape(height, width)
        frame = cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
    elif encoding in ("rgba8", "bgra8"):
        frame = rows[:, :width * 4].reshape(height, width, 4)
        code = cv2.COLOR_RGBA2BGR if encoding == "rgba8" else cv2.COLOR_BGRA2BGR
        frame = cv2.cvtColor(frame, code)
    else:
        raise ValueError("unsupported image encoding: " + encoding)
    return np.ascontiguousarray(frame)


def parse_vision_output(raw):
    value = raw.strip()
    if value.startswith("```"):
        value = value.split("\n", 1)[-1]
        if value.endswith("```"):
            value = value[:-3].strip()
    text, answer_text = value, ""
    try:
        obj = json.loads(value)
        text = str(obj.get("text", ""))
        answer_text = str(obj.get("answer", ""))
    except (ValueError, AttributeError):
        pass
    match = re.search(r"(?<!\d)(31|32|33|40|41|42|49|50|51)(?!\d)", answer_text)
    if not match:
        raise RuntimeError("model returned no valid yellow-cell answer: " + answer_text)
    return text, match.group(1)


def vision(frame):
    if not VISION_LOCK.acquire(False):
        raise RuntimeError("vision backend busy")
    try:
        key = required_env("ARK_API_KEY")
        base_url = os.environ.get("ARK_BASE_URL", "https://ark.cn-beijing.volces.com/api/v3")
        model = os.environ.get("ARK_VISION_MODEL", "doubao-seed-2-1-lite-260915")
        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
        if not ok:
            raise RuntimeError("jpeg encode failed")
        data_url = "data:image/jpeg;base64," + base64.b64encode(encoded.tobytes()).decode("ascii")
        prompt = (
            "请识别图片中的题目，先原样描述完整题目信息，再完成题目。"
            "answer只能填写对应的黄色格号码，且只能是31、32、33、40、41、42、49、50、51之一。"
            "只输出JSON：{\"text\":\"...\",\"answer\":\"...\"}，不要输出其他内容。"
        )
        payload = {
            "model": model,
            "messages": [{"role": "user", "content": [
                {"type": "text", "text": prompt},
                {"type": "image_url", "image_url": {"url": data_url}},
            ]}],
            "thinking": {"type": "disabled"},
        }
        response = requests.post(
            base_url.rstrip("/") + "/chat/completions",
            headers={"Authorization": "Bearer " + key,
                     "Content-Type": "application/json"},
            json=payload, timeout=60)
        if response.status_code != 200:
            raise RuntimeError("Ark HTTP {}: {}".format(
                response.status_code, response.text[:1000]))
        value = response.json()
        raw = value["choices"][0]["message"]["content"]
        text, answer = parse_vision_output(raw)
        return {"success": True, "text": text, "answer": answer}
    finally:
        VISION_LOCK.release()


def red_detect(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([0, 100, 50], np.uint8),
                       np.array([10, 255, 255], np.uint8))
    mask |= cv2.inRange(hsv, np.array([160, 100, 50], np.uint8),
                        np.array([180, 255, 255], np.uint8))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    area = int(cv2.countNonZero(mask))
    minimum = int(os.environ.get("RED_MIN_AREA", "500"))
    cx = cy = -1
    if area:
        moments = cv2.moments(mask)
        if moments["m00"]:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
    return {"detected": area >= minimum, "area": area,
            "center_x": cx, "center_y": cy}


def parse_tts_response(data, audio_file):
    if len(data) < 4:
        raise RuntimeError("incomplete TTS response")
    header_size = (data[0] & 0x0F) * 4
    message_type = data[1] >> 4
    flags = data[1] & 0x0F
    compression = data[2] & 0x0F
    payload = data[header_size:]
    if message_type == 0x0F:
        code = int.from_bytes(payload[:4], "big") if len(payload) >= 4 else -1
        detail = payload[8:] if len(payload) >= 8 else b""
        if compression == 1 and detail:
            detail = gzip.decompress(detail)
        raise RuntimeError("TTS error {}: {}".format(code, detail.decode("utf-8", "replace")))
    if message_type == 0x0B:
        if flags == 0:
            return False
        if len(payload) < 8:
            raise RuntimeError("incomplete TTS audio frame")
        sequence = int.from_bytes(payload[:4], "big", signed=True)
        size = int.from_bytes(payload[4:8], "big")
        audio = payload[8:8 + size]
        if len(audio) != size:
            raise RuntimeError("TTS audio length mismatch")
        audio_file.write(audio)
        return sequence < 0 or flags in (2, 3)
    if message_type == 0x0C:
        return False
    raise RuntimeError("unknown TTS message type: {}".format(message_type))


def synthesize(text, output_path):
    appid = required_env("VOLC_TTS_APPID")
    token = required_env("VOLC_TTS_TOKEN")
    body = gzip.compress(json.dumps({
        "app": {"appid": appid, "token": token,
                "cluster": os.environ.get("VOLC_TTS_CLUSTER", "volcano_tts")},
        "user": {"uid": "abot_competition"},
        "audio": {"voice_type": os.environ.get("VOLC_TTS_VOICE", "BV002_streaming"),
                  "encoding": "mp3", "speed_ratio": 0.9,
                  "volume_ratio": 1.0, "pitch_ratio": 1.0},
        "request": {"reqid": str(uuid.uuid4()), "text": text,
                    "text_type": "plain", "operation": "submit"},
    }, ensure_ascii=False).encode("utf-8"))
    packet = bytearray(b"\x11\x10\x11\x00")
    packet.extend(len(body).to_bytes(4, "big"))
    packet.extend(body)
    connection = websocket.create_connection(
        "wss://openspeech.bytedance.com/api/v1/tts/ws_binary",
        header=["Authorization: Bearer; " + token], timeout=15)
    try:
        connection.send_binary(bytes(packet))
        with open(output_path, "wb") as audio_file:
            while True:
                response = connection.recv()
                if isinstance(response, str):
                    raise RuntimeError("TTS returned text instead of binary audio")
                if parse_tts_response(response, audio_file):
                    break
    finally:
        connection.close()


def tts(text):
    if not TTS_LOCK.acquire(False):
        raise RuntimeError("TTS backend busy")
    audio_path = None
    try:
        player = shutil.which(os.environ.get("TTS_PLAYER", "mplayer"))
        if not player:
            raise RuntimeError("mplayer not found")
        handle, audio_path = tempfile.mkstemp(prefix="abot_tts_", suffix=".mp3")
        os.close(handle)
        synthesize(text, audio_path)
        played = subprocess.run([player, "-really-quiet", audio_path],
                                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                                timeout=30, check=False)
        if played.returncode:
            raise RuntimeError("audio player failed: " +
                               played.stderr.decode("utf-8", "replace"))
        return {"success": True}
    finally:
        if audio_path and os.path.exists(audio_path):
            os.remove(audio_path)
        TTS_LOCK.release()


class Handler(BaseHTTPRequestHandler):
    server_version = "ABOT-AI/1.0"

    def log_message(self, fmt, *args):
        print("[ai_backend] " + (fmt % args), flush=True)

    def reply(self, status, value):
        body = json.dumps(value, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/health":
            with VOICE_LOCK:
                event = VOICE_EVENT
            self.reply(200, {"success": True, "offline_voice": True,
                             "voice_event": event, "vision_model":
                             os.environ.get("ARK_VISION_MODEL", "doubao-seed-2-1-lite-260915")})
            return
        if self.path == "/voice-event":
            with VOICE_LOCK:
                event = VOICE_EVENT
            self.reply(200, {"success": True, "event": event})
            return
        else:
            self.reply(404, {"success": False, "error": "not found"})

    def do_POST(self):
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > MAX_BODY:
                raise ValueError("invalid request size")
            value = json.loads(self.rfile.read(length).decode("utf-8"))
            if self.path == "/vision":
                result = vision(decode_ros_image(value["image"]))
            elif self.path == "/red":
                result = red_detect(decode_ros_image(value["image"]))
                result["success"] = True
            elif self.path == "/tts":
                text = str(value.get("text", "")).strip()
                if not text:
                    raise ValueError("empty TTS text")
                result = tts(text)
            else:
                self.reply(404, {"success": False, "error": "not found"})
                return
            self.reply(200, result)
        except Exception as exc:
            self.reply(500, {"success": False, "error": str(exc)})


def main():
    required_env("ARK_API_KEY")
    required_env("VOLC_TTS_APPID")
    required_env("VOLC_TTS_TOKEN")
    if not shutil.which(os.environ.get("TTS_PLAYER", "mplayer")):
        raise RuntimeError("mplayer not found")
    start_offline_voice()
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    print("[ai_backend] ready at http://{}:{}".format(HOST, PORT), flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
