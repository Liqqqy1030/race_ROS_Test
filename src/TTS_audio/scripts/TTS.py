#!/usr/bin/env python3
"""ROS service for serialized Chinese speech synthesis and playback."""
import asyncio
import gzip
import json
import os
import shutil
import subprocess
import tempfile
import threading
import uuid
import queue

import rospy
from websockets.legacy.client import connect
from std_msgs.msg import String
from TTS_audio.srv import StringService, StringServiceResponse


def _config():
    appid = os.environ.get("VOLC_TTS_APPID", "").strip()
    token = os.environ.get("VOLC_TTS_TOKEN", "").strip()
    cluster = rospy.get_param("~cluster", os.environ.get("VOLC_TTS_CLUSTER", "volcano_tts"))
    voice_type = rospy.get_param("~voice_type", os.environ.get("VOLC_TTS_VOICE", "BV002_streaming"))
    host = rospy.get_param("~host", "openspeech.bytedance.com")
    if not appid or not token:
        raise RuntimeError("请设置环境变量 VOLC_TTS_APPID 和 VOLC_TTS_TOKEN")
    return appid, token, cluster, voice_type, host


def _parse_response(data, audio_file):
    if len(data) < 4:
        raise RuntimeError("语音服务返回了不完整的数据包")
    header_size = (data[0] & 0x0F) * 4
    message_type = data[1] >> 4
    flags = data[1] & 0x0F
    if header_size < 4 or len(data) < header_size:
        raise RuntimeError("语音服务返回了无效的数据包头")
    payload = data[header_size:]

    if message_type == 0x0F:
        error_code = int.from_bytes(payload[:4], "big") if len(payload) >= 4 else "unknown"
        message = payload[8:].decode("utf-8", "replace") if len(payload) >= 8 else ""
        raise RuntimeError("语音服务错误 {}: {}".format(error_code, message))
    if message_type == 0x0B:
        if flags == 0:
            return False
        if len(payload) < 8:
            raise RuntimeError("语音服务返回了不完整的音频帧")
        sequence = int.from_bytes(payload[:4], "big", signed=True)
        size = int.from_bytes(payload[4:8], "big")
        audio = payload[8:8 + size]
        if len(audio) != size:
            raise RuntimeError("语音音频帧长度不匹配")
        audio_file.write(audio)
        return sequence < 0 or flags in (2, 3)
    if message_type == 0x0C:
        return False
    raise RuntimeError("语音服务返回了未知消息类型: {}".format(message_type))


async def _synthesize(text, output_path, timeout):
    appid, token, cluster, voice_type, host = _config()
    request = {
        "app": {"appid": appid, "token": token, "cluster": cluster},
        "user": {"uid": str(os.environ.get("VOLC_TTS_UID", "robot"))},
        "audio": {
            "voice_type": voice_type,
            "encoding": "mp3",
            "speed_ratio": float(rospy.get_param("~speed_ratio", 0.9)),
            "volume_ratio": float(rospy.get_param("~volume_ratio", 1.0)),
            "pitch_ratio": float(rospy.get_param("~pitch_ratio", 1.0)),
        },
        "request": {
            "reqid": str(uuid.uuid4()),
            "text": text,
            "text_type": "plain",
            "operation": "submit",
        },
    }
    body = gzip.compress(json.dumps(request, ensure_ascii=False).encode("utf-8"))
    packet = bytearray(b"\x11\x10\x11\x00")
    packet.extend(len(body).to_bytes(4, "big"))
    packet.extend(body)
    url = "wss://{}/api/v1/tts/ws_binary".format(host)

    async with connect(
        url,
        extra_headers={"Authorization": "Bearer; " + token},
        ping_interval=None,
        open_timeout=timeout,
        close_timeout=1,
        max_size=None,
    ) as ws:
        await ws.send(packet)
        with open(output_path, "wb") as audio_file:
            while True:
                response = await asyncio.wait_for(ws.recv(), timeout=timeout)
                if isinstance(response, str):
                    raise RuntimeError("语音服务返回了非二进制数据")
                if _parse_response(response, audio_file):
                    break
    if os.path.getsize(output_path) == 0:
        raise RuntimeError("语音服务没有返回音频")


def _play(audio_path, timeout):
    player = rospy.get_param("~player", "mplayer")
    executable = shutil.which(player)
    if not executable:
        raise RuntimeError("找不到音频播放器: {}".format(player))
    completed = subprocess.run(
        [executable, "-really-quiet", audio_path],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        timeout=timeout,
        check=False,
    )
    if completed.returncode != 0:
        detail = completed.stderr.decode("utf-8", "replace").strip()
        raise RuntimeError("音频播放失败 ({}): {}".format(completed.returncode, detail))


_request_lock = threading.Lock()
_topic_queue = queue.Queue(maxsize=20)


def handle_tts_request(req):
    text = req.data.strip()
    if not text:
        return StringServiceResponse("error:播报文本为空")

    if not _request_lock.acquire(False):
        return StringServiceResponse('error:TTS busy')
    try:
        timeout = float(rospy.get_param("~timeout", 20.0))
        if timeout <= 0:
            return StringServiceResponse("error:timeout 必须大于 0")
        audio_path = None
        loop = None
        try:
            fd, audio_path = tempfile.mkstemp(prefix="tts_", suffix=".mp3")
            os.close(fd)
            loop = asyncio.new_event_loop()
            loop.run_until_complete(asyncio.wait_for(_synthesize(text, audio_path, timeout), timeout=timeout))
            _play(audio_path, timeout)
            return StringServiceResponse("ok")
        except Exception as exc:
            rospy.logerr("TTS 请求失败: %s", exc)
            return StringServiceResponse("error:{}".format(exc))
        finally:
            if loop is not None:
                try:
                    loop.run_until_complete(loop.shutdown_asyncgens())
                except Exception:
                    pass
                loop.close()
            if audio_path and os.path.exists(audio_path):
                try:
                    os.remove(audio_path)
                except OSError as exc:
                    rospy.logwarn("清理临时音频失败: %s", exc)

    finally:
        _request_lock.release()


def _topic_callback(msg):
    """兼容原工程使用的 /robot_voice/tts_topic。"""
    text = msg.data.strip()
    if not text:
        return
    try:
        _topic_queue.put_nowait(text)
    except queue.Full:
        rospy.logwarn("语音播报队列已满，丢弃消息: %s", text)


def _topic_worker():
    """依次播放任务点信息，避免多条中文语音互相覆盖。"""
    while not rospy.is_shutdown():
        try:
            text = _topic_queue.get(timeout=0.2)
        except queue.Empty:
            continue
        try:
            request = type("TopicRequest", (), {"data": text})()
            result = handle_tts_request(request)
            if result.result != "ok":
                rospy.logerr("话题播报失败: %s", result.result)
        finally:
            _topic_queue.task_done()


def tts_server():
    rospy.init_node("tts_server")
    _config()
    if not shutil.which(rospy.get_param('~player', 'mplayer')):
        raise RuntimeError('mplayer is required')
    rospy.Service("/tts_service", StringService, handle_tts_request)
    legacy_topic = rospy.get_param("~legacy_topic", "/robot_voice/tts_topic")
    if legacy_topic:
        rospy.Subscriber(legacy_topic, String, _topic_callback, queue_size=20)
    worker = threading.Thread(target=_topic_worker)
    worker.daemon = True
    worker.start()
    rospy.loginfo("TTS 服务已启动：/tts_service；兼容话题：%s", legacy_topic)
    rospy.spin()


if __name__ == "__main__":
    tts_server()
