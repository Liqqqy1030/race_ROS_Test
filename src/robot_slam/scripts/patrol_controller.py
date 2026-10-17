#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""四次读题 -> 按答案访问黄色格 -> 终点。兼容Python2/3语法。
视觉使用带request_id的接口；不接收旧的1~8编号，以免错走格子。
"""
from __future__ import unicode_literals
import json
import math
import re
import threading
import time
import uuid
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from TTS_audio.srv import StringService


class Patrol(object):
    def __init__(self):
        self.points = rospy.get_param('~read_points')
        self.targets = rospy.get_param('~targets')
        self.finish = rospy.get_param('~finish')
        self.answer_numbers = {
            '31': 1, '32': 2, '33': 3,
            '40': 4, '41': 5, '42': 6,
            '49': 7, '50': 8, '51': 9,
        }
        if len(self.points) != 4 or set(self.targets) != set(
                ['31', '32', '33', '40', '41', '42', '49', '50', '51']):
            raise ValueError('Expected four read points and nine yellow cells')
        for pose in self.points + list(self.targets.values()) + [self.finish]:
            if len(pose) != 3 or any(math.isnan(float(v)) or math.isinf(float(v)) for v in pose):
                raise ValueError('Invalid waypoint')
        self.run_id = uuid.uuid4().hex
        self.stage = 'BOOT'
        self.started = threading.Event()
        self.lock = threading.Lock()
        self.pending = None
        self.result = None
        self.stage_pub = rospy.Publisher('/patrol/stage', String, queue_size=10, latch=True)
        self.request_pub = rospy.Publisher('/patrol/vision/request', String, queue_size=1)
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.tts = rospy.ServiceProxy('/tts_service', StringService)
        rospy.Subscriber('/start_mission', String, self.on_start)
        rospy.Subscriber('/patrol/vision/result', String, self.on_result)
        rospy.on_shutdown(self.stop)

    def set_stage(self, stage):
        self.stage = stage
        self.stage_pub.publish(String(data=json.dumps(dict(run_id=self.run_id, stage=stage))))
        rospy.loginfo(stage)

    def on_start(self, msg):
        if self.stage == 'WAIT_START' and msg.data.strip() in ('start', '比赛开始'):
            self.started.set()
            return
        try:
            command = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if (self.stage == 'WAIT_START' and isinstance(command, dict)
                and command.get('command') == 'start'
                and command.get('run_id') == self.run_id):
            self.started.set()

    def on_result(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                return
            with self.lock:
                if self.pending and self.result is None and all(
                        value.get(k) == self.pending[k] for k in ('run_id', 'request_id', 'task_id')):
                    self.result = value
        except (ValueError, TypeError):
            rospy.logwarn('Invalid vision JSON')

    def stop(self):
        self.client.cancel_goal()

    def speak(self, text):
        # 同步等待播报完成，但给网络/TTS设置独立墙钟超时。
        done = threading.Event()
        outcome = []
        def call():
            try:
                outcome.append(self.tts(text).result)
            except Exception as exc:
                outcome.append('error:' + str(exc))
            finally:
                done.set()
        worker = threading.Thread(target=call)
        worker.daemon = True
        worker.start()
        deadline = time.time() + 45
        while not done.wait(0.1):
            if rospy.is_shutdown() or time.time() >= deadline:
                raise RuntimeError('TTS timeout; mission stopped')
        if not outcome or outcome[0] not in ('ok', 'TTS处理完成'):
            raise RuntimeError('TTS failed')

    def go(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(pose[0])
        goal.target_pose.pose.position.y = float(pose[1])
        yaw = math.radians(float(pose[2]))
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2)
        self.client.send_goal(goal)
        deadline = time.time() + float(rospy.get_param('~navigation_timeout', 90))
        while not rospy.is_shutdown():
            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.sleep(float(rospy.get_param('~settle_seconds', 2)))
                return
            if state in (GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED,
                         GoalStatus.RECALLED, GoalStatus.LOST) or time.time() > deadline:
                self.client.cancel_goal()
                raise RuntimeError('Navigation failed or timed out')
            time.sleep(0.1)
        raise RuntimeError('Shutdown')

    def recognize(self, task_id):
        for attempt in range(int(rospy.get_param('~vision_attempts', 3))):
            request = dict(run_id=self.run_id, request_id=uuid.uuid4().hex, task_id=task_id)
            with self.lock:
                self.pending, self.result = request, None
            self.request_pub.publish(String(data=json.dumps(request)))
            deadline = time.time() + float(rospy.get_param('~vision_timeout', 30))
            value = None
            while not rospy.is_shutdown() and time.time() < deadline:
                with self.lock:
                    value = self.result
                if value is not None:
                    break
                time.sleep(0.1)
            with self.lock:
                self.pending = None
            if value and value.get('success') is True:
                raw_answer = str(value.get('answer', '')).strip()
                match = re.search(r'(?<!\d)(31|32|33|40|41|42|49|50|51)(?!\d)', raw_answer)
                answer = match.group(1) if match else raw_answer
                if answer in self.targets:
                    return answer, value.get('text', '')
            if rospy.is_shutdown():
                break
        raise RuntimeError('No valid yellow-cell answer')

    def run(self):
        if not self.client.wait_for_server(rospy.Duration(30)):
            raise RuntimeError('move_base unavailable')
        rospy.wait_for_service('/tts_service', timeout=30)
        self.set_stage('WAIT_VISION')
        while (rospy.get_param('~require_vision_ready', True)
               and self.request_pub.get_num_connections() == 0):
            if rospy.is_shutdown():
                return
            time.sleep(0.1)
        self.speak('系统已就绪，请下达开始口令')
        self.set_stage('WAIT_START')
        while not self.started.wait(0.1):
            if rospy.is_shutdown():
                return
        self.set_stage('READING')
        self.speak('比赛开始')
        answers = []
        for index, pose in enumerate(self.points, 1):
            self.speak('前往第%d个读题点' % index)
            self.go(pose)
            self.set_stage('RECOGNIZING_%d' % index)
            answer, text = self.recognize(index)
            answers.append(answer)
            self.speak('第%d个线索是%d号' % (index, self.answer_numbers[answer]))
        rospy.set_param('/patrol/answers', answers)
        self.set_stage('EXECUTING')
        # 保留重复答案：题目要求两次访问同一格时，也按顺序执行。
        for answer in answers:
            self.go(self.targets[answer])
            self.speak('到达任务点%d号' % self.answer_numbers[answer])
        self.set_stage('RETURNING')
        self.speak('前往终点')
        self.go(self.finish)
        self.speak('导航结束')
        self.set_stage('DONE')
        rospy.spin()  # 保持完成状态，不自动重跑。


if __name__ == '__main__':
    rospy.init_node('patrol_controller')
    patrol = None
    try:
        patrol = Patrol()
        patrol.run()
    except Exception as exc:
        if patrol is not None:
            patrol.stop()
            patrol.set_stage('ABORTED')
        rospy.logerr('Patrol stopped: %s', exc)
        if not rospy.is_shutdown():
            rospy.spin()
