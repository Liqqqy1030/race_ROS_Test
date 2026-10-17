#!/usr/bin/env python
# _*_ coding:utf-8 _*_

import rospy
import numpy as np
import os,cv2
import math
import imutils

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import  BoundingBox2D, Detection2DArray,ObjectHypothesisWithPose,Detection2D


class HsvDetect():
    def __init__(self):
        rospy.init_node('abot_ball_detect_node', log_level=rospy.INFO)
        self.rate = rospy.get_param('~rate', 5)
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/image_raw/')
        self.real_area = rospy.get_param('~real_area', 600)

        c1 = rospy.get_param("~color1", [16, 101, 171, 77, 255, 255, 'red'])
        c2 = rospy.get_param("~color2", [20, 100, 100, 30, 255, 255, 'yellow'])
        c3 = rospy.get_param("~color3", [35, 43, 35, 90, 255, 255, 'green'])

        r = rospy.Rate(self.rate)
        self.name = rospy.get_name()
        rospy.on_shutdown(self.shutdown)

        self.sub_image_type = "raw"
        self.pub_face_type = "raw"
        self.cv_image = None


        self.color_dist = {
             c1[6]: {'Lower': np.array([c1[0], c1[1], c1[2]]), 'Upper': np.array([c1[3], c1[4], c1[5]])},
             c2[6]: {'Lower': np.array([c2[0], c2[1], c2[2]]), 'Upper': np.array([c2[3], c2[4], c2[5]])},
             c3[6]: {'Lower': np.array([c3[0], c3[1], c3[2]]), 'Upper': np.array([c3[3], c3[4], c3[5]])},
            }


        if self.sub_image_type == "compressed":
            self.sub_image_original = rospy.Subscriber(self.camera_topic+'compressed', CompressedImage, self.ImageCallback, queue_size = 1)
        elif self.sub_image_type == "raw":
            self.sub_image_original = rospy.Subscriber(self.camera_topic, Image, self.ImageCallback, queue_size = 1)


        if self.pub_face_type == "compressed":
            self.pub_face_image = rospy.Publisher('/object_image/compressed', CompressedImage, queue_size=1)
        else:
            self.pub_face_image = rospy.Publisher('/object_image', Image, queue_size=1)

        self.object_pub = rospy.Publisher("/detect_node/detections", Detection2DArray,  queue_size=1)

        self.cvBridge = CvBridge()
	center = None
       

        while not rospy.is_shutdown():

            # Resize frame of video to 1/4 size for faster face recognition processing
            if self.cv_image is not None:
		blurred = cv2.GaussianBlur(self.cv_image, (5, 5), 0)
        	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        	erode_hsv = cv2.erode(hsv, None, iterations=2)
                for i in self.color_dist:
                    mask = cv2.inRange(erode_hsv, self.color_dist[i]['Lower'], self.color_dist[i]['Upper'])
                    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
                    if len(cnts) > 0:
                        c = max(cnts, key=cv2.contourArea)
                        rect = cv2.minAreaRect(c)
                        box = cv2.boxPoints(rect)
                        cv2.drawContours(self.cv_image, [np.int0(box)], -1, (0, 255, 255), 2)
                        cx, cy = rect[0]
                        h, w = rect[1]
                        self.c_angle = rect[2]

                        # only proceed if the radius meets a minimum size
                        if h*w > self.real_area:
                            # draw the circle and centroid on the frame,
                            # then update the list of tracked points
                            cv2.circle(self.cv_image, (int(cx), int(cy)), 3, (216, 0, 255), -1)
                            objArray = Detection2DArray()
                            objArray.detections =[]
                            objArray.header.frame_id = i
                            obj=Detection2D()
                            obj_hypothesis= ObjectHypothesisWithPose()
                            obj.header.frame_id = i
                            obj_hypothesis.id = 1
                            obj_hypothesis.score = 1
                            obj.results.append(obj_hypothesis)
                            obj.bbox.size_y = h
                            obj.bbox.size_x = w
                            obj.bbox.center.x = cx
                            obj.bbox.center.y = cy
                            objArray.detections.append(obj)
                            self.object_pub.publish(objArray)

	        if self.pub_face_type == "compressed":
                # publishes traffic sign image in compressed type
                    self.pub_face_image.publish(self.cvBridge.cv2_to_compressed_imgmsg(self.cv_image, "jpg"))

                elif self.pub_face_type == "raw":
                # publishes traffic sign image in raw type
                    self.pub_face_image.publish(self.cvBridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
            r.sleep()


    def ImageCallback(self, image_msg):
        if self.sub_image_type == "compressed":
            #converting compressed image to opencv image
            np_arr = np.fromstring(image_msg.data, np.uint8)
            self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == "raw":
            self.cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")


    def shutdown(self):
        # Release handle to the webcam
        rospy.logwarn("now will shutdown face_location_node ...")

if __name__ == '__main__':
    try:
        detect = HsvDetect()
        rospy.spin()
    except Exception, e:
        rospy.logerr("%s", str(e))
        os._exit(1)

