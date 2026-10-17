#!/usr/bin/env python
# _*_ coding:utf-8 _*_

import rospy
import numpy as np
import os,cv2
import math
import imutils

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage

def nothing(x):
    pass



class AbotHsvDetect():
    def __init__(self):
        rospy.init_node('abot_hsv_tools_node', log_level=rospy.INFO)
        self.rate = rospy.get_param('~rate', 20)
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/image_raw/')
        r = rospy.Rate(self.rate)
        rospy.on_shutdown(self.shutdown)

        self.sub_image_type = "raw"
        self.pub_face_type = "raw"
        self.cv_image = None
	cv2.namedWindow("AbotHSVBars")
        cv2.createTrackbar("LH", "AbotHSVBars", 0, 179, nothing)
        cv2.createTrackbar("LS", "AbotHSVBars", 0, 255, nothing)
        cv2.createTrackbar("LV", "AbotHSVBars", 0, 255, nothing)
        cv2.createTrackbar("UH", "AbotHSVBars", 179, 179, nothing)
        cv2.createTrackbar("US", "AbotHSVBars", 255, 255, nothing)
        cv2.createTrackbar("UV", "AbotHSVBars", 255, 255, nothing)



        if self.sub_image_type == "compressed":
            self.sub_image_original = rospy.Subscriber(self.camera_topic+'compressed', CompressedImage, self.ImageCallback, queue_size = 1)
        elif self.sub_image_type == "raw":
            self.sub_image_original = rospy.Subscriber(self.camera_topic, Image, self.ImageCallback, queue_size = 1)

        self.cvBridge = CvBridge()

        while not rospy.is_shutdown():

            # Resize frame of video to 1/4 size for faster face recognition processing
            if self.cv_image is not None:
                
		hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

    		l_h = cv2.getTrackbarPos("LH", "AbotHSVBars")
    		l_s = cv2.getTrackbarPos("LS", "AbotHSVBars")
    		l_v = cv2.getTrackbarPos("LV", "AbotHSVBars")
    		u_h = cv2.getTrackbarPos("UH", "AbotHSVBars")
    		u_s = cv2.getTrackbarPos("US", "AbotHSVBars")
    		u_v = cv2.getTrackbarPos("UV", "AbotHSVBars")
 		lower_blue = np.array([l_h, l_s, l_v])
   	 	upper_blue = np.array([u_h, u_s, u_v])
    		mask = cv2.inRange(hsv, lower_blue, upper_blue)

    		result = cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask)

    		cv2.imshow("AbotOrgin", self.cv_image), cv2.waitKey(1)
    		cv2.imshow("AbotMask", mask), cv2.waitKey(1)
    		cv2.imshow("AbotResult", result), cv2.waitKey(1)

            #r.sleep()


    def ImageCallback(self, image_msg):
        if self.sub_image_type == "compressed":
            #converting compressed image to opencv image
            np_arr = np.fromstring(image_msg.data, np.uint8)
            self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == "raw":
            self.cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")


    def shutdown(self):
        # Release handle to the webcam
        rospy.logwarn("now will shutdown hsv_tools_node ...")

if __name__ == '__main__':
    try:
        detect = AbotHsvDetect()
        rospy.spin()
    except Exception, e:
        rospy.logerr("%s", str(e))
        os._exit(1)

