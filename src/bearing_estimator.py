#!/usr/bin/env python

from std_msgs.msg import Bool, Float32, String
import rospy
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np


import sys



class Bearing_Estimator():

    def __init__(self):
        rospy.init_node('bearing_estimator')

        # default_camera = "/intel-realsense"  # ch`aneg the default camera
        # camera = rospy.get_param('camera', default_camera)
        # pan = rospy.get_param('pan')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.detector)

    # def _image_callback(self, sensor_msgs::ImageConstPtr& msg):
    #     self._img_msg = msg

    def detector(self,msg):
        print("Received an image!")
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)
        # try:
        #     # Convert your ROS Image message to OpenCV2
        #     cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # except CvBridgeError, e:
        #     print(e)
        # else:
        #     # Save your OpenCV2 image as a jpeg
        #     time = msg.header.stamp
        #     cv2.imwrite('' + str(time) + '.jpeg', cv2_img)
        #     rospy.sleep(1)

        PINK_MIN = np.array([165, 50, 50], np.uint8)
        PINK_MAX = np.array([175, 255, 255], np.uint8)

        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # img = cv2.medianBlur(img,7)
        frame_threshed = cv2.inRange(img, PINK_MIN, PINK_MAX)
        # cv2.imwrite('output2.jpg', frame_threshed)
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        output = cv2.bitwise_and(img, img, mask=frame_threshed)
        cv2.imshow("images", np.hstack([img, output]))

        # For real-sense
        kernel = np.ones((5, 5), np.uint8)
        opening = cv2.morphologyEx(frame_threshed, cv2.MORPH_OPEN, kernel)
        # cv2.imshow("opening", opening)

        # find contours in the binary image
        im2, contours, hierarchy = cv2.findContours(opening, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if (len(contours) == 0):
            stro = "No contour detected for image " + "lllllllllllll"
            print(stro)

        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < 28:
            stro = "Detected contour too small"

        x, y, w, h = cv2.boundingRect(c)
        # draw the book contour (in green)
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

        M = cv2.moments(c)

        # calculate x,y coordinate of center
        if M["m00"] != 0:
            print "centroid detected"
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            print "centroid not detected"
            stro = "No centroid detected for image " + str(sys.argv[1])
            print(stro)
        cX, cY = 0, 0



    def bearing_calculator(self, x_location,y_location,height_image, width_image):

        pass

if __name__ == '__main__':
    try:
        bear_estim = Bearing_Estimator()

    except rospy.ROSInterruptException:
        pass
    rospy.spin()
