#!/usr/bin/env python
from std_msgs.msg import Bool, Float32, String
import rospy
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from bearing_estimator.srv import estimate_bearing
import numpy as np

class BearingEstimator:
    def __init__(self):

        self.bridge = CvBridge()
        self.img = []

        s = rospy.Service('estimate_bearing', 
                            estimate_bearing, 
                            self.handle_estimate_bearing)

        rospy.Subscriber("/camera/image_raw", 
                                    Image, 
                                    self.image_loader)

    def image_loader(self,msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def handle_estimate_bearing(self, req):
        # Include CV bearing calculations
        print("Caluclating bearing")
        ret = estimate_bearingResponse()
        ret.detected = True
        ret.bearing = np.random.rand()*2*np.pi
        return ret


if __name__ == "__main__":
    try:
        # Init ROS node
        rospy.init_node('bearing_estimator')

        # Instatiated the class
        NC = BearingEstimator()

        # Wait
        rospy.spin()
    except rospy.ROSInterruptException:
        pass