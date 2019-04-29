#!/usr/bin/env python
from std_msgs.msg import Bool, Float32, String
import math
import rospy
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from bearing_estimator.msg import bearing_msg
from bearing_estimator.srv import ground_truth_bearing, ground_truth_bearingResponse
import numpy as np
from geometry_msgs.msg import PoseWithCovariance
from piksi_rtk_msgs.msg import BaselineNed

class BearingEstimator:
    def __init__(self):

        rospy.Service('ground_truth_bearing',
                            ground_truth_bearing,
                            self.handle_ground_truth_bearing)

        rospy.Subscriber("/ak2/piksi/baseline_ned",BaselineNed, self.calculate_rtk_bearing)

        # pose_msg  = rospy.wait_for_message("/piksi/enu_pose_fix", PoseWithCovariance)
        # self.base_station_x = pose_msg.pose.position.x
        # self.base_station_y = pose_msg.pose.position.y
        # self.compass_angle = 0

        self.base_station_x = 0
        self.base_station_y = 0
        self.compass_angle = 0

        self.pub = rospy.Publisher('/rtk_bearing', bearing_msg, queue_size=10)
        self.stationary_rover_to_base_station = np.eye(4)
        self.true_bearing = 0

    def calculate_rtk_bearing(self, msg):
        x = msg.e/1000.0
        y = msg.n/1000.0
        angle = math.atan2(x - self.base_station_x, y - self.base_station_y)
        bearing = angle - self.compass_angle
        self.true_bearing = bearing

    def handle_ground_truth_bearing(self, req):
        ret = ground_truth_bearingResponse()
        print(self.true_bearing)
        if self.true_bearing:
            ret.detected = True
        else:
            ret.detected = False

        current_true_bearing = bearing_msg()
        current_true_bearing.bearing = self.true_bearing
        current_true_bearing.header.stamp = rospy.get_rostime()
        self.pub.publish(current_true_bearing)

        ret.bearing = current_true_bearing
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
