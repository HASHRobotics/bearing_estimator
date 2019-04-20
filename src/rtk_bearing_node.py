#!/usr/bin/env python
from std_msgs.msg import Bool, Float32, String
import rospy
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from bearing_estimator.msg import bearing_msg
from bearing_estimator.srv import ground_truth_bearing, ground_truth_bearingResponse
import numpy as np
from geometry_msgs.msg import PoseWithCovariance

class BearingEstimator:
    def __init__(self):

        rospy.Service('ground_truth_bearing',
                            ground_truth_bearing,
                            self.handle_ground_truth_bearing)

        rospy.Subscriber("/piksi/enu_pose_fix",PoseWithCovariance, self.calculate_rtk_bearing)
        self.pub = rospy.Publisher('/rtk_bearing', bearing_msg, queue_size=10)
        self.stationary_rover_to_base_station = np.eye(4)
        self.true_bearing = 0

    def calculate_rtk_bearing(self, msg):
        point = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.positon.z, 1]).T
        transformed_point = np.matmul(self.stationary_rover_to_base_station, point)
        bearing = math.atan2(transformed_point[2]/transformed_point[1])
        self.true_bearing = bearing

    def handle_ground_truth_bearing(self, req):
        ret = ground_truth_bearingResponse()
        if self.true_bearing:
            ret.detected = False
        else:
            ret.detected = True
        ret.bearing = self.true_bearing

        current_true_bearing = bearing_msg()
        current_true_bearing.bearing = self.true_bearing
        current_true_bearing.header.stamp = rospy.get_rostime()
        self.pub.publish(current_true_bearing)


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
