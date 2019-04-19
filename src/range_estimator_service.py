#!/usr/bin/env python
import rospy
import time
import serial
from std_msgs.msg import String
from sensor_msgs.msg import Range

from collections import deque

from bearing_estimator.srv import estimate_range, ground_truth_range, get_rangeResponse
from nav_msgs.msgs import Odometry

class RangeEstimator:
    def __init__(self):
        self.distance = None
        rospy.Service('estimate_range',
                        estimate_range,
                        self.handle_estimate_range)

        rospy.Service('ground_truth_range',
                        ground_truth_range,
                        self.handle_ground_truth_range)

        rospy.Subscriber("estimate_range",
                            Range,
                            self.set_estimated_distance)

        rospy.Subscriber("rtk_range",
                            Range,
                            self.set_rtk_range)

        self.estimated_pub = rospy.Publisher('estimated_range', Range, queue_size=10)
        self.true_pub = rospy.Publisher('true_range', Range, queue_size=10)
        self.true_distance = None
        self.estimated_distance = None

        def set_estimated_distance(self, msg):
            self.estimated_distance = msg.range

        def set_rtk_range(self, msg):
            self.true_distance = msg.range

    def handle_ground_truth_range(self, req):
        ret = get_rangeResponse()
        if self.true_distance == None:
            ret.detected = False
        else:
            ret.detected = True
        ret.range = self.true_distance

        current_true_range = Range()
        current_true_range.range = self.real_distance
        current_true_range.header.stamp = rospy.get_rostime()
        self.true_pub.publish(current_true_range)


    def handle_estimate_range(self, req):
        ret = get_rangeResponse()
        if self.estimated_distance == None:
            ret.detected = False
        else:
            ret.detected = True
        ret.range = self.estimated_distance

        current_estimated_range = Range()
        current_estimated_range.range = self.estimated_distance
        current_estimated_range.header.stamp = rospy.get_rostime()
        self.estimated_pub.publish(current_estimated_range)

        return ret



if __name__ == "__main__":
    try:
        # Init ROS node
        rospy.init_node('range_estimator')

        # Instatiated the class
        NC = RangeEstimator()

        # Wait
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
