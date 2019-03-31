#!/usr/bin/env python
import rospy
import time
import serial
from std_msgs.msg import String
from sensor_msgs.msg import Range

from collections import deque

from bearing_estimator.srv import get_range, get_rangeResponse

class RangeEstimator:
    def __init__(self):
        self.distance = None
        s = rospy.Service('get_range', 
                            get_range, 
                            self.handle_estimate_range)

        pub = rospy.Publisher('range', Range, queue_size=10)
        
        while not rospy.is_shutdown():
            ser = serial.Serial(
                port='/dev/ttyACM0',
                baudrate=115200,
                timeout=0.1
            )

            ser.write(b'\r\r')
            res=ser.read(100000)
            time.sleep(1)
            ser.write(b'lec\r')
            print(res)

            q = deque()
            total = 0
            while True:
                res=ser.read(100)
                distance = Range()

                if len(res)>0:
                    readings = res.split('\r\n')
                    for reading in readings:
                        if 'DIST' in reading:
                            try:
                                range_deca = float(reading.split(',')[-1])
                                if range_deca:
                                    q.append(range_deca)
                                    if len(q) <= 10:
                                        total += range_deca
                                    else:
                                        removed_element = q.popleft()
                                        print(q)
                                        total = total - removed_element + range_deca
                                        distance.range = total/len(q)
                                        distance.header.stamp = rospy.get_rostime()
                                        rospy.loginfo(distance)
                                        pub.publish(distance)
                                        self.distance = distance
                            except ValueError as e:
                                rospy.loginfo("A string found")

    def handle_estimate_range(self, req):
        # Include CV bearing calculations
        print("Caluclating Range")
        ret = get_rangeResponse()
        current_range = Range()
        if self.distance == None:
            ret.detected = False
        else:
            ret.detected = True
        current_range.range = self.distance
        ret.range = current_range
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