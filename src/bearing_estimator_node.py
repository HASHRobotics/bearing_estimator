#!/usr/bin/env python
from std_msgs.msg import Bool, Float32, String
import rospy
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from bearing_estimator.msg import bearing_msg
from bearing_estimator.srv import estimate_bearing, estimate_bearingResponse
import numpy as np
import imutils

class BearingEstimator:
	def __init__(self):

		self.bridge = CvBridge()
		self.img = []
		self.centroid_detected = False

		s = rospy.Service('estimate_bearing', 
							estimate_bearing, 
							self.handle_estimate_bearing)

		rospy.Subscriber("/camera/image_color", 
									Image, 
									self.image_loader)

	def get_good_contours(self, c, new_width, new_height):
		'''
		INPUT: Takes in the contour and the height and width of the image
		OUTPUT: Returns the contours which lie within the specified screen range provided
		'''
		l = []
		for j, elt in enumerate(c):
			rightmost = tuple(elt[elt[:,:, 0].argmax()][0])
			topmost = tuple(elt[elt[:,:, 1].argmin()][0])
			if ((topmost[1] > 3 * new_height / 10 and topmost[1] < 7 * new_height / 10) and (
					rightmost[0] > (0.4 * new_width))):
				l.append(elt)
		return l

	def image_loader(self,msg):
		self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
		print(self.img.shape)
		new_height, new_width, channels = self.img.shape
		self.new_height = 600
		self.new_width = 900
		img = imutils.resize(self.img, height=self.new_height, width=self.new_width)

		output = self.img

		COLOR_MIN = np.array([50,25,0], np.uint8)
		COLOR_MAX = np.array([90,255,255], np.uint8)

		output = cv2.cvtColor(output, cv2.COLOR_BGR2HSV)

		frame_threshed = cv2.inRange(output, COLOR_MIN, COLOR_MAX)



		frame_threshed = frame_threshed/255.0
		struct1 = np.array([[1,1,1],[1,1,1],[1,1,1]])
		# struct1 = np.array([[0,1,0],[1,1,1],[0,1,0]])
		# frame_threshed = scipy.ndimage.morphology.binary_erosion(frame_threshed, structure = struct1, iterations = 1)
		# frame_threshed = scipy.ndimage.morphology.binary_dilation(frame_threshed, iterations = 4)
		frame_threshed = frame_threshed*255.0
		frame_threshed = frame_threshed.astype("uint8")

		im2, contours = cv2.findContours(frame_threshed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		print(len(contours))
		if (len(contours) == 0):
			stro = "No contour detected for image"
			
		contours = self.get_good_contours(im2, new_width, new_height)
		if (len(contours) == 0):
			stro = "No Good contour detected for image"
			
		# Detecting max contour & making sure it is bigger than a threshold
		c = max(contours, key=cv2.contourArea)
		if not cv2.contourArea(c) < 0:
			stro = "Detected contours"
			x,y,w,h = cv2.boundingRect(c)
			print(stro)
			self.cX = int(x + (w / 2))
			self.cY = int(y + (h / 2))
			



	def handle_estimate_bearing(self):
		# Include CV bearing calculations
		print("Caluclating bearing")
		ret = estimate_bearingResponse()
		current_bearing = bearing_msg()
		current_bearing.bearing = np.arctan2([self.cY- self.new_height],[self.cX - self.new_width])
		ret.detected = True
		ret.bearing = current_bearing
		print(current_bearing)
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
