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
import scipy.ndimage

class BearingEstimator:
	def __init__(self):

        #self.name = rospy.get_param("name")
		self.bridge = CvBridge()
		self.img = []
		self.centroid_detected = False
		self.cX = 0
		self.cY = 0
		self.counter = 0
		self.hue_min = rospy.get_param('hue_min', '70')
		self.hue_max = rospy.get_param('hue_max', '90')


		s = rospy.Service('estimate_bearing',
							estimate_bearing,
							self.handle_estimate_bearing)

        self.pub = rospy.Publisher('/bearing{0}'.format(self.name), bearing_msg, queue_size=10)


		# rospy.Subscriber("/camera/image_color",
		# 							Image,
		# 							self.image_loader)

	def get_good_contours(self, c, new_width, new_height):
		'''
		INPUT: Takes in the contour and the height and width of the image
		OUTPUT: Returns the contours which lie within the specified screen range provided
		'''
		l = []
		for j, elt in enumerate(c):
			# rightmost = tuple(elt[elt[:,:, 0].argmax()][0])
			# topmost = tuple(elt[elt[:,:, 1].argmin()][0])
			# if ((topmost[1] > 3 * new_height / 10 and topmost[1] < 7 * new_height / 10) and (rightmost[0] > (0.4 * new_width))):
			# 	l.append(elt)
			l.append(elt)
		return l


	def circular_mask(self, radius, height, width, layer = 3):
		a= int(height/2)
		b = int(width/2)
		y, x = np.ogrid[-a:height-a, -b:width-b]
		mask = x ** 2 + y ** 2 <= radius ** 2
		mask_1 = np.ones((height, width))
		mask_1[mask] = 0
		# mask = -1 * mask.astype(float) + 1
		# mask_1 = convolve(background, mask) - sum(sum(mask)) + 1
		mask_ = np.zeros((height, width,layer))
		for i in range(layer):
			mask_[:,:,i] = mask_1
		return mask_


	def handle_estimate_bearing(self, req):
		#image loader and centroid estimation
		self.counter += 1
		image_msg = rospy.wait_for_message("/camera/image_color", Image)

		self.img = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")

		new_height, new_width, channels = self.img.shape
		self.new_height = 600
		self.new_width = 900
		self.img = imutils.resize(self.img, height=self.new_height, width=self.new_width)
##MASK in
		new_height, new_width, channels = self.img.shape
		center_mask = self.circular_mask(170, new_height, new_width, 3)
		self.img = np.multiply(center_mask,self.img)
		self.img = self.img.astype("uint8")

#Mask Out
		center_mask = self.circular_mask(350,new_height, new_width, 3)
		center_mask = 1 - center_mask
		self.img = np.multiply(center_mask,self.img)
		self.img = self.img.astype("uint8")

		output = self.img
		COLOR_MIN = np.array([70,25,0], np.uint8)
		COLOR_MAX = np.array([90,255,255], np.uint8)

		output = cv2.cvtColor(output, cv2.COLOR_BGR2HSV)

		frame_threshed = cv2.inRange(output, COLOR_MIN, COLOR_MAX)


		frame_threshed = frame_threshed/255.0
		struct1 = np.array([[1,1,1],[1,1,1],[1,1,1]])
		# struct1 = np.array([[0,1,0],[1,1,1],[0,1,0]])
		frame_threshed = scipy.ndimage.morphology.binary_erosion(frame_threshed, structure = struct1, iterations = 2)
		# frame_threshed = scipy.ndimage.morphology.binary_dilation(frame_threshed, iterations = 3 )
		frame_threshed = frame_threshed*255.0
		frame_threshed = frame_threshed.astype("uint8")

		# cv2.rectangle(self.img,(x,y),(x+w,y+h),(0,255,0),2)
		cv2.imwrite(''+str(self.counter)+'.jpg', frame_threshed)

		im2, contours = cv2.findContours(frame_threshed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


		if (len(contours) == 0):
			stro = "No contour detected for image"

		contours = self.get_good_contours(im2, new_width, new_height)
		print("contour", contours)

		if (len(contours) == 0):
			stro = "No Good contour detected for image"
		print("here")
		# Detecting max contour & making sure it is bigger than a threshold
		print(len(contours))
		if (len(contours) > 0):
			c = max(contours, key=cv2.contourArea)
			print("len > 0")
			# if not cv2.contourArea(c) < 0:
			stro = "Detected contours"
			x,y,w,h = cv2.boundingRect(c)
			print(stro)
			self.cX = int(x + (w / 2))
			self.cY = int(y + (h / 2))
				# print("centroid",self.cX,self.cY)
		   #CHANGE THIS PATH AS PER THE USE
			cv2.rectangle(self.img,(x,y),(x+w,y+h),(0,255,0),2)
			cv2.imwrite(''+str(self.counter)+'bb'+'.jpg', self.img)

			# Include CV bearing calculations
			print("Caluclating bearing")
			ret = estimate_bearingResponse()
			current_bearing = bearing_msg()
			if (self.cY and self.cX):
				current_bearing.bearing = np.arctan2([self.new_height/2 - self.cY],[self.cX - self.new_width/2]) *(180.0/3.14)

				ret.detected = True
				ret.bearing = current_bearing
				print("BEARING",current_bearing)
                self.pub.publish(current_bearing)
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
