#!/usr/bin/env python

from sensor_msgs.msg import Imu, LaserScan
from vitarana_drone.msg import *
import cv2
import math
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospy
from std_msgs.msg import Float32
import time
import tf
from vitarana_drone.msg import MarkerData


class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('detect_marker_cascade') #Initialise rosnode 

		self.img = np.empty([]) # This will contain your image frame from camera
		self.gray = np.empty([])
		self.bridge = CvBridge()
		self.logo_cascade = cv2.CascadeClassifier('/home/naman/Downloads/intro_cascade_classifiers_training_and_usage/data/cascade.xml')
      		self.sample_time = 1
		self.Z_m = 0
		rospy.Subscriber('/zm_val',Float32,self.range_finder_bottom)
	        self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic

		self.marker = rospy.Publisher('/edrone/marker_data', MarkerData, queue_size = 3)
		self.markobj = MarkerData()
		self.r1 = rospy.Rate(1)
		self.marker_detected = False

	def range_finder_bottom(self,down):   #defining range finder bottom
		self.Z_m = down.data

	# Callback function of amera topic
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
	         	self.gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY) # image, reject levels level weights.

			logo = self.logo_cascade.detectMultiScale(self.gray, scaleFactor=1.05)
			if len(logo) == 0:
				self.marker_detected = False
				self.markobj.marker_id = 1
				self.markobj.err_x_m = float("NaN")
				self.markobj.err_y_m = float("NaN")
				return
			self.marker_detected = True

			for (x, y, w, h) in logo:
 			   cv2.rectangle(self.img, (x, y), (x + w, y + h), (255, 255, 0), 2)
			centre_x_pixel = x + (w/2)
			centre_y_pixel = y + (h/2)
			focal_length = (400/2)/math.tan(1.3962634/2)
			self.markobj.marker_id = 1
			cv2.imshow('img' ,self.img)
			if (self.Z_m >25):
			    self.Z_m = 25
			#self.Z_m =5.2
			self.markobj.err_x_m = (200 - centre_x_pixel)*self.Z_m/focal_length
			self.markobj.err_y_m = (200 - centre_y_pixel)*self.Z_m/focal_length
			#print(self.markobj.err_x_m, self.markobj.err_y_m ,self.Z_m)
			cv2.waitKey(1)

		except CvBridgeError as e:
			print(e)
			return
	def main(self):
	    try:
	        #self.gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY) # image, reject levels level weights.
		self.marker.publish(self.markobj)
		print(self.markobj)
	    except CvBridgeError as e:
		    print(e)
		    return

if __name__ == '__main__':
    	image_proc_obj = image_proc()
    	r = rospy.Rate(1)
    	while not rospy.is_shutdown():
    		image_proc_obj.main()
        	r.sleep()

