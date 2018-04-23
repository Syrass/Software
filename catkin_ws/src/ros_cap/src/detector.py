#!/usr/bin/env python
# - - - - - - - - - - - - - - - - - - - - - - -
# imports

import rospy
import cv2 as cv
from std_msgs.msg import String, Int32

#from geometry_msgs.msg import Twist

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

import numpy as np


# - - - - - - - - - - - - - - - - - - - - - - -
# yellow colors

lower_yellow = np.array([40, 50, 50])
upper_yellow = np.array([100, 255, 255])


# - - - - - - - - - - - - - - - - - - - - - - -
# detector

class Detector(object):

	def __init__(self):

		super(Detector, self).__init__()
		self.img_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, self.callback)
		self.publisher = rospy.Publisher("detector_imagenfiltrada", Image, queue_size=10)
		self.cv_image = Image()
		self.bridge = CvBridge()

	#def publicar(self):		<-- idk what da hell is dis


	def callback(self, msg):
		
		# exception
		try:
	 	       self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print(e)

		# - - - - - - - - - - - - - - - - - - - - - - -
		# drama	shit
		
		# renaming received image
		img_bgr = self.cv_image

		# BGR to HSV - color type
		img_hsv = cv.cvtColor(img_bgr, cv.COLOR_BGR2HSV)

		# filter
		mask = cv.inRange(img_hsv, lower_yellow, upper_yellow)

# here!		# processing
		processed = cv.threshold(mask, 127, 255, cv.THRESH_BINARY)
#		processed = cv.cvtColor(processed, cv.COLOR_HSV2BGR)

#		img_out = cv.cvtColor(processed, cv.COLOR_GRAY2BGR)


		# final image
		final_img = self.bridge.cv2_to_imgmsg(img_bgr, "bgr8")	#BGR8

		# - - - - - - - - - - - - - - - - - - - - - - -
		# publish
		
		self.publisher.publish(final_img)

#		self.publisher.publish(self.bridge.cv2_to_imgmsg(processed, "bgr8"))


# - - - - - - - - - - - - - - - - - - - - - - -
# ???	error404 - file not found

def main():
	rospy.init_node('test') #creacion y registro del nodo!

	obj = Detector() # Crea un objeto del tipo Detector, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
