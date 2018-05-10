#!/usr/bin/env python
# - - - - - - - - - - - - - - - - - - - - - - -
# imports

import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from cv_bridge import CvBridge, CvBridgeError


# - - - - - - - - - - - - - - - - - - - - - - -
# yellow color limit values in HSV

#lower_yellow = np.array([40, 50, 50])		# original
lower_yellow = np.array([25, 98, 195])
#upper_yellow = np.array([100, 255, 255])	# original
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

		kernel = np.ones((5,5),np.uint8)			# Matriz de 1's 5x5

# here!		# processing AND
		img_out = cv.bitwise_and(img_hsv, img_hsv, mask = mask)
		img_out = cv.cvtColor(img_out, cv.COLOR_HSV2BGR)

		# eroding - dilating
		mask = cv.erode(mask, kernel, iterations = 2)
		mask = cv.dilate(mask, kernel, iterations = 12)

		# blob contour
		image, contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
		rectanglelist = [cv.contourArea(c) for c in contours]
		rectangles = len(rectanglelist)

		print 'rectangles: ' + str(rectangles)

		if rectangles > 0:
			for c in contours:
				x,y,w,h = cv.boundingRect(c)

				cv.rectangle(img_bgr, (x,y), (x+w, y+h), (0,255,0), 2)

		# final image as msg
		final_img = self.bridge.cv2_to_imgmsg(img_bgr, "bgr8")
		##final_img = self.bridge.cv2_to_imgmsg(img_out, "bgr8")				# ESTO ES PARA VER LA MASCARA-

		# - - - - - - - - - - - - - - - - - - - - - - -
		# publish

		self.publisher.publish(final_img)


# - - - - - - - - - - - - - - - - - - - - - - -
# ???	error404 - file not found

def main():
	rospy.init_node('test') #creacion y registro del nodo!

	obj = Detector() # Crea un objeto del tipo Detector, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
