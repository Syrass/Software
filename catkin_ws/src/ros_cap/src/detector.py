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
# yellow color limit values en HSV

#lower_yellow = np.array([40, 50, 50])		# original
lower_yellow = np.array([50, 150, 150])
#lower_yellow = np.array([100, 255, 255])	# original
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
		
		mask = cv.erode(mask, kernel, iterations = 4)
		mask = cv.dilate(mask, kernel, iterations = 16)

# here!		# processing AND
		img_out = cv.bitwise_and(img_hsv, img_hsv, mask)
		img_out = cv.cvtColor(img_bgr, cv.COLOR_HSV2BGR)

		# blob
#		contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
#		x, y, w, h = cv.boundingRect(cnt)

		# final image as msg
		final_img = self.bridge.cv2_to_imgmsg(img_out, "bgr8")

		# - - - - - - - - - - - - - - - - - - - - - - -
		# publish
		
		self.publisher.publish(final_img)

#		self.publisher.publish(self.bridge.cv2_to_imgmsg(processed, "bgr8"))

'''
BITACORA -

> Logre finalmente que se viera algo!!
Aunque sigue fallando.

No se si es mejor usar
img_out = cv.bitwise_and(img_bgr, img_bgr, mask)
o
img_out = cv.bitwise_and(img_hsv, img_hsv, mask)
img_out = cv.cvtColor(img_bgr, cv.COLOR_HSV2BGR)
(el actual).

Creo que la mascara no esta considerando los limites de color, he
probado con valores muy extremos y sigue mostrando lo mismo...
Whatever, es demasiado tarde para seguir intentando y/o ver donde
esta el problema.

-Syrass

'''


# - - - - - - - - - - - - - - - - - - - - - - -
# ???	error404 - file not found

def main():
	rospy.init_node('test') #creacion y registro del nodo!

	obj = Detector() # Crea un objeto del tipo Detector, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
