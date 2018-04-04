#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import Twist2DStamped

class P01(object):
	def __init__(self):
		super(P01, self).__init__()
		self.publisher = rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd", Twist2DStamped, queue_size=10)
		self.subscriber = rospy.Subscriber("/duckiebot/joy", Joy, self.callback)
		self.twist = Twist2DStamped()


	#def publicar(self):

	#def callback(self,msg):
	def callback(self,msg):
		speed = msg.axes[1]
		self.twist.v = speed
#		print self.twist.v

		ebrake = msg.buttons[2]

		angle = msg.axes[0]
		self.twist.omega = 10*angle
		print self.twist.omega
		
		if ebrake == 1:
			self.twist.v = 0
			self.twist.omega = 0

		self.publisher.publish(self.twist)

def main():
	rospy.init_node('test') #creacion y registro del nodo!

	obj = P01() # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
