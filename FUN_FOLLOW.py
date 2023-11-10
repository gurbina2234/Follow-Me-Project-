#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from duckietown_msgs.msg import Twist2DStamped
from geometry_msgs.msg import Twist, Point# importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image
import cv2 # importar libreria opencv
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy


class Template(object):
	def __init__(self, args):
		
		super(Template, self).__init__()
		self.args = args
		self.sub = rospy.Subscriber("/duckiebot/posicion_bot",Point,self.procesar_dis)
		self.sub1= rospy.Subscriber("/duckiebot/filtro", Twist2DStamped,self.procesar_twist)
		self.publisher = rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd", Twist2DStamped, queue_size = 1)
		self.dx=300
		self.dz =300
		self.sep=0
		self.dis_area=[]
		self.twist= Twist2DStamped()
	#def publicar(self):

	#def callback(self,msg):
	centro_img = Point() 
	centro_img.x=160
	centro_img.y=120
	centro_img.z=0
	
	Z_segura=9.75 #para considerar que 87 es el centro de la "pantalla"
	#funcion que retorna una lista compuesta por la distancia en el eje "x" y en el eje "z"
	def procesar_dis(self, dis):
		self.dx=dis.x #distancia en eje x de la camara
		self.dz = dis.z #distancia en eje z de la camara
		self.dis_area=[self.dx,self.dz] 
		return self.dis_area 
		

	def procesar_twist(self, twist):
	
	# se definen las variables dx, distancia en eje x, y da, area del blob	
		dx=self.dis_area[0]
		da=self.dis_area[1]		
		if self.dx == 0:
			self.sep = 0

		else:
			self.sep = 87-self.dx
		print(da)
		
	#Se definen las velocidades lineales y angulares dependiendo de la distancia del duckiebot al blob, usando el area		
		if abs(da)<800:
			c=0.04
		elif abs(da)<1400 and abs(da)>=800:
			c=0.01
			
		elif abs(da)< 2000 and abs(da)>=1400:
			c=0.008
		
		if  da==0 or da >= 2000:
			c=0
			
 
		self.twist.v = -c*(da/10) 

		if da==0:
			self.sep=3
			k=3.6
		elif abs(self.sep)>=50:
			k=0.09
		elif abs(self.sep)<50 and abs(self.sep)>=30:
			k=0.09
			
		elif abs(self.sep)<30 and abs(self.sep)>=10:
			k=0.17	
			
		else: 
			k=0.49	
			
		self.twist.omega = k*self.sep
		self.publisher.publish(self.twist)
#pato 		   
def main():
	rospy.init_node('FOLLOW_COLOR') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
		
