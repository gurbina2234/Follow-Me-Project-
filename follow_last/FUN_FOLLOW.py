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
		#se definen las variables del tipo self a utilizar en las funciones
		super(Template, self).__init__()
		self.args = args
		self.sub = rospy.Subscriber("/duckiebot/posicion_bot",Point,self.procesar_dis)
		self.sub1= rospy.Subscriber("/duckiebot/filtro", Twist2DStamped,self.procesar_twist)
		self.publisher = rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd", Twist2DStamped, queue_size = 1)
		self.dx=300 #se crea una variable de tipo self arbitraria en el eje x
		self.dz =300 #se crea una variable de tipo self arbitraria en el eje z
		self.sep=0 #se crea una variable de tipo self que determina la ubicacion del blob en el eje x 
		self.dis_area=[] #se crea una lista vacia para el area del
		self.twist= Twist2DStamped() 

	#variables que indican el centro de la imagen captada por la camara
	centro_img = Point() 
	centro_img.x=160
	centro_img.y=120
	centro_img.z=0

	#funcion que retorna una lista compuesta por la distancia en el eje "x" y en el eje "z"
	def procesar_dis(self, dis):
		self.dx=dis.x #distancia en eje x de la camara
		self.dz = dis.z #distancia en eje z de la camara
		self.dis_area=[self.dx,self.dz] 
		return self.dis_area 
		
	def procesar_twist(self, twist):	
		# se definen las variables dx, distancia en eje x, y da, area del blob con indices extraidos de la lista self.dis_area
			
		dx=self.dis_area[0]
		da=self.dis_area[1]	
		 	
		if self.dx == 0:
			self.sep = 0 #se devuelve la separacion si la distancia en x es nula
		else:
			self.sep = 87-self.dx #se devuelve la separacion si la distancia x es distinto a 0

		print(da) #se hace un print para ver que area detecta el blob
		
		#se define una constante c dependiendo del tamano de area del blob	
		if abs(da)<800:
			c=0.03 
			
		elif abs(da)<1100 and abs(da)>=800:
			c=0.009
			
		elif abs(da)<1400 and abs(da)>=1100:
			c=0.005	
				
		elif abs(da)< 2000 and abs(da)>=1400:
			c=0.003
		
		if  da==0 or da >= 2000:
			c=0
			
 
		self.twist.v = -c*(da/10) #se multiplica la constante c por la normalizacion del area del blob y retorna la velocidad lineal 

		#se define una constante k dependiendo de self.sep 
		if da==0:
			self.sep=3.3
			k=3.3
			
		elif abs(self.sep)>=50: #se aplica valor absoluto a la distancia para que sea valido en valores negativos
			k=0.15
			
		elif abs(self.sep)<50 and abs(self.sep)>=30:
			k=0.15
			
		elif abs(self.sep)<30 and abs(self.sep)>=10:
			k=0.1
			
		else: 
			k=0.09
			
		self.twist.omega = k*self.sep #se multiplica la constante k por la separacion en el eje x y retorna la velocidad lineal 
		
		self.publisher.publish(self.twist) #se publica self.twist ...........
		   
def main():
	rospy.init_node('FOLLOW_COLOR') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
		
