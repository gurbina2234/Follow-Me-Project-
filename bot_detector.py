#!/usr/bin/env python
import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist, Point# importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image
import cv2 # importar libreria opencv
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy


class Template(object):
	def __init__(self, args):
		
		super(Template, self).__init__()
		self.args = args
		self.publisher = rospy.Publisher("/duckiebot/dist_bot",Image, queue_size=10)
		self.sub1 = rospy.Subscriber("/duckiebot/camera_node/image/rect",Image,self.procesar_img)
		self.publisher2 = rospy.Publisher("/duckiebot/posicion_bot",Point, queue_size=10)
	#def publicar(self):
	
	#se definen valores arbitarios para cada eje 
	#def callback(self,msg):
	centro_img = Point() 
	centro_img.x=160
	centro_img.y=120
	centro_img.z=0
	
	#Se define una mascara para definir la deteccion por color	
	def procesar_img(self, msg):
		# Cambiar espacio de color
		bridge=CvBridge()
		image_in= bridge.imgmsg_to_cv2(msg, "bgr8")           
		image_out= cv2.cvtColor(image_in,cv2.COLOR_BGR2HSV)
		lower_limit=np.array([50,100,100])
		upper_limit=np.array([90,255,255])
		mask=cv2.inRange(image_out,lower_limit,upper_limit)
		kernel= np.ones((5,5), np.uint8)
		mask = cv2.erode(mask,kernel, iterations = 1)
		mask = cv2.dilate(mask,kernel, iterations = 4)
		image_od=cv2.bitwise_and(image_in,image_in,mask=mask)
		# Filtrar rango util
			

		# Aplicar mascara
		# Aplicar transformaciones morfologicas

		# Definir blobs		
		_,contours, hierarchy=cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

		#print(contours)
					 
						 
		if contours != []:   
		# Dibujar rectangulos de cada blob al detectarlo 
			for cnt in contours:   
				x,y,w,h=cv2.boundingRect(cnt)
				a=(101.85916357881302*4.5)/h
				print(a)
				dis=Point()
				dis.x=(x+w)/2
				dis.y=(y+h)/2
				dis.z=w*h
				self.publisher2.publish(dis)
		
				cv2.rectangle(image_od,(x,y),(x+w,y+h),(0,0,255),2)
				
		#funcion que retorna "no hay nada" cuando la camara no detecta el blob de color
		else:
			dis=Point()
			print("no hay nada")
			dis.x=0
			dis.y=0
			dis.z=0
			self.publisher2.publish(dis)
					 

							

			 
		# Publicar imagen final
		msg = bridge.cv2_to_imgmsg(image_od, "bgr8")
		self.publisher.publish(msg)

			
def main():
	rospy.init_node('bot_detector') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
