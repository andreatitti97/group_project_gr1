#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-


# Python libs
import sys, time

# numpy and scipy
import numpy as np
#from scipy.ndimage import filters

# OpenCV
""" ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'

if ros_path in sys.path:

    sys.path.remove(ros_path)

import cv2

sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') """
import cv2


# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError
import socket 
import os
import struct
FORMAT = "utf-8"      

def sub_server(indirizzo, backlog=1): # blacklog quante richieste può accettare  
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(indirizzo) # collegamento dell socket server al'ip della macchina che lo ospita 
        s.listen(backlog) # mi metto in ascolto di richieste specificando il backlog, cioè quante ne posso gestire 
        print("server initializated, listening...")
    except socket.error as error:
        print("server crash {error}")
        print("try to rerun the server")
        sub_server(indirizzo, backlog=1)
        
    count = 0
    path = os.path.dirname(os.path.abspath(__file__)) + '/imagesReceived'
    print(path)
    # path = 'D:\GroupProject\AndroidStudioProjects\GP-main\imagesReceived'
    while True:
        conn, indirizzo_client = s.accept() # accetto la richiesta di un client, 
                                            # funzione che ritorna la connessione (il socket del client) e l'inidrizzo del client 
        
        string = ''
        buf = bytes(string).encode("utf-8")
        while len(buf)<4:
            buf += conn.recv(4 - len(buf))
        size = struct.unpack('!i', buf)
        print("receiving %s bytes" % size)
        
        #filename = path + '/image'+str(count)+'.png'
        #filename = path + '\image.png'
        data1=''
        """ with open('/home/rick/my_ws/src/ass3/exp_assignment3/scripts/imagesReceived/image.png', 'wb') as file:
            while True:
                data = conn.recv(60000000)
                print("received data")
                #print(data)
                #data1 =  np.int8(data)
                data1  = data1+data
                print(sys.getsizeof(data1))
                if not data:
                    break
                file.write(data)
        print("Image received!")
        count = count+1 """
        while True:
            data = conn.recv(1024)
            print("received data")
            #print(data)
            data1  = data1+data
            print(sys.getsizeof(data1))
            if not data:
                break

        #file.close()  
        #data2 =  np.int8(data1)
        rospy.sleep(0.1)
        img = CompressedImage()
        img.data = data1
        img.format = "png"
        image_pub.publish(img)
        
        """ np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR) """
        
    conn.close() 

if __name__ == "__main__":
    rospy.init_node('image_publisher', anonymous=True)
    image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage)
    hostname = socket.gethostname()
	## getting the IP address using socket.gethostbyname() method
    ip_address = socket.gethostbyname(hostname)
	## printing the hostname and ip_address
    print("Hostname: {hostname}")
    print("IP Address: {ip_address}")
    sub_server(("192.168.1.195",8888)) 
	#  "" = prende l'IP della macchina su cui sta girando però la porta deve essere specificata  
	## getting the hostname by socket.gethostname() method
	
