#!/usr/bin/env python

from __future__ import print_function
from turtle import pos
from std_msgs.msg import String, Header
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CompressedImage, PointCloud, PointCloud2, PointField, Image
from cv_bridge import CvBridge, CvBridgeError
from numpy import interp

import sensor_msgs
import roslib
#roslib.load_manifest('zed_nodes')
import sys
import rospy
import cv2
import numpy as np
import struct
import ctypes
import pandas as pd
from datetime import datetime, timedelta
from crazyflie_driver.msg import GenericLogData
import random
import signal
import csv

puntos = []
data_matrix = []

def guardar_datos_en_csv(datos, archivo_csv):
    datos.insert(0, ["x", "y", "z", "yaw"])

    with open(archivo_csv, 'w', newline='') as archivo:
        writer = csv.writer(archivo)
        writer.writerows(datos)
    print("Datos guardados en el archivo CSV:", archivo_csv)

def signal_handler(sig, frame):
    # Lógica para guardar los datos en el archivo CSV antes de finalizar el nodo
    guardar_datos_en_csv(data_matrix, 'datos.csv')
    sys.exit(0)

def callback(data):
    global puntos
    global pos_actual
    global data_matrix
    x,y,z,yaw = data.values[0],data.values[1],data.values[2],data.values[3]
    r,g,b = 255,255,255
    puntos_total=[]
    puntoss=[]
    prueba=[0,0,0,0]
    #print("Received matrix:", data.values)

    punto=[x,y,z]
    rgb=struct.unpack('I', struct.pack('BBBB', b, g, r, int(random.randint(0, 255))))[0]
    rgb_act_pos=struct.unpack('I', struct.pack('BBBB', 0, 0, 255, int(random.randint(0, 255))))[0]

    punto_pos_act=[x,y,z]

    punto.append(rgb)
    puntos.append(punto)

    data_matrix_position=[x,y,z,yaw]
    data_matrix.append(data_matrix_position)
    
    punto_pos_act.append(rgb_act_pos)
    pos_actual.append(punto_pos_act)
    #print(type(puntos),type(puntoss))

    if len(pos_actual) > 7:
        pos_actual = pos_actual[-7:]
    #print(pos_actual)

    puntoss=puntos+pos_actual
    #print(puntoss)    

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('rgb', 12, PointField.UINT32, 1),
          #PointField('rgba', 12, PointField.UINT32, 1),
          ]
    
    header = Header()
    header.frame_id = "map"
    pc2 = sensor_msgs.point_cloud2.create_cloud(header, fields, puntoss)

    pub.publish(pc2)


def listener():
    rospy.init_node("point_cloud3", anonymous=False)
    global pub
    pub=rospy.Publisher("point_cloud3", PointCloud2, queue_size=10)
    topic = rospy.get_param("~topic", "/crazyflie1/log1")
    rospy.Subscriber(topic, GenericLogData, callback)    
    

    #rate = rospy.Rate(40)
    #while not rospy.is_shutdown():
    #    rospy.Subscriber('/crazyflie/log1', GenericLogData, callback)

    #    rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    puntos=[]
    pos_actual=[0,0,0]
    signal.signal(signal.SIGINT, signal_handler)
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
