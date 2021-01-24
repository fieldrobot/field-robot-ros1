#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
import numpy
import time
import os
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from timeit import default_timer as timer
from skimage import measure
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from first_camera_processing.msg import Pixel
from first_camera_processing.msg import Blobs
import std_msgs.msg
from sys import exit

#Mit einer HoughTransformtaion wird hier eine erstellte Karte in ein Bild umgewandelt, in diesem werden die Reihen erkannt und wieder drauf gezeichnet,
#und abschliessend wird das Bild als eine zweite Karte zur Verfuegung gestellt.
class Generator:
    def __init__(self):
        #Dies beschreibt die Hoehe und Breite des Bildes. Die Werte werden noch sinnvoll aktualisiert.
        self.height = 0
        self.width = 0

        #OpenCV zur Bildbearbeiten
        self.bridge = cv_bridge.CvBridge()
        #Die  Metadaten der bestehenden Karte empfangen.
        self.map_meta_sub = rospy.Subscriber('/map_metadata', MapMetaData, self.map_meta_callback)
        #Zur Pufferung warten. Es muessen erst Metadaten empfangen werden.
        time.sleep(0.1)
        #Die bestehende Karte wird empfangen.
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        #Und so wird die ueberarbeitete Karte publiziert.
        self.line_map_pub = rospy.Publisher('/line_map', OccupancyGrid, queue_size=1)


    #Den Metainformatoinen wird die Hoehe und Breite der Karte entnommen und in die Variablen geschrieben.
    def map_meta_callback(self, msg):
        self.height = msg.height
        self.width = msg.width
        print('initiate')

    def map_callback(self, msg):
        #Timer
        print('start')
        start_time = timer()
        #Erstellung eines numpy Arrays (Bild) fuer OpenCV
        self.image = numpy.zeros((self.height,self.width))
        #Das Numpy Array mit den Werten der Karte fuellen.
        #Hier ist eine Schleife notwendig, da die Karte alle Pixel linear und nicht in 2D speichert.
        for x in range(self.height):
            self.image[x] = msg.data[((self.width)*x):((self.width*(x+1)))]

        #Alle unbekannten Felder haben in der Karte den Wert -1, freie Feldre 0 und belegte 100. Hier werden die freien und unbekannten zusammengefasst.
        #frei und unbekannt wird zu 0 und belegt wird zu 1
        self.filtered_image = (self.image==100).astype(numpy.uint8)

        #alle belegten werden auf hunder gesetzt.
        (thresh, self.baw_image) = cv2.threshold(self.filtered_image, 0, 100, cv2.THRESH_BINARY)

        #Timer
        print('image creation complete')
        end_time = timer()
        time = (end_time - start_time)
        print("image creation complete/s", time)

        #zur besseren Erkennung "fetten"
        self.baw_image = cv2.dilate(self.baw_image, None, iterations=1)
        #Linien erkennen
        lines = cv2.HoughLinesP(self.baw_image,1,numpy.pi/18000,1,minLineLength=18,maxLineGap=7)
        #wieder "entfetten"
        self.baw_image = cv2.erode(self.baw_image,None,iterations=1)
        #Zaehlvariable fuer die folgende Schleife
        counter = 0
        #Die Linien werden nun auf das Bild gezeichnet
        for line in lines:
            for x1,y1,x2,y2 in lines[counter]:
                cv2.line(self.baw_image, (x1,y1), (x2,y2), (100,100,100), 1)
            counter = counter + 1

        #Timer
        print('image processing complete')
        end_time = timer()
        time = (end_time - start_time)
        print("image processing complete/s", time)

        #Aus dem 2D-Bild wird wieder 1D gemacht sowie der Datentyp fuer die Map wieder in int umgewandelt.
        self.image_flattened = numpy.append([],self.baw_image).astype(int)
        #Timer
        end_time = timer()
        time = (end_time - start_time)
        print("image flattening complete/s", time)

        #die Nachricht zur Publikation ueber ROS vorbereiten.
        OcuGrid = OccupancyGrid()
        OcuGrid.header = msg.header
        OcuGrid.info = msg.info

        #self.image_flattened[self.image_flattened == 255] = 100
        #das numpy Array muss fuer die map in eine Liste umgewandelt werden.
        list = self.image_flattened.tolist()
        #Die Liste der Nachricht hnzufuegen
        OcuGrid.data = list
        #Timer
        end_time = timer()
        time = (end_time - start_time)
        print("image to list converstion complete/s", time)

        #Publikation
        self.line_map_pub.publish(OcuGrid)
        #Timer
        end_time = timer()
        time = (end_time - start_time)
        print("success/s", time)
        print('success')


rospy.init_node('line_map_generator')
generator = Generator()
rospy.spin()
# END ALL
