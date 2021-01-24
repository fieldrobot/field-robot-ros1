#!/usr/bin/python3.8

import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from timeit import default_timer as timer
from skimage import measure
from nav_msgs.msg import Odometry
from field_robot.msg import Pixel
from field_robot.msg import Blobs
import std_msgs.msg
from sys import exit

#In dieser Datei wird das Kamera-Bild analysiert und die Blobs werden ermittelt. Die Position der gefundenen Blobs in Pixeln wird publiziert.
class Follower:
    def __init__(self):
        #openCV Instanz zur Bildverarbeitung
        self.bridge = cv_bridge.CvBridge()
        #Es wird das Kamerabild des Roboters subscribed
        self.image_sub = rospy.Subscriber('/camera/image', Image, self.image_callback)
        #Hier werden spaeter im selbst definierten Format die gefundenen Blobs publiziert
        self.blob_pixel_pub = rospy.Publisher('/camera/blob_pixel', Blobs, queue_size=1)

    #Callback-Function fuer neue Bilder
    def image_callback(self, msg):
        #Start der Zeitmessung zur Bestimmung der Rechenzeit
        start_ = timer()
        #Erstelung eines Objektes des selbst definierten Dateiformates zur Blob-Publizierung. Hier werden die Daten reingeschrieben. Das Objekt wird nachher publiziert.
        blobsMes = Blobs()

        #Bild von ROS zu OpenCV umwandeln
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #Farmraum von RGB zur HSV umwandeln
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #Bild nach den organgen Blobs filtern: die Filtergrenzen festlegen
        lower_orange = numpy.array([0, 90, 81])
        upper_orange = numpy.array([45, 255, 255])
        #Bild nach den organgen Blobs filtern: das eigentliche Filtern
        thresh = cv2.inRange(hsv, lower_orange, upper_orange)
        #Erode und Dilate zur Glaettung der Kanten und zum Entfernen von stoerendem Rauschen
        thresh = cv2.erode(thresh, None, iterations=1)
        thresh = cv2.dilate(thresh, None, iterations=1)
        #Alle getrennten Bereiche des Bildes (der Hintergrund und die Blobs) werden erkannt und all zu einem Bereich gehoerenden Pixel werden auf jeweils einen spezifischen Wert gesetzt.
        labels = measure.label(thresh)
        #mask = numpy.zeros(thresh.shape, dtype="uint8") UNNOETIG: zum Testen und Ueberrest von frueheren Versionen
        #Alle spezifische Werte werden ermittelt. Jeder taucht in diesem Array nur noch EINMAL auf.
        blobs = numpy.unique(labels)
        #Jede erkannte Region kann nun ein Blob sein und wird deswegen weiter getrennt betrachtet.
        for label in blobs:
            #Hiermit soll der Hintergund aussortiert werden. Optimierungsbedarf: ist ein Blob links oben im Bild, wird dieser aussortiert.
            if label == 0:
                continue
            #Hier wird ein leeres "Bild" erzeugt, mit Hilfe welchem, die eine Region (der eine Blob) betrachtet werden soll.
            labelMask = numpy.zeros(thresh.shape, dtype="uint8")
            #Alle zum Bereich gehoerenden Pixel werden weiss gefaerbt. Denn label gibt die aktuelle Bereichnummer an und labels ist die Maske, in welcher alle Bereiche markiert sind.
            labelMask[labels == label] = 255
            # numPixels = cv2.countNonZero(labelMask) UNNOETIG: zum Testen und Ueberrest von frueheren Versionen
            # if numPixels > 30: UNNOETIG: zum Testen und Ueberrest von frueheren Versionen
            #mask = cv2.add(mask, labelMask) UNNOETIG: zum Test und Ueberrest von frueheren Versionen
            #Berechnung des Mittelpunktes von EINEM Blob
            uniqueM = cv2.moments(labelMask)
            uniqueX = int(uniqueM["m10"] / uniqueM["m00"])
            uniqueY = int(uniqueM["m01"] / uniqueM["m00"])

            #Die Pixelkoordinate des einen Blobs wird in einem Objekt des selbst definierten Dateiformates Pixel geschrieben.
            pixel = Pixel()
            pixel.x = uniqueX
            pixel.y = uniqueY

            #Dieser eine Blob wird nun dem Array in der eigentlichen Nachricht hinzugefuegt, welche alle Blobs enthaelt.
            blobsMes.pixel.append(pixel)

        #Der Header wird erzeugt und mit grundlegenden Werten gefuellt.
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()

        #Der Nachricht wird nun der header hinzugefuegt.
        blobsMes.header = header
        #Die Zeitmessung fuer die Dauer dieser Prozedur wird beendet und angezeigt.
        end_time = timer()
        time = 1000 * (end_time - start_)
        print("time/ms", time)
        #Die Information mit der Position aller Blobs wird veroeffentlicht.
        self.blob_pixel_pub.publish(blobsMes)


#allgemeines Zeug
rospy.init_node('blob_detection')
follower = Follower()
rospy.spin()
# END ALL