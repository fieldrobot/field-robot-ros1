#!/usr/bin/python3.8

import rospy
import numpy as np
import tf
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from field_robot.msg import Pixel
from field_robot.msg import Blobs
import std_msgs.msg

#Aus der Information, bei welchen Pixeln die einzelnen Blobs liegen wird in dieser Funktion die PointCloud generiert, welche fuer die folgenden Prozesse benoetigt wird.
class CamToSepPoiClo:
    def __init__(self):
        # general parameter definition
        self.goalFrame = 'base_footprint' #hier base_footprint und nicht base_link, das sich base_link bei unserem Roboter spaeter neigt.
        self.cameraFrame = 'camera' #Frame des Kamerabildes
        self.pointCloudTopic = '/camera/blob_cloud' #topic, auf welchem die PointCloud2 publiziert werden soll

        # camera parameter definition: das habe ich berechnet. Wie genau fuehre ich hier nicht aus, da dann der Tag vorbei waere ;)
        self.pixelSize = 0.003257794325
        self.focalLength = 1
        self.height = 240
        self.width = 420

        # publisher and subscriber definition
        self.tfListener = tf.TransformListener() #Hiermit kann ich die Transformationen zwischen den einzelnene Frames abfragen.
        self.tfListener.waitForTransform(self.goalFrame, self.cameraFrame, rospy.Time(0), rospy.Duration(3)) #Hier wird gewartet, bis ich Transformationen empfangen kann. Ansonsten kommt eine Fehlermeldung in der Art von: noch nicht
        self.pub = rospy.Publisher(self.pointCloudTopic, PointCloud2, queue_size=10) #Hiermit wird nachher die PointCloud2 publiziert
        self.blob_pixel_sub = rospy.Subscriber('/camera/blob_pixel', Blobs, self.blob_pixel_callback) #Das ist der Subscriber fuer die Blob-Pixel. Diese Information stammt aus blob_detection.py

    def blob_pixel_callback(self, msg):
        points = [] #in diesem Array werden die einzelnen Punkt in 3D-gespeichert

        #in dieser Schleife wird jeder der einzelnen Blobs betrachtet und eine entsprechende 3D-Position auf dem Boden berechnet.
        for pixel in msg.pixel:
            #Hier werden die Transformationsaenerung und Rotationsaenderungen von dem Kamera-Frame zum Roboter-Frame berechnet. Die Funktion ist von ROS gegeben.
            (transform_from_cam, rotation_from_cam) = self.tfListener.lookupTransform(self.goalFrame, self.cameraFrame, rospy.Time(0))
            #Ergebnis: Pixel-Bild-Koordinate in ROS-Koordinate (in m) ungewandelt (Beschreibung siehe bei der Funktion9
            cam_coordinate_unrotated = self.pixel_to_point(pixel.x, pixel.y)
            #aus der einfachen 3D-Koordinate, die bisher keinerlei Drehungen der Kamera beruecksichtigt, wird hier nun eine neue Koordinate generiert, die die Drehung beruecksichtigt.
            #Somit zeigen die Koordinaten von dem Frame der Kamera aus nun in die richtige Richtung.
            cam_coordinate = self.qv_mult(rotation_from_cam, cam_coordinate_unrotated)

            #es werden ausschliesslich Koordinaten behandelt, welche aus Sicht der Kamera nach unten zeigen. Deren z-Kooridnate also <0 ist.
            if cam_coordinate[2] < 0:
                #Jede ermittelte Koordinate ist ja nur eine von unendlich vielen moeglichen Punkten eines Blobs. Dabei liegen alle Punkte auf einer Gerade.
                #Hier wird jetzt die Koordinate dieser Gerade germittelt, die auf dem Boden liegt und somit den echten Blob darstellt.
                cam_coordinate_ground = self.ground_coordinate(cam_coordinate, transform_from_cam)
                #Hier wird die Kooridnate zu angepasst, dass sie sich nun auf einen anderen Bezugsframe(base_footprint, anstelle von camera) bezieht.
                link_coordinate = self.transform_coordinate(cam_coordinate_ground, transform_from_cam)

                #points.append([0.5, 0.5, 0])
                #die ermittelte Koordiante wird nun dem Array aller Punkte hinzugefuegt.
                points.append([link_coordinate[0], link_coordinate[1], link_coordinate[2]])
                print(points)

                #[[link_coordinate[0], link_coordinate[1], link_coordinate[2]]]

        #Der Header wird erstellt und an die Gegebenheiten, wie den Bezugsframe angepasst.
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.goalFrame

        #Dies ist eine gegebene Funktion, welche aus dem Punkte-Array und einem Header eine PointCloud2 macht.
        pointCloud = pc2.create_cloud_xyz32(header, points)
        #Diese PointCloud2 wird nun ueber ROS publiziert.
        self.pub.publish(pointCloud)
        print('juhu')

    #ein gegebenes Pixelpaar wird in eine ROS 3D-Koordinate umgewandelnt
    def pixel_to_point(self, x_im, y_im):
        #Verschiebung von (0,0) von der linken oberen Ecke des Bildes in die Mitte des Bildes. Dies entspricht der Koordinatendarstellung.
        x_im = -x_im + (self.width / 2)
        y_im = -y_im + (self.height / 2)
        #Die Pixel werden nun mit der Pixelgroesse von Pixel in m umgerechnet, um als Koordinate angegeben werden zu koennen.
        y_co = x_im * self.pixelSize
        z_co = y_im * self.pixelSize
        #Abhaengig von der verwendeten Brennweite, wird hier die x-Koordinate fest gesetzt.
        x_co = 1
        #Der ermittelte 3D-Punkt fuer den uebergebenen Pixel wird zurueckgegeben.
        return np.array([x_co, y_co, z_co])

    #Rotationen werden in ROS in Quaternions statt in xyz-Rotationen angegebne. Das hat verschiedenste Vorteile, u. A. sind die Bewegungen von Robotern eleganter.
    #Siehe z. B. Gimbal Lock auf Wikipedia. Der einzige Nachteil von Quaternions: Sie sind ein 4D-Zahlensystem, das erstaunlich aehnlich zu den komplexen Zahlen ist.
    #Es hat mich mehr als eine Woche gekostet das ausreichend zu verstehen. Wenn dich der Grund fuer den Code interessiert: einfach melden.
    #Im Allgemeinen wird hier jedoch eine gegebene Koordinate nur nach einer Rotationsanweisung um den Kooridnatenursprung gedreht.
    def qv_mult(self, q1, v1):
        #v1 = tf.transformations.unit_vector(v1) #does not change the vector since its already a unit vector
        q2 = list(v1)
        q2.append(0.0)
        return tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(q1, q2),
            tf.transformations.quaternion_conjugate(q1)
        )[:3]

    #Hie wird eine gegebene Koordinate in Bezug zur Kamera so gestreckt, dass diese nun auf dem Boden liegt. Diese neue Kooridnate wird zurueckgegeben.
    def ground_coordinate(self, cam_coordinate, transform):
        #Hier wird der Streckungs-/Stauchungsfaktor der Koordinate bestimmt, damit diees auf dem Boden liegt.
        factor = (0-transform[2])/cam_coordinate[2]
        #Die entsprechend angepasste Koordinate wird zurueckgegeben.
        return cam_coordinate*factor

    #Hier wird eine gegebene Koordinate um gegebene xyz-Werte einfach verschoben. Dies dient der translatorischen Umwandlung zwischen verschiedenen Frames.
    def transform_coordinate(self, coordinate, transform):
        return coordinate+transform


rospy.init_node('camera_to_separate_point_cloud')
ctspc = CamToSepPoiClo()
rospy.spin()
