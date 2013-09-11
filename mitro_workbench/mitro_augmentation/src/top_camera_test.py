#!/usr/bin/env python

import roslib
roslib.load_manifest('mitro_augmentation')
import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from math import pi, cos, sin
from numpy import matrix, array

def ROS2OpenCV(x, y, z):
    # height
    h = 1.625 #1.29
    z = z - h    
    return (-y, -z, x)

def proj(x, y, z):
    (x, y, z) = ROS2OpenCV(x, y, z)
    # rotation                                                                                                         
    th = 26.21
    th = th / 180.0 * pi
    R = matrix("1 0 0; 0 %f %f; 0 %f %f"%(cos(th), -sin(th), sin(th), cos(th)))
    P = matrix([x, y, z]).transpose()
    (x, y, z) = R*P

    # distortion                                                                                                       
    (k1, k2, p1, p2, k3) = (0.000991, -0.060764, 0.000059, 0.000923, 0.000000)

    # projection                                                                                                       
    (fx, fy, cx, cy) = (544.097107, 547.663330, 322.301049, 246.070287)

    x_p = x / z
    y_p = y / z

    r2 = x_p * x_p + y_p * y_p

    x_pp = x_p * (1 + k1 * r2 + k2 * (r2 * r2) + k3 * (r2 * r2 * r2)) + 2 * p1 * x_p * y_p + p2 * (r2 + 2 * x_p * x_p)
    y_pp = y_p * (1 + k1 * r2 + k2 * (r2 * r2) + k3 * (r2 * r2 * r2)) + p1 * (r2 + 2 * y_p * y_p) + 2 * p2 * x_p * y_p


    #x_pp = x_p
    #y_pp = y_p


    u = fx * x_pp + cx
    v = fy * y_pp + cy
    print "%f %f %f -> %d %d"%(x, y, z, u, v)
    return (int(u),int(v))

class test_vision_node:

    def __init__(self):
        rospy.init_node('test_vision_node')

        """ Give the OpenCV display window a name. """
        self.cv_window_name = "OpenCV Image"

        """ Create the window and make it re-sizeable (second parameter = 0) """
        cv.NamedWindow(self.cv_window_name, 0)

        """ Create the cv_bridge object """
        self.bridge = CvBridge()

        """ Subscribe to the raw camera image topic """
        self.image_sub = rospy.Subscriber("/gscam/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            """ Convert the raw image to OpenCV format """
            cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
          print e
  
        
        """ Get the width and height of the image """
        (width, height) = cv.GetSize(cv_image)

        """ Overlay some text onto the image display """
        #text_font = cv.InitFont(cv.CV_FONT_HERSHEY_DUPLEX, 2, 2)
        #cv.PutText(cv_image, "OpenCV Image", (50, height / 2), text_font, cv.RGB(255, 255, 0))
 

        #cv.Line(cv_image, (width/2, 0), (width/2, height), cv.RGB(255, 0, 0))
        #cv.Line(cv_image, (0, height/2), (width, height/2), cv.RGB(255, 0, 0))

        (fx, fy, cx, cy) = (544.097107, 547.663330, 322.301049, 246.070287)
        cx = int(cx)
        cy = int(cy)
        cv.Line(cv_image, (cx, 0), (cx, height), cv.RGB(255, 0, 0))
        cv.Line(cv_image, (0, cy), (width, cy), cv.RGB(255, 0, 0))


        w = 1.22
        h = 0.88
        x = 2# - w/2
        y = 0 - h/2

        cv.Line(cv_image, proj(x, y, 0), proj(x, y+h, 0), cv.RGB(255, 0, 0))
        cv.Line(cv_image, proj(x, y+h, 0), proj(x+w, y+h, 0), cv.RGB(255, 0, 0))
        cv.Line(cv_image, proj(x+w, y+h, 0), proj(x+w, y, 0), cv.RGB(255, 0, 0))
        cv.Line(cv_image, proj(x+w, y, 0), proj(x, y, 0), cv.RGB(255, 0, 0))


        # cv.Circle(cv_image, proj(2, 0, 0), 5, cv.RGB(255, 0, 0)) 
        # cv.Circle(cv_image, proj(2, 0.5, 0), 5, cv.RGB(0, 255, 0)) 
        # cv.Circle(cv_image, proj(2, -0.5, 0), 5, cv.RGB(0, 0, 255)) 
       # cv.Circle(cv_image, proj(1, 0, 0), 5, cv.RGB(0, 255, 0)) 

        
 
        """ Refresh the image on the screen """
        cv.ShowImage(self.cv_window_name, cv_image)
        cv.WaitKey(3)

def main(args):
      vn = test_vision_node()
      try:
        rospy.spin()
      except KeyboardInterrupt:
        print "Shutting down vison node."
      cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    
