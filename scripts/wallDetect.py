#!/usr/bin/env python

'''
Created on Jan 17, 2013
@author: mllofriu
'''

import rospy  # @UnresolvedImport

from sensor_msgs.msg import Image

from cv2 import *
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import math

class LineDetector(object):
    '''
    This class takes the raw image and detects tape lines on the floor. 
    The topic lines is then published with the detected lines. 
    '''

    pub = None
    bridge = None
    maxLen = 10

    def __init__(self):
        self.bridge = CvBridge()
        
        rospy.Subscriber("/cam/image_raw", Image, self.processImage)
   
        while not rospy.is_shutdown():
            waitKey(5)
        
    def processImage(self, rosImage):
        rospy.loginfo("Image received")
        try:
           img = self.bridge.imgmsg_to_cv2(rosImage, desired_encoding="passthrough")
        except CvBridgeError, e:
            print "Macana:", e
            
        img = np.asarray(img)
        
        img = resize(img, (640,480))       
        y,u,v = split(img)
        
        img = y
        
        imshow("Yuv", img)
        
        ret, thrs = threshold(img,180,255,THRESH_BINARY)
        
        canny = Canny(thrs, 0, 200)
        imshow("Canny", canny)
        
        lines = HoughLinesP(canny, 1, cv.CV_PI/180, 100, minLineLength=400,maxLineGap=400)
         
        if (lines != None):
            for l in lines[0]:
                line(img, (l[0], l[1]),(l[2],l[3]), cv.CV_RGB(255, 0, 0), 3, 8)

        imshow("Lines", img)
    
if __name__ == "__main__":
    rospy.init_node('lineDetector')
    namedWindow("Lines")
    LineDetector()
