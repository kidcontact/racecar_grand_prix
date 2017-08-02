#!/usr/bin/python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ackermann_msgs.msg import AckermannDriveStamped 
import numpy as np
from std_msgs.msg import Int32
from ar_localization import TrackPosition

class lightDetectorNode:
    def __init__(self):
        rospy.Subscriber('/zed/left/image_rect_color', Image, self.blobDetectorCallback)
        self.bridge = CvBridge()
        
        #to run without robot
        #img = cv2.imread('yellow.jpeg')
        #self.blobDetectorCallback(img)
        
        self.HSV_RANGES = [np.array([75,10,230]), np.array([90,30,255])]
        self.pub = rospy.Publisher('vision_test', Image, queue_size=10)
        self.track_pub = rospy.Publisher('track_position', Int32, queue_size=10)
        self.img = 0
        self.error_dist = 0
        self.track_pos = 0

    def blobDetectorCallback(self, msg):

            
        #create openCV image (frame)
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        #cut frame

        #frame = frame[200:400,:]
        self.img = frame
        
        #call thresholdImg function and return the thresholded image as tImage
        tImage = self.thresholdImg(frame)
        
        #call getContours function and return the contour data as found_contours and the image as cImage
        found_contours, cImage = self.getContours(tImage)
        cImage = self.drawRect(tImage,cImage)
        rospy.loginfo("helloworld")
        
        self.pub.publish(self.bridge.cv2_to_imgmsg(cImage, 'bgr8'))
        
        
    def convertToHSV(self, image):
        image_dup = image
        img = cv2.cvtColor(image_dup, cv2.COLOR_BGR2HSV)
        return img
        
    def thresholdImg (self, img1):
        image = cv2.inRange(self.convertToHSV(img1), self.HSV_RANGES[0], self.HSV_RANGES[1])
        return image
     
    def getContours(self, img):
        _, contours, _ = cv2.findContours(img, 1,2)
        image = cv2.drawContours(self.img.copy(), contours, -1, (0,255,0), 3)
        return contours, image
    def drawRect(self, Thresh, frame):
        #TODO return contors and label self.image with detected contours
        _, contours, _ = cv2.findContours(Thresh,1,2)
        largest=0
        if len(contours)>0:
            largest = self.sortContours(contours)[0]
        
        x,y,w,h = cv2.boundingRect(largest)
        cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
        centerX = x+w/2
        centerY = y+h/2
        
        #find distance from center
        self.error_dist = centerX-len(frame[0])/2
        
        #display error on top of box
        text = str(self.error_dist)
        linetype = 4
        font = cv2.FONT_HERSHEY_SIMPLEX
        color = (255,255,255)
        fontScale = 1
        point = (x-100 ,y)
        cv2.putText(frame,text,point,font,fontScale,color,linetype)
        cv2.imshow('image', frame)
        cv2.waitKey(10000)
        return frame
        """ this stuff is driving stuff
        
        size = h
        
        speed = .5
        angle = 0
        constant = 15
        
        
        
       
        if size > 130:
            speed = 0
            
        else:
            if distOff >  constant:
                angle = .1
            elif distOff < -constant:
                angle = .25
        
        output_msg = AckermannDriveStamped()
        output_msg.drive.speed = rospy.Time.now()
        output_msg.drive.steering_angle = angle
        output_msg.drive.speed = speed    
        self.cmd_pub.publish(output_msg)
        """
        cv2.waitKey(1)
        
    def sortContours(self, contours):
        #TODO fill in this code (Stephan I think is the only one with a finished function for this)
        def greater(a, b):
            area1 = cv2.contourArea(a)
            area2 = cv2.contourArea(b)
            if area1 > area2:
                return -1
            return 1
        
        contours.sort(greater)
        return contours
    
    #find the center point of the cone using the contours so that we can calculate the angle of error
    def findCenter(self):
        pass        
        
    def show(self, image=None, window='image'):
        """ (completed helper function)
        shows an image with cv2.imshow but does not
        use waitKey
        """
        if image is None:
            image = self.image
        cv2.imshow(window, image)

    def startDisplay(self, time=0):
        """ (completed helper function)
        start display after loading image with self.show
        """
        cv2.waitKey(time)
        
    def clear(self, window=None):
        """ (completed helper function)
        clear display or displays
        """
        if window is not None:
            cv2.destroyWindow(window)
        else:        
            cv2.destroyAllWindows()
    
    def setHSV(self, min_hsv, max_hsv):
        """ (completed helper function)
        update minimum and maximum hsv values to use with inrange
        when doing the thresholding
        """
        assert len(min_hsv) == 3 and len(max_hsv) == 3
        min_val = np.array(min_hsv, np.uint8)
        max_val = np.array(max_hsv, np.uint8)
        self.HSV_RANGES = [min_val, max_val]

    def detect(self, debug=True):
        """ (completed helper function)
        object detector function described in pseudo code from lecture 
        Contains debug functions to view seperate components of object detection
        """
        hsv_img = self.convertToHSV(self.image)
        img_thresh = self.thresholdImage(hsv_img)
        contours, labeled_image = self.getContours(img_thresh)
        if len(contours) > 0:
            largest_contour = self.sortContours(contours)[0]
        else:
            largest_contour = None
        if debug:
            print "Largest Contour found", largest_contour
            print "type any key to quit showing windows"
            #DONT FORGET ABOUT THIS SHIT
            #self.show(self.image,window="original")
            #self.show(hsv_img,window="hsv")
            #self.show(img_thresh,window="threshold")
            #self.show(labeled_image, window="contoursFound")
            #self.startDisplay()
            #self.clear()
        return largest_contour




        
if __name__ == '__main__':
    rospy.init_node('lightDetectorNode')
    node = lightDetectorNode()
rospy.spin()
