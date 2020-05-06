#!/usr/bin/env python

import numpy as np
import cv2
import time
import rospy
import sys

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def read_rgb_image(image_name, show):
    # rgb_image = cv2.imread(image_name)
    rgb_image = image_name
    if show:
        cv2.imshow("RGB Image", rgb_image)
    return rgb_image

def filter_color(rgb_image, lower_bound_color, upper_bound_color):
    #convert image into the HSV color space
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv image", hsv_image)

    mask = cv2.inRange(hsv_image, lower_bound_color, upper_bound_color)
    return mask

def getContours(binary_image):
    _, contours, hierarchy = cv2.findContours(binary_image.copy(),
                                              cv2.RETR_EXTERNAL,
	                                          cv2.CHAIN_APPROX_SIMPLE)
    return contours

def draw_ball_contour(binary_image, rgb_image, contours):
    black_image = np.zeros([binary_image.shape[0], binary_image.shape[1],3],'uint8')
    
    for c in contours:
        area = cv2.contourArea(c)
        perimeter= cv2.arcLength(c, True)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        if (area > 3000):
            cv2.drawContours(rgb_image, [c], -1, (150,250,150), 1)
            cv2.drawContours(black_image, [c], -1, (150,250,150), 1)
            cx, cy = get_contour_center(c)
            cv2.circle(rgb_image, (cx,cy),(int)(radius),(0,0,255),1)
            cv2.circle(black_image, (cx,cy),(int)(radius),(0,0,255),1)
            cv2.circle(black_image, (cx,cy),5,(150,150,255),-1)
    #         print ("Area: {}, Perimeter: {}".format(area, perimeter))
    # print ("number of contours: {}".format(len(contours)))
    # cv2.imshow("RGB Image Contours",rgb_image)
    cv2.imshow("Black Image Contours",black_image)

def get_contour_center(contour):
    M = cv2.moments(contour)
    cx=-1
    cy=-1
    if (M['m00']!=0):
        cx= int(M['m10']/M['m00'])
        cy= int(M['m01']/M['m00'])
    return cx, cy


def detect_ball_in_a_frame(image_frame):
    # better tracking version
    # yellowLower =(30, 100, 50)
    # yellowUpper = (60, 255, 255)

    # bad tracking
    # yellowLower =(30, 150, 100)
    # yellowUpper = (50, 255, 255)

    #test tracking
    yellowLower =(30, 255, 135)
    yellowUpper = (40, 255, 185)

    # frame we read is in rgb form
    rgb_image = image_frame
    # rgb_image = read_rgb_image(image_frame, True)

    binary_mask_image = filter_color(rgb_image, yellowLower, yellowUpper)
    contours = getContours (binary_mask_image)
    draw_ball_contour(binary_mask_image, rgb_image, contours)

def image_callback(ros_image):
    print ('got an Image')
    global bridge
    #convert ros_image into an opencv-compatible image
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60:
        detect_ball_in_a_frame(cv_image)
    try:
        image_pub = rospy.Publisher("/image_converter/output_video",Image,queue_size=1)
        # mask_pub = rospy.Publisher("/image_converter/image_mask",Image,queue_size=1)
    except CvBridgeError as e:
            print(e)
    # font = cv2.FONT_HERSHEY_SIMPLEX
    # cv2.putText(cv_image,'Webcam Activated with ROS & OpenCV!',(10,350), font, 1,(255,255,255),2,cv2.LINE_AA)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)  

def main(args):
    rospy.init_node('ros_ball_tracer', anonymous=True)
    image_sub = rospy.Subscriber("/usb_cam/image_raw",Image, image_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()
   

if __name__ == '__main__':
    main(sys.argv)

