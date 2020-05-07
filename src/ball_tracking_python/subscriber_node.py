#!/usr/bin/env python

'''
 Read the frames from a topic.
'''

import numpy as np
import cv2
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ball_tracking_python.publisher_node import filter_color, getContours,draw_ball_contour

yellowLower =(30, 150, 100)
yellowUpper = (50, 255, 255)

def image_callback(ros_image):
    print ('got an image')
    #convert ros_image into an opencv-compatible image
    try:
        cv_image = CvBridge().imgmsg_to_cv2(ros_image, "bgr8")
        binary_image_mask = filter_color(cv_image, yellowLower, yellowUpper)
        contours = getContours(binary_image_mask)
        draw_ball_contour(binary_image_mask, cv_image, contours)
        cv2.waitKey(1) #this is required by cv (shows the delay for processing the next frame). If value is zero then press key to continue
    except CvBridgeError as e:
        print(e)
    #from now on, you can work exactly like with opencv


def main():
    _name = "tennis_ball_listener"
    rospy.loginfo("Main starts for ", _name)
    rospy.init_node(_name)

    #subscribe to the topic
    topic_name='tennis_ball_image'
    image_sub = rospy.Subscriber(topic_name,Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
