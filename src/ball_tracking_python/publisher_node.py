#!/usr/bin/env python

'''
 Read the frames from a video and publish them on a topic.
'''

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def read_frames_video(video_capture):
    ret, frame = video_capture.read()  
    return frame


def publisher_frame():
    topic_name='tennis_ball_image'
    framePub = rospy.Publisher(topic_name, Image, queue_size=10)  
    return framePub


def publish_frame(rgb_frame, framePub):    
    msg_frame = CvBridge().cv2_to_imgmsg(rgb_frame, "bgr8")
    framePub.publish(msg_frame)

def main():
    _name = "tennis_ball_publisher"
    rospy.loginfo("Main starts for ", _name)
    rospy.init_node(_name)
    framePub = publisher_frame()
    video_capture = cv2.VideoCapture('/home/karlo/catkin_ws/src/scratch/ros_essentials_cpp/src/topic03_perception/video/tennis-ball-video.mp4')
    refRate = rospy.Rate(30)
    while not rospy.is_shutdown():       
        refRate.sleep()
        rgb_frame = read_frames_video(video_capture)
        publish_frame(rgb_frame, framePub)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()