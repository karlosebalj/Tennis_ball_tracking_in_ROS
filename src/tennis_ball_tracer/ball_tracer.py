#!/usr/bin/env python

import numpy as np
import cv2
import time

def read_rgb_image(image_name, show):
    # rgb_image = cv2.imread(image_name)
    rgb_image = image_name
    if show:
        cv2.imshow("RGB Image", rgb_image)
    return rgb_image

def filter_color(rgb_image, lower_bound_color, upper_bound_color):
    # convert image into the HSV color space
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv image", hsv_image)

    # Apply HSV treshold 
    mask = cv2.inRange(hsv_image, lower_bound_color, upper_bound_color)

    # to remove small particles from the picture - optional dilate + erode steps
    # #- dilate makes the in range areas larger
    # mask = cv2.dilate(mask, None, iterations=2)
    # #- Show HSV Mask
    # cv2.imshow("Dilate Mask", mask)   
    # cv2.waitKey(0)
    # mask = cv2.erode(mask, None, iterations=2)
    # #- Show dilate/erode mask
    # cv2.imshow("Erode Mask", mask)
    # cv2.waitKey(0)
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
        #deafault area was 3000, put it down so it can recognize the ball
        if (area > 300):
            cv2.drawContours(rgb_image, [c], -1, (150,250,150), 1)
            cv2.drawContours(black_image, [c], -1, (150,250,150), 1)
            cx, cy = get_contour_center(c)
            cv2.circle(rgb_image, (cx,cy),(int)(radius),(0,0,255),1)
            cv2.circle(black_image, (cx,cy),(int)(radius),(0,0,255),1)
            cv2.circle(black_image, (cx,cy),5,(150,150,255),-1)
            print ("Area: {}, Perimeter: {}".format(area, perimeter))
    print ("number of contours: {}".format(len(contours)))
    cv2.imshow("RGB Image Contours",rgb_image)
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
    yellowLower =(30, 100, 50)
    yellowUpper = (60, 255, 255)

    # bad tracking
    # yellowLower =(30, 150, 100)
    # yellowUpper = (50, 255, 255)

    #test tracking
    # yellowLower =(30, 255, 135)
    # yellowUpper = (40, 255, 185)


    # frame we read is in rgb form
    rgb_image = image_frame
    # rgb_image = read_rgb_image(image_frame, True)

    binary_mask_image = filter_color(rgb_image, yellowLower, yellowUpper)
    contours = getContours (binary_mask_image)
    draw_ball_contour(binary_mask_image, rgb_image, contours)

def main():
    video_capture = cv2.VideoCapture(0)
    # video_capture = cv2.VideoCapture('/home/karlo/catkin_ws/src/turtlesim_cleaner/src/video/tennis-ball-video.mp4')
    if (video_capture.isOpened() == False):
        print ("Error opening stream or a file")
    while (True):
        ret, frame = video_capture.read()
        detect_ball_in_a_frame(frame)
        time.sleep(0.033)
        if ret == True:
            cv2.imshow('RGB Video', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): # delay of a one milisecond
            break
    video_capture.release()
    cv2.destroyAllWindows()

if __name__ == main():
    main()

