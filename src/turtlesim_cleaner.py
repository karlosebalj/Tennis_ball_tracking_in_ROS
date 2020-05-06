#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

x=0
y=0
z=0 
yaw =0

def poseCallback(pose_message):
    global x
    global y, yaw
    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta

def move(speed, distance, is_forward):
    #declare a Twist msg to send velocity commands
    velocity_message = Twist()
    #get current location
    global x, y
    x0 = x
    y0 = y
    if (is_forward):
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)
    
    distance_moved = 0.0
    loop_rate = rospy.Rate(10) #we publish the velocity at 10 Hz(10 times a second)
    cmd_vel_topic='/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    while True:
        rospy.loginfo("Turtle moves forward")
        velocity_publisher.publish(velocity_message)

        loop_rate.sleep()
        #rospy.Duration(1.0)

        distance_moved = distance_moved + abs(0.5 * math.sqrt((x-x0)**2)+ (y-y0)**2)
        print (distance_moved)
        if not (distance_moved < distance):
            rospy.loginfo("reached")
            break
    #stop the robot when it reaches the target        
    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)

def rotate(angular_speed_degree, relative_angle_degree, clockwise):
    global yaw
    velocity_message = Twist()
    velocity_message.linear.x = 0
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0
    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    velocity_message.angular.z = 0

    #get current location
    theta0 = yaw
    angular_speed = math.radians(abs(angular_speed_degree))

    if (clockwise):
        velocity_message.angular.z = -abs(angular_speed)
    else:
        velocity_message.angular.z = abs(angular_speed)
        
    angle_moved = 0.0
    loop_rate = rospy.Rate(10)
    cmd_vel_topic = '/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    t0 = rospy.Time.now().to_sec()

    while True:
        rospy.loginfo("Turtlesim rotates")
        velocity_publisher.publish(velocity_message)

        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0) * angular_speed_degree
        loop_rate.sleep()

        if (current_angle_degree >relative_angle_degree):
            rospy.loginfo("reached")
            break
    velocity_message.angular.z =0
    velocity_publisher.publish(velocity_message)

def go_to_goal(x_goal, y_goal):
    global x
    global y, z, yaw

    velocity_message = Twist()
    cmd_vel_topic='/turtle1/cmd_vel'

    while (True):
        K_linear = 0.5 
        distance = abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))

        linear_speed = distance * K_linear

        K_angular = 4.0
        desired_angle_goal = math.atan2(y_goal-y, x_goal-x)
        angular_speed = (desired_angle_goal-yaw)*K_angular

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed

        velocity_publisher.publish(velocity_message)
        print ('x=', x, 'y=',y)


        if (distance <0.01):
            break


def setDesiredOrientation(desired_angle_radians):
    relative_angle_radians = desired_angle_radians - yaw
    if relative_angle_radians < 0:
        clockwise = 1
    else:
        clockwise = 0
    print (relative_angle_radians)
    print (desired_angle_radians)
    rotate(30 ,math.degrees(abs(relative_angle_radians)), clockwise)




if __name__ == '__main__':
    try:
        rospy.init_node('turtlesim_cleaner', anonymous=True)

        #declare velocity publisher
        cmd_vel_topic='/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)
        time.sleep(2)

        # move(1.0, 2.0, True)
        # rotate(30, 90, True)
        # go_to_goal(1.0, 1.0)
        x_goal = rospy.get_param("x_goal")
        y_goal = rospy.get_param("y_goal")

        print('x_goal = ', x_goal)
        print('y_goal = ', y_goal)
        go_to_goal(x_goal, y_goal)

        setDesiredOrientation(math.radians(90))
 
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
