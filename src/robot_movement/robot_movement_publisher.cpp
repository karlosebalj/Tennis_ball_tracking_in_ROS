
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <math.h>       /* sqrt */
#include <sstream>


void poseRobot(const turtlesim::Pose& pose) {
    float x = pose.x;
    float y = pose.y;
    float theta = pose.theta;
    float linear_velocity = pose.linear_velocity;
    float angular_velocity = pose.angular_velocity;
    
    std::cout << "------" << x << std::endl;
    std::cout << "x:" << x << std::endl;
    std::cout << "y:" << y << std::endl;
    std::cout << "theta:" << theta << std::endl;
    std::cout << "linear_velocity:" << linear_velocity << std::endl;
    std::cout << "angular_velocity:" << angular_velocity << std::endl;
    std::cout << "------" << x << std::endl;

}

void moveRobotForward(float speed) {
	ros::NodeHandle n;
	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	ros::Rate loop_rate(10); //10 messages per second

    geometry_msgs::Twist msg;
    msg.linear.x = speed;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;

    float distance_moved = 0.0;

    turtlesim::Pose pose;
    float  x0 = pose.x;
    float  y0 = pose.y;
    int count;

  

    while (ros::ok()) // Keep spinning loop until user presses Ctrl+C
    {
        // ROS_INFO("Robot moves forward");
        velocity_publisher.publish(msg);
        ros::spinOnce(); 
        loop_rate.sleep();
        count++;
        if (count == 5) {
           break;
        }

    //     distance_moved = distance_moved + abs(0.5 * sqrt((pow(msg.linear.x-x0,2) + pow(msg.linear.y -y0,2))));
    //     std::cout << "distance" << distance_moved << std::endl;
    //     if (distance_moved > distance) {
    //         std::cout << "goal reached!" << std::endl;
    //         break;
    //    }
    }
}

void moveRobotRotate(float speed) {
	ros::NodeHandle n;
	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	ros::Rate loop_rate(1); //1 message per second
    int count = 0;
    while (true) // Keep spinning loop until user presses Ctrl+C
    {
	   geometry_msgs::Twist msg;

       msg.linear.x = 0.0;
       msg.linear.y = 0.0;
       msg.linear.z = 0.0;
       msg.angular.x = 0.0;
       msg.angular.y = 0.0;
       msg.angular.z = 1.0;
       velocity_publisher.publish(msg);
       ros::spinOnce(); 
       loop_rate.sleep();
       count++;
       if (count == 2) {
           break;
       }
    }
}

int main(int argc, char **argv)
{
	// Initiate new ROS node named "talker"
	ros::init(argc, argv, "robot_move_publisher");
    ros::NodeHandle n;
    ros::Subscriber pose_subscriber = n.subscribe("/turtle1/pose", 1000, poseRobot);
    moveRobotForward(1.0);
    moveRobotRotate(1.0);



}



