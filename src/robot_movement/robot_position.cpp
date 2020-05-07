#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
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


int main(int argc, char **argv)
{
	// Initiate new ROS node named "talker"
	ros::init(argc, argv, "robot_pose");
    ros::NodeHandle node;
    ros::Subscriber pose_subscriber = node.subscribe("/turtle1/pose", 1000, poseRobot);
    ros::spin();
}
