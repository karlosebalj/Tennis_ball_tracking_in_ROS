#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

int main(int argc, char* argv[]) 
{
    //Setup ROS
    ros::init(argc, argv, "internal_cam_testing");

    ROS_INFO("Testing Camera");
    cv::VideoCapture cap(1);
    if (!cap.isOpened())
    {
        std::cout << "Camera can not be opened" << std::endl;
        ROS_INFO("Camera can not be opened");
        std::cin.get();
        return -1;
    }  
    ROS_INFO("Camera tested");

    double fps = cap.get(CV_CAP_PROP_FPS);
    std::cout << "Frames per seconds: " << fps << std::endl;
    std::string window_name = "My_ros_video";

    cv::namedWindow(window_name, cv::WINDOW_NORMAL);

    while (true) {
        cv::Mat frame;
        bool success = cap.read(frame);
        if (success == false)
        {
            std::cout << "Video ended" << std::endl;
            break;
        }

        cv::imshow(window_name, frame);
         //wait for for 10 ms until any key is pressed.  
        //If the 'Esc' key is pressed, break the while loop.
        //If the any other key is pressed, continue the loop 
        //If any key is not pressed withing 10 ms, continue the loop
        if (cv::waitKey(10) == 27)
        {
            std::cout << "Esc key is pressed by user. Stoppig the video" << std::endl;
            break;
        }
        
        
    }

}