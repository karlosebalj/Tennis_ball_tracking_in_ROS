#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include "opencv2/opencv.hpp"

int main() 
{
    cv::Mat image;
    
    //LOAD image
    image = cv::imread("/home/karlo/catkin_ws/src/turtlesim_cleaner/src/images/chess.jpg", CV_LOAD_IMAGE_COLOR);
    if (!image.data) //check for invalid input
    {
        std::cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    //DISPLAY image
    cv::namedWindow ("window", CV_WINDOW_AUTOSIZE);
    cv::imshow("window" , image);

    //SAVE IMAGE
    cv::imwrite("/home/karlo/catkin_ws/src/turtlesim_cleaner/src/images/copy/copy_image.jpg", image);

    cv::waitKey(0);
}