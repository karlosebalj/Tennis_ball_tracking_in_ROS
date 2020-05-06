#include "opencv2/opencv.hpp"

int main(int, char**) {
    cv::VideoCapture video_capture(0); //open the default camera
    if (!video_capture.isOpened()) //check if we succeded
        return -1;
    
    cv::Mat gray_image;
    cv::namedWindow("gray_image", 1);
    while(true)
    {
        cv::Mat frame;
        video_capture >> frame; //get a new frame from the camera
        cv::cvtColor(frame, gray_image, cv::COLOR_BGR2HSV);
        // cv::GaussianBlur (gray_image, gray_image, cv::Size(7, 7), 1.5, 1.5);
        // cv::Canny (gray_image, gray_image, 0, 30, 3);
        cv::imshow("gray_image", gray_image);
        if (cvWaitKey(30) >= 0)
            break;
    }
    //the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}