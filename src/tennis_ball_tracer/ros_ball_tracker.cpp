#include "ros/ros.h"
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <nav_msgs/Odometry.h>

static const std::string OPENCV_WINDOW = "Image window";

static int x, y, radius;


class BallDetector {
private:
    ros::NodeHandle nodeHandle_;
    image_transport::ImageTransport imageTransport_;
    image_transport::Subscriber imageSub_;
    image_transport::Publisher imagePub_;
public:
    BallDetector() : imageTransport_(nodeHandle_) 
    {
        //Subscribe to input video feed and publish output video feed
        imageSub_ = imageTransport_.subscribe(
            "/usb_cam/image_raw", 10, &BallDetector::imageCallback, this);
        imagePub_ = imageTransport_.advertise("/image_converter/output_video", 10);

        cv::namedWindow(OPENCV_WINDOW, 0);
    }

    ~BallDetector() 
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
        cv_bridge::CvImagePtr cvPtr;
        cv_bridge::CvImagePtr cvGrayPtr;
        cv::Mat rgbIMG, hsvIMG, thresholdedIMG, lowerBoundColor, upperBoundColor;

        lowerBoundColor =cv::Scalar(30, 100, 50);
        upperBoundColor = cv::Scalar(60, 255, 255);

        std::vector<cv::Vec3f> v3fCircles;		// 3 element vector of floats, this will be the pass by reference output of HoughCircles()
        try
        {
            cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            rgbIMG = cvPtr->image;
            // cv::imshow("RGB window", rgbIMG);

            // convert it into HSV color space
            cv::cvtColor(rgbIMG, hsvIMG,  cv::COLOR_BGR2HSV);
            // cv::imshow("HSV window", hsvIMG);

            cv::inRange(hsvIMG, cv::Scalar(30, 100, 50), cv::Scalar(60, 255, 255), thresholdedIMG); //Threshold the image
            
            //morphological opening (remove small objects from the foreground)
            cv::erode(thresholdedIMG, thresholdedIMG, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
            cv::dilate( thresholdedIMG, thresholdedIMG, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 

            //morphological closing (fill small holes in the foreground)
            cv::dilate( thresholdedIMG, thresholdedIMG, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
            cv::erode(thresholdedIMG, thresholdedIMG, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

            // cv::GaussianBlur(thresholdedIMG, thresholdedIMG, cv::Size(3, 3), 0);   //Blur Effect
            cv::GaussianBlur(thresholdedIMG, thresholdedIMG, cv::Size(9, 9), 0);   //Blur Effect

            cv::HoughCircles(thresholdedIMG, v3fCircles, CV_HOUGH_GRADIENT, 1, hsvIMG.rows/8, 100, 20, 0, 0);

            cv::imshow("Thresholded Image", thresholdedIMG); //show the thresholded image
            // cv::imshow("Mask image", thresholdedIMG);
        }
        catch(const std::exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // cv::circle(cvPtr->image, cv::Point(50,50), 100, CV_RGB(255, 0, 0));


        for (size_t i = 0; i < v3fCircles.size(); i++) {
            std::cout << "Ball position X = "<< v3fCircles[i][0]			// x position of center point of circle
				<<",\tY = "<< v3fCircles[i][1]								// y position of center point of circle
				<<",\tRadius = "<< v3fCircles[i][2]<< "\n";					// radius of circle

            // x = static_cast<int>(round(v3fCircles[i][0]));
            // y = static_cast<int>(round(v3fCircles[i][1]));
            // radius = static_cast<int>(round(v3fCircles[i][2]));
            // cv::Point center(x, y);

            // cv::circle(cvPtr->image, cv::Point(50,50), 10, CV_RGB(255, 0, 0));

            // cv::line(srcIMG, center, center, black, 2);

            std::cout << "Ball position X = "<< x			        // x position of center point of circle
				      <<",\tY = "<< y								// y position of center point of circle
				      <<",\tRadius = "<< radius<< "\n";				// radius of circle
            cv::circle(rgbIMG,												    // draw on original image
				cv::Point((int)v3fCircles[i][0], (int)v3fCircles[i][1]),		// center point of circle
				3,																// radius of circle in pixels
				cv::Scalar(0, 255, 0),											// draw green
				CV_FILLED);														// thickness

																				// draw red circle around object detected 
			cv::circle(rgbIMG,												    // draw on original image
				cv::Point((int)v3fCircles[i][0], (int)v3fCircles[i][1]),		// center point of circle
				(int)v3fCircles[i][2],											// radius of circle in pixels
				cv::Scalar(0, 0, 255),											// draw red
				3);	
            cv::imshow("RGB window", rgbIMG);
        }
        
        // update GUI video
        cv::imshow(OPENCV_WINDOW, cvPtr->image);
        if (cv::waitKey(10) == 27)
        {
            std::cout << "Esc key is pressed by user. Stoppig the video" << std::endl;
            cv::destroyAllWindows();
        }

        imagePub_.publish(cvPtr->toImageMsg());
    }
};

int main(int argc, char* argv[]) 
{
    ros::init(argc, argv, "ros_ball_tracker");
    ROS_INFO("Node is initialized");
    BallDetector ball;
    ros::spin();
    return 0;
}