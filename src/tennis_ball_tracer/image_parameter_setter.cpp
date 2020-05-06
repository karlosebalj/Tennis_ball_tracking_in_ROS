#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
    cv::Mat image;
    
    //LOAD image
    image = cv::imread("/home/karlo/catkin_ws/src/turtlesim_cleaner/src/images/tennisball01.jpg", CV_LOAD_IMAGE_COLOR);
    if (!image.data) //check for invalid input
    {
        std::cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    int iLowH = 30;
    int iHighH = 60;

    int iLowS = 100; 
    int iHighS = 255;

    int iLowV = 50;
    int iHighV = 255;

    //Create trackbars in "Control" window
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);


    Mat imgHSV;

    cvtColor(image, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
    
    Mat imgThresholded;

    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
        
    //morphological opening (remove small objects from the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

    //morphological closing (fill small holes in the foreground)
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    imshow("Thresholded Image", imgThresholded); //show the thresholded image
    imshow("Original", image); //show the original image

    if (waitKey(0) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
    {
        cout << "esc key is pressed by user" << endl;
        cv::destroyAllWindows();
    }
    
}

