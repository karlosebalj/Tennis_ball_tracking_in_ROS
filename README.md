Tracking a tennis ball using OpenCV made inside ROS for a mobile robot

Ball detection algorithm consists of:
	1. Read input image from USB camera as RGB
	2. Filter the color of the ball using HSV method
	3. Apply contour detection algorithm to find edges and contours
	4. Get center position of the ball and print those values
	5. Display information of the ball (position, center)
