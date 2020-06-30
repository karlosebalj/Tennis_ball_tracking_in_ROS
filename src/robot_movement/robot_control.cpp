#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"

#include <sstream>

const double x_min = 0.0;
const double y_min = 0.0;
const double x_max = 11.0;
const double y_max = 11.0;

const double PI = 3.14159265359;

class RobotControl
{
private:
	ros::NodeHandle nodeHandle_;
	ros::Publisher velocity_pub;
	ros::Subscriber pose_sub;
	turtlesim::Pose turtlesim_pose;

public:
	RobotControl() 
	{
		velocity_pub = nodeHandle_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
		pose_sub = nodeHandle_.subscribe("/turtle1/pose", 10, &RobotControl::poseCallback, this);
	}

	void moveLinear(double speed, double distance, bool direction);
	void roatateAngular (double angular_speed, double angle, bool clockwise);

	double degrees2radians(double angle_in_degrees);
	void setDesiredOrientation (double desired_angle_radians);
	
	void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
	void moveToAPose(turtlesim::Pose goalPosition, double distance_tolerance);

	void gridClean();
	void spiralClean();
};

// Gibanje robota prema naprijed sa željenom linearnom brzinom 
// za definiranu udaljenost pravocrtno prema naprijed ili nazad

void RobotControl::moveLinear(double speed, double distance, bool direction){
	geometry_msgs::Twist vel_msg;

	// Određivanje smjera prema naprijed ili prema nazad
	if (direction)
		vel_msg.linear.x =abs(speed);
	else
		vel_msg.linear.x =-abs(speed);
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;

	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z =0;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(100);
	while(current_distance<distance){
		velocity_pub.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
		//std::cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<std::endl;
	}

	//zaustavi robota
	vel_msg.linear.x =0;
	velocity_pub.publish(vel_msg);
}


void RobotControl::roatateAngular(double angular_speed, double relative_angle, bool clockwise){

	geometry_msgs::Twist vel_msg;
	// postavljanje linearnih brzina na nulu
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	// postavljanje x i y komponenata na nulu
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

	if (clockwise)
		vel_msg.angular.z = -fabs(angular_speed);
	else
		vel_msg.angular.z = fabs(angular_speed);

	double current_angle = 0.0;
	double t0 = ros::Time::now().toSec();
	ros::Rate loop_rate(100);
	while(current_angle<relative_angle){
		velocity_pub.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	};

	//zaustavi robota
	vel_msg.angular.z =0;
	velocity_pub.publish(vel_msg);

}

double RobotControl::degrees2radians(double angle_degrees){
	return (angle_degrees *PI) /180.0;
}


void RobotControl::setDesiredOrientation (double desired_angle_radians){
	double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
	bool clockwise = ((relative_angle_radians<0)?true:false);
	std::cout << "Zeljeni iznos kuta[rad]: " << desired_angle_radians << std::endl;
	std::cout << "Kut zakreta robota[rad]: " << turtlesim_pose.theta << std::endl;
	std::cout << "Relativan kut zakreta robota[rad]: " << relative_angle_radians <<std::endl;
	std::cout << "Smjer kazaljke na satu = 1, obrnuti smjer = 0:  " << clockwise <<std::endl;
	std::cout << ""  <<std::endl;

	roatateAngular(degrees2radians(10), fabs(relative_angle_radians), clockwise);

}

void RobotControl::poseCallback(const turtlesim::Pose::ConstPtr &pose_message){
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;
}

double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

void RobotControl::moveToAPose(turtlesim::Pose goalPosition, double distance_tolerance){

	geometry_msgs::Twist vel_msg;

	ros::Rate loop_rate(100);
	double tolerance = 0.01;
	do{
		/****** Proporcionalni rtolerancegulator******/
		//linearana brzina u smjeru x-osi
		double Kp=1.0;
		double error = getDistance(turtlesim_pose.x, turtlesim_pose.y, goalPosition.x, goalPosition.y);
		double tolerance = tolerance+error;
		//Kp = v0 * (exp(-alpha)*error*error)/(error*error);
		vel_msg.linear.x = (Kp*error);
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;

		//kutna brzina u smjeru z-osi
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =4*(atan2(goalPosition.y-turtlesim_pose.y, goalPosition.x-turtlesim_pose.x)-turtlesim_pose.theta);

		velocity_pub.publish(vel_msg);
		ros::spinOnce();
		loop_rate.sleep();

	}while(getDistance(turtlesim_pose.x, turtlesim_pose.y, goalPosition.x, goalPosition.y)>distance_tolerance);
	std::cout << "Stigao na cilj! "<<std::endl;
	vel_msg.linear.x =0;
	vel_msg.angular.z = 0;
	velocity_pub.publish(vel_msg);
}



void RobotControl::gridClean(){

	ros::Rate loop(0.5);

	// kreiranje varijable pozicije
	turtlesim::Pose pose;
	pose.x=1;
	pose.y=1;
	pose.theta=0;

	// odlazak u zeljenu poziciju
	moveToAPose(pose, 0.01);
	loop.sleep();
	setDesiredOrientation(0);
	loop.sleep();
	
	// vozi linearno 
	moveLinear(2.0, 9.0, true);
	loop.sleep();

	// rotacija za 90 stupnjeva i pomicanje gore
	roatateAngular(degrees2radians(10), degrees2radians(90), false);
	loop.sleep();
	moveLinear(2.0, 9.0, true);

	// rotacija za 90 stupnjeva i pomicanje lijevo
	roatateAngular(degrees2radians(10), degrees2radians(90), false);
	loop.sleep();
	moveLinear(2.0, 1.0, true);

	// rotacija za 90 stupnjeva i pomicanje dolje
	roatateAngular(degrees2radians(10), degrees2radians(90), false);
	loop.sleep();
	moveLinear(2.0, 9.0, true);

	// rotacija za 90 stupnjeva i pomicanje lijevo
	roatateAngular(degrees2radians(30), degrees2radians(90), true);
	loop.sleep();
	moveLinear(2.0, 1.0, true);

	// rotacija za 90 stupnjeva i pomicanje gore
	roatateAngular(degrees2radians(30), degrees2radians(90), true);
	loop.sleep();
	moveLinear(2.0, 9.0, true);

	double distance = getDistance(turtlesim_pose.x, turtlesim_pose.y, x_max, y_max);

}

int main(int argc, char **argv)
{
	// Inicijalizacija ROS čvora robot_control
	ros::init(argc, argv, "robot_control");

	//instanciranje robot objekta
	RobotControl robot;

	// inicijalizacija varijabli 
	double linearSpeed = 0.0; 
	double angularSpeed = 0.0;
	double distance = 0.0 ; 
	double angle = 0.0 ;
	bool direction = true;
	bool clockwise = true;

	//Program za testiranje upravljanja robota
/* 	// ROS_INFO("\n\n\n******POCETAK TESTIRANJA************\n");
	// std::cout << "Unesi zeljenu linearnu brzinu: ";
	// std::cin >> linearSpeed;
	// std::cout << "Unesi zeljenu udaljenost: ";
	// std::cin >> distance;
	// std::cout << "Naprijed = 1, Nazad = 0: ";
	// std::cin >> direction;
	// robot.moveLinear(linearSpeed, distance, direction);
	// ROS_INFO("Zavrseno linearno gibanje");

	// std::cout <<"Unesi zeljenu kutnu brzinu (degree/sec): ";
	// std::cin >> angularSpeed;
	// std::cout << "Unesi zeljeni kut (degrees): ";
	// std::cin >> angle;
	// std::cout << "Smjer kazaljke na satu = 1, Suprotan smjer kazaljke na satu = 0: ";
	// std::cin >> clockwise;
	// robot.roatateAngular(robot.degrees2radians(angularSpeed), robot.degrees2radians(angle), clockwise);
	// ROS_INFO("Gotova rotacija robota"); */

	
	/* 	ROS_INFO("\n*********Testiranje zakretanja robota za zeljeni kut************\n");
	ros::Rate loop_rate(0.5);
	robot.setDesiredOrientation(robot.degrees2radians(120));
	loop_rate.sleep();
	robot.setDesiredOrientation(robot.degrees2radians(-60));
	loop_rate.sleep();
	robot.setDesiredOrientation(robot.degrees2radians(0));  */

/* 	ROS_INFO("\n\n\n**********Gibanje robota u zeljenu poziciju u prostoru************\n");
	ros::Rate loop_rate(0.5);
	turtlesim::Pose goalPosition;
	goalPosition.x = 10;
	goalPosition.y = 10;
	goalPosition.theta=0;
	robot.moveToAPose(goalPosition, 0.01);
	loop_rate.sleep(); 
 */
	// ros::Rate loop(0.5);
	// turtlesim::Pose pose;
	// pose.x=1;
	// pose.y=1;
	// pose.theta=0;
	// moveToAPose(pose, 0.01);

    //#############******########
	// pose.x=6;
	// pose.y=6;
	// pose.theta=0;
	// moveToAPose(pose, 0.01);
	
	// ROS_INFO("Gibanje robota po zadanoj mreži");
	// robot.gridClean();

	ros::spin();
	return 0;
}








