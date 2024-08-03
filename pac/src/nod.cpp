#include <ostream>
#include <stdlib.h>
#include <valarray>

//#include <chrono>
#include <functional>
#include <memory>
//#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;	using std::cout, std::endl, std::valarray;	//I planned to use valarrays to hold information about the past locations, but did not have the chance to use them since I had problems integrating the IMU sensor inside the robot.

class node : public rclcpp::Node
{
	public:
		node()
		: Node("nod"), log(false)
		{
			sdec = this->create_subscription<sensor_msgs::msg::Image>("dec", 1, std::bind(&node::cbdec, this, _1));	//depth camera
			skep = this->create_subscription<std_msgs::msg::Int32>("kep", 1, std::bind(&node::cbkep, this, _1));	//key-press data from Gazebo

			slto = this->create_subscription<nav_msgs::msg::Odometry>("lto", 1, std::bind(&node::cblto, this, _1));	//left track odometry
			srto = this->create_subscription<nav_msgs::msg::Odometry>("rto", 1, std::bind(&node::cbrto, this, _1));	//right track odometry

			//movement 
			pcv = this->create_publisher<geometry_msgs::msg::Twist>("cv", 1);	//cmd_vel to the robot
		}
		volatile bool log;

	private:
		void cbdec(const sensor_msgs::msg::Image::SharedPtr msg) const	//callback - depth camera
		{
			#define ssize 2	//sensorsize	sensor is a square of ssize*ssize cells
			float (*damda)[ssize] = (float(*)[ssize])msg->data.data();
			float *daa = (float*)msg->data.data();

			if (log)	//rshift toggles log
				{for (int a = 0; a < msg->data.size() / 4; a++)	cout << daa[a] << " ";	cout<<endl;}

			static geometry_msgs::msg::Twist mcv;	//message cmd_vel
			static unsigned char state=255, timec=0;	//time counter counts for random amount of turn.
			/*state
			255	initial
			0	inside the panel, free to move (moves straight)
			1..4	emergency occured (one of the dec sensor's cells show indfinite)
			*/

			unsigned char emerg=0, newstate=0;

			if (timec)	timec--;	//turns for a random amount of time (this function cbdec is called for every depth camera sensor input, so timec/dec_freq is the amount of the time)
			else
			{
				for (int a=0; a<4; a++)	if (daa[a]==INFINITY)	newstate=emerg=a+1;	//if emergency occured (one of the dec sensor's cells show indfinite), change the state accordingly
				if (newstate-state)	//state changed
				{
					switch (emerg)	//compute which way to turn in case of emergency or go straight if emergency==0
					{
						case 1:	mcv.angular.z=-1, mcv.linear.x=0;	break;
						case 2:	mcv.angular.z=1, mcv.linear.x=0;	break;
						case 3:	mcv.angular.z=-1, mcv.linear.x=0;	break;
						case 4:	mcv.angular.z=1, mcv.linear.x=0;	break;
						default:	mcv.angular.z=0, mcv.linear.x=1;
					}
					if (newstate!=0)	timec=random()%8;	//emergency occurs, for a random time keep turning
					pcv->publish(mcv);
					if (log)	cout << "publishing - " << (int)timec << endl;
					state=newstate;
				}
			}
		}
		void cbkep(const std_msgs::msg::Int32::SharedPtr msg)	//callback - keypress
		{	//you can control the robot via keypresses too
			static geometry_msgs::msg::Twist mcv;	
			switch (msg->data)
			{
				case 16777234:	mcv.angular.z=1, mcv.linear.x=0;	break;	//left
				case 16777235:	mcv.angular.z=0, mcv.linear.x=1;	break;	//up
				case 16777236:	mcv.angular.z=-1, mcv.linear.x=0;	break;	//right
				case 16777237:	mcv.angular.z=0, mcv.linear.x=-1;	break;	//down
				case 16777248:	log=!log;	break;	//right shift
				default:	mcv.angular.z=0, mcv.linear.x=0;	//stop
			}
			pcv->publish(mcv);
		}
		//these are for getting the track odometries: how much the track has moved forward
		void cblto(const nav_msgs::msg::Odometry::SharedPtr msg)	{cbto(0,msg);}	//left track
		void cbrto(const nav_msgs::msg::Odometry::SharedPtr msg)	{cbto(1,msg);}	//right track
		void cbto(bool which, auto msg)	//callback - track odometry
		{
		/*
		This function calculates the deltas using static variables. And tries to calculate how much the robot has turned. Then wants to log this info inside a valarray to smartly clean the panel in the future. However, without any IMU, I cannot fuse and calculate a accurate orientation value.
		What I planned was: to make a calibration algorithm using the simulation:
			I'd first use a errorless IMU in the simulation to be able to know absolutely how much the robot is turned.
			Then make a way to map the derivative of the tracks' cumulative turn values to the derivative of the orientation.
			Validate this mapping using different track_efficiency parameters inside the wor.sdf
			Then add a second IMU which is noised.
			Try to fuse 2 sensor values together using the anoised IMU values to be able to estimate the real orientation derivative.
		*/
			static valarray<float> xarra,yarra;	//x array and y array.	These valarrays was meant to be used for the robot to avoid freshly cleaned points inside the panel and go to uncleaned points. And also give a output of "how cleaned the solar panel is" when desired.
			static float x,y, direc=M_PI_2;
			static bool lread, rread;	static float lforwa,rforwa;
			static float lforwadelta,rforwadelta;
			if (which)	rread=true, rforwadelta = msg->pose.pose.position.x - rforwa;
			else	lread=true, lforwadelta = msg->pose.pose.position.x - lforwa;
			if (lread && rread)
			{
				lread=rread=false;
				float forwa = (lforwadelta+rforwadelta)/2;
				x += sinf(direc) * forwa, y += cosf(direc) * forwa;
				direc += (rforwadelta-lforwadelta)/2;
				lforwa+=lforwadelta, rforwa+=rforwadelta;
			}
			//if (log)	cout << x << " x - y " << y << endl;
			if (log)	cout << direc << endl;
		}
		
		//s is for subscriver and p is for publisher
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sdec;
		rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr skep;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr slto,srto;

		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pcv;
};

int main(int argc, char * argv[])
{
	srandom(time(0));	//initialize the randomizer to give different values for every run of the program

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<node>());
	rclcpp::shutdown();
	return 0;
}
