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
using namespace std::chrono_literals;	using std::cout, std::endl, std::valarray;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class node : public rclcpp::Node
{
	public:
		node()
		: Node("nod"), log(false)
		{
			sdec = this->create_subscription<sensor_msgs::msg::Image>("dec", 1, std::bind(&node::cbdec, this, _1));
			skep = this->create_subscription<std_msgs::msg::Int32>("kep", 1, std::bind(&node::cbkep, this, _1));

			slto = this->create_subscription<nav_msgs::msg::Odometry>("lto", 1, std::bind(&node::cblto, this, _1));
			srto = this->create_subscription<nav_msgs::msg::Odometry>("rto", 1, std::bind(&node::cbrto, this, _1));

			//movement 
			pcv = this->create_publisher<geometry_msgs::msg::Twist>("cv", 1);
		}
		volatile bool log;

	private:
		void cbdec(const sensor_msgs::msg::Image::SharedPtr msg) const
		{
			#define ssize 2	//sensorsize	start to use when you start to change the sensor size frequently
			float (*damda)[2] = (float(*)[2])msg->data.data();
			float *daa = (float*)msg->data.data();

			if (log)
				{for (int a = 0; a < msg->data.size() / 4; a++)	cout << daa[a] << " ";	cout<<endl;}

			static geometry_msgs::msg::Twist mcv;
			static unsigned char state=255, timec=0;

			unsigned char emerg=0, newstate=0;

			if (timec)	timec--;
			else
			{
				for (int a=0; a<4; a++)	if (daa[a]==INFINITY)	newstate=emerg=a+1;
				if (newstate-state)	//state changed
				{
					switch (emerg)
					{
						case 1:	mcv.angular.z=-1, mcv.linear.x=0;	break;
						case 2:	mcv.angular.z=1, mcv.linear.x=0;	break;
						case 3:	mcv.angular.z=-1, mcv.linear.x=0;	break;
						case 4:	mcv.angular.z=1, mcv.linear.x=0;	break;
						default:	mcv.angular.z=0, mcv.linear.x=1;
					}
					if (newstate!=0)	timec=random()%8;	//emergency occurs, randomly keep turning
					pcv->publish(mcv);
					cout << "publishing - " << (int)timec << endl;
					state=newstate;
				}
			}
		}
		void cbkep(const std_msgs::msg::Int32::SharedPtr msg)
		{
			static geometry_msgs::msg::Twist mcv;	
			switch (msg->data)
			{
				case 16777234:	mcv.angular.z=1, mcv.linear.x=0;	break;	//left
				case 16777235:	mcv.angular.z=0, mcv.linear.x=1;	break;	//up
				case 16777236:	mcv.angular.z=-1, mcv.linear.x=0;	break;	//right
				case 16777237:	mcv.angular.z=0, mcv.linear.x=-1;	break;	//down
				case 16777248:	log=!log;	break;	//right shift
				default:	mcv.angular.z=0, mcv.linear.x=0;
			}
			pcv->publish(mcv);
		}
		void cblto(const nav_msgs::msg::Odometry::SharedPtr msg)
			{cbto(0,msg);}
		void cbrto(const nav_msgs::msg::Odometry::SharedPtr msg)
			{cbto(1,msg);}
		void cbto(bool which, auto msg)
		{
			static valarray<float> xarra,yarra;
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
		
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sdec;
		rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr skep;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr slto,srto;

		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pcv;
};

int main(int argc, char * argv[])
{
	srandom(time(0));

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<node>());
	rclcpp::shutdown();
	return 0;
}