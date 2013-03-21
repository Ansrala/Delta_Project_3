//Motor control
//Joe Moore

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <std_msgs/String.h>

//our custom messages
#include <p3a_delta/command.h>

using namespace std;

void getCommand(const p3a_delta::command& msg);
geometry_msgs::Twist getTwistMsg(int turnDirection);
//0 forward,    1 left,   2 right,  3 back,  4 stop
int turnDirection;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "p3_delta_motor");
	ros::NodeHandle nh;

	ros::Rate loop_rate(10);
  
	ros::Subscriber info_get = nh.subscribe("command", 10, getCommand);
	ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	
	while(ros::ok())
	{
		
		cmd_vel_pub.publish(getTwistMsg());
		ros::spinOnce();
		loop_rate.sleep();

	}

 return 0;
}
void getCommand(const p3a_delta::command& msg)
{
	if(msg.forward)
	   turnDirection = 0;
	else if(msg.turnLeft)
	   turnDirection = 1;
	else if(msg.turnRight)
	   turnDirection = 2;
	else if(msg.backUp)
	   turnDirection = 3;
	else 
	   turnDirection = 4;
}
geometry_msgs::Twist getTwistMsg()
{
	geometry_msgs::Twist twist;
	switch(turnDirection){
	case 0:
	  twist.linear.x = 0.25f;
          twist.angular.z = 0.0f;	
	  break;
	case 1:
	  twist.linear.x = 0.0f;
          twist.angular.z = 0.3f;	
	  break;
	case 2:
	  twist.linear.x = 0.0f;
          twist.angular.z = -0.3f;	
	  break;
	case 3:
	  twist.linear.x = -0.25f;
          twist.angular.z = 0.0f;	
	  break;
	case default:
	  twist.linear.x = 0.0f;
          twist.angular.z = 0.0f;	
	  break;
	}
	return twist;
}
