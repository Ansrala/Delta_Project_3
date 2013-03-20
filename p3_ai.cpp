

#include "ros/ros.h"
#include "serializer/SensorState.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <std_msgs/String.h>

//our custom messages

#include <p3a_delta/command.h>

using namespace std;


serializer::SensorState serialSensors;

bool tryingLeft;
bool tryingRight;
bool tryingForward;
bool tryingBackward;

void getCurrCommand(const p3a_delta::command& msg);

void checkSensorChange(const serializer::SensorState& msg);


float getValue();


int main(int argc, char **argv)
{
  ros::init(argc, argv, "p3_delta_ai");
  ros::NodeHandle n;

  ros::Publisher cmd_vel_pub = n.advertise<p3a_delta::command>("command", 10);

  geometry_msgs::Twist output;

ros::Rate loop_rate(10);

  //the state messenger doesn't need the same resolution as the cmd::velocity channel.  
//Thus, we only publish 5 times a second.
  ros::Publisher command_pub = n.advertise<geometry_msgs::Twist>("FState", 5);

ros::topic::waitForMessage<nav_msgs::Odometry>(std::string("odom"), n,ros::Duration(30));
    geometry_msgs::Twist msg;


   //we listen a little faster than we publish, since we don't know when it will change
    ros::Subscriber subc = n.subscribe("serializer/sensors", 10, checkSensorChange);
	//['right_ir', 'left_ir', 'voltage', 'touch_1', 'touch_2'] 

	
   
//begin your methodology
ROS_INFO("STARTING LOOP");

while(ros::ok())
{

	
	


	ros::spinOnce();
	loop_rate.sleep();
}


//end your methadology


  return 0;
}







void checkSensorChange(const serializer::SensorState& msg)
{
	//ROS_INFO("
  	for(int i = 0; i < msg.name.size(); i++)
	{
		serialSensors.name.push_back(msg.name[i]);	
	}

	for(int i = 0; i < msg.value.size(); i++)
	{
		serialSensors.value.push_back(msg.value[i]);
	}
}



//acutally, shit.  Keep this for the motor node
void getCurrCommand(const p3a_delta::command& msg)
{
	tryingLeft = msg.turnLeft;
	tryingRight = msg.turnRight;
	tryingForward = msg.forward;
	tryingBackward = msg.backward;
}


float getValue()
{
	if(serialSensors.value.size() > 0)
  		return serialSensors.value[1];
	else return -1;
}





