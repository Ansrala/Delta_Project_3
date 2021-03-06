
#define OCC_THRESHOLD .15f
#define TRIP_THRESHOLD .4f
#define OFFCENTER 2

#define robotWidth .29f


#include "ros/ros.h"
#include "serializer/SensorState.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <std_msgs/String.h>
#include <cmath>

//our custom messages
#include <p3a_delta/zoneCount.h>
#include <p3a_delta/command.h>

using namespace std;


serializer::SensorState serialSensors;
p3a_delta::zoneCount zones;

bool tryingLeft;
bool tryingRight;
bool tryingForward;
bool tryingBackward;

enum zoneStates {FREE, OCCUPIED, TRIPPED};

//void getCurrCommand(const p3a_delta::command& msg);
void getCurrZones(const p3a_delta::zoneCount msg);

void checkSensorChange(const serializer::SensorState& msg);
void doNothing(p3a_delta::command &msg);


float getValue();


int main(int argc, char **argv)
{
  ros::init(argc, argv, "p3_delta_ai");
  ros::NodeHandle n;

  ros::Publisher commandPub = n.advertise<p3a_delta::command>("command", 0);

  p3a_delta::command outBound;
  
  bool escape = false;

ros::Rate loop_rate(5);



ros::topic::waitForMessage<nav_msgs::Odometry>(std::string("odom"), n,ros::Duration(30));
    geometry_msgs::Twist msg;

	ros::Subscriber subz = n.subscribe("zones", 0, getCurrZones);

   //we listen a little faster than we publish, since we don't know when it will change
    ros::Subscriber subc = n.subscribe("serializer/sensors", 0, checkSensorChange);
	//['right_ir', 'left_ir', 'voltage', 'touch_1', 'touch_2'] 

	
   
//begin your methodology
ROS_INFO("STARTING LOOP");

//how many zones are we talkin'
int zoneCount = 0;// = zones.zone.size();
int zonePointSize = 0;// = zones.pointNum / zoneCount;
float zoneAngleSize = 0.0f;// = zonePointSize * zones.angleIncrement;
vector<zoneStates> states;
//states.resize(zoneCount);

//how many zones open do we need to go forward?
float lsquared = 0.0f;// = zones.minScanDist * zones.minScanDist;
float angleMinOpen = 0.0f;// = acos((robotWidth*robotWidth - 2 * lsquared)/(2 * lsquared));
int zoneMinOpen = 0;// = angleMinOpen / (zonePointSize * zones.angleIncrement);
//if (zoneMinOpen == 0)
//	zoneMinOpen = 1;
	
float totalRange = zones.pointNum * zones.angleIncrement;

int count = 0;

bool justBackedUp = false;
bool sleep = false;
bool justWentForward = false;
while(ros::ok())
{
	zoneCount = zones.zone.size();
	
	if (zoneCount == 0)
	{
		ros::spinOnce();
		loop_rate.sleep();
		continue;
	}
	
		
	zonePointSize = zones.pointNum / zoneCount;
	zoneAngleSize = zonePointSize * zones.angleIncrement;
	if (states.size() != zoneCount)
		states.resize(zoneCount);
		
	lsquared = zones.minScanDist * zones.minScanDist;
	angleMinOpen = acos((robotWidth*robotWidth - 2 * lsquared)/(2 * lsquared));
	zoneMinOpen = angleMinOpen / (zonePointSize * zones.angleIncrement);
	if (zoneMinOpen == 0)
		zoneMinOpen = 1;
		
	totalRange = zones.pointNum * zones.angleIncrement;

	doNothing(outBound);
	//process zones
	for ( int i = 0; i < zoneCount; i ++)
	{
		if( (zones.zone[i]/(float(zonePointSize))) >= OCC_THRESHOLD)
		{
			states[i] = OCCUPIED;
		}
		else
		{
			states[i] = FREE;
		}
	}
	
	//rotate from the right, take the last free zone detected
	count = 0;
	for (int i = 0; i < zoneCount; i++)
	{
		if(states[i] == FREE)
			count++;
	
	}
	ROS_INFO("Count %d",count);
	//everything blocked
	if ( count == 0)
	{
			outBound.backUp = true;
			justBackedUp = true;
	}	
	//nothing blocking
	else if (count == zoneCount){
		if(justBackedUp){
			outBound.turnRight = true;
			sleep = true;
			justBackedUp = false;
		}
		else if(justWentForward){
		 	outBound.turnLeft = true;
		 	justWentForward = false;
		 	sleep = true;
		}
		else
		outBound.forward = true;
	}
	else 
	{
		if(justBackedUp){
			outBound.turnRight = true;
			sleep = true;
			justBackedUp = false;
		}
		else if(states[1] != FREE){
			if(states[0] == FREE)
				outBound.turnRight = true;
			else 
				outBound.turnLeft = true;
			
		}
		else{
			outBound.forward = true;
			justWentForward = true;
		}
	
		 
		
	}
	//checkable side
		
	commandPub.publish(outBound);
	if(sleep){
	sleep = false;
	ros::Duration(1.0).sleep();
	}
	
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
	tryingBackward = msg.backUp;
}


void getCurrZones(const p3a_delta::zoneCount msg)
{
	zones.pointNum = msg.pointNum;
	zones.angleIncrement = msg.angleIncrement;
	
	zones.zone.erase(zones.zone.begin(), zones.zone.end());
	for(int i = 0; i < msg.zone.size(); i++)
	{
		zones.zone.push_back(msg.zone[i]);
	}
}

float getValue()
{
	if(serialSensors.value.size() > 0)
  		return serialSensors.value[1];
	else return -1;
}

void doNothing(p3a_delta::command &msg)
{
	msg.turnLeft = false;
	msg.turnRight = false;
	msg.backUp = false;
	msg.forward = false;
}




