
#define OCC_THRESHOLD .15f
#define TRIP_THRESHOLD .3f
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

void checkSensorChange(const serializer::SensorState& msg);
void doNothing(p3a_delta::command &msg);


float getValue();


int main(int argc, char **argv)
{
  ros::init(argc, argv, "p3_delta_ai");
  ros::NodeHandle n;

  ros::Publisher commandPub = n.advertise<p3a_delta::command>("command", 10);

  p3a_delta::command outBound;
  
  bool escape = false;

ros::Rate loop_rate(10);

  //the state messenger doesn't need the same resolution as the cmd::velocity channel.  
//Thus, we only publish 5 times a second.
  ros::Publisher command_pub = n.advertise<geometry_msgs::Twist>("FState", 5);

ros::topic::waitForMessage<nav_msgs::Odometry>(std::string("odom"), n,ros::Duration(30));
    geometry_msgs::Twist msg;

	ros::Subscriber subz = n.subscribe("p3a_delta/zoneCount", 10, checkSensorChange);

   //we listen a little faster than we publish, since we don't know when it will change
    ros::Subscriber subc = n.subscribe("serializer/sensors", 10, checkSensorChange);
	//['right_ir', 'left_ir', 'voltage', 'touch_1', 'touch_2'] 

	
   
//begin your methodology
ROS_INFO("STARTING LOOP");

//how many zones are we talkin'
int zoneCount;// = zones.zone.size();
int zonePointSize;// = zones.pointNum / zoneCount;
float zoneAngleSize;// = zonePointSize * zones.angleIncrement;
vector<zoneStates> states;
//states.resize(zoneCount);

//how many zones open do we need to go forward?
float lsquared;// = zones.minScanDist * zones.minScanDist;
float angleMinOpen;// = acos((robotWidth*robotWidth - 2 * lsquared)/(2 * lsquared));
int zoneMinOpen;// = angleMinOpen / (zonePointSize * zones.angleIncrement);
//if (zoneMinOpen == 0)
//	zoneMinOpen = 1;
	
float totalRange = zones.pointNum * zones.angleIncrement;

int count = 0;
bool fromRight = true;
bool forwardOpen = true;
int left = 0;
int right = 0;
bool forwardRight = true;
bool forwardLeft = true;
int k = 0;
	


while(ros::ok())
{
	ROS_INFO("GOT INTO LOOP %d", zoneCount);
	zoneCount = zones.zone.size();
	
	if (zoneCount == 0)
	{
		ros::spinOnce();
		loop_rate.sleep();
		continue;
	}
	ROS_INFO("ESCAPED DEADLOCK");
		
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
		if( (zones.zone[i]/(float(zonePointSize))) >= TRIP_THRESHOLD)
		{
			states[i] = TRIPPED;
		}
		else if( (zones.zone[i]/(float(zonePointSize))) >= OCC_THRESHOLD)
		{
			states[i] = OCCUPIED;
		}
		else
		{
			states[i] = FREE;
		}
	}
	
	//rotate from the left, take the last free zone detected
	count = 0;
	fromRight = true;
	for (int i = zoneCount-1; i >= 0; i--)
	{
		if(states[i] == FREE)
		{
			count++;
		}
		else
			break;
	}
	
	//the immediate right is blocked
	if ( count == 0)
	{
		ROS_INFO("LEFT BLOCKED");
		fromRight = false;
		count = 0;
		//search from right
		for (int i = 0; i < zoneCount; i++)
		{
			if(states[i] == FREE)
			{
				count++;
			}
			else
				break;
		}
		//shit, the right is blocked
		if(count == 0)
		{
			ROS_INFO("RIGHT BLOCKED");
			//forward maybe?
			forwardRight = true;
			forwardLeft = true;
			k = 0;
			while ((forwardLeft || forwardRight) && (k < (zoneCount/2)))
			{
				//check left
				if(states[zoneCount/2 -k] == FREE)
				{
					left++;
				}
				else
				{
					forwardLeft = false;
				}
				
				//check right
				if(states[zoneCount/2 +k] == FREE)
				{
					right++;
				}
				else
				{
					forwardRight = false;
				}
				k++;
			}
			
			//if the two sides are big enough to pass, consider going forward
			if((left + right) >= zoneMinOpen)
			{
				//see if the opening is centered
				//it is not.
				if( (left-right) > OFFCENTER	)
				{
					if (left > right)
					{
						outBound.turnLeft = true;
					}
					else
					{
						outBound.turnRight = true;
					}
				}
				else //it is centered, and big enough
				{
					outBound.forward = true;
				}
				
			}
			//it simply cannot go forward
			else
			{
				ROS_INFO("FORWARD BLOCKED");
				outBound.backUp = true;
			}
		}//end of forward check
	}//end of right check
		
	//one of the sides was checkable
	if (count > 0)
	{
		
		if(fromRight)
		{
			ROS_INFO("GOING FROM RIGHT");
			if(zoneAngleSize * count < (totalRange/6 *2))
			{
				//go forward
				outBound.forward = true;
			}
			else 
			{
				outBound.turnRight = true;
			}
		}
		else if(!fromRight)
		{
			ROS_INFO("GOING FROM LEFT");
			if(zoneAngleSize * count > (totalRange/6 * 4))
			{
				//go forward
				outBound.forward = true;
			}
			else 
			{
				outBound.turnLeft = true;
			}
		}
	}
	//checkable side
		
	commandPub.publish(outBound);

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




