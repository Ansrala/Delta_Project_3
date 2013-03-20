//sensor node
//interprets data from sensors
//posts them to "worldinfo" topic

#define ANGLE_RES 7
#define OB_SIZE 7
#define THRESHOLD 1.0f
#define MIN_POINT_COUNT 20
#define PI 3.14592f

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <stdlib.h>
#include <cmath>
#include <iostream>
#include <vector>

//our custom messages
#include <p3a_delta/zoneCount.h>


using namespace std;

sensor_msgs::LaserScan now;

void loadLaser(const sensor_msgs::LaserScan& msg);





int main(int argc, char **argv)
{
	ros::init(argc, argv, "p3_delta_sensors");
	ros::NodeHandle nh;

	ros::Rate loop_rate(10); 
	p3a_delta::zoneCount  outBound;
  
	ros::Subscriber info_get = nh.subscribe("scan", 500, loadLaser);
	ros::Publisher info_pub1 = nh.advertise<p3a_delta::zoneCount>("zones", 50);
	
	int zoneSize = now.ranges.size() / 3;
	int currentZone = 0;
	int counts[3] = {0,0,0};
	

	while(ros::ok())
	{
		//parse points
		for (int i = 0; i < now.ranges.size(); i++)
		{
			if ( i < (currentZone +1) * zoneSize)
			{
				currentZone = 0;
			}
			if ( i < (currentZone + 2) * zoneSize && i > (currentZone +1) * zoneSize )
			{
				currentZone = 1;
			}
			if (i > (currentZone + 2) * zoneSize)
			{
				currentZone = 3;
			}
			
			//is it in an acceptable range?
			if ( !(now.ranges[i] >.6f && now.ranges[i] < 3))
			{
				counts[currentZone] ++;
			}
			
		}
			
			
		outBound.zone1 = counts[0];
		outBound.zone2 = counts[1];
		outBound.zone3 = counts[2];	

		
		info_pub1.publish(outBound);


		ros::spinOnce();
		loop_rate.sleep();

	}



	//end your methadology


  return 0;
}




//todo:: make info gathering function
void loadLaser(const sensor_msgs::LaserScan& msg)
{
	//make a deep copy
	//these are in radians
	//ROS_INFO("Sensors updated: point count = %d", msg.ranges.size());
	now.angle_min = msg.angle_min;
	now.angle_max = msg.angle_max;
	now.angle_increment = msg.angle_increment;

	//time info
	now.time_increment = msg.time_increment;
	now.scan_time = msg.scan_time;	

	now.ranges.erase(now.ranges.begin(), now.ranges.end()); 
	for(int i = 0; i < msg.ranges.size(); i++)
	{
		now.ranges.push_back(msg.ranges[i]);
	}

}




