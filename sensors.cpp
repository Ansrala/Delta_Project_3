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

void checkStateChange(const std_msgs::String& msg);

void cleanLinesOut(p2_delta::lineList &linesOut, p2_delta::pointList &pointsOut);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "p2_delta_sensors");
  ros::NodeHandle nh;
  //ros::NodeHandle s;
  ros::NodeHandle talk1;
  ros::NodeHandle talk2;
  ros::NodeHandle sense;
 // ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  ros::Rate loop_rate(10);

//test
ros::Publisher pub = nh.advertise<std_msgs::String>("topic_name", 5);
std_msgs::String str;
str.data = "hello world";
pub.publish(str);



//ros::topic::waitForMessage<nav_msgs::Odometry>(string("odom"), n,ros::Duration(30));
    geometry_msgs::Twist msg;

  
   ros::Subscriber info_get = sense.subscribe("scan", 500, loadLaser);
   ros::Publisher info_pub1 = talk1.advertise<p2_delta::pointList>("worldinfoPoints", 50);
   ros::Publisher info_pub2 = talk2.advertise<p2_delta::lineList>("worldinfoLines", 50);

   
   

ROS_INFO("lines out size: %d",linesOut.x1.size());
info_pub2.publish(linesOut);
info_pub1.publish(pointsOut);


//ROS_INFO("ENDING LOOP Count: %d", testCount);
//testCount++;
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

void cleanLinesOut(p2_delta::lineList &linesOut, p2_delta::pointList &pointsOut)
{
	linesOut.x1.erase(linesOut.x1.begin(), linesOut.x1.end());
	linesOut.y1.erase(linesOut.y1.begin(), linesOut.y1.end());
	linesOut.x2.erase(linesOut.x2.begin(), linesOut.x2.end());
	linesOut.y2.erase(linesOut.y2.begin(), linesOut.y2.end());

	pointsOut.x.erase(pointsOut.x.begin(), pointsOut.x.end());
	pointsOut.y.erase(pointsOut.y.begin(), pointsOut.y.end());
}


