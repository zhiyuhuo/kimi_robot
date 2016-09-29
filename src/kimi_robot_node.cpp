#include "ros/ros.h"
#include "stdio.h"
#include "stdlib.h"
#include <cstdlib>
#include <vector>
#include <math.h>
#include <time.h>
#include <sys/timeb.h>
#include <unistd.h>
#include <pthread.h>
#include <iostream>
#include <fstream>
#include <dirent.h>

#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

#include "kimi_robot/GeometryR/geometryR_header.h"
#include "kimi_robot/Perception/perception.h"

double _x = 0;
double _y = 0;
double _th = 0;

double _xp = 0;
double _yp = 0;
double _thp = 0;
ros::Time _lastframetime;

double _vx = 0;
double _vy = 0;
double _vth = 0;

std::vector<VecPosition> _laserframe;
CPerception _perception;

bool _ifgetvel = false;
bool _ifgetlaser = false;

void TwistCallback(const geometry_msgs::Twist::ConstPtr& msg);
void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
void PredictRobotPose();
void UpdateRobotPose();
void UpdateRobotPose();

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kimi_nav_node");
  ros::NodeHandle n;
  _lastframetime = ros::Time::now();
  ros::Subscriber velsub = n.subscribe("cmd_vel", 1000, TwistCallback);
  ros::Subscriber laserSub = n.subscribe("base_scan", 1000, LaserCallback);
  
  while (! _ifgetlaser) {	ros::spinOnce();	}
  
  ros::Rate looprate(10);
  while (ros::ok()) {
    
    _perception.ReadLaserFrame(_laserframe);
    
    
    std::vector<CCorner> corners = _perception.ExtractCornerFromLaserPoints();
    
    std::cout << "show corners:__________\n";
//     for (int i = 0; i < corners.size(); i++) {
//       std::cout << corners[i].m_pos.GetX() << ",  "
// 		<< corners[i].m_pos.GetY() << ",  "
// 		<< corners[i].m_angle << ",  "
// 		<< corners[i].m_normal << std::endl;
//     }
    
    
    //UpdateRobotPose();
    ros::spinOnce();
    looprate.sleep();
  }
  return 0;
}

void TwistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{ 
  PredictRobotPose();
  _vx = msg->linear.x;
  _vy = msg->linear.y;
  _vth = msg->angular.z;
  //std::cout << "vx: " << _vx << "	vy: " << _vy << "	vth: " << _vth << std::endl;
  
  _ifgetvel = true;
}

void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  _laserframe.clear();
  std::vector<float> ranges = msg->ranges;
  int N = ranges.size();
  float minAngle = msg->angle_min;
  float angleIncrement = msg->angle_increment;
  for (int i = 0; i < N; i++) {
    VecPosition p = VecPosition::GetVecPositionFromPolar(ranges[i], minAngle + angleIncrement * i);
    //std::cout << i << " : " << p.GetX() << " " << p.GetY() << std::endl;
    _laserframe.push_back(p);
  }
  
  _ifgetlaser = true;
}

void PredictRobotPose()
{   
  double dt = (ros::Time::now() - _lastframetime).toSec();
  double dx = (_vx * cos(_thp) - _vy * sin(_thp)) * dt;
  double dy = (_vx * sin(_thp) + _vy * cos(_thp)) * dt;
  double dth = _vth * dt;    
    
  //std::cout << "dt: " << dt << "	dx: " << dx << "	dy: " << dy << "	dth: " << dth << std::endl;
   
  _xp += dx;
  _yp += dy;
  _thp += dth;
    
  _lastframetime = ros::Time::now();
    
  //std::cout << "xp: " << _xp << "	yp: " << _yp << "	thp: " << _thp << std::endl;
}

void UpdateRobotPose()
{
  
}

void LaserLocalize()
{
  
}







