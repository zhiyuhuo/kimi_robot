#ifndef PERCEPTION_PERCEPTION_H_
#define PERCEPTION_PERCEPTION_H_

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

#include "../GeometryR/geometryR_header.h"

class CCorner {
public:
  VecPosition m_pos;
  double m_angle;
  double m_normal;

public:
  CCorner(VecPosition pos, double angle, double normal){
    m_pos = pos;
    m_angle = angle;
    m_normal = normal;
  }
};

class CWall {
  
};

class CPerception {
  
private:
  std::vector<VecPosition> m_laserframe;
  
public:
  int ReadLaserFrame(std::vector<VecPosition> laserframe) 
  {
    m_laserframe.clear();
    m_laserframe = laserframe;
    return m_laserframe.size();
  }
  
  std::vector<CCorner> ExtractCornerFromLaserPoints()
  {
    std::vector<VecPosition> laserframe = this->m_laserframe;
    
    std::vector<CCorner> res;
    std::vector<VecPosition> segset;
    
    for (int i = 5; i < laserframe.size(); i ++) {
      VecPosition si = laserframe[i] - laserframe[i-5];
      if (si.GetMagnitude() < 0.5) {
	for (int j = i+1; j < laserframe.size(); j++) {
	  VecPosition sj = laserframe[j] - laserframe[j-5];
	  
	  if (sj.GetMagnitude() > 0.5)
	    break;
	  
	  double th = fabs(VecPosition::NormalizeAngle(si.GetDirection() - sj.GetDirection()));
	  if (th > PI * 0.45) {
// 	    std::cout << i << ": " << si.GetX() << " " << si.GetY() << ", " 
// 	    	      << j << ": " << sj.GetX() << " " << sj.GetY() << ", "<< th << std::endl;
	    
	    double mindth = PI;
	    int mindthid = -1;
	    VecPosition sl;
	    VecPosition sr;
	    for (int k = i-4; k < j-1; k++) {
	      sl = laserframe[k] - laserframe[i-5];
	      sr = laserframe[j] - laserframe[k];
	      double th2 = fabs(VecPosition::NormalizeAngle(sr.GetDirection() - sl.GetDirection()));
	      if (fabs(th2 - th) < mindth) {
		mindth = fabs(th2 - th);
		mindthid = k;
	      }
	    }
	    
	    std::cout << mindthid << " " << mindth << std::endl;
	    if (mindthid < 0 || mindth > 0.1)
	      break;
	    
	    double normalvalue = VecPosition::NormalizeAngle2PI((sr.GetDirection() + s.GetDirection())/2);
	    
	    std::cout << mindthid << std::endl;
	    CCorner cnr(laserframe[mindthid], th, normalvalue);
	    std::cout << cnr.m_pos.GetX() << ",  "
		<< cnr.m_pos.GetY() << ",  "
		<< cnr.m_angle << ",  "
		<< cnr.m_normal << std::endl;
	    res.push_back(cnr);
	    i = j;
	    break;
	  }
	}
      }
      
    }
    
    
    return res;
  }
  
};

#endif