/*
Adept MobileRobots Advanced Robotics Navigation and Localization (ARNL)
Version 1.9.0

Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006-2009 MobileRobots Inc.
Copyright (C) 2010-2014 Adept Technology, Inc.

All Rights Reserved.

Adept MobileRobots does not make any representations about the
suitability of this software for any purpose.  It is provided "as is"
without express or implied warranty.

The license for this software is distributed as LICENSE.txt in the top
level directory.

robots@mobilerobots.com
Adept MobileRobots
10 Columbia Drive
Amherst, NH 03031
+1-603-881-7960

*/
/****************************************************************************
 * 
 * File: ArRobotPoseProb.h
 * 
 * Function: Header file for the robotposeprob.cpp file.
 *
 * Created:  George V. Paul. gvp@activmedia.com. December 6 2002.
 *
 *****************************************************************************/
#ifndef ARROBOTPOSEPROB_H
#define ARROBOTPOSEPROB_H

#include <stdio.h>
#include <math.h>
#include "Aria.h"
/* 
  @class ArRobotPoseProb.
  @internal
  @brief Specialized class holds pose and its probability in one place.
*/
class ArRobotPoseProb
{

public:
  /// Base Constructor.
  ArRobotPoseProb(void)
  {
    myPose[0]=0; myPose[1]=0; myPose[2]=0;
    myProb=0;
  }
  /// Constructor with the all 4 params.
  ArRobotPoseProb(double x, double y, double t, double p)
  {
    myPose[0]=x; myPose[1]=y; myPose[2]=t;
    myProb=p;
  }
  /// Constructor with the all 5 params.
  ArRobotPoseProb(double x, double y, double t, double p, double s)
  {
    myPose[0]=x; myPose[1]=y; myPose[2]=t;
    myProb=p; myScore=s;
  }
  /// Constructor with the all 4 params.
  ArRobotPoseProb(ArPose pose, double p)
  {
    myPose[0]=pose.getX(); myPose[1]=pose.getY(); myPose[2]=pose.getTh();
    myProb=p;
  }
  /// Base Destructor.
//  ~ArRobotPoseProb(void){}

  /// Get the X coords in mm.
  double   getX(void){return myPose[0];}
  /// Get the Y coords in mm.
  double   getY(void){return myPose[1];}
  /// Get the Theta coords in degs.
  double   getTh(void){return myPose[2];}
  /// Get the pose as a double pointer.
  ArPose   getPose(void){return ArPose(myPose[0], myPose[1],myPose[2]);}
  /// Get the probability of this pose.
  double   getProb(void){return myProb;}
  /// Get the score.
  double   getScore(void){return myScore;}
  /// Set the X coords in mm.
  void     setX(double x){myPose[0] = x;}
  /// Set the Y coords in mm.
  void     setY(double y){myPose[1] = y;}
  /// Set the Theta coords in degs.
  void     setTh(double th){myPose[2] = th;}
  /// Set the pose of this poseprob.
  void     setPose(double a, double b, double c)
  {
    myPose[0] = a; myPose[1] = b; myPose[2] = c;
  }
  /// Set the pose of this poseprob.
  void     setPose(ArPose p)
  {
    myPose[0] = p.getX(); myPose[1] = p.getY(); myPose[2] = p.getTh();
  }
  /// Set the probablity of this poseprob.
  void     setProb(double a){myProb = a;}
  /// Set the score.
  void     setScore(double a){myScore = a;}
  /// Copy Constructor
  ArRobotPoseProb(const ArRobotPoseProb &pp)
  {
    myPose[0] = pp.myPose[0]; 
    myPose[1] = pp.myPose[1];
    myPose[2] = pp.myPose[2];
    myProb = pp.myProb; 
    myScore = pp.myScore;
  }

private:
  double myPose[3];
  double myProb;
  double myScore;
};

#endif // ARROBOTPOSEPROB_H
