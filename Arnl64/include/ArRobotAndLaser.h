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
/*****************************************************************************
 * 
 * File: ArRobotAndLaser.h
 * 
 * Function: Header file for the robotandlaser.cpp file.
 *
 * Created:  George V. Paul. gvp@activmedia.com. December 3 2002.
 *
 *****************************************************************************/
#ifndef ARROBOTANDLASER_H
#define ARROBOTANDLASER_H

#include <stdio.h>
#include <math.h>
#include "Aria.h"

/* 
  @class ArRobotAndLaser.
  @internal This class is used internally by Arnl
  @brief Class holds details about the robot and laser pertaining to
         localization.
*/
class ArRobotAndLaser
{

public:

  /// Base Constructor.
  ArRobotAndLaser(ArRobot* robot, ArRangeDevice* laser) : 
  myRobot(robot), myLaser(laser) {}
  /// Base Destructor.
  ~ArRobotAndLaser(void) {}

  /// Convert the laser scan range array into global coords arrays.
  bool     scanToGlobalCoords(ArPose rp, std::vector<ArPose>& xyl, 
			      double increment);
  /// Return the pointer to the range data from the current scan.
  std::vector<int> scanData(void) {return myScanData;}
  /// Return the pointer to the X data from the current scan.
  std::vector<ArPose> scanXY(void) {return myScanXY;}
  /// Store the robot's Pose at which the current scan was taken.
  void     setPoseTaken(ArPose a) {myPoseTaken = a;}
  /// Store the robot's Pose at which the current scan was taken.
  void     setEncoderPoseTaken(ArPose a) {myEncoderPoseTaken = a;}
  /// Store the robot's time at which the current scan was taken.
  void     setTimeTaken(ArTime a) {myTimeTaken = a;}
  /// Get the robot's Pose at which the current scan was taken.  
  ArPose   getPoseTaken(void) {return myPoseTaken;}
  /// Get the robot's Pose at which the current scan was taken.  
  ArPose   getEncoderPoseTaken(void) {return myEncoderPoseTaken;}
  /// Get the robot's time at which the current scan was taken.
  ArTime    getTimeTaken(void) {return myTimeTaken;}
  /// Store the current range data in the laser buffer into data array.
  bool     scanLaserIntoArray(ArRangeDevice* l, int refThres = 31);
  /// Convert reflections to global coordinates.
  bool     scanReflectionsToGlobalCoords(ArPose rp, 
					 std::vector<ArPose>& gPoses);
  /// Get the list of reflections.
  std::vector<ArPose> getReflections(void) {return myReflection;}
  /// Gets the number of scan points.
  int      getNumOfScanPoints(void){ return myScanXY.size();}
  /// Gets the number of reflections.
  int      getNumOfReflections(void){ return myReflection.size();}
  /// Finds the centers of each reflection from the set of reflections.
  std::vector<ArPose> getReflectionCenters(double refSize);
  /// Gets the angle increment.
  double   getAngleIncrement(void){ return myScanIncrement;}

private:
  ArRobot*            myRobot;
  ArRangeDevice*      myLaser;
  double              myScanIncrement;
  std::vector<int>    myScanData;
  std::vector<ArPose> myScanXY;
  std::vector<ArPose> myReflection;
  ArPose              myPoseTaken;
  ArPose              myEncoderPoseTaken;
  ArTime              myTimeTaken;
};

#endif // ARROBOTANDLASER.H
