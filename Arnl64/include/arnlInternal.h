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
#ifndef ARNLINTERNAL_H
#define ARNLINTERNAL_H

/// A class to hold a multirobot pose and radius
class ArMultiRobotPoseAndRadius
{
public:
  /// Constructor
  ArMultiRobotPoseAndRadius(ArPose pose, double radius)
    { myPose = pose; myRadius = radius; }
  /// Empty constructor
  ArMultiRobotPoseAndRadius() { myRadius = 0; }
  /// Destructor
  virtual ~ArMultiRobotPoseAndRadius() {}
  /// Gets the x position of the other robot
  double getX(void) { return myPose.getX(); }
  /// Gets the y position of the other robot
  double getY(void) { return myPose.getY(); }
  /// Gets the th of the other robot (probably always 0)
  double getTh(void) { return myPose.getTh(); }
  /// Gets the pose of the other robot
  ArPose getPose(void) { return myPose; }
  /// Gets the radius of the real robot
  double getRadius(void) { return myRadius; }

protected:
  ArPose myPose;
  double myRadius;
};


/// Class for global initialization of ARNL and ARNL directories
class Arnl
{
public:
  /// Sets up the Aria base directory to be the base directory of ARNL
  AREXPORT static void init(void);
  /// Gets the typical param file name (params/arnl.p or params/sonarnl.p)
  AREXPORT static const char *getTypicalParamFileName(void);
  /// Gets the typical param file name (params/default-arnl.p or params/default-sonarnl.p)
  AREXPORT static const char *getTypicalDefaultParamFileName(void);
protected:
  static const char *ourTypicalParamFileName;
  static const char *ourTypicalDefaultParamFileName;
};

#endif
