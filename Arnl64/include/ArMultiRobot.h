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
#ifndef ARMULTIROBOT_H
#define ARMULTIROBOT_H

#include "Aria.h"
#include "ArNetworking.h"
#include "arnlInternal.h"
#include "ArPathPlanningTask.h"
#include "ArBaseLocalizationTask.h"
#include "ArMultiRobotFlags.h"

///  handles the server side of dispensing information for multi robot driving
/**
   This class handles the server side of dispensing information to the
   central server.  It has no callback for the packet because it
   broadcasts it in each user task set
**/
class ArServerHandlerMultiRobot
{
public:
  AREXPORT ArServerHandlerMultiRobot(ArServerBase *server, ArRobot *robot,
				     ArPathPlanningTask *pathTask,
				     ArBaseLocalizationTask *locTask,
				     ArMapInterface *arMap);
  AREXPORT ~ArServerHandlerMultiRobot();
  /// gets the information about the robot's radius and path radius
  AREXPORT void multiRobotInfo(ArServerClient *client, ArNetPacket *packet);
  /// Gets the robot's precedence class
  AREXPORT int getPrecedenceClass(void);
  /// Sets the robot's precedence class
  AREXPORT void setPrecedenceClass(int precedenceClass = 0);
  /// Override map name (if you set this it'll use this instead of the real map name)
  AREXPORT void overrideMapName(const char *overrideMapName);
  /// Gets the override map name 
  AREXPORT const char *getOverrideMapName(void)
    { return myOverrideMapName.c_str(); }
  /// Gets the callback that gets the override map name
  AREXPORT ArRetFunctor<const char *> *getOverrideMapNameCB(void)
    { return &myGetOverrideMapNameCB; }
  /// processes our config and updates everyone that it changed
  AREXPORT bool processFile(char *errorBuffer, size_t errorBufferLen);
  /// Supress multirobot path sending this cycle
  AREXPORT void supressPathSendingThisCycle(void);

  /// Sets the overrides we use for our dimensions, 0 means use default
  AREXPORT void setOverrideValues(
	  int overrideWidthLeft, int overrideWidthRight,
	  int overrideLengthFront, int overrideLengthRear,
	  int overridePathHalfWidth);

  int getActualRadius(void) { return myActualRadius; }

  int getUsingWidthLeft(void) { return myUsingWidthLeft; }
  int getUsingWidthRight(void) { return myUsingWidthRight; }
  int getUsingLengthFront(void) { return myUsingLengthFront; }
  int getUsingLengthRear(void) { return myUsingLengthRear; }
  int getUsingPathHalfWidth(void) { return myUsingPathHalfWidth; } 

  int getDefaultWidthLeft(void) { return myDefaultWidthLeft; }
  int getDefaultWidthRight(void) { return myDefaultWidthRight; }
  int getDefaultLengthFront(void) { return myDefaultLengthFront; }
  int getDefaultLengthRear(void) { return myDefaultLengthRear; }
  int getDefaultPathHalfWidth(void) { return myDefaultPathHalfWidth; } 

protected:
  /// broadcasts the informationabout the position and path
  AREXPORT void userTask(void);
  /// Called when we get a new goal
  AREXPORT void newGoal(void);

  ArMutex myMutex;
  ArServerBase *myServer;
  ArRobot *myRobot;
  ArPathPlanningTask *myPathTask;
  ArBaseLocalizationTask *myLocTask;
  ArMapInterface *myMap;
  int myPrecedenceClass;
  std::string myOverrideMapName;

  bool mySupressPathSendingThisCycle;
  ArNetPacket myMultiRobotInfoPacket;

  int myActualRadius;
  int myActualWidthLeft;
  int myActualWidthRight;
  int myActualLengthFront;
  int myActualLengthRear;

  int myConfigRadiusAdjustment;
  int myConfigWidthLeftAdjustment;
  int myConfigWidthRightAdjustment;
  int myConfigLengthFrontAdjustment;
  int myConfigLengthRearAdjustment;
  int myConfigPathHalfWidthAdjustment;

  int myDefaultWidthLeft;
  int myDefaultWidthRight;
  int myDefaultLengthFront;
  int myDefaultLengthRear;
  int myDefaultPathHalfWidth;

  int myUsingWidthLeft;
  int myUsingWidthRight;
  int myUsingLengthFront;
  int myUsingLengthRear;
  int myUsingPathHalfWidth;

  int myOverrideWidthLeft;
  int myOverrideWidthRight;
  int myOverrideLengthFront;
  int myOverrideLengthRear;
  int myOverridePathHalfWidth;

  int myPathMaxLength;
  int myPathResolution;
  int myShortPathLength;

  unsigned char myPathNum;
  unsigned char myMultiRobotCapabilityFlags1;

  std::string myMultiRobotToServerCommand;  
  std::string myMultiRobotInfoCommand;

  ArFunctor2C<ArServerHandlerMultiRobot, 
	      ArServerClient *, ArNetPacket *> myMultiRobotInfoCB;
  ArRetFunctor2C<bool, ArServerHandlerMultiRobot, 
                            char *, size_t> myProcessFileCB;
  ArFunctorC<ArServerHandlerMultiRobot> myUserTaskCB;
  ArFunctorC<ArServerHandlerMultiRobot> myNewGoalCB;
  ArRetFunctorC<const char *, 
	       ArServerHandlerMultiRobot> myGetOverrideMapNameCB;
};

/// This is the class that will make range data from multiple robots
class ArMultiRobotRangeDevice : public ArRangeDevice
{
public:
  /// Constructor 
  AREXPORT ArMultiRobotRangeDevice(ArServerBase *serverBase);
  /// Destructor
  AREXPORT ~ArMultiRobotRangeDevice();
  /** Override ArRangeDevice::applyTransform() to skip transform from local to global coordinates (already in global coords.)
      @internal
  */
  virtual void applyTransform(ArTransform trans, bool doCumulative) 
	{}  
protected:
  /// Process our robot poses packet
  void netRobotPoses(ArServerClient *client, ArNetPacket *packet);
  /// Process our robot paths packet
  void netRobotPaths(ArServerClient *client, ArNetPacket *packet);
  /// Gets the other robot poses and radii in a threadsafe manner
public:
  AREXPORT std::list<ArMultiRobotPoseAndRadius> getOtherRobots(void);
  /// Gets the callback for other robots
  AREXPORT ArRetFunctor<std::list<ArMultiRobotPoseAndRadius> >*getOtherRobotsCB(void) { return &myOtherRobotsCB; }
protected:
  ArServerBase *myServer;
  
  std::list<ArMultiRobotPoseAndRadius> myOtherRobots;
  ArFunctor2C<ArMultiRobotRangeDevice, ArServerClient *, 
	      ArNetPacket *> myNetRobotPosesCB;
  ArFunctor2C<ArMultiRobotRangeDevice, ArServerClient *, 
	      ArNetPacket *> myNetRobotPathsCB;
  ArRetFunctorC<std::list<ArMultiRobotPoseAndRadius>, ArMultiRobotRangeDevice> myOtherRobotsCB;
};

#endif 
