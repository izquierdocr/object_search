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
#ifndef ARCENTRALMULTIROBOT_H
#define ARCENTRALMULTIROBOT_H

#include "Aria.h"
#include "ArNetworking.h"

#include "ArMultiRobotFlags.h"

/**
   Class that gathers information from multiple robots and helps them
   avoid each other.  Use this class in a central multi-robot server
   program.
**/
class ArCentralMultiRobot : public ArASyncTask
{
public:
  /// Constructor
  AREXPORT ArCentralMultiRobot(ArCentralManager *manager, ArConfig *config,
			       ArServerInfoDrawings *drawings,
			       ArServerHandlerCommands *handlerCommands = NULL,
			       ArServerHandlerPopup *popupHandler = NULL);
  /// Destructor
  AREXPORT virtual ~ArCentralMultiRobot();
  /// Called when a new forwarder is added
  AREXPORT void forwarderAdded(ArCentralForwarder *forwarder);
  /// Called when a new forwarder is added
  AREXPORT void forwarderRemoved(ArCentralForwarder *forwarder);
  /// Runs through all the clients and sends 'em the data (in the new way)
  AREXPORT void loopOnce(void);
  /// Gets data used for visualizing all the paths of the robots
  virtual ArDrawingData *getGetPathsDrawingData(void) 
    { return myGetPathsDrawingData; }
  AREXPORT virtual void setGetPathsDrawingData(ArDrawingData *data, 
						 bool takeOwnershipOfData);
  /// Sends the packet for all of the robot paths
  AREXPORT void netGetPaths(ArServerClient *client, ArNetPacket *packet);
  /// Gets data used for visualizing all the paths of the robots
  virtual ArDrawingData *getGetStraightPathsDrawingData(void) 
    { return myGetStraightPathsDrawingData; }
  AREXPORT virtual void setGetStraightPathsDrawingData(ArDrawingData *data, 
						 bool takeOwnershipOfData);
  /// Sends the packet for all of the robot paths
  AREXPORT void netGetStraightPaths(ArServerClient *client, ArNetPacket *packet);
  /// Sends the packet for all of the robot positions
  AREXPORT virtual void *runThread(void *arg);
public:
  void multiRobotToServer(ArNetPacket *packet, ArCentralForwarder *forwarder);
  void multiRobotInfo(ArNetPacket *packet, ArCentralForwarder *forwarder);
  void popupMultiRobotInfo(void);

  ArCentralManager *myManager;
  ArConfig *myConfig;
  ArServerInfoDrawings *myDrawings;
  ArServerHandlerCommands *myHandlerCommands;
  ArServerHandlerPopup *myPopupHandler;

  ArDrawingData *myGetPathsDrawingData;
  bool myOwnGetPathsDrawingData;
  ArDrawingData *myGetStraightPathsDrawingData;
  bool myOwnGetStraightPathsDrawingData;

  ArMutex myDataMutex;

  /**
     This class holds the information from the client robot and also
     holds the pointers to the functors to delete when this item is
     deleted.
  **/
  class Robot
  {
  public:
    /// Constructor
    /**
       @param robotName the robot's unique name
       @param precedenceBase the robot's precedence level
       @param multiRobotToServerCB the functor the ArCentralMultiRobot
       will use to parse the data, its just here so that it can all be
       cleaned up at once
    **/
    Robot(const char *robotName, unsigned int precedenceBase, 
	 ArFunctor2<ArNetPacket *, ArCentralForwarder *> *multiRobotToServerCB,
	  ArFunctor2<ArNetPacket *, ArCentralForwarder *> *multiRobotInfoCB)
      { 
	myRobotName = robotName; 
	myPrecedenceClass = 0;
	myPrecedenceBase = precedenceBase; 
	myPrecedence = myPrecedenceClass * 1000000 | myPrecedenceBase;
	myDataGood = false;
        myDataReceived = false; 
	myMultiRobotToServerCB = multiRobotToServerCB;
	myMultiRobotInfoCB = multiRobotInfoCB;
	myHaveThAndVel = false;
	myPathNumber = 0;
      }
    /// Destructor
    virtual ~Robot() 
      { delete myMultiRobotToServerCB; delete myMultiRobotInfoCB; } 
    /// Gets the robot's name
    const char *getRobotName(void) { return myRobotName.c_str(); }
    /// Gets the robot's precedence
    long getPrecedence(void) { return myPrecedence; }
    /// Gets the robot's precedence class
    int getPrecedenceClass(void) { return myPrecedenceClass; }
    /// Gets the robot's precedence base
    long getPrecedenceBase(void) { return myPrecedenceBase; }
    /// Checks all the things and just tells us if we want to use this
    /// robot or not
    bool isDataGood(void) 
      { return 
	(myDataGood && myDataReceived && myLastReceived.secSince() < 20); }
    /// Checks if we have th and vel
    bool haveThAndVel(void) { return true; } 
    /// Gets our current pose
    ArPose getPose(void) { return myPose; } 
    /// Gets our x 
    double getX(void) { return myPose.getX(); } 
    /// Gets our y
    double getY(void) { return myPose.getY(); } 
    /// Gets our th 
    double getTh(void) { return myPose.getTh(); } 
    /// Gets our map
    const char *getMapName(void) { return myMapName; }
    /// Gets the pointer to our path
    std::list<ArPose> *getPath(void) { return &myPath; }
    /// Gets the number of points in the short path
    int getShortPathNumPoints(void) { return myShortPathNumPoints; }
    /// Gets out velocity
    int getVel(void) { return myVel; }

    /// Gets the robot's actual radius
    int getActualRadius(void) { return myActualRadius; }
    /// Gets the robot's current left width
    int getWidthLeft(void) { return myWidthLeft; }
    /// Gets the robot's current right width
    int getWidthRight(void) { return myWidthRight; }
    /// Gets the robot's current length front
    int getLengthFront(void) { return myLengthFront; }
    /// Gets the robot's current length rear
    int getLengthRear(void) { return myLengthRear; }
    /// Gets the robot's current path half width
    int getPathHalfWidth(void) { return myPathHalfWidth; }

    /// Gets the flags
    unsigned char getFlags1(void) { return myFlags1; }
    /// Gets the capability flags
    unsigned char getCapabilityFlags1(void) { return myCapabilityFlags1; }
    /// Gets when we started waiting to fail
    ArTime getStartedWaitingToFail(void) { return myStartedWaitingToFail; }
    /// Gets the pointer to the straight part of our path
    std::list<ArPose> *getStraightPathPoints(void) 
      { return &myStraightPathPoints; }
    /// Gets the possibly extended straight path line segment 
    ArLineSegment getStraightPathLine(void) { return myStraightPathLine; }
    /// Gets the pointer to the non straight part of our path
    std::list<ArPose> *getNonStraightPathPoints(void) 
      { return &myNonStraightPathPoints; }
    /// sees if we have a new path
    bool hasNewPath(void) 
      {
	if (myOldPathNumber != myPathNumber) 
	  return true;
	else
	  return false;
      }
    /// Has this robot avoid the other robot
    void avoidRobot(Robot *otherRobot, bool logMore);
    /// Sees if this robot is avoiding the other robot
    bool isAvoidingRobot(Robot *otherRobot);
    /// Makes this robot no longer avoid the other robot
    void clearRobot(Robot *otherRobot);
    /// Gets the avoiding robot
    std::set<Robot *> *getAvoidingRobots(void);
    /// Sets the data from the frequent packet
    void setData(ArNetPacket *packet, int straightPathPointWidth,
		 int straightPathLengthAddition, int pathCheckPadding);
    /// sets the data from the occasional
    void setInfo(ArNetPacket *packet);
  protected:
    std::string myRobotName;
    ArPose myPose;
    std::list<ArPose> myPath;

    // the precedence is made up of the precedence class * the precedence base
    long myPrecedence;
    int myPrecedenceClass;
    long myPrecedenceBase;
    bool myDataReceived;
    bool myDataGood;
    
    int myActualRadius;
    int myWidthLeft;
    int myWidthRight;
    int myLengthFront;
    int myLengthRear;
    int myPathHalfWidth;

    char myMapName[1024];
    int myShortPathNumPoints;

    bool myHaveThAndVel;
    int myVel;
    unsigned char myPathNumber;
    unsigned char myOldPathNumber;

    unsigned char myCapabilityFlags1;
    unsigned char myFlags1;

    ArTime myStartedWaitingToFail;

    int myNumStraightPoints;
    std::list<ArPose> myStraightPathPoints;
    std::list<ArPose> myNonStraightPathPoints;
    ArPose myLastStraightPoint;
    ArLineSegment myStraightPathLine;

    std::set<Robot *> myAvoidingRobots;

    ArTime myLastReceived;
    ArFunctor2<ArNetPacket *, ArCentralForwarder *> *myMultiRobotToServerCB;
    ArFunctor2<ArNetPacket *, ArCentralForwarder *> *myMultiRobotInfoCB;
  };

  std::map<ArCentralForwarder *, Robot *> myClients;

  int myIgnorePathDist;
  bool myIgnoreDistIgnoresRobotsToo;
  int myShortPathMinDist;

  int myStraightPathPointWidth;
  int myStraightPathLengthAddition;
  int myRobotSameDirectionWidth;
  int myStoppedSpeed;
  int myMovingSpeed;
  int myPathCheckPadding;
  bool myLogMore;

  std::string myMultiRobotToServerCommand;  
  std::string myMultiRobotInfoCommand;
  std::string myMultiRobotPosesFromServerCommand;  
  std::string myMultiRobotPathsFromServerCommand;


  //std::map<ArCentralForwarder *, ArClientHandlerConfig *> myClientConfigs;

  ArFunctor1C<ArCentralMultiRobot, ArCentralForwarder *> myForwarderAddedCB;
  ArFunctor1C<ArCentralMultiRobot, ArCentralForwarder *> myForwarderRemovedCB;
  ArFunctor2C<ArCentralMultiRobot, ArServerClient *, 
	      ArNetPacket *> myNetGetPathsCB;
  ArFunctor2C<ArCentralMultiRobot, ArServerClient *, 
	      ArNetPacket *> myNetGetStraightPathsCB;
  ArFunctorC<ArCentralMultiRobot> myPopupMultiRobotInfoCB;
};


#endif 
