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
#ifndef ARMULTIROBOTPEER_H
#define ARMULTIROBOTPEER_H

#include "Aria.h"
#include "ArNetworking.h"
#include "arnlInternal.h"
#include "ArPathPlanningTask.h"
#include "ArBaseLocalizationTask.h"

///  handles the server side of dispensing information for multi robot driving
/**
   This class handles the server side of dispensing information for
   multi robot driving
**/
class ArServerHandlerMultiRobotPeer
{
public:
  AREXPORT ArServerHandlerMultiRobotPeer(ArServerBase *server, ArRobot *robot,
				     ArPathPlanningTask *pathTask,
				     ArBaseLocalizationTask *lockTask);
  AREXPORT ~ArServerHandlerMultiRobotPeer();
  /// Changes the precedence class this robot is in
  AREXPORT void setPrecedenceClass(int precedenceClass);
  /// Gets the precedence class this robot is in
  AREXPORT int getPrecedenceClass(void) { return myPrecedenceClass; }
  /// A function call called whenever precedence changes (and initially)
  AREXPORT void setNewPrecedenceCallback(
	  ArFunctor2<short, short> *newPrecedenceCallback);
  /// A function call called whenever fingerprint changes (and initially)
  AREXPORT void setNewFingerprintCallback(
	  ArFunctor1<unsigned char *> *newFingerprintCallback);
  /// Gets a functor that will give us a new fingerprint
  AREXPORT ArFunctor *getChangeFingerprintCB(void) 
    { return &myChangeFingerprintCB; }
  /// Function that changes our fingerprint
  AREXPORT void changeFingerprint(bool lockRobot = true);
  /// This function call will get a fingerprint of the robot
  AREXPORT void netGetFingerprint(ArServerClient *client, ArNetPacket *packet);
  /// This function call will adjust the precedence 
  AREXPORT void netAdjustPrecedence(ArServerClient *client, ArNetPacket *packet);
  /// Serves up the information about the robot's location and precedence
  AREXPORT void netRobotInfo(ArServerClient *client, ArNetPacket *packet);
  /// Serves up the information about the robot's path
  AREXPORT void netPathInfo(ArServerClient *client, ArNetPacket *packet);
  /// Serves up the information about the short version of robot's path
  AREXPORT void netShortPathInfo(ArServerClient *client, ArNetPacket *packet);
  /// processes the config change
  AREXPORT bool processFile(char *errorBuffer, size_t errorBufferLen);
  /// Supress multirobot path sending this cycle
  AREXPORT void supressPathSendingThisCycle(void);
protected:
  ArMutex myMutex;
  ArServerBase *myServer;
  ArRobot *myRobot;
  ArPathPlanningTask *myPathTask;
  ArBaseLocalizationTask *myLocTask;
  ArFunctor2<short, short> *myNewPrecedenceCallback;
  ArFunctor1<unsigned char *> *myNewFingerprintCallback;
  ArFunctor1C<ArServerHandlerMultiRobotPeer, bool> myChangeFingerprintCB;

  //bool myUseExplicitPrecedence;
  //int myExplicitPrecedence;
  int myPrecedenceBase;
  int myPrecedenceClass;
  int myRadius;
  int myRobotRadius;
  int myRobotRadiusAdjustment;
  int myPathRadius;
  int myPathRadiusAdjustment;
  int myPathMaxLength;
  int myPathResolutionAdjustment;
  int myPathResolution;
  int myShortPathLength;

  bool mySupressPathSendingThisCycle;

  // this is what generates our fingerprint
  ArMutex myFingerprintMutex;
  unsigned char myFingerprint[16];

  ArFunctor2C<ArServerHandlerMultiRobotPeer, 
	      ArServerClient *, ArNetPacket *> myNetGetFingerprintCB;
  ArFunctor2C<ArServerHandlerMultiRobotPeer, 
	      ArServerClient *, ArNetPacket *> myNetAdjustPrecedenceCB;
  ArFunctor2C<ArServerHandlerMultiRobotPeer, 
	      ArServerClient *, ArNetPacket *> myNetRobotCB;
  ArFunctor2C<ArServerHandlerMultiRobotPeer, 
                           ArServerClient *, ArNetPacket *> myNetPathCB;
  ArFunctor2C<ArServerHandlerMultiRobotPeer, 
                           ArServerClient *, ArNetPacket *> myNetShortPathCB;
  ArRetFunctor2C<bool, ArServerHandlerMultiRobotPeer, 
                            char *, size_t> myProcessFileCB;
};

/// This is the class that will make range data from multiple robots
class ArMultiRobotPeerRangeDevice : public ArRangeDevice
{
public:
  /// Constructor 
  AREXPORT ArMultiRobotPeerRangeDevice(ArMapInterface *arMap);
  /// Destructor
  AREXPORT ~ArMultiRobotPeerRangeDevice();
  /// The precedence to use 
  /// Our callback for processing the file
  AREXPORT bool processFile(char *errorBuffer, size_t errorBufferLen);
  /// Our sensor interp callback (does all our processing)
  AREXPORT void sensorInterpCallback(void);
  /// Our callback for a new precedence
  AREXPORT void setPrecedence(short precedenceBase, short precedenceClass);
  /// Our callback for a new precedence
  AREXPORT void setFingerprint(unsigned char *fingerprint);
  /// Gets our functor for setting the precedence
  ArFunctor2<short, short> *getSetPrecedenceCallback(void) 
  { return &mySetPrecedenceCallback; }
  /// Gets our functor for setting the fingerprint
  ArFunctor1<unsigned char *> *getSetFingerprintCallback(void) 
  { return &mySetFingerprintCallback; }
  /// Sets the functor we can use to change the fingerprint
  AREXPORT void setChangeFingerprintCB(ArFunctor *functor);
  /// Function to call when the map is changed
  AREXPORT void mapChanged(void);
  /// Sets the robot pointer, also attaches its process function to the
  /// sensorInterp of the robot
  AREXPORT virtual void setRobot(ArRobot *robot);
  /// we don't want to apply transforms since our readings come in global coords
  virtual void applyTransform(ArTransform trans, bool doCumulative) {}  
  /// Gets the other robot poses and radii
  AREXPORT std::list<ArMultiRobotPoseAndRadius> getOtherRobots(void);
  /// Gets the callback for other robots
  AREXPORT ArRetFunctor<std::list<ArMultiRobotPoseAndRadius> >*getOtherRobotsCB(void) { return &myOtherRobotsCB; }
  /// Enforces the that the server is using this protocol version
  AREXPORT void enforceProtocolVersion(const char *protocolVersion);

protected:
  
  class ConnectionHolder
  {
  public:
    /// Constructor
    ConnectionHolder(const char *hostname, int port, const char *user, 
		     const char *password, 
		     const char *enforceProtocolVersion);
    /// Destructor
    virtual ~ConnectionHolder();
    /// Sees if we're connected
    bool isConnected(void);
    /// Sees if we are ourself
    bool isSelf(void);
    /// Calls us
    void call(ArFunctor2<double, double> *addCurrentReadingFunctor,
	      ArFunctor2<double, double> *addCumulativeReadingFunctor,
	      ArFunctor1<ArMultiRobotPoseAndRadius> *addOtherRobotFunctor,
	      int precedence, int ignorePathDist, int shortPathMinDist,
	      ArPose currentPose, double timeoutInMins);
    /// Connects us
    void connect(void);
    /// Gets the packet that says where the other robot is
    void netRobotInfo(ArNetPacket *Packet);
    /// Gets the packet that gives us the path of the other robot
    void netPathInfo(ArNetPacket *Packet);
    /// Gets the map name
    void netMapName(ArNetPacket *Packet);
    /// Gets that the map was updated
    void netMapUpdated(ArNetPacket *Packet);

    /// Gets the packet that gives us the fingerprint of the robot
    void netFingerprint(ArNetPacket *Packet);
    /// Sets the fingerprint of this robot
    void setFingerprint(char fingerprint[16]);
    /// Sets the mapname of this robot
    void setMapName(const char* mapName);
  protected:
    int myRobotX, myRobotY, myRobotRadius;
    short myRobotPrecedenceBase;
    short myRobotPrecedenceClass;
    int myRobotPrecedence;
    int myRealRobotRadius;
    bool myRobotLost;
    ArTime myRobotLast;
    ArNetPacket myPathPacket;
    ArTime myPathLast;
    unsigned char myRobotFingerprint[16];
    bool myIsSelf;
    bool myNeedNewFingerprintCompare;
    char myFingerprint[16];

    ArTime myLastSend;
    std::string myRobotMapName;
    bool myNeedNewMapNameCompare;
    bool myDifferentMap;
    std::string myMapName;

    bool myNoMultiRobotPeer;

    std::string myHostname;
    int myPort;
    std::string myUser;
    std::string myPassword;
    bool myConnected;
    ArMutex myConnectedMutex;
    ArMutex myDataMutex;
    ArClientBase myClient;
    bool myPathRequested;
    bool myShortPathRequested;
    ArTime myLastInfo;
    ArTime myConnectedTime;
    ArTime myDisconnectedTime;
    ArFunctor1C<ArMultiRobotPeerRangeDevice::ConnectionHolder, ArNetPacket *> myNetFingerprintCB;
    ArFunctor1C<ArMultiRobotPeerRangeDevice::ConnectionHolder, ArNetPacket *> myNetRobotInfoCB;
    ArFunctor1C<ArMultiRobotPeerRangeDevice::ConnectionHolder, ArNetPacket *> myNetPathInfoCB;
    ArFunctor1C<ArMultiRobotPeerRangeDevice::ConnectionHolder, ArNetPacket *> myNetMapNameCB;
    ArFunctor1C<ArMultiRobotPeerRangeDevice::ConnectionHolder, ArNetPacket *> myNetMapUpdatedCB;
  };
  /** 
     this class runs a thread that connects all the data objects...
     It holds the last string of values, and deletes the 
  **/
  class ConnectorThread : public ArASyncTask
  {
  public:
    ConnectorThread();
    virtual ~ConnectorThread();
    void call(ArFunctor2<double, double> *addCurrentReadingFunctor,
	      ArFunctor2<double, double> *addCumulativeReadingFunctor,
	      ArFunctor1<ArMultiRobotPoseAndRadius> *addOtherRobotFunctor,
	      int precedence, int ignorePathDist, int shortPathMinDist,
	      ArPose currentPose, double timeoutInMins);
    void update(const char *hosts, const char *user, const char *password,
		const char *enforceProtocolVersion);
    void setFingerprint(unsigned char *fingerprint);
    void setMapName(const char *mapName);
    void setChangeFingerprintCB(ArFunctor *functor);
    virtual void *runThread(void *arg);
  protected:
    std::list<ConnectionHolder *> myList;
    ArMutex myListMutex;
    bool myChanged;
    char myFingerprint[16];
    bool myNewFingerprint;
    std::string myMapName;
    bool myNewMapName;
    bool myWarnedMatchs;
    ArFunctor *myChangeFingerprintCB;
    std::string myNewConnections;
    std::string myUser;
    std::string myPassword;
    std::string myEnforceProtocolVersion;
  };

  void addOtherRobot(ArMultiRobotPoseAndRadius otherRobot);

  std::list<ArMultiRobotPoseAndRadius> myOtherRobots;

  ArMapInterface *myMap;
  char myHosts[50000];
  char myUser[512];
  char myPassword[512];
  short myPrecedenceBase;
  short myPrecedenceClass;
  short myPrecedence;
  int myIgnorePathDist;
  int myShortPathMinDist;
  double myTimeoutInMins;
  ConnectorThread myThread;
  std::string myEnforceProtocolVersion;
  ArFunctorC<ArMultiRobotPeerRangeDevice> mySensorInterpCB;
  ArFunctor2C<ArRangeBuffer, double, double> myAddCurrentReadingCB;
  ArFunctor2C<ArRangeBuffer, double, double> myAddCumulativeReadingCB;
  ArFunctor1C<ArMultiRobotPeerRangeDevice, 
      ArMultiRobotPoseAndRadius> myAddOtherRobotCB;
  ArRetFunctor2C<bool, ArMultiRobotPeerRangeDevice, char *, size_t> myProcessFileCB;
  ArFunctor2C<ArMultiRobotPeerRangeDevice, short, short> mySetPrecedenceCallback;
  ArFunctor1C<ArMultiRobotPeerRangeDevice, unsigned char *> mySetFingerprintCallback;
  ArFunctorC<ArMultiRobotPeerRangeDevice> myMapChangedCB;
  ArRetFunctorC<std::list<ArMultiRobotPoseAndRadius>, ArMultiRobotPeerRangeDevice> myOtherRobotsCB;
};

#endif // ARMUTLIROBOTRANGEDEVICE_H
