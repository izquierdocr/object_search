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
#include "Aria.h"
#include "ArNetworking.h"
#include "ArExport.h"
//#include "ariaUtil.h"
#include "ArServerClasses.h"
#include "ArPathPlanningTask.h"
#include "ArBaseLocalizationTask.h"
#include <math.h>
#include <errno.h>

AREXPORT ArServerInfoPath::ArServerInfoPath(ArServerBase *server, 
					    ArRobot *robot,
					    ArPathPlanningTask *pathTask) :
  myGetPathCB(this, &ArServerInfoPath::getPath),
  myGetPathCurrentCB(this, &ArServerInfoPath::getPathCurrent),
  myGetSearchRectangleCB(this, &ArServerInfoPath::getSearchRectangle),
  mySearchRectangleEnableCB(this, &ArServerInfoPath::searchRectangleEnable),
  mySearchRectangleDisableCB(this, &ArServerInfoPath::searchRectangleDisable),
  myServerPathPlannerStatusCB(this, &ArServerInfoPath::serverPathPlannerStatus),
  myPathPlannerStateChangeCB(this, &ArServerInfoPath::pathPlannerStateChanged)
{
  myServer = server;
  myRobot = robot;
  myPathTask = pathTask;
  myServerInfoDrawings = NULL;
  myHandlerCommands = NULL;
  myAction = myPathTask->getPathPlanAction();
  myDrawSearchRectangle = false;
  mySearchRectangleDrawingData = NULL;
  myOwnSearchRectangleDrawingData = false;
  myPathPlannerStatus[0] = 0;

  setSearchRectangleDrawingData(
	  new ArDrawingData(
		  "polyLine", 
		  ArColor(166, 166, 166), // grey
		  1, // 1 mm
		  75, // layer 75
		  200, // refresh time
		  "DefaultOff"), // let people see it if they want, but by default don't show it
		  true);

  if (myServer != NULL)
  {
    myServer->addData("getPath", 
		      "gets the path of the robot to the goal as a series of points",
		      &myGetPathCB,
		      "none",
		      "int2: numPoints, repeats for numPoints byte4: x, byte4: y",
		      "NavigationInfo", "RETURN_SINGLE");
    myServer->addData("pathPlannerStatus", "path planner status", &myServerPathPlannerStatusCB, "none", "string", "Navigation", "RETURN_SINGLE");
  }

  pathTask->addStateChangeCB(&myPathPlannerStateChangeCB);
}

AREXPORT ArServerInfoPath::~ArServerInfoPath()
{

}

AREXPORT void ArServerInfoPath::getPath(ArServerClient *client, 
					ArNetPacket * /*packet*/)
{
  getPathInternal(client, true);
}

AREXPORT void ArServerInfoPath::getPathCurrent(ArServerClient *client, 
					ArNetPacket * /*packet*/)
{
  getPathInternal(client, false);
}

void ArServerInfoPath::getPathInternal(ArServerClient *client, 
						bool shortCount)
{
  ArNetPacket sending;
  
  myRobot->lock();
  if (!myAction->isActive() || !myPathTask->getInitializedFlag())
  {
    myRobot->unlock();
    sending.empty();
    if (shortCount)
      sending.byte2ToBuf(0);
    else
      sending.byte4ToBuf(0);
    client->sendPacketUdp(&sending);
    return;
  }

  int i; 
  int numPoints; // number of points to send in packet
  ArPose from = myRobot->getPose();
  
  myRobot->unlock();
  std::list<ArPose> points = myPathTask->getCurrentPath(from);

  numPoints = points.size();

  std::list<ArPose>::iterator it = points.begin();

  // skip first 2 points to avoid robot icon overlap
  if(numPoints <= 2) 
  {
     sending.empty();
     if(shortCount)
       sending.byte2ToBuf(0); // 0 points in packet
     else
       sending.byte4ToBuf(0);
     client->sendPacketUdp(&sending);
  	 return;
  }
  ++it; ++it;
  numPoints -= 2;

  // If there are too many path points to fit safely in a packet, degrade
  // it logarithmically from head to tail and maybe truncate it if still too
  // big.
  const int maxNumPoints = (ArNetPacket::MAX_LENGTH - ArNetPacket::HEADER_LENGTH - ArNetPacket::FOOTER_LENGTH - 4)/8 ;  // 8 bytes per point (4 for X, 4 for Y), numPoints may take 4 (or 2) bytes
  bool degradeLog = false;
  if(numPoints > maxNumPoints)
  {
    numPoints = (int)ceil(ArMath::log2(numPoints));
    degradeLog = true;
  }
  if(numPoints > maxNumPoints)  // still!
  {
    ArLog::log(ArLog::Normal, "ArServerInfoPath: Warning: Path is way too long for a net packet (%d, max is %d), will both degrade it logarithmically and truncate.", points.size(), maxNumPoints);
    numPoints = maxNumPoints;
  }
  else if(degradeLog)
  {
    ArLog::log(ArLog::Normal, "ArServerInfoPath: Warning: Path is too long for a net packet (%d, max is %d). Degrading logarithmically.", points.size(), maxNumPoints);
  }


  sending.empty();
  if(shortCount)
    sending.byte2ToBuf(numPoints); 
  else
    sending.byte4ToBuf(numPoints); 

  for (i = 0; it != points.end() && i < numPoints; it++)
  {
    sending.byte4ToBuf((int)(*it).getX());
    sending.byte4ToBuf((int)(*it).getY());
    if(degradeLog)
      i += i;
    else
      ++i;
  }

  client->sendPacketUdp(&sending);

}

AREXPORT void ArServerInfoPath::getSearchRectangle(ArServerClient *client, 
						   ArNetPacket *packet)
{
  if (!myDrawSearchRectangle || !myPathTask->getInitializedFlag())
  {
    ArNetPacket sendPacket;
    sendPacket.byte4ToBuf(0);
    client->sendPacketTcp(&sendPacket);
  }
  else
  {
    myPathTask->drawSearchRectangle(client, packet);
  }
}

void ArServerInfoPath::serverPathPlannerStatus(ArServerClient* client, ArNetPacket* /*pkt*/)
{
    ArNetPacket ret;
    ret.strToBuf(myPathPlannerStatus);
    client->sendPacketTcp(&ret);
}


void ArServerInfoPath::pathPlannerStateChanged()
{
  myPathTask->getStatusString(myPathPlannerStatus, sizeof(myPathPlannerStatus)-1);
  // todo, broadcast packet with new status?
}

AREXPORT void ArServerInfoPath::addControlCommands(
	ArServerHandlerCommands *handlerCommands)
{
  return;
  
  myHandlerCommands = handlerCommands;
  myHandlerCommands->addCommand(
	  "searchRectangleDrawingEnable",
	  "Enables display of the search area",
	  &mySearchRectangleEnableCB);
  myHandlerCommands->addCommand(
	  "searchRectangleDrawingDisable",
	  "Disables display of the search area",
	  &mySearchRectangleDisableCB);
}

AREXPORT void ArServerInfoPath::setSearchRectangleDrawingData(
	ArDrawingData *data, bool takeOwnershipOfData)
{
  if (mySearchRectangleDrawingData != NULL && myOwnSearchRectangleDrawingData)
  {
    delete mySearchRectangleDrawingData;
    mySearchRectangleDrawingData = NULL;
    myOwnSearchRectangleDrawingData = false;
  }
  mySearchRectangleDrawingData = data; 
  myOwnSearchRectangleDrawingData = takeOwnershipOfData; 
}

AREXPORT void ArServerInfoPath::addSearchRectangleDrawing(
	ArServerInfoDrawings *drawings)
{
  myDrawings = drawings;
  if (myDrawings != NULL)
  {
    drawings->addDrawing(mySearchRectangleDrawingData, 
			 "Path Planning Local Area", 
			 &myGetSearchRectangleCB);
    // MPL do this better later TODO
    /*
    drawings->addDrawing(new ArDrawingData(
				   "polyLine",
				   ArColor(0, 255, 255), // cyan
				   2,
				   74,
				   200,
				   "DefaultOn"),
			   "getPathCurrent",
			   &myGetPathCurrentCB);
    */
  }


}

AREXPORT void ArServerInfoPath::searchRectangleEnable(void)
{
  myDrawSearchRectangle = true;
}

AREXPORT void ArServerInfoPath::searchRectangleDisable(void)
{
  myDrawSearchRectangle = false;
}

AREXPORT ArServerInfoLocalization::ArServerInfoLocalization(
	ArServerBase *server, 
	ArRobot *robot,
	ArBaseLocalizationTask *locTask,
	ArBaseLocalizationTask *locTaskForPoints) :
  myGetLocPointsCB(this, &ArServerInfoLocalization::getLocPoints),
  myGetLocStateCB(this, &ArServerInfoLocalization::getLocState),
  myStateChangeCB(this, &ArServerInfoLocalization::stateChange)
{
  myServer = server;
  myRobot = robot;
  myLocTask = locTask;
  
  if (locTaskForPoints != NULL)
    myLocTaskForPoints = locTaskForPoints;
  else
    myLocTaskForPoints = locTask;

  myDataMutex.setLogName("ArServerInfoLocalization");

  if (locTask == NULL)
    return;

  if (myServer != NULL)
  {
    myServer->addData("getLocPoints", 
		      "gets localization points showing where robot might be", 
		      &myGetLocPointsCB, "none",
		      "byte2: numPoints, repeats for numPoints byte4: x, byte4: y",
		      "NavigationInfo", "RETURN_SINGLE");
    myServer->addData("getLocState", 
		      "gets localization state", 
		      &myGetLocStateCB, "none",
		      "ubyte: 0 == initialized (good), 1 == idle (probably mapping), 2 == uninitialized (lost), 3 == initializing (someone tried to localize, can't move until its done); ubyte2: locScore * 1000 (0 == lost, 1000 = perfect match)",
		      "NavigationInfo", "RETURN_SINGLE");
  }

  myState = GOOD;
  myLastState = myState;
  myStateChangeCB.setName("ArServerInfoLocalization");
  myLocTask->addStateChangeCB(&myStateChangeCB, 50);
  //myRobot->addUserTask("ArServerInfoLocalization", 50, &myUserTaskCB);
}

AREXPORT ArServerInfoLocalization::~ArServerInfoLocalization()
{

}


AREXPORT void ArServerInfoLocalization::getLocPoints(
	ArServerClient *client, ArNetPacket * /*packet*/ )
{
  ArNetPacket sending;
  
  std::list<ArPose> points = myLocTaskForPoints->getCurrentSamplePoses();
  std::list<ArPose>::iterator it;

  sending.empty();
  sending.byte2ToBuf(points.size()/10);
  
  int i;
  for (i = 0, it = points.begin(); it != points.end(); it++, i++)
  {
    if (i % 10 == 0)
    {
      //ArLog::log(ArLog::Normal, "%g %g\n", (*it).getX(), (*it).getY());
      sending.byte4ToBuf((int)(*it).getX());
      sending.byte4ToBuf((int)(*it).getY());
    }
  }

  client->sendPacketUdp(&sending);

}

AREXPORT void ArServerInfoLocalization::getLocState(
	ArServerClient *client, ArNetPacket * /*packet*/ )
{
  ArNetPacket sending;

  sending.uByteToBuf(myState);
  sending.uByte2ToBuf(
	  ArMath::roundInt(myLocTask->getLocalizationScore() * 1000));
  client->sendPacketUdp(&sending);
}



void ArServerInfoLocalization::stateChange(void)
{
  // MPL 
  //ArMap *arMap;
  //arMap = myLocTask->getAriaMap();
  
  ArBaseLocalizationTask::LocalizationState locState = myLocTask->getState();
  //bool init = myLocTask->getInitializedFlag();
  bool init = !myLocTask->getRobotIsLostFlag();

  myDataMutex.lock();

  /*
  if (myLocTask->getIdleFlag() || arMap == NULL || 
      (arMap->getNumPoints() == 0 && arMap->getNumLines() == 0))
  */
  if (locState == ArBaseLocalizationTask::LOCALIZATION_IDLE || 
      locState == ArBaseLocalizationTask::NOT_INITIALIZED)
    myState = IDLE;
  else if (locState == ArBaseLocalizationTask::LOCALIZATION_INIT_COMPUTING)
    myState = INITING;
  else if (init)
    myState = GOOD;
  else
    myState = LOST;

  State state = myState;
  State lastState = myLastState;

  myLastState = myState;

  myDataMutex.unlock();

  if (state != lastState)
  {
    if (lastState == INITING && state == GOOD)
      myInitSucceededCBList.invoke();
    if (lastState == INITING && state == LOST)
      myInitFailedCBList.invoke();
    // if we were initializing and are lost we don't call this because
    // the init failed callback happened... if we were idle and lost
    // we don't call this because we were probably mapping or
    // something like that
    if ((lastState != INITING && lastState != IDLE) && state == LOST)
      myLostCBList.invoke();
    if (state == INITING)
      myInitCBList.invoke();
  }
  
  //myLastState = myState;
  
}

AREXPORT void ArServerInfoLocalization::setToLostState(void)
{
  State origState = myState;
  myState = LOST;
  if (myState != origState)
    stateChange();
}


AREXPORT ArServerHandlerLocalization::ArServerHandlerLocalization(
	ArServerBase *server, ArRobot *robot, 
	ArBaseLocalizationTask *locTask,
	bool addResetToHome,
	bool setSimPoseOnLocalize) :
  myNetResetToHomeCB(this, &ArServerHandlerLocalization::netResetToHome),
  myNetLocalizeToPoseCB(this, &ArServerHandlerLocalization::netLocalizeToPose),
  myUserTaskCB(this, &ArServerHandlerLocalization::userTask),
  myRelativeLocalizeToPoseCB(
	  this, &ArServerHandlerLocalization::relativeLocalizeToPose),
  myRelativeLocalizeHeadingCB(
	  this, &ArServerHandlerLocalization::relativeLocalizeHeading)
{
  myServer = server;
  myRobot = robot;
  myLocTask = locTask;
  
  mySetSimPoseOnLocalize = setSimPoseOnLocalize;
  myLastLocalizeSimCounter = 0;
  myLastLocalizeMovedSince = true;

  if (myServer != NULL)
  {
    if (addResetToHome)
      myServer->addData("resetToHome", "relocalizes the robot to home",
			&myNetResetToHomeCB, "none", "none", "Localize", 
			"RETURN_NONE");
    myServer->addData("localizeToPose", "localizes the robot to given pose (with the option it can control the xy spread and th spread (both must be given if either is))",
		      &myNetLocalizeToPoseCB, "byte4:x, byte4:y, byte4:th, <optional uByte4:xy_spread uByte4:th_spread>",
		      "none", "Localize", "RETURN_NONE");
  }
  myUserTaskCB.setName("ArServerHandlerLocalization");
  myRobot->addUserTask("ArServerHandlerLocalization", 50, &myUserTaskCB);
  myLocalizeCallbackMutex.setLogName(
	  "ArServerHandlerLocalization::myLocalizeCallbackMutex");
}

AREXPORT ArServerHandlerLocalization::~ArServerHandlerLocalization()
{

}


AREXPORT void ArServerHandlerLocalization::netResetToHome(
	ArServerClient * /*client*/, ArNetPacket * /*packet*/)
{
  if (myLocTask != NULL)
    localizeToPose(myLocTask->getRobotHome());
}

AREXPORT void ArServerHandlerLocalization::netLocalizeToPose(
	ArServerClient *client, ArNetPacket *packet)
{
  double x, y, th;
  x = packet->bufToByte4();
  y = packet->bufToByte4();
  th = packet->bufToByte4();
  ArPose pose(x, y, th);

  double xySpread = -1;
  double thSpread = -1;
  // if the xyspread is here grab it
  if (packet->getDataReadLength() < packet->getDataLength())
    xySpread = packet->bufToUByte4();

  // if we have the the th spread grab it
  if (packet->getDataReadLength() < packet->getDataLength())
    thSpread = packet->bufToUByte4();

  ArLog::log(ArLog::Normal, "Localizing from ArNetworking connection %s",
	     client->getIPString());
  localizeToPose(pose, true, xySpread, thSpread);
}
 
 AREXPORT void ArServerHandlerLocalization::localizeToPose(
	 ArPose pose, bool lockRobot, double xySpread, double thSpread)
{

  bool isSim;

  if (mySetSimPoseOnLocalize && myRobot->getOrigRobotConfig() != NULL && 
      myRobot->getOrigRobotConfig()->hasPacketArrived() && 
      strcmp(myRobot->getOrigRobotConfig()->getSerialNumber(), "SIM") == 0)
    isSim = true;
  else
    isSim = false;

  printf("Localizing to pose %.0f %.0f %.1f\n", 
	 pose.getX(), pose.getY(), pose.getTh());

  if (!isSim)
    callBeforeLocalizeCallbacks(pose, ArPose(xySpread, xySpread, thSpread), -1);
  else
    callBeforeLocalizeCallbacks(pose, ArPose(0, 0, 0), -1);

  if (lockRobot)
    myRobot->lock();

  myPose = pose;
  mySpread.setX(xySpread);
  mySpread.setY(xySpread);
  mySpread.setTh(thSpread);
  myLastLocalizeEncoderPose = myRobot->getEncoderPose();
  myLastLocalizeMovedSince = false;

  myRobot->moveTo(myPose);


  // if we're in the sim, set the sim pose and set the counter so that
  // we'll localize after the sim updates our pose and sends us sensor
  if (isSim)
  {
    setSimPose(myPose, false);
    myLastLocalizeSimCounter = myRobot->getCounter();
  }

  if (lockRobot)
    myRobot->unlock();

  // if its not the sim then localize
  if (myLastLocalizeSimCounter == 0)
  {
    if (myLocTask != NULL)
      myLocTask->setRobotPose(myPose, ArPose(xySpread, xySpread, thSpread));
    callLocalizeCallbacks(myPose, ArPose(xySpread, xySpread, thSpread), -1);
  }

}

AREXPORT void ArServerHandlerLocalization::setSimPose(ArPose pose,
						      bool lockRobot)
{
  if (lockRobot)
    myRobot->lock();

  ArRobotPacket p;
  p.setID(ArCommands::SIM_SET_POSE);
  //p.uByteToBuf(0); // arg type, N/A for this command (dataN does this for us now)
  p.byte4ToBuf(ArMath::roundInt(pose.getX()));
  p.byte4ToBuf(ArMath::roundInt(pose.getY()));
  p.byte4ToBuf(ArMath::roundInt(pose.getTh()));
  p.finalizePacket();
  //myRobot->getDeviceConnection()->write(p.getBuf(), p.getLength());
  myRobot->comDataN(ArCommands::SIM_SET_POSE,
		    &p.getBuf()[p.getHeaderLength()], 
		    p.getLength() - p.getHeaderLength() - p.getFooterLength());
  if (lockRobot)
    myRobot->unlock();
  
}

void ArServerHandlerLocalization::userTask()
{
  // if we've moved since we last localized flag that
  if (!myLastLocalizeMovedSince && 
      myLastLocalizeEncoderPose.findDistanceTo(myRobot->getEncoderPose()) > 4 &&
      fabs(ArMath::subAngle(myLastLocalizeEncoderPose.getTh(),
			    myRobot->getEncoderTh())) > 1)
  {
    myLastLocalizeMovedSince = true;
  }

  if (myLastLocalizeSimCounter != 0 && 
      myRobot->getCounter() - myLastLocalizeSimCounter > 4)
  {
    myLastLocalizeSimCounter = 0;
    myRobot->moveTo(myPose);
    if (myLocTask != NULL)
      myLocTask->setRobotPose(myPose, ArPose(0, 0, 0), -1);
    callLocalizeCallbacks(myPose, ArPose(0, 0, 0), -1);
  }

  
}

AREXPORT void ArServerHandlerLocalization::addSimpleRelativeCommands(
	ArServerHandlerCommands *commands)
{
  commands->addStringCommand("RelativeLocalizeToPose",
		"Localized the robot to this offset from the current pose, input is '<x> <y> <heading>'",
			     &myRelativeLocalizeToPoseCB);
  commands->addStringCommand("RelativeLocalizeHeading",
		"Localized the robot to this offset from the current pose, input is '<heading>l",
			     &myRelativeLocalizeHeadingCB);
}

AREXPORT void ArServerHandlerLocalization::relativeLocalizeToPose(
	ArArgumentBuilder *arg)
{
  if (arg->getArgc() < 3 || !arg->isArgDouble(0) || !arg->isArgDouble(1) || 
      !arg->isArgDouble(2))
  {
    ArLog::log(ArLog::Normal, "Bad argument to relativeLocalizeToPose of %s", 
	       arg->getFullString());
    return;
  }
  double xOffset = arg->getArgDouble(0);
  double yOffset = arg->getArgDouble(1);
  double thOffset = arg->getArgDouble(2);

  ArPose pose;
  myRobot->lock();
  pose.setX(myRobot->getX() + xOffset);
  pose.setY(myRobot->getY() + yOffset);
  pose.setTh(ArMath::addAngle(myRobot->getTh(), thOffset));
  ArLog::log(ArLog::Normal, "Localizing from relativeLocalizeToPose %.0f %.0f %.0f", xOffset, yOffset, thOffset);
  myRobot->unlock();
  localizeToPose(pose, true, -1, -1);
}

AREXPORT void ArServerHandlerLocalization::relativeLocalizeHeading(
	ArArgumentBuilder *arg)
{
  if (arg->getArgc() < 1  || !arg->isArgDouble(0))
  {
    ArLog::log(ArLog::Normal, "Bad argument to relativeLocalizeHeading of %s", 
	       arg->getFullString());
    return;
  }
  double thOffset = arg->getArgDouble(0);

  ArPose pose;
  myRobot->lock();
  pose.setX(myRobot->getX());
  pose.setY(myRobot->getY());
  pose.setTh(ArMath::addAngle(myRobot->getTh(), thOffset));
  ArLog::log(ArLog::Normal, "Localizing from relativeLocalizeHeading %.0f", thOffset);
  myRobot->unlock();
  localizeToPose(pose, true, -1, -1);

}

AREXPORT void ArServerHandlerLocalization::addLocalizeCallback(
	ArFunctor3<ArPose, ArPose, int> *functor, int position)
{
  myLocalizeCallbackMutex.lock();
  myLocalizeCallbacks.insert(
	  std::pair<int, ArFunctor3<ArPose, ArPose, int> *>(-position, 
							    functor));
  myLocalizeCallbackMutex.unlock();
}

AREXPORT void ArServerHandlerLocalization::remLocalizeCallback(
	ArFunctor3<ArPose, ArPose, int> *functor)
{
  std::multimap<int, ArFunctor3<ArPose, ArPose, int> *>::iterator it;

  myLocalizeCallbackMutex.lock();
  for (it = myLocalizeCallbacks.begin(); it != myLocalizeCallbacks.end(); it++)
  {
    if ((*it).second == functor)
    {
      myLocalizeCallbacks.erase(it);
      myLocalizeCallbackMutex.unlock();
      remLocalizeCallback(functor);
      return;
    }
  }
  myLocalizeCallbackMutex.unlock();
}

/// @internal
AREXPORT void ArServerHandlerLocalization::callLocalizeCallbacks(
	ArPose pose, ArPose spread, int nsam)
{
  std::multimap<int, ArFunctor3<ArPose, ArPose, int> *>::iterator it;
  ArFunctor3<ArPose, ArPose, int> *functor;

  myLocalizeCallbackMutex.lock();

  for (it = myLocalizeCallbacks.begin(); it != myLocalizeCallbacks.end(); it++)
  {
    functor = (*it).second;
    if (functor == NULL)
      continue;
    functor->invoke(pose, spread, nsam);
  }

  myLocalizeCallbackMutex.unlock();
}

AREXPORT void ArServerHandlerLocalization::addBeforeLocalizeCallback(
	ArFunctor3<ArPose, ArPose, int> *functor, int position)
{
  myLocalizeCallbackMutex.lock();
  myBeforeLocalizeCallbacks.insert(
	  std::pair<int, ArFunctor3<ArPose, ArPose, int> *>(-position, 
							    functor));
  myLocalizeCallbackMutex.unlock();
}

AREXPORT void ArServerHandlerLocalization::remBeforeLocalizeCallback(
	ArFunctor3<ArPose, ArPose, int> *functor)
{
  std::multimap<int, ArFunctor3<ArPose, ArPose, int> *>::iterator it;

  myLocalizeCallbackMutex.lock();
  for (it = myBeforeLocalizeCallbacks.begin(); 
       it != myBeforeLocalizeCallbacks.end(); 
       it++)
  {
    if ((*it).second == functor)
    {
      myBeforeLocalizeCallbacks.erase(it);
      myLocalizeCallbackMutex.unlock();
      remBeforeLocalizeCallback(functor);
      return;
    }
  }
  myLocalizeCallbackMutex.unlock();
}

/// @internal
AREXPORT void ArServerHandlerLocalization::callBeforeLocalizeCallbacks(
	ArPose pose, ArPose spread, int nsam)
{
  std::multimap<int, ArFunctor3<ArPose, ArPose, int> *>::iterator it;
  ArFunctor3<ArPose, ArPose, int> *functor;

  myLocalizeCallbackMutex.lock();

  for (it = myBeforeLocalizeCallbacks.begin(); 
       it != myBeforeLocalizeCallbacks.end(); 
       it++)
  {
    functor = (*it).second;
    if (functor == NULL)
      continue;
    functor->invoke(pose, spread, nsam);
  }

  myLocalizeCallbackMutex.unlock();
}

/// @internal
AREXPORT bool ArServerHandlerLocalization::getLastLocalizationIfNotMovedSince(
	ArPose *pose, ArPose *spread, int *nsam)
{
  //myRobot->lock();
  if (myLastLocalizeMovedSince)
  {
    //myRobot->unlock();
    return false;
  }
  (*pose) = myPose;
  (*spread) = mySpread;
  (*nsam) = -1;
  //myRobot->unlock();
  return true;
}


/*!
 * 
 */
AREXPORT ArServerModeGotoLLA::ArServerModeGotoLLA(
	ArServerBase *server, ArRobot *robot, 
	ArBaseLocalizationTask* locTask, ArPathPlanningTask *pathTask,
	ArMapInterface *arMap, ArPose home, 
	ArRetFunctor<ArPose> *getHomePoseCB) :
  ArServerMode(robot, server, "GotoLLA"),
  myGoalDoneCB(this, &ArServerModeGotoLLA::goalDone),
  myGoalFailedCB(this, &ArServerModeGotoLLA::goalFailed),
  myGotoLLACB(this, &ArServerModeGotoLLA::gotoLLA),
  myTourGoalsInListLLACB(this, &ArServerModeGotoLLA::tourGoalsInListCommand)
{
  myServer = server;
  myRobot = robot;
  myLocTask = locTask;
  myPathTask = pathTask;
  myGoingHome = false;
  myTouringGoals = false;
  myMap = arMap;
  myHome = home;

  myPathTask->addGoalDoneCB(&myGoalDoneCB);
  myPathTask->addGoalFailedCB(&myGoalFailedCB);
}

AREXPORT ArServerModeGotoLLA::~ArServerModeGotoLLA()
{

}

AREXPORT void 
ArServerModeGotoLLA::addGotoLLACommand(ArServerHandlerCommands *commandsServer)
{
  commandsServer->addStringCommand("GotoGoalLLA", 
	   "Go to a goal defined by Latitude, Longitude, and Altitude coords ",
				   &myGotoLLACB);
}

AREXPORT void 
ArServerModeGotoLLA::gotoLLA(ArArgumentBuilder *arg)
{
  ArLog::log(ArLog::Normal, "GotoLLA: Begin");

  if (arg->getArgc() < 3 || !arg->isArgDouble(0) || !arg->isArgDouble(1) || 
      !arg->isArgDouble(2))
  {

    ArLog::log(ArLog::Normal, 
	       "Bad argument to convertLLA2RobotCoordsCallBack %s",
	       arg->getFullString());
    return;
  }
    
  double lat = arg->getArgDouble(0);
  double lon = arg->getArgDouble(1);
  double alt = arg->getArgDouble(2);
  if(fabs(lat) > 360.0 || fabs(lon) > 360.0)
  {
    ArLog::log(ArLog::Normal, 
	       "Bad latitude or longitude or both %g %g",
	       lat, lon);
    return;
  }

  ArLog::log(ArLog::Normal, "gotoLLA: LLA %g %g %g",
	     lat, lon, alt);
  double ea, no, up;
  bool worked = myLocTask->convertLLA2RobotCoords(lat, lon, alt,
						ea, no, up);
  if(worked)
  {
    ArLog::log(ArLog::Normal, "convertLLA2RobotCoords: ENU %g %g %g",
	       ea, no, up);
    ArPose goal(ea, no);
    ArPose llaGoal(lat, lon, alt);
    gotoPose(goal, llaGoal, false);
  }
  else
    ArLog::log(ArLog::Normal, "convertLLA2RobotCoords: false");

}

AREXPORT void 
ArServerModeGotoLLA::activate(void)
{
  ArLog::log(ArLog::Normal, "ArServerModeGotoLLA::activate");
  if (!baseActivate())
  {
    return;
  }
  
  if(myTouringGoals)
  {
    planToNextTourGoal();
  }
  else
  {
    if(!myPathTask->pathPlanToPose(myGoalPose, myUseHeading))
    {
      ArLog::log(ArLog::Terse, "Error: Could not plan a path to point.");
      myStatus = "Failed to plan to point";
    }
  }
  myDone = false;
  setActivityTimeToNow();
}

AREXPORT void 
ArServerModeGotoLLA::deactivate(void)
{
  baseDeactivate();
  myPathTask->cancelPathPlan();
}

AREXPORT void 
ArServerModeGotoLLA::userTask(void)
{
  if (!myDone)
  {
    setActivityTimeToNow();

  }
}

size_t 
ArServerModeGotoLLA::numGoalsTouring()
{
  if(!myTouringGoals) 
    return 0;
  if(myAmTouringGoalsInList)
    return myTouringGoalsList.size();
  else
    return 0;
}

void 
ArServerModeGotoLLA::findNextTourGoal(void)
{
  char llaName[256];
  if(myAmTouringGoalsInList)
  {
    // If we are selecting goals from a list, return the head
    // and move it to the back.
    myGoal = myTouringGoalsList.front();
    myLLAGoal = myTouringLLAGoalsList.front();
    myTouringGoalsList.pop_front();
    myTouringLLAGoalsList.pop_front();
    myTouringGoalsList.push_back(myGoal);
    myTouringLLAGoalsList.push_back(myLLAGoal);
    ArLog::log(ArLog::Normal, 
	       "Tour goals: popped next goal \"%g %g\" from user's list.", 
	       myGoal.getX(), myGoal.getY());
    sprintf(llaName, " %.6f %.6f %.2f", 
	    myLLAGoal.getX(), myLLAGoal.getY(), myLLAGoal.getTh()); 
  }

  myStatus = "Touring to LLA point";
  myStatus += llaName;
}

void 
ArServerModeGotoLLA::planToNextTourGoal()
{
  size_t failedCount = 0;
  size_t numGoals = numGoalsTouring();
  while(failedCount < numGoals) 
  {
    findNextTourGoal();
    if(myPathTask->pathPlanToPose(myGoal, false))
    {
      return;
    }
    else
    {
      ++failedCount;
      ArLog::log(ArLog::Normal,
		 "Tour goals: Warning: failed to plan a path to \"%g %g\".",
		 myGoal.getX(), myGoal.getY());
    }
  }
  ArLog::log(ArLog::Normal, 
	     "Tour goals: Warning: failed to find a path to any goal.");
  myStatus = "Failed touring goals: All goals failed.";
}

void 
ArServerModeGotoLLA::goalDone(ArPose /*pose*/)
{
  if (!myIsActive)
    return;
  if (myGoingHome)
  {
    myDone = true;
    myStatus = "Returned home";
  }
  else if (myTouringGoals)
  {
    planToNextTourGoal();
  }
  else
  {
    myDone = false;
    myStatus = "Arrived at LLA point";
    char llaName[256];    
    sprintf(llaName, " %.6f %.6f %.2f", 
	    myLLAGoalPose.getX(), myLLAGoalPose.getY(), 
	    myLLAGoalPose.getTh()); 
    myStatus += llaName;
  }
}

void 
ArServerModeGotoLLA::goalFailed(ArPose /*pose*/)
{
  if (!myIsActive)
    return;

  if (myTouringGoals)
  {
    if (ArUtil::strcasecmp(myStatus, "Robot lost") == 0)
    {
      myStatus = "Failed touring because robot lost";
    }
    else
    {
      planToNextTourGoal();
    }
  }
  else
  {
    myDone = true;
    std::string oldStatus;
    oldStatus = myStatus;
    if (myGoingHome)
      myStatus = "Failed to get home";
    else
    {
      myStatus = "Failed to get to LLA point";
      char llaName[256];    
      sprintf(llaName, " %.6f %.6f %.2f", 
	      myLLAGoalPose.getX(), myLLAGoalPose.getY(), 
	      myLLAGoalPose.getTh()); 
      myStatus += llaName;
    }

    if (ArUtil::strcasecmp(oldStatus, "Robot lost") == 0)
    {
      myStatus += " because robot lost";
    }
    else 
    {
      char failureStr[512];
      myPathTask->getFailureString(failureStr, sizeof(failureStr));
      myStatus += " (" + std::string(failureStr) + ")";
    }
  }
}

AREXPORT void 
ArServerModeGotoLLA::gotoPose(ArPose pose, ArPose llaPose, bool useHeading)
{
  myGoalPose = pose;
  myLLAGoalPose = llaPose;
  myUseHeading = useHeading;
  myStatus = "Going to LLA point";
  char llaName[256];    
  sprintf(llaName, " %.6f %.6f %.2f", 
	  myLLAGoalPose.getX(), myLLAGoalPose.getY(), myLLAGoalPose.getTh()); 
  myStatus += llaName;
  myMode = "Goto point";
  activate();
}
/** Does not check if goals listed are valid goals. */
AREXPORT void 
ArServerModeGotoLLA::tourGoalsInList(std::list<ArPose> goalList,
				     std::list<ArPose> llaGoalList)
{
  myTouringGoals = true;
  myAmTouringGoalsInList = true;
  myTouringGoalsList = goalList;
  myTouringLLAGoalsList = llaGoalList;
  myMode = "Touring goals";
  ArLog::log(ArLog::Normal, "Tour goals: touring %d goals from given list",
	     goalList.size());
  char str[2056];
  std::list<ArPose>::iterator iter;
  sprintf(str, "Goals are ");
  for(iter = goalList.begin(); iter != goalList.end(); iter++)
  {
    ArPose p = *(iter);
    sprintf(str, "%s%g %g, ", str, p.getX(), p.getY());
  }
  ArLog::log(ArLog::Normal, "%s", str);
  // reactivate (start tour over again)
  activate();
}
AREXPORT void 
ArServerModeGotoLLA::tourGoalsInListCommand(ArArgumentBuilder *args)
{
  ArLog::log(ArLog::Normal, "tourGoalsInListCommand: Begin");
  int nArgs = args->getArgc();
  std::list<ArPose> goals;
  std::list<ArPose> llaGoals;

  if (nArgs%3 != 0)
  {
    ArLog::log(ArLog::Normal, 
	       "Bad argument to tourGoalsInListCommand %s",
	       args->getFullString());
    return;
  }
  int nSets = nArgs/3;
  for(int i = 0; i < nSets; i++)
  {
    int ii = i*3;
    double lat = args->getArgDouble(ii);
    double lon = args->getArgDouble(ii+1);
    double alt = args->getArgDouble(ii+2);
    if(fabs(lat) > 360.0 || fabs(lon) > 360.0)
    {
      ArLog::log(ArLog::Normal, 
		 "Bad latitude or longitude or both %g %g",
		 lat, lon);
      return;
    }

    ArLog::log(ArLog::Normal, "tourGoalsInListCommand: LLA %g %g %g",
	       lat, lon, alt);
    double ea, no, up;
    bool worked = myLocTask->convertLLA2RobotCoords(lat, lon, alt,
						  ea, no, up);
    if(worked)
    {
      ArLog::log(ArLog::Normal, "convertLLA2RobotCoords: ENU %g %g %g",
		 ea, no, up);
      ArPose goal(ea, no);
      goals.push_back(goal);
      ArPose lla(lat, lon, alt);
      llaGoals.push_back(lla);
    }
    else
      ArLog::log(ArLog::Normal, "convertLLA2RobotCoords: false");
  }

  tourGoalsInList(goals, llaGoals);
}
AREXPORT void 
ArServerModeGotoLLA::addTourGoalsInListCommand(
	ArServerHandlerCommands *commandsServer)
{
  commandsServer->addStringCommand("TourLLAGoalsList", 
  "Tour goals defined by latitude longitude and altitude in the given list.",
				   &myTourGoalsInListLLACB);
}

/* ----- Goto mode: ---------- */

AREXPORT ArServerModeGoto::ArServerModeGoto(
	ArServerBase *server, ArRobot *robot, ArPathPlanningTask *pathTask,
	ArMapInterface *arMap, ArPose home, ArRetFunctor<ArPose> *getHomePoseCB) :
  ArServerMode(robot, server, "Goto"),
  myGoalDoneCB(this, &ArServerModeGoto::goalDone),
  myGoalFailedCB(this, &ArServerModeGoto::goalFailed),
  myServerGetGoalsCB(this, &ArServerModeGoto::serverGetGoals),
  myServerGotoGoalCB(this, &ArServerModeGoto::serverGotoGoal),
  myServerGotoPoseCB(this, &ArServerModeGoto::serverGotoPose),
  myServerHomeCB(this, &ArServerModeGoto::serverHome),
  myServerTourGoalsCB(this, &ArServerModeGoto::serverTourGoals),
  myServerGoalNameCB(this, &ArServerModeGoto::serverGoalName),
  myTourGoalsInListSimpleCommandCB(this, &ArServerModeGoto::tourGoalsInListCommand)
{

  myServer = server;
  myRobot = robot;
  myPathTask = pathTask;
  myGoingHome = false;
  myTouringGoals = false;
  myMap = arMap;
  myHome = home;
  myGetHomePoseCB = getHomePoseCB;
  myAmTouringGoalsInList = false;

  myPathTask->addGoalDoneCB(&myGoalDoneCB);
  myPathTask->addGoalFailedCB(&myGoalFailedCB);
  addModeData("gotoGoal", "sends the robot to the goal", 
	      &myServerGotoGoalCB, 
	      "string: goal", "none", "Navigation", "RETURN_NONE");
    
  addModeData("gotoPose", 
	      "sends the robot to a given x, y and optional heading", 
	      &myServerGotoPoseCB, 
	      "byte4: x byte4: y (optional) byte4: th", "none", "Navigation",
	      "RETURN_NONE");
  addModeData("home", "Sends the robot to where it started up",
	      &myServerHomeCB, "none", "none", "Navigation", "RETURN_NONE");
  myServer->addData("goalName", "current goal name", &myServerGoalNameCB, "none", "string", "Navigation", "RETURN_SINGLE");
  if (myMap != NULL)
  {
    addModeData("tourGoals",
		"sends the robot on a tour of all the goals",
		&myServerTourGoalsCB, "none", 
		"none", "Navigation", "RETURN_NONE");
  }
  myServer->addData("getGoals", "gets the list of goals", 
		    &myServerGetGoalsCB, "none", 
		    "<repeat> string: goal", "NavigationInfo", 
		    "RETURN_SINGLE");
}

AREXPORT ArServerModeGoto::~ArServerModeGoto()
{

}

AREXPORT void ArServerModeGoto::activate(void)
{

  if (!baseActivate())
  {
    return;
  }

  if(myTouringGoals)
  {
    planToNextTourGoal();
  }
  else
  {
    if (myGoalName.size() > 0)
    {
      if(!myPathTask->pathPlanToGoal(myGoalName.c_str()))
      {
        ArLog::log(ArLog::Terse, "Error: Could not plan a path to \"%s\".", myGoalName.c_str());
        myStatus = "Failed to plan to ";
        myStatus += myGoalName;
      }
    }
    else
    {
      if(!myPathTask->pathPlanToPose(myGoalPose, myUseHeading))
      {
        ArLog::log(ArLog::Terse, "Error: Could not plan a path to point.");
        myStatus = "Failed to plan to point";
      }
    }
  }

  myDone = false;
  setActivityTimeToNow();
}

AREXPORT void ArServerModeGoto::deactivate(void)
{
  baseDeactivate();
  myPathTask->cancelPathPlan();
}

AREXPORT void ArServerModeGoto::userTask(void)
{
  
  if (!myDone)
  {
    setActivityTimeToNow();

  }
}

AREXPORT void ArServerModeGoto::gotoPose(ArPose pose, bool useHeading)
{
  reset();
  myGoalPose = pose;
  myUseHeading = useHeading;
  myStatus = "Going to point";
  myMode = "Goto point";
  activate();
}

AREXPORT void ArServerModeGoto::home(void)
{
  reset();
  if (myGetHomePoseCB != NULL)
    myGoalPose = myGetHomePoseCB->invokeR();
  else
    myGoalPose = myHome;
  myUseHeading = true;
  myGoingHome = true;
  myStatus = "Returning home";
  myMode = "Go home";
  activate();
}

AREXPORT void ArServerModeGoto::gotoGoal(const char *goal)
{
  reset();
  myGoalName = goal;
  myMode = "Goto goal";
  myStatus = "Going to ";
  myStatus += goal;
  activate();
}

AREXPORT void ArServerModeGoto::tourGoals(void)
{
  std::string onGoal;

  onGoal = myGoalName;
  reset();
  myGoalName = onGoal;
  myTouringGoals = true;
  myAmTouringGoalsInList = false;
  myMode = "Touring goals";
  ArLog::log(ArLog::Normal, "Touring goals");
  //findNextTourGoal(); moved to activate()
  activate();
}

/** Does not check if goals listed are valid goals. */
AREXPORT void ArServerModeGoto::tourGoalsInList(std::deque<std::string> goalList)
{
  std::string onGoal = myGoalName;
  reset();
  myGoalName = onGoal;
  myTouringGoals = true;
  myAmTouringGoalsInList = true;
  myTouringGoalsList = goalList;
  myMode = "Touring goals";
  ArLog::log(ArLog::Normal, "Tour goals: touring %d goals from given list", goalList.size());
  //findNextTourGoal(); moved to activate()
  
  // reactivate (start tour over again)
  activate();
}

AREXPORT void ArServerModeGoto::addTourGoalsInListSimpleCommand(ArServerHandlerCommands *commandsServer)
{
  commandsServer->addStringCommand("TourGoalsList", 
    "Tour goals in the given list. Separate goal names with commas. "\
    "To add multiple goals with a common prefix, use the prefix followed by a *.",
    &myTourGoalsInListSimpleCommandCB);
}



AREXPORT bool ArServerModeGoto::isAutoResumeAfterInterrupt()
{
  return myTouringGoals;
}

#ifdef WIN32
// on Windows, strtok() uses thread-local storage, and can be safely called by multiple threads
char *strtok_r(char *str, const char *delim, char **thing)
{
  return strtok(str, delim);
}
#endif

AREXPORT void ArServerModeGoto::tourGoalsInListCommand(ArArgumentBuilder *args)
{
  char *str = strdup(args->getFullString());
  char *strtokpriv;
  char *tok = strtok_r(str, ",", &strtokpriv);
  //ArArgumentBuilder splitArgs(512, ','); // ArgumentBuilder always splits on
  //space
  //splitArgs.add(args->getArg(0));
  std::deque<std::string> goals;
  for(size_t i = 0; tok != NULL; i++)
  {
    // Strip preceding and following whitespace
    while(*tok && isspace(*tok))
    {
      ++tok;
    }
    char *tokend = tok + (strlen(tok) - 1);
    while(tokend && *tokend && isspace(*tokend))
    {
      *tokend-- = '\0';
    }

    // If a goal by this name exists, add it
    std::string s = tok;
    if(s.size() > 0) // skip empty strings
    {
      // if it has a * as the last char, then search for matching goals,
      // otherwise just push it into the list, if it exists.
      std::string::size_type starPos = s.find('*');
      if(starPos == s.npos)
      {
        myMap->lock();
        if(myMap->findMapObject(s.c_str(), "Goal") || myMap->findMapObject(s.c_str(), "GoalWithHeading"))
        {
          ArLog::log(ArLog::Normal, "Tour goals: adding \"%s\" to tour list.", s.c_str());
          goals.push_back(s);
        }
        else
        {
          ArLog::log(ArLog::Terse, "Tour goals: Warning: not adding \"%s\" to tour list; no goal by that name found in the map.", s.c_str());
        }
        myMap->unlock();
      }
      else if(starPos == s.size()-1)
      {
        // Find matching goals
        std::string prefix = s.substr(0, starPos);
        ArLog::log(ArLog::Normal, "Tour goals: searching for goals with prefix \"%s\"...", prefix.c_str());
        myMap->lock();
        for(std::list<ArMapObject*>::const_iterator i = myMap->getMapObjects()->begin();
            i != myMap->getMapObjects()->end(); i++)
        {
          if(!(*i)) continue;
          if( ! (strcasecmp((*i)->getType(), "GoalWithHeading") == 0 ||
                 strcasecmp((*i)->getType(), "Goal") == 0) )
            continue;
          const char *goalName = (*i)->getName();
          if(strncmp(goalName, prefix.c_str(), prefix.size()) == 0)
          {
            ArLog::log(ArLog::Normal, "\t...Adding matching goal \"%s\" to tour.", goalName);
            goals.push_back(goalName);
          }
        }
        myMap->unlock();
      }
      else
      {
        ArLog::log(ArLog::Terse, "Tour goals: Error in goal list; the \'*\' wildcard must be the last character in the goal name (in \"%s\"). starPos=%d, npos=%d, size=%d", s.c_str(), starPos, s.npos, s.size());
        free(str);
        return;
      }
    }

    // Find next token
    tok = strtok_r(NULL, ",", &strtokpriv);
  }
  free(str);
  tourGoalsInList(goals);
}


size_t ArServerModeGoto::numGoalsTouring()
{
  if(!myTouringGoals) return 0;
  if(myAmTouringGoalsInList)
  {
    return myTouringGoalsList.size();
  }
  else
  {
    if(!myMap) return 0;
    size_t count = 0;
    myMap->lock();
    for (std::list<ArMapObject*>::const_iterator i = myMap->getMapObjects()->begin(); 
         i != myMap->getMapObjects()->end(); 
         i++)
    {
      ArMapObject *obj = (*i);
      if ((strcasecmp(obj->getType(), "GoalWithHeading") == 0 ||
         strcasecmp(obj->getType(), "Goal") == 0))
      {
        ++count;
      }
    }
    myMap->unlock();
    return count;
  }
}

void ArServerModeGoto::findNextTourGoal(void)
{
  if (myMap == NULL)
  {
    myGoalName = "";
    return;
  }


  if(myAmTouringGoalsInList)
  {
    // If we are selecting goals from a list, return the head
    // and move it to the back.
    myGoalName = myTouringGoalsList.front();
    myTouringGoalsList.pop_front();
    myTouringGoalsList.push_back(myGoalName);
    ArLog::log(ArLog::Verbose, "Tour goals: popped next goal \"%s\" from user's list.", myGoalName.c_str());
  }
  else
  {
    // Otherwise, search the map's goals for the current goal,
    // and return the next one.
    std::list<ArMapObject *>::iterator objIt;
    ArMapObject* obj;
    bool nextGoalIt = false;
    bool gotGoal = false;
    std::string firstGoal;
    myMap->lock();
    for (objIt = myMap->getMapObjects()->begin(); 
         objIt != myMap->getMapObjects()->end(); 
         objIt++)
    {
      obj = (*objIt);
      if ((strcasecmp(obj->getType(), "GoalWithHeading") == 0 ||
         strcasecmp(obj->getType(), "Goal") == 0))
      {
        if (nextGoalIt)
        {
          myGoalName = obj->getName();
          gotGoal = true;
          break;
        }
        if (strcasecmp(obj->getName(), myGoalName.c_str()) == 0)
        {
          nextGoalIt = true;
        }
        if (firstGoal.size() <= 0)
          firstGoal = obj->getName();
      }
    }
    myMap->unlock();
    if (!gotGoal)
      myGoalName = firstGoal;
  }

  myStatus = "Touring to ";
  myStatus += myGoalName;
  //myPathTask->unlock();
  //myRobot->unlock();

}


void ArServerModeGoto::reset(void)
{
  myGoingHome = false;
  myTouringGoals = false;
  myGoalName = "";
  myUseHeading = true;
}

// keep trying to plan to goals in tour, until either one suceeds or all goals fail
void ArServerModeGoto::planToNextTourGoal()
{
  size_t failedCount = 0;
  size_t numGoals = numGoalsTouring();
  while(failedCount < numGoals) 
  {
    findNextTourGoal();
    if(myPathTask->pathPlanToGoal(myGoalName.c_str()))
    {
      return;
    }
    else
    {
      ++failedCount;
      ArLog::log(ArLog::Terse, "Tour goals: Warning: failed to plan a path to \"%s\".", myGoalName.c_str());
    }
  }
  ArLog::log(ArLog::Terse, "Tour goals: Warning: failed to find a path to any goal.");
  myStatus = "Failed touring goals: All goals failed.";
}

void ArServerModeGoto::goalDone(ArPose /*pose*/)
{
  if (!myIsActive)
    return;
  if (myGoingHome)
  {
    myDone = true;
    myStatus = "Returned home";
  }
  else if (myTouringGoals)
  {
    planToNextTourGoal();
  }
  else if (myGoalName.size() > 0)
  {
    myDone = true;
    myStatus = "Arrived at ";
    myStatus += myGoalName;
  }
  else
  {
    myDone = false;
    myStatus = "Arrived at point";
  }
}

void ArServerModeGoto::goalFailed(ArPose /*pose*/)
{
  if (!myIsActive)
    return;
  if (myPathTask->getAriaMap() == NULL || 
      strlen(myPathTask->getAriaMap()->getFileName()) <= 0)
  {
    myDone = true;
    myStatus = "Failed driving because map empty";
    ArLog::log(ArLog::Normal, "Failed driving because map empty");
    return;
  }
  if (myTouringGoals)
  {
    if (ArUtil::strcasecmp(myStatus, "Robot lost") == 0)
    {
      myStatus = "Failed touring because robot lost";
    }
    else
    {
      planToNextTourGoal();
    }
  }
  else
  {
    myDone = true;
    std::string oldStatus;
    oldStatus = myStatus;
    if (myGoingHome)
      myStatus = "Failed to get home";
    else if (myGoalName.size() > 0)
    {
      myStatus = "Failed to get to ";
      myStatus += myGoalName;
    }
    else
    {
      myStatus = "Failed to get to point";
    }

    
    if (ArUtil::strcasecmp(oldStatus, "Robot lost") == 0)
    {
      myStatus += " because robot lost";
    }
    else 
    {
      char failureStr[512];
      myPathTask->getFailureString(failureStr, sizeof(failureStr));
      myStatus += " (" + std::string(failureStr) + ")";
    }
  }
}

AREXPORT void ArServerModeGoto::serverGotoGoal(ArServerClient * /*client*/, 
					       ArNetPacket *packet)
{
  char buf[512];
  packet->bufToStr(buf, sizeof(buf)-1);
  ArLog::log(ArLog::Normal, "Going to goal %s", buf);
  //myRobot->lock();
  gotoGoal(buf);
  //myRobot->unlock();
}

AREXPORT void ArServerModeGoto::serverGotoPose(ArServerClient * /*client*/, 
					       ArNetPacket *packet)
{
  ArPose pose;
  bool useHeading = false;

  pose.setX(packet->bufToByte4());
  pose.setY(packet->bufToByte4());
  if (packet->getDataLength() > packet->getDataReadLength())
  {
    useHeading = true;
    pose.setTh(packet->bufToByte4());
  }
  //myRobot->lock();
  ArLog::log(ArLog::Normal, "Going to point");
  gotoPose(pose, useHeading);
  //myRobot->unlock();
}


AREXPORT void ArServerModeGoto::serverHome(ArServerClient * /*client*/, 
					   ArNetPacket * /*packet*/)
{
  ArLog::log(ArLog::Normal, "Going home");
  //myRobot->lock();
  home();
  //myRobot->unlock();
}

AREXPORT void ArServerModeGoto::serverTourGoals(ArServerClient * /*client*/,
						ArNetPacket * /*packet*/ )
{
  ArLog::log(ArLog::Normal, "Touring goals");
  //myRobot->lock();
  tourGoals();
  //myRobot->unlock();
}


AREXPORT void ArServerModeGoto::serverGetGoals(ArServerClient *client, 
					       ArNetPacket * /*packet*/ )
{
  ArNetPacket sendPacket;
  ArLog::log(ArLog::Verbose, "getGoals requested");

  if (myMap == NULL)
  {
    client->sendPacketTcp(&sendPacket);
    myPathTask->unlock();
    return;
  }

  myMap->lock();
  std::list<ArMapObject *>::iterator objIt;
  ArMapObject* obj;
  for (objIt = myMap->getMapObjects()->begin(); 
       objIt != myMap->getMapObjects()->end(); 
       objIt++)
  {
    obj = (*objIt);
    if (strcasecmp(obj->getType(), "GoalWithHeading") == 0 ||
	strcasecmp(obj->getType(), "Goal") == 0)
    {
      sendPacket.strToBuf(obj->getName());
    }
  }
  myMap->unlock();
  client->sendPacketTcp(&sendPacket);
}







void ArServerModeGoto::serverGoalName(ArServerClient *client, ArNetPacket * /*pkt*/)
{
    ArNetPacket retPkt;
    retPkt.strToBuf(myGoalName.c_str());
    client->sendPacketTcp(&retPkt);
}




/**
   This class will draw the current path planning destination on the client.
   
   @param infoDrawings the place to add our functor to draw too
   @param pathTask the path planning task to draw the destination of
   @param name the name to use for the drawing
 **/
AREXPORT ArServerDrawingDestination::ArServerDrawingDestination(
	ArServerInfoDrawings *infoDrawings,
	ArPathPlanningTask *pathTask,
	const char *name) :
  myDrawDestinationCB(this, &ArServerDrawingDestination::drawDestination),
  myProcessFileCB(this, &ArServerDrawingDestination::processFile)
{
  myInfoDrawings = infoDrawings;
  myPathTask = pathTask;
  myName = name;
  myDrawingData = NULL;
  myOwnDrawingData = false;
  myOn = false;
  myWasInitialized = false;
  setFlashingParameters(1, 0);
  
  setDrawingData(new ArDrawingData("polyDots",
				   ArColor(0xff, 0xff, 0x0),
				   800, // size
				   49), // just below the robot
		 true);
  // make sure our data's valid, don't check return on addDrawing
  // since it warns itself and we don't really care if it doesn't work
  if (myInfoDrawings != NULL && myPathTask != NULL && myDrawingData != NULL)
  {
    myInfoDrawings->addDrawing(myDrawingData, myName.c_str(),
			       &myDrawDestinationCB);
  }
}

AREXPORT ArServerDrawingDestination::~ArServerDrawingDestination()
{
  
}

/**
   @param onMSec if onMSec and offMSec are both > 0 then the
   destination will flash on for onMSec and off for offMSec, if onMSec
   is 0 it'll never draw

   @param offMSec if onMSec and offMSec are both > 0 then the
   destination will flash on for onMSec and off for offMSec, if offMSec
   is 0 it'll just stay solid on   
**/
AREXPORT void ArServerDrawingDestination::setFlashingParameters(int onMSec, 
								int offMSec) 
{
  if (onMSec < 0)
    myOnMSec = 0;
  else
    myOnMSec = onMSec;
  if (offMSec < 0)
    myOffMSec = 0;
  else
    myOffMSec = offMSec;
}

/**
   @param drawingData how to draw the destination
   @param ownDrawingData whether this instance owns the drawing data or not
*/
AREXPORT void ArServerDrawingDestination::setDrawingData(
	ArDrawingData *drawingData,  bool ownDrawingData) 
{
  if (myDrawingData != NULL && myOwnDrawingData)
  {
    delete myDrawingData;
    myDrawingData = NULL;
    myOwnDrawingData = false;
  }
  myDrawingData = drawingData;
  myOwnDrawingData = ownDrawingData;
}

AREXPORT void ArServerDrawingDestination::drawDestination(
	ArServerClient *client, ArNetPacket * /*packet*/)
{
  ArNetPacket sendPacket;
  ArPose currentGoal;

  currentGoal = myPathTask->getCurrentGoal();

  // first see if we're off all together
  if (!myPathTask->getInitializedFlag())
  {
    myWasInitialized = false;
    myOn = false;
  }
  // if we just started the goal (so we force on)
  else if ((!myWasInitialized || 
	   myLastGoal.squaredFindDistanceTo(currentGoal) > 1) && 
	   myOnMSec != 0)
  {
    myWasInitialized = true;
    myOn = true;
    myLastSwitch.setToNow();
  }
  // next see if we should switch on 
  else if (!myOn && myOnMSec != 0 && myLastSwitch.mSecSince() > myOffMSec)
  {
    myOn = true;
    myLastSwitch.setToNow();
  }
  // see if we should switch off
  else if (myOn && myOffMSec != 0 && myLastSwitch.mSecSince() > myOnMSec)
  {
    myOn = false;
    myLastSwitch.setToNow();
  }
  myLastGoal = currentGoal;

  if (myOn)
  {
    sendPacket.byte4ToBuf(1);
    sendPacket.byte4ToBuf(ArMath::roundInt(currentGoal.getX()));
    sendPacket.byte4ToBuf(ArMath::roundInt(currentGoal.getY()));
  }
  else
  {
    sendPacket.byte4ToBuf(0);
  }
  if (client != NULL)
    client->sendPacketUdp(&sendPacket);

}

AREXPORT void ArServerDrawingDestination::addToConfig(ArConfig *config)
{
  if (config == NULL)
    return;
  

  sprintf(myConfigDrawMode, "AlwaysOff");
  myConfigFlashMSec = 1000;
  sprintf(myConfigShape, "polyDots");
  myConfigColorRGB = 0xFFFF00;
  myConfigSecondaryColorRGB = 0x000000;
  myConfigSize = 800;
  myConfigLayer = 49;
  myConfigRefreshTime = 200;

  std::string section;
  section = "Destination Drawing";
  config->setSectionComment(section.c_str(), "Controls the appearance of the destination.  Changes to the drawing mode (and flash rate) values will take effect immediately, but all other changes require MobileEyes to be restarted.");
  /***
  config->setSectionComment(section.c_str(), "This will control if the destination is drawn, set OnMSec to 1 and OffMSec to 0 to draw the destination steadily, set OnMSec to 0 and OffMSec to 0 to never draw the destnation, set these otherwise to make the destination flash on for OnMSec and off for OffMSec...  To get everything other than OnMSec and OffMSec to take effect you'll need to restart MobileEyes after changing these values");
  **/
  config->addParam(
	  ArConfigArg("DrawingMode", myConfigDrawMode, 
		            "How the destination indicator is drawn.",
		            sizeof(myConfigDrawMode)),
	  section.c_str(), ArPriority::DETAILED,
	  "Choices:AlwaysOff;;AlwaysOn;;Flash");

  config->addParam(
	  ArConfigArg("FlashMSec", &myConfigFlashMSec, 
                "The rate (in ms) at which the destination should be flashed; valid only when DrawingMode is set to Flash.  \
If set to 0, the destination will not be drawn.", 0),
	  section.c_str(), ArPriority::DETAILED);

  config->addParam(
	  ArConfigArg("Shape", myConfigShape, 
		            "Shape to be drawn at the destination.",
		            sizeof(myConfigShape)),
	  section.c_str(), ArPriority::DETAILED,
	  "Choices:polyDots;;polyArrows;;polyLine;;polyPoints;;polySegments");

  config->addParam(	  
	  ArConfigArg("PrimaryColor", &myConfigColorRGB, 
		            "The primary drawing color", 
		            0, 0xFFFFFF),
	  section.c_str(), ArPriority::DETAILED,
    "Color");

  config->addParam(	  
	  ArConfigArg("SecondaryColor", &myConfigSecondaryColorRGB, 
		            "The secondary drawing color", 
		            0, 0xFFFFFF),
	  section.c_str(), ArPriority::DETAILED,
    "Color");

  config->addParam(	  
	  ArConfigArg("Size", &myConfigSize, 
		      "The size (in mm) of the destination to be drawn", 
		      0),
	  section.c_str(), ArPriority::DETAILED);

  config->addParam(	  
	  ArConfigArg("Layer", &myConfigLayer, 
		      "The layer on which the destination is drawn", 
		      0),
	  section.c_str(), ArPriority::DETAILED);

  config->addParam(	  
	  ArConfigArg("RefreshTime", &myConfigRefreshTime, 
		      "Rate (in msecs) at which to refresh the drawing",
		      50),
	  section.c_str(), ArPriority::DETAILED);

  myProcessFileCB.setName("ArServerDrawingDestination");
  config->addProcessFileCB(&myProcessFileCB, 10);

}

AREXPORT bool ArServerDrawingDestination::processFile(void)
{  

  if (myDrawingData == NULL)
    return true;

  if (ArUtil::strcasecmp(myConfigDrawMode, "AlwaysOff") == 0) {
    setFlashingParameters(0, 1);
  }
  else if (ArUtil::strcasecmp(myConfigDrawMode, "AlwaysOn") == 0) {
    setFlashingParameters(1, 0);
  }
  else if (ArUtil::strcasecmp(myConfigDrawMode, "Flash") == 0) {
    setFlashingParameters(myConfigFlashMSec, myConfigFlashMSec);
  }

  myDrawingData->setShape(myConfigShape);
  myDrawingData->setPrimaryColor(ArColor(myConfigColorRGB));
  myDrawingData->setSecondaryColor(ArColor(myConfigSecondaryColorRGB));
  myDrawingData->setSize(myConfigSize);
  myDrawingData->setLayer(myConfigLayer);
  myDrawingData->setDefaultRefreshTime(myConfigRefreshTime);

  return true;
}



/**
 *  @param locTask  Localization task to get localization score from
 *  @param pathTask If not NULL, then cancel this path planner's goal if lost
 *    and the path planner is active.
 *  @param serverMode Server mode to activate if lost
 *  @param name Optional, name for this action
 */
AREXPORT ArActionLost::ArActionLost(
	ArBaseLocalizationTask *locTask,
	ArPathPlanningTask *pathTask,
	ArServerMode *serverMode,
	const char *name) :
  ArAction(name, "Stops the robot and fails the goal if losts occur"),
  myEnableCB(this, &ArActionLost::enable),
  myDisableCB(this, &ArActionLost::disable)
{
  myDataMutex.setLogName("ArActionLost::myDataMutex");
  myLocTask = locTask;
  myPathTask = pathTask;
  myServerMode = serverMode;
  myIsEnabled = true;
}

AREXPORT ArActionLost::~ArActionLost()
{
}

AREXPORT ArActionDesired *ArActionLost::fire(
	ArActionDesired /*currentDesired*/ )
{

  // if we're disabled just skip this all
  myDataMutex.lock();
  if (!myIsEnabled)
  {
    myDataMutex.unlock();
    return NULL;
  }
  myDataMutex.unlock();
  ArBaseLocalizationTask::LocalizationState state = myLocTask->getState();

  // if localization is lost or disabled don't let us drive
  if (myLocTask->getRobotIsLostFlag() || 
      state == ArBaseLocalizationTask::LOCALIZATION_INIT_COMPUTING || 
      state == ArBaseLocalizationTask::LOCALIZATION_IDLE)
  {
    // if the path planning isn't initialized just skip it
    if (myPathTask != NULL && myPathTask->getInitializedFlag())
    {
      if (state == ArBaseLocalizationTask::LOCALIZATION_IDLE)
	ArLog::log(ArLog::Terse, "Localization idle, failing");
      if (state == ArBaseLocalizationTask::LOCALIZATION_INIT_COMPUTING)
	ArLog::log(ArLog::Terse, 
		   "Localization initializing, failing");
      if (myLocTask->getRobotIsLostFlag())
	ArLog::log(ArLog::Terse, 
		   "Localization lost (not initialized), failing");
      myPathTask->goalFailed(myPathTask->getCurrentGoal(), "Robot lost");
    }
    if (myServerMode != NULL)
    {
      myServerMode->setStatus("Robot lost");
    }
    myDesired.reset();
    myDesired.setMaxVel(0);
    myDesired.setMaxNegVel(0);
    if (myRobot->hasLatVel())
    {
      myDesired.setMaxLeftLatVel(0);
      myDesired.setMaxRightLatVel(0);
    }
    return &myDesired;
  }
  return NULL;

}

AREXPORT void ArActionLost::enable(void)
{
  myDataMutex.lock();
  myIsEnabled = true;
  myDataMutex.unlock();
}

AREXPORT void ArActionLost::disable(void)
{
  myDataMutex.lock();
  myIsEnabled = false;
  myDataMutex.unlock();
}

/*!
 * This is the constructor, note the use of constructor chaining with the
 * ArAction... also note how it uses setNextArgument, which makes it so that 
 * other things can see what parameters this action has, and set them.
 * It also initializes the classes variables.

 *  @param lt: The pointer to the localization task.
 *
 */
AREXPORT
ArActionSlowDownWhenNotCertain::ArActionSlowDownWhenNotCertain(
	ArBaseLocalizationTask* lt
    ) :  ArAction("SlowDownWhenNotCertain")
{
  myLocalizationTask = lt;
}
/*!
 * Function to call when arnl.p is processed.
 */
bool     
ArActionSlowDownWhenNotCertain::reconfigureAction(void)
{
  // Behavior to allow another action to work the movement params.
  ArConfig* config = Aria::getConfig();

  if(config)
  { 
    ArConfigSection* section = config->findSection("Path planning settings");
    if(section)
    {
      ArConfigArg* arg = section->findParam("MaxSpeed");
      if(arg)
	myMaxVel = arg->getDouble();
      arg = section->findParam("MaxRotSpeed");
      if(arg)
	myMaxRotVel = arg->getDouble();
    }
    section = config->findSection("Robot config");
    if(section)
    {
      ArConfigArg* arg = section->findParam("LatVelMax");
      if(arg)
	myMaxLatVel = arg->getDouble();
    }
  }  
  return true;
}
/*!
 * Destructor.
 */
AREXPORT 
ArActionSlowDownWhenNotCertain::~ArActionSlowDownWhenNotCertain(void)
{
  if(myProcessFileCB)
  {
    ArConfig* config = Aria::getConfig();
    if(config)
      config->remProcessFileCB(myProcessFileCB);
    delete myProcessFileCB;
  }
}
/*!  Sets the myRobot pointer (all setRobot overloaded functions must
 *  do this),
 *  @param robot: The pointer to the robot class.
 *
 */
AREXPORT void 
ArActionSlowDownWhenNotCertain::setRobot(ArRobot *robot)
{
  myRobot = robot;
  myPower = 1;
  myMaxThreshold = 0.9;
  myMinThreshold = 0.1;
  // Behavior to allow another action to work the movement params.
  ArConfig* config = Aria::getConfig();

  if(config)
  { 
    std::string strPrefix = "SlowDown";
    std::string name;
#ifndef SONARNL
    char* nameSection = "Localization settings";
#else
    char* nameSection = "Sonar localization settings";
#endif

    myMaxVel = 750;
    myMaxRotVel = 100;
    myMaxLatVel = 750;
    myMinThreshold = 0.2;

    ArConfigSection* section = config->findSection("Path planning settings");
    if(section)
    {
      ArConfigArg* arg = section->findParam("MaxSpeed");
      if(arg)
	myMaxVel = arg->getDouble();
      // MPL adding this so that it'll chagne when the config changes
      config->addParam(ArConfigArg("MaxSpeed", &myMaxVel), 
		       "Path planning settings", ArPriority::DETAILED);
      arg = section->findParam("MaxRotSpeed");
      if(arg)
	myMaxRotVel = arg->getDouble();
      // MPL adding this so that it'll chagne when the config changes
      config->addParam(ArConfigArg("MaxRotSpeed", &myMaxRotVel), 
		       "Path planning settings", ArPriority::DETAILED);
    }
    else
    {
      name = strPrefix;
      name += "TransVelMax";
      config->addParam(ArConfigArg(name.c_str(), &myMaxVel, 
				   "Maximum translational velocity.",
				   0), nameSection, ArPriority::DETAILED);
      name = strPrefix;
      name += "RotVelMax";
      config->addParam(ArConfigArg(name.c_str(), &myMaxRotVel, 
				   "Maximum rotational velocity if it is \
not set by path planning.",
				   0), nameSection, ArPriority::DETAILED);
      
    }

    /// SEEKUR
    section = config->findSection("Robot config");
    if(myRobot->hasLatVel() && section)
    {
      ArConfigArg* arg = section->findParam("LatVelMax");
      if(arg)
	myMaxLatVel = arg->getDouble();
      config->addParam(ArConfigArg("LatVelMax", &myMaxLatVel), 
		       "Robot config", ArPriority::DETAILED);
    }
    else
      myMaxLatVel = 0;
    // taking this out since if it wasn't there then this probably doesn't have lat vel (can't check the robot here because there isn't one, and its not worth refactoring for)
    /*
      else
      {
      name = strPrefix;
      name += "LatVelMax";
      config->addParam(ArConfigArg(name.c_str(), &myMaxLatVel, 
      "Maximum lateral velocity.",
      0), nameSection, ArPriority::DETAILED);
      }
    */


    section = config->findSection("Localization settings");
    if(section)
    {
      ArConfigArg* arg = section->findParam("PassThreshold");
      if(arg)
	myMinThreshold = arg->getDouble();
      config->addParam(ArConfigArg("PassThreshold", &myMinThreshold), 
		       nameSection, ArPriority::DETAILED);

    }
    else
    {
      name = strPrefix;
      name += "MinThreshold";
      config->addParam(ArConfigArg(name.c_str(), &myMinThreshold,
				   "Minimum localization score below \
which the slow down speed is not interpolated. The SlowDownTransVelMin \
will be used in this case.",
				   0, 1), nameSection, ArPriority::DETAILED);
    }

    myEnableFlag = false;
    myMinVel = 250.0;
    myMinRotVel = 25.0;
    myMinLatVel = 250.0;

    name = strPrefix;
    name += "EnableFlag";
    config->addParam(ArConfigArg(name.c_str(), &myEnableFlag,
				 "This flag when false will disable the \
slow down action without having to tune it off with the other parameters."), 
		     nameSection, ArPriority::DETAILED);

    name = strPrefix;
    name += "TransVelMin";
    config->addParam(ArConfigArg(name.c_str(), &myMinVel, 
				 "Minimum translation velocity when the ArServerClasses.cpp localization score drops to zero (this actually is lowerbounded at 100 now, but since the old default was 0 not raising the min... if you want it to actually stop change the LostTresholdDist and PassThreshold so it gets lost). This parameter is part of an action which slows down the robot when its localization score goes down from 1.0.",
				 0), nameSection, ArPriority::DETAILED);
    name = strPrefix;
    name += "RotVelMin";
    config->addParam(ArConfigArg(name.c_str(), &myMinRotVel, 
				 "Minimum rotational velocity when the localization score drops to zero (this actually is lowerbounded at 10 now, but since the old default was 0 not raising the min... if you want it to actually stop change the LostTresholdDist and PassThreshold so it gets lost). This parameter is part of an action which slows down the robot when its localization score goes down from 1.0.",
				 0), nameSection, ArPriority::DETAILED);

    /// SEEKUR (add this somehow)
    if (myRobot != NULL && myRobot->hasLatVel())
    {
      name = strPrefix;
      name += "LatVelMin";
      config->addParam(ArConfigArg(name.c_str(), &myMinLatVel, 
				   "Minimum lateral velocity when the ArServerClasses.cpp localization score drops to zero (this actually is lowerbounded at 100 now, but since the old default was 0 not raising the min... if you want it to actually stop change the LostTresholdDist and PassThreshold so it gets lost). This parameter is part of an action which slows down the robot when its localization score goes down from 1.0.",
				   0), nameSection, ArPriority::DETAILED);
    }
    name = strPrefix;
    name += "MaxThreshold";
    config->addParam(ArConfigArg(name.c_str(), &myMaxThreshold,
				 "Maximum localization score above \
which the robot is not slowed down. The TransVelMax will be used in \
this case. (The complementary min threshold is the pass threshold \
for localization)",
				 0, 1), nameSection, ArPriority::EXPERT);

    name = strPrefix;
    name += "Power";
    config->addParam(ArConfigArg(name.c_str(), &myPower,
				 "The degree to which the localization score \
is raised to compute the slow down speed when the localization score is \
between the min and max thresholds.",
				 1), nameSection, ArPriority::EXPERT);


    myProcessFileCB = new ArRetFunctorC<bool, 
    ArActionSlowDownWhenNotCertain> (this, 
		     &ArActionSlowDownWhenNotCertain::reconfigureAction);
    myProcessFileCB->setName("ArActionSlowDownWhenNotCertain");
    config->addProcessFileCB(myProcessFileCB, 40);
  }
  ArAction::setRobot(robot);
}
/*!  
 * Deactivates the action.
 *
 */
AREXPORT void
ArActionSlowDownWhenNotCertain::deactivate(void)
{
  ArAction::deactivate();
}

/*!
 * Core of the action.
 */
AREXPORT ArActionDesired*
ArActionSlowDownWhenNotCertain::fire(ArActionDesired /*currentDesired*/)
{
  myDesired.reset();

  if(!myEnableFlag)
    return NULL;

  double minVel = ArUtil::findMax(myMinVel, 100.0);
  double minRotVel = ArUtil::findMax(myMinRotVel, 10.0);
  double minLatVel = ArUtil::findMax(myMinLatVel, 100.0);

  if(myLocalizationTask)
  {
    //if(myLocalizationTask->getInitializedFlag())
    if (!myLocalizationTask->getRobotIsLostFlag())
    {
      myMinThreshold = myLocalizationTask->getLocalizationThreshold();
      double vel, rotVel = 25;
      double latVel;
      double score = myLocalizationTask->getLocalizationScore();
      bool setMaxes = true;
      if (myLocalizationTask->getState() == 
	  ArBaseLocalizationTask::LOCALIZATION_INIT_COMPUTING)
      {
	setMaxes = true;
	vel = 0;
	rotVel = 0;
	latVel = 0;
	ArLog::log(ArLog::Verbose, 
		   "SlowDownWhenUncertain: Stopping because of localization initializing", 
		   vel);
      }
      else if (score <= myMinThreshold)
      {
	setMaxes = true;
	vel = minVel;
	rotVel = minRotVel;
	latVel = minLatVel;
	if (myRobot->getVel() > vel)
	  ArLog::log(ArLog::Normal, "SlowDownWhenUncertain: Velocity %.0f because score %g < %g", 
		     vel, score, myMinThreshold);
	ArLog::log(ArLog::Verbose, "SlowDownWhenUncertain: Velocity %.0f because score %g < %g", 
		   vel, score, myMinThreshold);
      }
      else if (score >= myMaxThreshold)
      {
	setMaxes = false;
	vel = myMaxVel;
	rotVel = myMaxRotVel;
	latVel = myMaxLatVel;
      }
      else if(score <= myMaxThreshold && score >= myMinThreshold)
      {
	setMaxes = true;
	score = pow(score, myPower);
	vel = (myMaxVel - minVel)*score + minVel;
	rotVel = (myMaxRotVel - minRotVel)*score + minRotVel;
	latVel = (myMaxLatVel - minLatVel)*score + minLatVel;
	ArLog::log(ArLog::Verbose, 
		   "SlowDownWhenUncertain: Going to %.0f because of uncertainty", 
		   vel);
      } 
      else
      {
	ArLog::log(ArLog::Terse, "# Slow down bad state!");
      }

      if (setMaxes)
      {
	myDesired.setMaxVel(vel);
	myDesired.setMaxRotVel(rotVel);
	if (myRobot->hasLatVel())
	{
	  myDesired.setMaxLeftLatVel(vel);
	  myDesired.setMaxRightLatVel(vel);
	}
      }
    }
  }
  return &myDesired;
}

AREXPORT ArSimMapSwitcher::ArSimMapSwitcher(ArRobot *robot, ArMapInterface *arMap) :
  myMapChangedCB(this, &ArSimMapSwitcher::mapChanged)
{
  myRobot = robot;
  myMap = arMap;
  myMapChangedCB.setName("ArSimMapSwitcher");
  myMap->addMapChangedCB(&myMapChangedCB, ArListPos::FIRST);
}

AREXPORT ArSimMapSwitcher::~ArSimMapSwitcher()
{
  myMap->remMapChangedCB(&myMapChangedCB);
}

AREXPORT void ArSimMapSwitcher::mapChanged(void)
{
  // if its not the sim just return;
  if (myRobot->getOrigRobotConfig() == NULL ||
      !myRobot->getOrigRobotConfig()->hasPacketArrived() ||
      strcmp(myRobot->getOrigRobotConfig()->getSerialNumber(), "SIM") != 0)
    return;


  size_t fileNameLen = strlen(myMap->getFileName());

  if(fileNameLen == 0)
    return;

  struct sim_ctrl_map_t {
    ArTypes::Byte2 code;
    ArTypes::UByte2 len;
    char mapFile[190];
  };
  struct sim_ctrl_map_t cmd;
  cmd.code = 1;
  cmd.len = (ArTypes::UByte2) fileNameLen + 1;
  if ( fileNameLen + 1 > sizeof(cmd.mapFile))
  {
    ArLog::log(ArLog::Terse, 
	       "ArSimMapSwitcher: Cannot load map since its filename is too long");
    return;
  }
  ArLog::log(ArLog::Normal, "ArSimMapSwitcher: Having sim load map '%s'", 
	     cmd.mapFile);
  strncpy(cmd.mapFile, myMap->getFileName(), fileNameLen);
  myRobot->comDataN(ArCommands::SIM_CTRL, (const char*)&cmd, strlen(cmd.mapFile) + 1);
}


/**
   @param robot the robot to attach to and to write the pose from
   @param baseDirectory the base directory to write into
   @param addAriaExitCB whether to add the aria exit callback or not
   @param poseWriteIntervalInMSecs if this is non-zero then the class
   will call savePose every poseWriteInternvalInMSecs milliseconds
 **/
AREXPORT ArPoseStorage::ArPoseStorage(
	ArRobot *robot, const char *baseDirectory, 
	int poseWriteIntervalInMSecs, bool addAriaExitCB) :
  myAriaExitCB(this, &ArPoseStorage::savePose),
  myUserTaskCB(this, &ArPoseStorage::userTask)
{
  myRobot = robot;
  if (baseDirectory != NULL && strlen(baseDirectory) > 0)
    myBaseDir = baseDirectory;
  else
    myBaseDir = "";
  myAriaExitCB.setName("ArPoseStorageExit");
  if (addAriaExitCB)
    Aria::addExitCallback(&myAriaExitCB, 100);
  myWriteInterval = poseWriteIntervalInMSecs;
  myLastWrite.setToNow();
  myUserTaskCB.setName("ArPoseStorage");
  myRobot->addUserTask("ArPoseStorage", 50, &myUserTaskCB);
  mySaving = false;
}

AREXPORT ArPoseStorage::~ArPoseStorage()
{

}

AREXPORT bool ArPoseStorage::savePose(void)
{
  FILE *file;
  std::string realFileName;
  std::string tmpFileName;
  myLastWrite.setToNow();
  if ((file = ArUtil::fopen(myTmpFileName.c_str(), "w")) == NULL)
  {
    ArLog::log(ArLog::Terse, 
	   "ArPoseStorage::savePose: Could not open file %s for writing",
	       tmpFileName.c_str());
    return false;
  }

  fprintf(file, "%.0f %.0f %.0f\n", myRobot->getX(), myRobot->getY(), 
	  myRobot->getTh());
  fclose(file);
#ifdef WIN32
  // rename() will fail on Windows if the old file still exists. 
  unlink(myRealFileName.c_str());
#endif
  if (rename(myTmpFileName.c_str(), myRealFileName.c_str()) != 0)
  {
    ArLog::log(ArLog::Terse, 
   "ArPoseStorage::savePose: Could not move temp file %s to real file %s (errno %d)",
	       myTmpFileName.c_str(), myRealFileName.c_str(), errno);
    if(errno == EACCES)
      ArLog::log(ArLog::Terse, "ArPoseStorage::savePose: New file %s already exists or could not be written.", myRealFileName.c_str());
    return false;
  }

  return true;
}

AREXPORT bool ArPoseStorage::restorePose(const char *fileName)
{
  FILE *file;

  char line[10000];

  if (fileName != NULL || fileName[0] == '\0')
    myFileName = fileName;
  else
    myFileName = "robotPose";

  if (myFileName[0] == '/' || myFileName[0] == '\\')
  {
    myRealFileName = myFileName;
  }
  else
  {
    myRealFileName = myBaseDir;
    myRealFileName += myFileName;
  }
  myTmpFileName = myRealFileName + ".tmp";

  ArLog::log(ArLog::Normal, 
	     "Opening pose file for reading %s from fileName given %s", 
	     myRealFileName.c_str(), myFileName.c_str());
  
  if ((file = ArUtil::fopen(myRealFileName.c_str(), "r")) == NULL)
  {
    ArLog::log(ArLog::Terse, 
      "ArPoseStorage::restorePose: Could not open file %s for reading", 
	       myRealFileName.c_str());
    return false;
  }

  ArArgumentBuilder builder;
  if (fgets(line, sizeof(line), file) != NULL)
  {
    
    builder.add(line);
    fclose(file);
    //unlink(fileName);
    if (builder.isArgDouble(0) && builder.isArgDouble(1) && 
	builder.isArgDouble(2))
    {
      ArLog::log(ArLog::Normal, "Restoring pose to %.0f %.0f %.0f from line %s", 
		 builder.getArgDouble(0), builder.getArgDouble(1), 
		 builder.getArgDouble(2), line);
      myRobot->moveTo(ArPose(builder.getArgDouble(0), builder.getArgDouble(1), 
			     builder.getArgDouble(2)));
      mySaving = true;
      return true;
    }
    else
    {
      ArLog::log(ArLog::Terse, 
"ArPoseStorage::restorePose: Bad line '%s' (should be x, y, th)", 
		 line);
      mySaving = true;
      return false;
    }
  }
  fclose(file);
  //unlink(fileName);
  ArLog::log(ArLog::Terse, "ArPoseStorage::restorePose: Empty file");
  mySaving = true;
  return false;
  
}

AREXPORT void ArPoseStorage::userTask(void)
{
  if (mySaving && myWriteInterval > 0 && 
      myLastWrite.mSecSince() > myWriteInterval)
    savePose();
}
