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
#include "ArDocking.h"
#include "ArPathPlanningInterface.h"
#include "ArLocalizationTask.h"

/* -------- Dock: --------- */

// TODO make something so that docks are set at startup where possible... probably in AramServer (what to do for research?)

AREXPORT ArServerModeDock *ArServerModeDock::createDock(
	ArServerBase *serverBase, ArRobot *robot, ArLocalizationTask *locTask, 
	ArPathPlanningInterface *pathTask, ArFunctor *shutdownFunctor)
{
  if (!robot->isConnected())
  {
    ArLog::log(ArLog::Normal, "ArServerModeDock::createDock: Robot is not connected, cannot create docking mode for specific robot.");
    return NULL;
  }

  ArServerModeDock *modeDock = NULL;
  const ArRobotConfigPacketReader *origConfig;
  if (ArUtil::strcasecmp(robot->getRobotName(), "simler") == 0 || 
      ArUtil::strcasecmp(robot->getRobotName(), "MobileSim") == 0)
  {
    ArLog::log(ArLog::Normal, "Creating simulator dock mode");
    modeDock = new ArServerModeDockSimulator(serverBase, robot, locTask, 
               pathTask, shutdownFunctor);
  }
  else if( ArUtil::strcasecmp(robot->getRobotType(), "mtx") == 0 )
  {
    if ( ArUtil::strcasecmp(robot->getRobotSubType(), "pioneer-lx") == 0 ||
      ArUtil::strcasecmp(robot->getRobotSubType(), "lx") == 0 ||
      ArUtil::strcasecmp(robot->getRobotSubType(), "marc_devel") == 0 ||
      ArUtil::strcasecmp(robot->getRobotSubType(), "lynx") == 0 
    )
    {
      ArLog::log(ArLog::Normal, "Creating LX/Lynx dock mode");
      modeDock = new ArServerModeDockLynx(serverBase, robot, locTask, pathTask, shutdownFunctor);
    }
  }
  else if ((origConfig = robot->getOrigRobotConfig()) != NULL)
  {
    if (origConfig->getHasCharger() == 6)
    {
      ArLog::log(ArLog::Normal, "Creating patrolbot NiMH dock mode");
      modeDock = new ArServerModeDockPatrolBotNiMH(serverBase, robot, locTask, 
						   pathTask, shutdownFunctor);
    }
    else if (origConfig->getHasCharger() == 3)
    {
      ArLog::log(ArLog::Normal, "Creating patrolbot dock mode");
      modeDock = new ArServerModeDockPatrolBot(serverBase, robot, locTask, 
					       pathTask, shutdownFunctor);
    }
    else if (origConfig->getHasCharger() == 2)
    {
      ArLog::log(ArLog::Normal, "Creating old powerbot dock mode");
      modeDock = new ArServerModeDockPowerBot(serverBase, robot, locTask, 
					      pathTask, true, 
					      shutdownFunctor, false, 0);
    }
    else if (origConfig->getHasCharger() == 4)
    {
      ArLog::log(ArLog::Normal, "Creating new ARCOS powerbot dock mode");
      modeDock = new ArServerModeDockPowerBot(serverBase, robot, locTask, 
					      pathTask, false, 
					      shutdownFunctor, false, 0);
    }
    else if (origConfig->getHasCharger() == 5)
    {
      ArLog::log(ArLog::Normal, "Creating new uARCS powerbot dock mode");
      modeDock = new ArServerModeDockPowerBot(serverBase, robot, locTask, 
					      pathTask, false, 
					      shutdownFunctor, true, 0);
    }
    else if (origConfig->getHasCharger() == 1)
    {
      ArLog::log(ArLog::Normal, "Creating pioneer dock mode");
      modeDock = new ArServerModeDockPioneer(serverBase, robot, locTask, 
					     pathTask, shutdownFunctor);
    }
  }
  return modeDock;
}

AREXPORT ArServerModeDock::ArServerModeDock(ArServerBase *server, 
					    ArRobot *robot, 
					    ArLocalizationTask *locTask, 
					    ArPathPlanningInterface *pathTask, 
					    bool useChargeState, 
					    ArFunctor *shutdownFunctor) : 
  ArServerMode(robot, server, "dock"),
  myDockMapChangedCB(this, &ArServerModeDock::dockMapChanged),
  myDockUserTaskCB(this, &ArServerModeDock::dockUserTask),
  myServerDockCB(this, &ArServerModeDock::serverDock),
  myServerUndockCB(this, &ArServerModeDock::serverUndock),
  myServerDockInfoCB(this, &ArServerModeDock::serverDockInfo),
  myServerGetAutoDockCB(this, &ArServerModeDock::serverGetAutoDock),
  myServerSetAutoDockCB(this, &ArServerModeDock::serverSetAutoDock),
  myServerDockingAtToServerCB(this, &ArServerModeDock::serverDockingAtToServer),
  myServerDockingAtFromServerCB(this, &ArServerModeDock::serverDockingAtFromServer),
  myServerAutoDockingDisableCB(this, &ArServerModeDock::setAutoDock, false),
  myServerAutoDockingEnableCB(this, &ArServerModeDock::setAutoDock, true),
  myServerDockInfoPopupTerseCB(this, &ArServerModeDock::dockInfoPopup, true),
  myServerDockInfoPopupVerboseCB(this, &ArServerModeDock::dockInfoPopup, 
				 false),
  myProcessFileCB(this, &ArServerModeDock::processFile),
  myWriteDockFileCB(this, &ArServerModeDock::writeDockFile),
  myEraseDockFileCB(this, &ArServerModeDock::eraseDockFile),
  myDefaultDistanceCB(ArPose::distanceBetween)
{
  myMode = "Dock";
  myLocTask = locTask;
  myPathTask = pathTask;
  myUseChargeState = useChargeState;
  myShutdownFunctor = shutdownFunctor;
  myGetOverrideMapNameFunctor = NULL;

  myDistanceCB = &myDefaultDistanceCB;

  myDockInfoLogLevel = ArLog::Verbose;
  myDockModeLogLevel = ArLog::Verbose;
  
  myDockInfoMutex.setLogName("ArServerModeDock::dockInfoMutex");

  myForcedDockCBList.setName("ArServerModeDock::forcedDockCBList");
  myIdleDockCBList.setName("ArServerModeDock::idleDockCBList");
  myRequestedDockCBList.setName("ArServerModeDock::requestedDockCBList");
  myDockedCBList.setName("ArServerModeDock::dockedCBList");
  myUndockingCBList.setName("ArServerModeDock::undockingCBList");
  myUndockedCBList.setName("ArServerModeDock::undockedCBList");
  myDockNowUnforcedCBList.setName("ArServerModeDock::dockNowUnforcedCBList");
  myDockNowForcedCBList.setName("ArServerModeDock::dockNowForcedCBList");
  mySingleShotDockedCBList.setName("ArServerModeDock::singleShotDockedCBList");
  mySingleShotDockedCBList.setSingleShot(true);

  myDrivingToDockCBList.setName("ArServerModeDock::drivingToDockCBList");
  myDrivingIntoDockCBList.setName("ArServerModeDock::drivingIntoDockCBList");

  addDrivingIntoDockCB(&myWriteDockFileCB, 100);
  addDockedCB(&myWriteDockFileCB, 100);
  addUndockedCB(&myEraseDockFileCB, 100);

  setDockFileName("dockFile");
  setDockBaseDirectory("");

  myRobot->addUserTask("dock", 50, &myDockUserTaskCB);
  if (myServer != NULL)
  {
    addModeData("dock", "sends the robot to the dock", 
			  &myServerDockCB,
			  "ubyte: 0 normal unforced dock (default), 1 forced dock (for internal use)", "none", "Navigation", "RETURN_NONE");
    addModeData("undock", "undocks the robot", &myServerUndockCB,
		"none", "none", "Navigation", "RETURN_NONE");
    myServer->addData("dockInfo", "get info about the docking state", 
			  &myServerDockInfoCB, "none", 
			  "ubyte: 0 undocked, 1 docking, 2 docked, 3 undocking; ubyte: 0 docking not forced, 1 docking forced (can't undock now); ubyte2: shutdown_countdown, 0 means no shutdown imminent, any other value means a shutdown will occur in that many seconds",
		      "NavigationInfo", "RETURN_SINGLE");
    myServer->addData("dockInfoChanged", 
											"Broadcast when the docking state changes", 
			 								NULL, 
											"none", 
			  						  "ubyte: 0 docked, 1 docking, 2 docked, 3 undocking; ubyte: 0 docking not forced, 1 docking forced (can't undock now); ubyte2: shutdown_countdown, 0 means no shutdown imminent, any other value means a shutdown will occur in that many seconds",
		      						"NavigationInfo", 
											"RETURN_SINGLE"); 
    myServer->addData("getAutoDock", "gets whether we auto dock or not",
		      &myServerGetAutoDockCB, "none", 
		      "ubyte: 0 no auto docking, 1 autodocking",
		      "NavigationInfo", "RETURN_SINGLE");
    myServer->addData(
	    "setAutoDock", "Sets whether we auto dock or not",
	    &myServerSetAutoDockCB, 
	    "ubyte: 0 = no autdocking, anything else = auto docking",
	    "none",
	    "Navigation", "RETURN_NONE");

    myServer->addData(
	    "dockingAtToServer", "Gets which dock the robot is using...  Whenever the dock this robot is using changes this gets broadcast, it can be requested once to get the current data",
	    &myServerDockingAtToServerCB,
	    "none", "str mapName; str dockName; str overrideMapName; byte4: x; byte4: y; byte2 th;", 
	    "MultiRobotCoordination", "RETURN_NONE");

    myServer->addData(
	    "dockingAtFromServer", "",
	    &myServerDockingAtFromServerCB,
	    "none", "uByte2 numRobots; repeating for numRobots (str robotName; str mapName; str dockName; str overrideMapName; byte4: x; byte4: y; byte2 th;)", 
	    "MultiRobotCoordination", "RETURN_NONE");

  }
  myDockingVoltage = 0;
  myDockingStateOfCharge = 0;
  myDockingIdleTime = 5;

  myDoneChargingVoltage = 0;
  myDoneChargingStateOfCharge = 0;
  if (myUseChargeState)
  {
    myDoneChargingAtFloat = true;
    myDoneChargingMinutes = 0;
  }
  else
  {
    myDoneChargingAtFloat = false;
    myDoneChargingMinutes = 60;
  }
  myMinimumMinutesBetweenAutoDock = 0;
  clearInterrupted();
  myForcedDock = false;
  myIsForcedDockAvailable = true;
  myIsAutoDockAvailable = true;
  myGoToDockSent = false;
  myLastForcedDock = false;
  myForcedDockRequested = false;
  myIdleDock = false;
  myFailedGotoNum = 0;
  myFailedDriveInNum = 0;
  myFailedDriveInIgnoreMinutes = 60;
  myFailedDriveInAttemptsBeforeIgnore = 3;
  myHandlerCommands = NULL;
  myHaveDocked = false;
  myHaveBeenNotDocked = false;
  myAutoDock = false;
  myLastAutoDock = false;
  myUsingAutoDock = false;
  myState = UNDOCKED;
  myLastState = UNDOCKED;

  myPreferredDock[0] = '\0';
  myPreferredDockMapObject = NULL;
  myOnlyUsePreferredDock = false;

  checkDock();
  myStartedState.setToNow();
  myProcessFileCB.setName("ArServerModeDock");
  //myWasChargerPowerGood = (myRobot->getFlags() & ArUtil::BIT10);
  myWasChargerPowerGood = myRobot->isChargerPowerGood();
  
  myShutdownMinutesIdle = 0;
  myShutdownMinutesForced = 0;
  myShuttingDownSeconds = 0;
  myLastShuttingDownSeconds = 0;

  myFoundDockMapName = "";
  myFoundDockOverrideMapName = "";
  myFoundDockMapObject = NULL;

  myLogLockedFirst = true;
  
  myDockMapChangedCB.setName("ArServerModeDock");
  if (myPathTask->getAriaMap() != NULL)
    myPathTask->getAriaMap()->addMapChangedCB(&myDockMapChangedCB);
  myDocksMutex.setLogName("ArServerModeDock::myDocksMutex");
}

AREXPORT ArServerModeDock::~ArServerModeDock()
{
  myRobot->remUserTask(&myDockUserTaskCB);
}

AREXPORT void ArServerModeDock::addControlCommands(
	ArServerHandlerCommands *handlerCommands)
{
  myHandlerCommands = handlerCommands;
  myHandlerCommands->addCommand(
	  "autoDockDisable",
	  "This disables the auto docking and undocking",
	  &myServerAutoDockingDisableCB);
  myHandlerCommands->addCommand(
	  "autoDockEnable",
	  "This enables the auto docking and undocking",
	  &myServerAutoDockingEnableCB);
}

AREXPORT void ArServerModeDock::addInfoCommands(
	ArServerHandlerCommands *handlerCommands, 
	ArServerHandlerPopup *handlerPopup)
{
  myHandlerCommands = handlerCommands;
  myHandlerPopup = handlerPopup;
  myHandlerCommands->addCommand(
	  "DockInfoPopupTerse",
	  "Pops up a window with important information about docking",
	  &myServerDockInfoPopupTerseCB);
  myHandlerCommands->addCommand(
	  "DockInfoPopupVerbose",
	  "Pops up a window with all information about docking",
	  &myServerDockInfoPopupVerboseCB);
}

void ArServerModeDock::dockInfoPopup(bool terse)
{
  if (!myHandlerPopup)
  {
    ArLog::log(ArLog::Terse, 
	 "ArServerModeDock::dockInfoPopup: called when popupHandler is NULL");
    return;
  }

  myDockInfoMutex.lock();

  std::string body;
  body = "";
  
  if (!myDockInfoForcedDockDesired.empty())
  {
    body += myDockInfoForcedDockDesired;
    body += "\n";
  }

  if (!terse && !myDockInfoDockDesired.empty())
  {
    body += myDockInfoDockDesired;
    body += "\n";
  }

  if (!terse && !myDockInfoForcedDockAchieved.empty())
  {
    body += myDockInfoForcedDockAchieved;
    body += "\n";
  }

  if (!terse && !myDockInfoDockAchieved.empty()) 
  {
    body += myDockInfoDockAchieved;
    body += "\n";
  }

  if (!myDockInfoUndockingFromForced.empty())
  {
    body += myDockInfoUndockingFromForced;
    body += "\n";
  }

  if (!terse && !myDockInfoUndocking.empty())
  {
    body += myDockInfoUndocking;
    body += "\n";
  }
  
  char buf[1024];
  buf[0] = '\0';
  
  subclassGetDockInfoString(buf, sizeof(buf), terse);
  
  body += buf;

  if (body.empty())
    body = "No dock info yet";

  myDockInfoMutex.unlock();
    
  ArServerHandlerPopupInfo popupInfo(
	  NULL, "Dock info popup", body.c_str(), 
	  ArServerHandlerPopup::INFORMATION,
	  0, 0, 0, NULL, "OK", "Done viewing dock info");


  myHandlerPopup->createPopup(&popupInfo);  

}

AREXPORT void ArServerModeDock::dockUserTask(void)
{
  ArMapInterface *arMap;
  if (myIsActive && myState == DOCKED)
  {
    myLastDocked.setToNow();
    myHaveDocked = true;
  } 
  else if (myIsActive && myState != DOCKED)
  {
    myLastNotDocked.setToNow();
    myHaveBeenNotDocked = true;
  }

  if (myIsActive && (myState == DOCKING || myState == UNDOCKING))
  {
    setActivityTimeToNow();
  }

  double dockingVoltage;
  double dockingStateOfCharge;
  // if our docking voltage is set to auto figure it out then do that
  if (!myRobot->haveStateOfCharge())
  {
    if (myDockingVoltage < .1)
    {
      if (myRobot->getOrigRobotConfig() != NULL && 
	  myRobot->getOrigRobotConfig()->hasPacketArrived() && 
	  myRobot->getOrigRobotConfig()->getLowBattery() > .1)
	dockingVoltage = myRobot->getOrigRobotConfig()->getLowBattery() * .1;
      else
	dockingVoltage = 12;
    }
    // otherwise just use the one we have
    else
      dockingVoltage = myDockingVoltage;
  }
  else
  {
    if (myDockingStateOfCharge < .1)
    {
      if (myRobot->getOrigRobotConfig() != NULL && 
	  myRobot->getOrigRobotConfig()->hasPacketArrived() && 
	  myRobot->getOrigRobotConfig()->getStateOfChargeLow() > .1)
	dockingStateOfCharge = myRobot->getOrigRobotConfig()->getStateOfChargeLow();
      else
	dockingStateOfCharge = 20;
    }
    // otherwise just use the one we have
    else
      dockingStateOfCharge = myDockingStateOfCharge;
  }

  // dock if auto docking is enabled and we're below docking voltage
  // and its been at least the minimum time between docks (or we
  // haven't docked)...  
  // BIT10 is replaced by the check
  // down below to myRobot->isChargerPowerGood...  it's done this way now so
  // that we can let folks know if this is happening and since the
  // charging on the MPX line isn't through the normal microcontroller  
  bool subclassCausedAutoDock = false;
  char subclassNeedsAutoDockReason[1024];
  if (!myIsActive && myUsingAutoDock && //!myServer->idleProcessingPending() &&
      //!(myRobot->getFlags() & ArUtil::BIT10) &&
      ((!myRobot->haveStateOfCharge() && 
	myRobot->getRealBatteryVoltage() < dockingVoltage) || 
       (myRobot->haveStateOfCharge() && 
	myRobot->getStateOfCharge() < dockingStateOfCharge) || 
       myForcedDockRequested || 
       (subclassCausedAutoDock = subclassNeedsAutoDock(
	       subclassNeedsAutoDockReason, 
	       sizeof(subclassNeedsAutoDockReason)))) &&
      (myLastDocked.secSince() > myMinimumMinutesBetweenAutoDock * 60 ||
       !myHaveDocked) && 
      (arMap = myPathTask->getAriaMap()) != NULL)
  {
    if (myRobot->isChargerPowerGood())
    {
      if (myLastOnChargerComplaint.secSince() >= 60)
      {
	ArLog::log(ArLog::Normal, "Docking not autodocking since the robot is already on a charger");
	myLastOnChargerComplaint.setToNow();
      }
    }
    else if (findDock(arMap) != NULL)
    {
      myForcedDock = true;
      activate();
      if (myIsActive)
      {
	lockMode(false);
	if (subclassCausedAutoDock && subclassNeedsAutoDockReason[0] != '\0')
	{
	  ArLog::log(ArLog::Normal, 
		     subclassNeedsAutoDockReason);
	}
	else if (myForcedDockRequested)
	  ArLog::log(ArLog::Normal,
		     "Docking because forced dock requested");
	else if (!myRobot->haveStateOfCharge())
	  ArLog::log(ArLog::Normal,
		     "Docking because voltage is %.1f (docks at %.1f)",
		     myRobot->getRealBatteryVoltage(), dockingVoltage);
	else 
	  ArLog::log(ArLog::Normal,
	     "Docking because state of charge is %.1f%% (docks at %.1f%%)",
		     myRobot->getStateOfCharge(), dockingStateOfCharge);
      }
      myForcedDockRequested = false;
    }
  }

  // if we're docking already but are unforced, see if it becomes
  // forced on the way (otherwise if there's no idle shutdown we may
  // wind up discharging the batteries)
  if (myIsActive && myUsingAutoDock && myState == DOCKING && !myForcedDock  &&
      ((!myRobot->haveStateOfCharge() && 
	myRobot->getRealBatteryVoltage() < dockingVoltage) || 
       (myRobot->haveStateOfCharge() && 
	myRobot->getStateOfCharge() < dockingStateOfCharge) || 
       myForcedDockRequested))
  {
    myForcedDockRequested = false;
    myForcedDock = true;
    if (myIsActive)
    {
      lockMode(false);
      if (myForcedDockRequested)
	ArLog::log(ArLog::Normal,
		   "Making docking forced because forced dock requested");
      else if (!myRobot->haveStateOfCharge())
	ArLog::log(ArLog::Normal,
		   "Making docking forced because voltage is %.1f (docks at %.1f)",
		   myRobot->getRealBatteryVoltage(), dockingVoltage);
      else 
	ArLog::log(ArLog::Normal,
		   "Making docking forced because state of charge is %.1f%% (docks at %.1f%%)",
		   myRobot->getStateOfCharge(), dockingStateOfCharge);
    }
  }

  // dock if auto docking is enabled and not docked and we're below
  // docking voltage and its been at least the minimum time between
  // docks (or we haven't docked).... BIT10 is replaced by the check
  // down below to myRobot->isChargerPowerGood...  it's done this way now so
  // that we can let folks know if this is happening and since the
  // charging on the MPX line isn't through the normal microcontroller  
  if (!myIsActive &&  myUsingAutoDock && //!myServer->idleProcessingPending() &&
      //!(myRobot->getFlags() & ArUtil::BIT10) && 
      myDockingIdleTime > .0000001 && 
      getActiveMode() != NULL && getActiveMode()->hasSetActivityTime() &&
      getActiveMode()->getActivityTime().secSince() >= myDockingIdleTime * 60 &&
      (myLastDocked.secSince() > myMinimumMinutesBetweenAutoDock * 60 ||
       !myHaveDocked) && 
      (arMap = myPathTask->getAriaMap()) != NULL)
  {
    if (myRobot->isChargerPowerGood()) 
    {
      if (myLastOnChargerComplaint.secSince() >= 60)
      {
	ArLog::log(ArLog::Normal, "Docking not idle docking since the robot is already on a charger");
	myLastOnChargerComplaint.setToNow();
      }
    }
    else if (findDock(arMap) != NULL)
    {
      myIdleDock = true;
      ArTime activityTime = getActiveMode()->getActivityTime();
      activate();
      if (isActive())
	
	ArLog::log(ArLog::Normal,
		   "Docking because idle for %d minutes %d seconds (docks at %g minutes idle)",
		   activityTime.secSince() / 60, 
		   activityTime.secSince() % 60, 
		   myDockingIdleTime);
    }
  }
  if (!myUsingAutoDock && myForcedDock)
  {
    myForcedDock = false;
    lockMode(true);
    stateChanged();
  }
  // if we're to a high enough voltage undock or for long enough time
  // or at float
  if (myIsActive && myForcedDock && myState == DOCKED && 
      !subclassNotDoneCharging())
  {
    if (subclassDoneCharging())
    {
      // the sub class should have logged why it was done charging...
      myForcedDock = false;
      lockMode(true);
    }
    if (myDoneChargingMinutes == -1)
    {
      ArLog::log(ArLog::Normal, 
	 "Done docking because MinutesToChargeFor set to debug value (-1)");
      myForcedDock = false;
      lockMode(true);
    }
    else if (myUseChargeState && myDoneChargingAtFloat && 
	myRobot->getChargeState() == ArRobot::CHARGING_FLOAT)
    {
      ArLog::log(ArLog::Normal, 
	 "Done docking because charging finished (got to float)");
      myForcedDock = false;
      lockMode(true);
    }
    else if (!myRobot->haveStateOfCharge() && myDoneChargingVoltage > 0.1 && 
	     myRobot->getRealBatteryVoltage() > myDoneChargingVoltage)
    {
      ArLog::log(ArLog::Normal, 
		 "Done docking because voltage is %.1f (finishes at %.1f)", 
		 myRobot->getRealBatteryVoltage(), myDoneChargingVoltage);
      myForcedDock = false;
      lockMode(true);
    }
    else if (myRobot->haveStateOfCharge() && 
	     myDoneChargingStateOfCharge > 0.1 && 
	     myRobot->getStateOfCharge() > myDoneChargingStateOfCharge)
    {
      ArLog::log(ArLog::Normal, 
		 "Done docking because state of charge is %.1f%% (finishes at %.1f%%)", 
		 myRobot->getStateOfCharge(), myDoneChargingStateOfCharge);
      myForcedDock = false;
      lockMode(true);
    }
    else if (myDoneChargingMinutes > 0 && 
	     myStartedState.secSince() > myDoneChargingMinutes * 60)
    {
      ArLog::log(ArLog::Normal, 
		 "Done docking because docked for %d minutes", 
		 myDoneChargingMinutes);
      myForcedDock = false;
      lockMode(true);
    }
    // now see if we should undock
    if (!myForcedDock)
    {
      // first just change the string so its clear we're not forced anymore
      myStatus = "Docked";
      stateChanged();
      resumeInterrupted(false);
    }
  }

  if (myShutdownFunctor != NULL && myIsActive && myState == DOCKING)
  {
    int secSince;

    if (myShutdownLastPose.squaredFindDistanceTo(myRobot->getPose()) > 20 * 20)
    {
      myShutdownLastPose = myRobot->getPose();
      myShutdownLastMove.setToNow();
      //printf("1\n");
      if (myShuttingDownSeconds > 0)
      {
	ArLog::log(ArLog::Normal, "ServerModeDock: Robot moved, shutdown cancelled");
	myShuttingDownSeconds = 0;
      }
    }
    else if ((myForcedDock && myShutdownMinutesForced > 0 && 
	      myShutdownLastMove.secSince() >= 
	       myShutdownMinutesForced * 60 + 30) ||
	     (!myForcedDock && myShutdownMinutesIdle > 0 && 
	      myShutdownLastMove.secSince() >= 
	       myShutdownMinutesIdle * 60 + 30))
    {
      //printf("2\n");
      if (myForcedDock)
	ArLog::log(ArLog::Normal, "ServerModeDock: Shutting down since have not made progress towards the dock (forced) in %d minutes", myShutdownMinutesForced);
      else
	ArLog::log(ArLog::Normal, "ServerModeDock: Shutting down since have not made progress towards the dock (idle) in %d minutes", myShutdownMinutesIdle);
      myShutdownFunctor->invoke();
    }
    else if ((secSince = myShutdownLastMove.secSince()) > 30)
    {
      //printf("3 %d %d %d\n", myForcedDock, secSince, myShuttingDownSeconds);
      bool wasShuttingDown = (myShuttingDownSeconds != 0);
      if (myForcedDock && myShutdownMinutesForced > 0)
	myShuttingDownSeconds = myShutdownMinutesForced * 60 + 30 - secSince;
      else if (!myForcedDock && myShutdownMinutesIdle > 0)
	myShuttingDownSeconds = myShutdownMinutesIdle * 60 + 30 - secSince;
      if (!wasShuttingDown && myShuttingDownSeconds > 0)
	ArLog::log(ArLog::Normal, "ServerModeDock: Robot has not moved in 30 seconds while trying to dock (%s), shutting down in %d seconds", 
		   myForcedDock ? "forced" : "idle",
		   myShuttingDownSeconds);
    }
    //printf("4 %d\n", myShuttingDownSeconds);
    if (myShuttingDownSeconds != myLastShuttingDownSeconds)
      broadcastDockInfoChanged();
    myLastShuttingDownSeconds = myShuttingDownSeconds;
  }


  // now we see if we're undocked but our robot started charging suddently, if
  // so we call checkDock
  if (myState == UNDOCKED && !myWasChargerPowerGood && 
      // (myRobot->getFlags() & ArUtil::BIT10))
      myRobot->isChargerPowerGood())
  {
    checkDock();
  }
  
  //myWasChargerPowerGood = (myRobot->getFlags() & ArUtil::BIT10);
  myWasChargerPowerGood = myRobot->isChargerPowerGood();

}

AREXPORT void ArServerModeDock::serverDock(ArServerClient *client, ArNetPacket * packet)
{
  int requestForcedDockByte = 0;

  if (packet->getDataLength() - packet->getDataReadLength() > 0)
    requestForcedDockByte = packet->bufToUByte();

  if (requestForcedDockByte != 1)
  {
    ArLog::log(ArLog::Normal, "Docking for %s", client->getIPString());
    myRobot->lock();
    dock();
    myRobot->unlock();
  }
  else
  {
    if (myUsingAutoDock)
      ArLog::log(ArLog::Normal, "Forced docking for %s", client->getIPString());
    else if (!myForcedDockRequested)
      ArLog::log(ArLog::Normal, "Forced docking requested for %s", client->getIPString());
    else if (myIsActive && !myForcedDock) 
      ArLog::log(ArLog::Normal, "Forced docking now for %s", client->getIPString());
    myRobot->lock();
    requestForcedDock();
    myRobot->unlock();
  }
}

AREXPORT void ArServerModeDock::serverUndock(ArServerClient *client, ArNetPacket * /*packet*/)
{
  ArLog::log(ArLog::Normal, "Undocking for %s", client->getIPString());
  myRobot->lock();
  undock();
  myRobot->unlock();
}

AREXPORT void ArServerModeDock::serverDockInfo(ArServerClient *client, ArNetPacket * /*packet*/)
{
  ArNetPacket send;
  myRobot->lock();
  makeDockInfoPacket(&send);
  myRobot->unlock();
  client->sendPacketTcp(&send);
}


AREXPORT void ArServerModeDock::serverGetAutoDock(ArServerClient *client, ArNetPacket * /*packet*/ )
{
  ArNetPacket send;
  myRobot->lock();
  if (myUsingAutoDock)
    send.uByteToBuf(1);
  else
    send.uByteToBuf(0);
  myRobot->unlock();
  client->sendPacketTcp(&send);
}

AREXPORT void ArServerModeDock::serverSetAutoDock(ArServerClient *client, ArNetPacket *packet)
{
  ArNetPacket send;
  bool autoDock;
  if (packet->bufToUByte() == 0)
  {
    ArLog::log(ArLog::Normal, "Clearing autodock for %s", 
	       client->getIPString());
    autoDock = false;
  }
  else
  {
    ArLog::log(ArLog::Normal, "Setting autodock for %s", 
	       client->getIPString());
    autoDock = true;
  }
  myRobot->lock();
  // MPL made this use the actual function so the virtual stuff can happen
  //myUsingAutoDock = autoDock;
  setAutoDock(autoDock);
  myRobot->unlock();
}

AREXPORT void ArServerModeDock::activate()
{
  bool idleDock = myIdleDock;
  myIdleDock = false;
  if (myIsActive)
    return;

  ArServerMode *activeMode = getActiveMode();
  ArServerMode *lastActiveMode = getLastActiveMode();

  // if we aren't the active mode, the active mode is locked, and the
  // active mode won't unlock... then just return (otherwise we spam
  // the logs enough to make them useless)
  if (activeMode != NULL && activeMode != this && ArServerMode::isLocked() && 
      !ArServerMode::willUnlockIfRequested())
  {
    if (myLogLockedFirst || myLogLockedTime.secSince() >= 60)
    {
      ArLog::log(ArLog::Normal, "Dock mode cannot activate since another mode (%s) is locked and will not unlock if requested", activeMode->getName());
      myLogLockedTime.setToNow();
      myLogLockedFirst = false;
    }
    return;
  }

  if (activeMode == NULL && lastActiveMode == ArServerMode::getIdleMode())
  {
    ArLog::log(myDockModeLogLevel, "DockMode: activeMode was NULL, but ourLastActiveMode was idle, so using that");
    activeMode = lastActiveMode;
  }

  if (activeMode != NULL)
    ArLog::log(myDockModeLogLevel, "DockMode: %s was activeMode", 
	       activeMode->getName());
  if (activeMode != NULL && activeMode == ArServerMode::getIdleMode())
  {
    activeMode = ArServerMode::getIdleMode()->getModeInterrupted();
    if (activeMode != NULL)
      ArLog::log(myDockModeLogLevel, "DockMode: Made %s new activeMode", 
		 activeMode->getName());
    else
      ArLog::log(myDockModeLogLevel, "DockMode: NULL new activeMode...");
  }

  if (myForcedDock && activeMode != NULL)
  {
    //printf("Interrupted %s\n", getActiveMode()->getName());
    if (activeMode->isAutoResumeAfterInterrupt()) 
    {
      myModeInterrupted = activeMode;
      ArLog::log(myDockModeLogLevel, "DockMode: Made %s new modeInterrupted",
		 myModeInterrupted->getName());
    }
  }
  else
  {
    clearInterrupted();
  }
  if (!baseActivate())
  {
    ArLog::log(myDockModeLogLevel, 
	       "DockMode: Couldn't activate, clearing interrupted");
    clearInterrupted();
    return;
  }

  // reset the last move time since we're truly activating from not
  // being active
  myShutdownLastPose = myRobot->getPose();
  myShutdownLastMove.setToNow();
  
  if (myHaveDocked)
  {
    long secs;
    secs = myLastDocked.secSince();

    char buf[1024]; 

    myDockInfoMutex.lock();

    if (myForcedDock)
    {
      sprintf(buf, 
	      "DockInfo:ForcedDockDesired: Last docked %02ld:%02ld:%02ld ago",
	      secs / 3600, (secs % 3600) / 60, secs % 60);
      myDockInfoForcedDockDesired = buf;
    }
    else
    {
      sprintf(buf, 
	      "DockInfo:DockDesired: Last docked %02ld:%02ld:%02ld ago",
	      secs / 3600, (secs % 3600) / 60, secs % 60);
      myDockInfoDockDesired = buf;
    }
    
    myDockInfoMutex.unlock();

    ArLog::log(myDockModeLogLevel, buf);
  }

  lockMode(true);
  if (myState != DOCKED)
  {
    if (myForcedDock)
      myForcedDockCBList.invoke();
    else if (idleDock)
      myIdleDockCBList.invoke();
    else 
      myRequestedDockCBList.invoke();
    dock();
  }
}

AREXPORT void ArServerModeDock::deactivate()
{
  if (myState == UNDOCKED)
  {
    myPathTask->cancelPathPlan();
    myForcedDock = false;
    myShuttingDownSeconds = 0;
    broadcastDockInfoChanged();
    //ArLog::log(ArLog::Normal, "########### 0");
    if (myFoundDockMapObject != NULL)
    {
      delete myFoundDockMapObject;
      myFoundDockMapObject = NULL;
      dockingAtChanged();
    }
    baseDeactivate();
  }
  else
    undock();
}

AREXPORT void ArServerModeDock::requestUnlock()
{
  if (!myForcedDock || myState == UNDOCKED)
    deactivate();
}

AREXPORT void ArServerModeDock::forceUnlock(void)
{
  myState = UNDOCKED;
  ArServerMode::forceUnlock();
}

void ArServerModeDock::switchState(State state)
{
  State oldState = myState;
  myState = state;
  myStartedState.setToNow();
  if (oldState != myState)
  {
    if (myState == DOCKED)
    {
      myDockedCBList.invoke();
      mySingleShotDockedCBList.invoke();
    }
    else if (myState == UNDOCKING)
      myUndockingCBList.invoke();
    else if (myState == UNDOCKED)
      myUndockedCBList.invoke();
  }

  if (myShutdownFunctor != NULL && myState == DOCKING && 
      myShutdownLastPose.squaredFindDistanceTo(myRobot->getPose()) > 20 * 20)
  {
    myShutdownLastPose = myRobot->getPose();
    myShutdownLastMove.setToNow();
  }

  stateChanged();
}


AREXPORT void ArServerModeDock::setDockingVoltage(double dockingVoltage)
{ 
  myDockingVoltage = dockingVoltage;
}

AREXPORT double ArServerModeDock::getDockingVoltage(void) const
{ 
  return myDockingVoltage; 
}

AREXPORT void ArServerModeDock::setDoneChargingVoltage(double doneChargingVoltage)
{ 
  myDoneChargingVoltage = doneChargingVoltage; 
}

AREXPORT double ArServerModeDock::getDoneChargingVoltage(void) const
{ 
  return myDoneChargingVoltage; 
}

AREXPORT void ArServerModeDock::setDoneChargingMinutes(int doneChargingMinutes)
{ 
  myDoneChargingMinutes = doneChargingMinutes;
}

AREXPORT int ArServerModeDock::getDoneChargingMinutes(void)
{ 
  return myDoneChargingMinutes; 
}

AREXPORT void ArServerModeDock::setMinimumMinutesBetweenAutoDock(
	int minutesBetween)
{ 
  myMinimumMinutesBetweenAutoDock = minutesBetween;
}

AREXPORT int ArServerModeDock::getMinimumMinutesBetweenAutoDock(void)
{ 
  return myMinimumMinutesBetweenAutoDock;
}

AREXPORT bool ArServerModeDock::getUseChargeState(void)
{
  return myUseChargeState;
}

AREXPORT void ArServerModeDock::setDoneChargingAtFloat(
	bool doneChargingAtFloat)
{
  myDoneChargingAtFloat = doneChargingAtFloat;
}

AREXPORT bool ArServerModeDock::getDoneChargingAtFloat(void)
{
  return myDoneChargingAtFloat;
}



AREXPORT void ArServerModeDock::setAutoDock(bool autoDocking) 
{ 
  if (myUsingAutoDock != autoDocking)
  {
    if (autoDocking)
      ArLog::log(ArLog::Normal, "Enabling autodocking");
    else
      ArLog::log(ArLog::Normal, "Disabling autodocking");
    // reset the shutdown time, and the seconds so we don't get
    // strange instant events or mismatched timing
    myShutdownLastPose = myRobot->getPose();
    myShutdownLastMove.setToNow();
    myShuttingDownSeconds = 0;
  }
  myUsingAutoDock = autoDocking; 
}

AREXPORT bool ArServerModeDock::getAutoDock(void)
{ 
  return myUsingAutoDock; 
}

AREXPORT bool ArServerModeDock::processFile(void)
{
  if (myAutoDock != myLastAutoDock)
    myUsingAutoDock = myAutoDock;
  myLastAutoDock = myAutoDock;
  // call the map changed CB so that everything gets updated
  dockMapChanged();

  /* MPL 2012/8/10 Don't set the dock this way anymore, since it's
   * found more dynamically now

  if (myDockName.empty())
  {
    ArMapInterface *arMap;
    arMap = myPathTask->getAriaMap();
    ArMapObject *dockObject = NULL;
    dockObject = findDock(arMap);
    if (dockObject != NULL)
      myDockName = dockObject->getName();
    else 
      myDockName = myPreferredDock;
  }
  */
  return true;
}

/**
   This call will have the server mode do a forced dock
**/ 
AREXPORT void ArServerModeDock::requestForcedDock(void)
{
  if (!myIsActive)
  {
    myForcedDockRequested = true;
    //myUsingAutoDock = true;
  }
  else
  {
    myForcedDock = true;
    lockMode(false);
    //myUsingAutoDock = true;
  }
}

AREXPORT void ArServerModeDock::addToConfig(ArConfig *config)
{
  std::string section;
  section = "Docking";

  if (config == NULL)
    return;
  
  char displayHint[2048];
  if (myDockType.empty())
    sprintf(displayHint, "MapItem:Dock|Parent|Optional");
  else
    sprintf(displayHint, "MapItem:%s|SubTypeAndParent|Optional", 
	   myDockType.c_str());

  config->addParam(
	  ArConfigArg("AutoDock", &myAutoDock, 
		      "True if we should automatically dock"), 
	  section.c_str(), ArPriority::NORMAL);

  if (myRobot->haveStateOfCharge())
    config->addParam(
	  ArConfigArg("AutoDockStateOfCharge", &myDockingStateOfCharge, 
		      "If auto docking the robot state of charge to automatically dock at", 0, 100),
	  section.c_str(), ArPriority::NORMAL);
  else
    config->addParam(
	  ArConfigArg("AutoDockVoltage", &myDockingVoltage, 
		      "If auto docking the robot voltage to automatically dock at (0 will use the lowBatteryVoltage from the firmware if it exists, if it does not exist it will use 12 volts)", 0),
	  section.c_str(), ArPriority::NORMAL);
  config->addParam(
	  ArConfigArg("AutoDockIdleTime", &myDockingIdleTime, 
		      "If auto docking the idle time in minutes until auto dock (0 means never auto dock because of idle)", 0),
	  section.c_str(), ArPriority::NORMAL);

  config->addParam(
	  ArConfigArg("PreferredDock", myPreferredDock,
		      "If there is a charger with this name this robot will use that dock if either OnlyUsePreferredDock is true or the dock is available.  If one of those conditions isn't met or a charger of this name does not exist then the default behavior of using the closest dock (by geometric distance) will apply.",
		      sizeof(myPreferredDock)),
	  section.c_str(), ArPriority::NORMAL, displayHint);

  config->addParam(
	  ArConfigArg("OnlyUsePreferredDock", &myOnlyUsePreferredDock,
		      "If this is true the robot will only use the preferred dock (which will prevent other robots from using it in a central multirobot setup)"),
	  section.c_str(), ArPriority::NORMAL, displayHint);


  if (!subclassAddParamsDoneCharging(config, section.c_str()))
    baseAddParamsDoneCharging(config, section.c_str());

  config->addParam(
	  ArConfigArg("MinutesBetweenAutoDock",
		      &myMinimumMinutesBetweenAutoDock,
		      "If 0 don't limit how often the robot can charge, if greater than 0 then this many minutes will elapse between when the robot last docked and when it will autodock",
		      0),
	  section.c_str(), ArPriority::EXPERT);

  myFailedDriveInIgnoreMinutes = 60;
  config->addParam(
	  ArConfigArg("FailedDriveInIgnoreMinutes",
		      &myFailedDriveInIgnoreMinutes,
		      "The number of minutes to ignore a dock we've failed to drive into too many times (FailedDriveInAttemptsBeforeIgnore)...  0 not to ignore these docks.",
		      0),
	  section.c_str(), ArPriority::EXPERT);

  myFailedDriveInAttemptsBeforeIgnore = 3;
  config->addParam(
	  ArConfigArg("FailedDriveInAttemptsBeforeIgnore",
		      &myFailedDriveInAttemptsBeforeIgnore,
		      "The number of times to try to drive into a dock before ignoring it (for FailedDriveInIgnoreMinutes)...  0 means never to ignore a dock for this reason.",
		      0),
	  section.c_str(), ArPriority::EXPERT);

  mySameDockDistanceTolerance = 1000;
  config->addParam(
	  ArConfigArg("SameDockDistanceTolerance",
		      &mySameDockDistanceTolerance,
		      "For cases where overrideMapName has been used docks within this distance and within SameDockAngleTolerance are counted as the same for purposes of seeing which docks are occupied by other robots.",
		      1),
	  section.c_str(), ArPriority::EXPERT);
  
  mySameDockAngleTolerance = 10;
  config->addParam(
	  ArConfigArg("SameDockAngleTolerance",
		      &mySameDockAngleTolerance,
		      "For cases where overrideMapName has been used docks within this angular tolerance and within SameDockDistanceTolerance are counted as the same for purposes of seeing which docks are occupied by other robots.",
		      1),
	  section.c_str(), ArPriority::EXPERT);
    


  Aria::getConfig()->addParam(
	  ArConfigArg(ArConfigArg::SEPARATOR), section.c_str(),
	  ArPriority::NORMAL);

  if (myShutdownFunctor != NULL)
  {
    config->addParam(
	    ArConfigArg("AutoDockShutdownMinutes",
			&myShutdownMinutesForced,
			"This controls how long the robot should wait before shutting down if it cannot move on its way to the dock while in a low battery situation.  0 means do not automatically shutdown for this reason, any other value is how many minutes to wait before shutting down",
			0, 60),
	    section.c_str(), ArPriority::TRIVIAL);
    config->addParam(
	    ArConfigArg("UnforcedDockShutdownMinutes",
			&myShutdownMinutesIdle,
			  "This controls how long the robot should wait before shutting down if it cannot move on its way to the dock if docking because the robot was idle or an explicit dock request was made.  0 means do not automatically shutdown for this reason, any other value is how many minutes to wait before shutting down",
			0, 60),
	      section.c_str(), ArPriority::TRIVIAL);
  }
  config->addProcessFileCB(&myProcessFileCB, 50);
}

AREXPORT void ArServerModeDock::baseAddParamsDoneCharging(ArConfig *config,
							  const char *section)
{
  if (myUseChargeState)
    baseAddParamDockUntilDoneCharging(config, section);

  baseAddParamMinutesToChargeFor(config, section);

  baseAddParamToChargeTo(config, section);

}

AREXPORT void ArServerModeDock::baseAddParamDockUntilDoneCharging(
	ArConfig *config, const char *section)
{
  if (myUseChargeState)
    config->addParam(
	    ArConfigArg("DockUntilDoneCharging", &myDoneChargingAtFloat,
			"If true then the robot will dock until it is done charging (in float mode)"),
	    section, ArPriority::NORMAL);
}

AREXPORT void ArServerModeDock::baseAddParamMinutesToChargeFor(
	ArConfig *config, const char *section)
{
  config->addParam(
	  ArConfigArg("MinutesToChargeFor", &myDoneChargingMinutes,
		      "If 0 don't finish charging based on time, if -1 then just undock as soon as docked (mostly for testing) if greater than 0 then be done charging after this many minutes", 
		      -1),
	  section, ArPriority::NORMAL);
}


AREXPORT void ArServerModeDock::baseAddParamToChargeTo(
	ArConfig *config, const char *section)
{
  if (myRobot->haveStateOfCharge())
    config->addParam(
	    ArConfigArg("StateOfChargeToChargeTo", 
			&myDoneChargingStateOfCharge,
			"If 0 don't finish charging based on state of charge, if greater than 0 then be done charging after robot's state of charge reaches this point", 
			0, 100),
	    section, ArPriority::NORMAL);
  else
    config->addParam(
	    ArConfigArg("VoltageToChargeTo", &myDoneChargingVoltage,
			"If 0 don't finish charging based on voltage, if greater than 0 then be done charging after robot's voltage reachs this point", 
			0),
	    section, ArPriority::ADVANCED);
}

AREXPORT ArMapObject *ArServerModeDock::findDock(ArMapInterface *arMap)
{
  // when this is non-NULL we have found the dock we'll use
  ArMapObject *dockObject = NULL;

  myDocksMutex.lock();

  // this is the list of docks we can't go to because we've failed to
  // drive to them...
  std::set<std::string> failedDocks;

  /*
    If we've failed to dock at this goal some number of times then put
    it in the list of the docks we've failed to drive into...  
  */
  if (myFoundDockMapObject != NULL && 
      myFailedDriveInAttemptsBeforeIgnore > 0 && 
      myFailedDriveInNum >= myFailedDriveInAttemptsBeforeIgnore)
  {
    
    if (myFailedDriveInIgnoreMinutes > 0)
      ArLog::log(ArLog::Normal, 
      	 "Failed to drive in to dock '%s' %d times, ignoring it for %g minutes (or until the map changes, or it's the last dock that can be tried)",
		 myFoundDockMapObject->getName(), myFailedDriveInNum,
		 myFailedDriveInIgnoreMinutes);
	       
    myFailedDriveInDocks[myFoundDockMapObject->getName()] = ArTime();
  }


  long long longestAgo = -1;
  std::string longestAgoDockName;

  if (myFailedDriveInIgnoreMinutes > 0)
  {
    // walk through the docks we've failed to drive into, and add any
    // we've failed to drive into in the last hour
    long long secSince;
    std::string dockName;

    std::map<std::string, ArTime>::iterator it;
    for (it = myFailedDriveInDocks.begin();
	 it != myFailedDriveInDocks.end();
	 it++)
    {
      secSince = (*it).second.secSince();
      dockName = (*it).first;
      if (myOccupiedDocks.find(dockName) != myOccupiedDocks.end())
	continue;
      if (secSince < myFailedDriveInIgnoreMinutes * 60.0)
	failedDocks.insert(dockName);
      // this should make sure the dock isn't occupied too
      if (secSince > longestAgo)
      {
	longestAgo = secSince;
	longestAgoDockName = dockName;
      }
    }

    // if we have no valid docks for now we pull out the oldest failed
    // drive into dock we just added... so that the robot doesn't
    // block the dock for some other robot that might be able to dock
    // (in case the problem is HW failure on the robot)...  Later we
    // won't do this, and instead we should fire off the 'help mode'
    // we'll add
    if (failedDocks.size() + myOccupiedDocks.size() >= myDocks.size() && 
	longestAgo >= 0)
    {
      ArLog::log(ArLog::Normal, 
		 "All possible docks are occupied or have been failed, so using dock '%s' since that was failed longest ago (%g minutes)",   
		 longestAgoDockName.c_str(), longestAgo / 60.0);
      failedDocks.erase(longestAgoDockName);
    }
  }
    

  // if we have a preferred dock... and we're either ONLY using the
  // preferred dock or the dock is free, then use it
  if (myPreferredDockMapObject != NULL && 
      
      (myOnlyUsePreferredDock || 
      
       (failedDocks.find(myPreferredDockMapObject->getName()) == 
	failedDocks.end() && 
	myOccupiedDocks.find(myPreferredDockMapObject->getName()) == 
	myOccupiedDocks.end())))
  {
    dockObject = myPreferredDockMapObject;
  }

  // if we don't have a dock yet, find the closest available one
  if (dockObject == NULL)
  {
    // first try to find the closest dock
    int dist = 0;
    int closestDist = INT_MAX;
    ArMapObject *closestDock = NULL;
    ArMapObject *dock = NULL;
    std::set<ArMapObject *>::iterator dockIt;
    for (dockIt = myDocks.begin(); 
	 dockIt != myDocks.end(); 
	 dockIt++)
    {
      dock = (*dockIt);
      // if this dock is one of the unavailable ones skip it
      if (failedDocks.find(dock->getName()) != 
	  failedDocks.end())
	continue;
      if (myOccupiedDocks.find(dock->getName()) != 
	  myOccupiedDocks.end())
	continue;
      //dist = dock->getPose().findDistanceTo(myRobot->getPose());
      dist = myDistanceCB->invokeR(dock->getPose(), myRobot->getPose());
      if (dist < closestDist)
      {
	closestDist = dist;
	closestDock = dock;
      } 
    }
    dockObject = closestDock;
  }

  /* the old simple code that just finds the first one
  if (dockObject == NULL && !myDockType.empty())
    dockObject = arMap->findMapObject(NULL, myDockType.c_str());
  if (dockObject == NULL)
    dockObject = arMap->findMapObject(NULL, "Dock");
  */


  std::string overrideMapName;
  if (myGetOverrideMapNameFunctor != NULL && 
      myGetOverrideMapNameFunctor->invokeR() != NULL && 
      myGetOverrideMapNameFunctor->invokeR()[0] != '\0')
    overrideMapName = myGetOverrideMapNameFunctor->invokeR();
  else
    overrideMapName = arMap->getFileName();

  // if the dock is different send off a message
  bool foundDifferent = true;
  if (ArUtil::strcmp(myFoundDockMapName, arMap->getFileName()) == 0 &&
      ArUtil::strcmp(myFoundDockOverrideMapName, overrideMapName) == 0 &&
      dockObject != NULL && myFoundDockMapObject != NULL &&
      ArUtil::strcmp(myFoundDockMapObject->getName(), 
		     dockObject->getName()) == 0)
    foundDifferent = false;      
  else if (myFoundDockMapObject == NULL && dockObject == NULL)
    foundDifferent = false;

  myFoundDockMapName = arMap->getFileName();
  myFoundDockOverrideMapName = overrideMapName;
  if (myFoundDockMapObject != NULL)
  {
    delete myFoundDockMapObject;
    myFoundDockMapObject = NULL;
  }
  if (dockObject != NULL)
    myFoundDockMapObject = new ArMapObject(*dockObject);

  myDocksMutex.unlock();

  // if it's a different one... first reset some counters, then send
  // the info to the server
  if (foundDifferent)
  {
    myFailedGotoNum = 0;
    myFailedDriveInNum = 0;
    // insert sending info to server
    dockingAtChanged();
  }

  /*
  if (myFoundDockMapObject != NULL)
    ArLog::log(ArLog::Normal, "########### 1 %s %d", 
	       myFoundDockMapObject->getName(), foundDifferent);
  else
    ArLog::log(ArLog::Normal, "########### 1 %s %d", 
	       "NULL", foundDifferent);
  */

  return myFoundDockMapObject;
}

void ArServerModeDock::dockingAtChanged(void)
{
  myDockingAtToServerPacket.empty();

  if (myFoundDockMapObject != NULL)
  {
    myDockingAtToServerPacket.strToBuf(myFoundDockMapName.c_str());
    myDockingAtToServerPacket.strToBuf(myFoundDockMapObject->getName());
    myDockingAtToServerPacket.strToBuf(
	    myFoundDockOverrideMapName.c_str());
    myDockingAtToServerPacket.byte4ToBuf(
	    (int)myFoundDockMapObject->getPose().getX());
    myDockingAtToServerPacket.byte4ToBuf(
	    (int)myFoundDockMapObject->getPose().getY());
    myDockingAtToServerPacket.byte2ToBuf(
	    (int)myFoundDockMapObject->getPose().getTh());
  }
  else if (myPreferredDockMapObject != NULL && myOnlyUsePreferredDock)
  {
    myDockingAtToServerPacket.strToBuf(myPreferredDockMapName.c_str());
    myDockingAtToServerPacket.strToBuf(
	    myPreferredDockMapObject->getName());
    myDockingAtToServerPacket.strToBuf(
	    myPreferredDockOverrideMapName.c_str());
    myDockingAtToServerPacket.byte4ToBuf(
	    (int)myPreferredDockMapObject->getPose().getX());
    myDockingAtToServerPacket.byte4ToBuf(
	    (int)myPreferredDockMapObject->getPose().getY());
    myDockingAtToServerPacket.byte2ToBuf(
	    (int)myPreferredDockMapObject->getPose().getTh());
  }
  else
  {
    myDockingAtToServerPacket.strToBuf("");
    myDockingAtToServerPacket.strToBuf("");
    myDockingAtToServerPacket.strToBuf("");
    myDockingAtToServerPacket.byte4ToBuf(0);
    myDockingAtToServerPacket.byte4ToBuf(0);
    myDockingAtToServerPacket.byte2ToBuf(0);
  }
  
  myServer->broadcastPacketTcp(&myDockingAtToServerPacket, 
			       "dockingAtToServer");

  if (myState == DOCKED)
    writeDockFile();
}

AREXPORT void ArServerModeDock::serverDockingAtToServer(ArServerClient *client, ArNetPacket *packet)
{
  client->sendPacketTcp(&myDockingAtToServerPacket);
}

AREXPORT void ArServerModeDock::serverDockingAtFromServer(ArServerClient *client, ArNetPacket *packet)
{
  myDocksMutex.lock();

  myDockingAtFromServerPacket = (*packet);

  checkOccupiedDocks();

  myDocksMutex.unlock();
}

void ArServerModeDock::checkOccupiedDocks(void)
{
  int numRobots;
  int ii;
  char otherRobotName[10000];
  char otherMapName[10000];
  char otherDockName[10000];
  char otherOverrideMapName[10000];
  ArPose otherDockPose;


  myDocksMutex.lock();

  myOccupiedDocks.clear();

  numRobots = myDockingAtFromServerPacket.bufToByte2();
  ArLog::log(ArLog::Normal, 
	     "CentralServer::DockHelper reports %d robots", 
	     numRobots);

  std::set<ArMapObject *>::iterator it;
  ArMapObject *dock;
  bool matchedDock;
  for (ii = 0; ii < numRobots; ii++)
  {
    matchedDock = false;

    myDockingAtFromServerPacket.bufToStr(otherRobotName, 
					  sizeof(otherRobotName));
    myDockingAtFromServerPacket.bufToStr(otherMapName, 
					  sizeof(otherMapName));
    myDockingAtFromServerPacket.bufToStr(otherDockName, 
					  sizeof(otherDockName));
    myDockingAtFromServerPacket.bufToStr(otherOverrideMapName, 
					  sizeof(otherOverrideMapName));
    otherDockPose.setX(myDockingAtFromServerPacket.bufToByte4());
    otherDockPose.setY(myDockingAtFromServerPacket.bufToByte4());
    otherDockPose.setTh(myDockingAtFromServerPacket.bufToByte2());    

    if (ArUtil::strcasecmp(otherRobotName, Aria::getIdentifier()) == 0)
    {
      ArLog::log(ArLog::Normal, "Robot '%s' is this robot", 
		 otherRobotName);
      continue;
    }

    if (otherMapName[0] == '\0' || otherDockName[0] == '\0')
    {
      ArLog::log(ArLog::Normal, "Robot %s is not docked", otherRobotName);
      continue;
    }
    
    if (ArUtil::strcasecmp(otherMapName, myDocksMapName) == 0)
    {
      for (it = myDocks.begin(); it != myDocks.end(); it++)
      {
	dock = (*it);
	// see if it's supposed to be the same goal
	if (ArUtil::strcasecmp(dock->getName(), otherDockName) == 0)
	{
	  // make sure it actually is...
	  if (dock->getPose().findDistanceTo(otherDockPose) <= 1 && 
	      (ArMath::subAngle(dock->getPose().getTh(), 
				otherDockPose.getTh()) <= 1))
	  {
	    matchedDock = true;
	    myOccupiedDocks.insert(dock->getName());
	    ArLog::log(ArLog::Normal, 
		       "Robot '%s' is using dock '%s' in this map.",
		       otherRobotName, dock->getName());
	  }
	  else
	  {
	    ArLog::log(ArLog::Normal, 
		       "\t\tRobot '%s' reports it is using dock '%s' in this map, but it's position is wrong...  It reports dock '%s' in map '%s' (overrideMap '%s' pose %g %g %g... In this map it is at %g %g %g)",
		       otherRobotName, dock->getName(),
		       otherDockName, otherMapName, otherOverrideMapName, 
		       otherDockPose.getX(), otherDockPose.getY(), 
		       otherDockPose.getTh(),
		       dock->getPose().getX(), dock->getPose().getY(), 
		       dock->getPose().getTh());
	    break;
	  }
	}
      }
    }

    if (myGetOverrideMapNameFunctor != NULL && 
	myGetOverrideMapNameFunctor->invokeR() != NULL && 
	myGetOverrideMapNameFunctor->invokeR()[0] != '\0' && 
	ArUtil::strcasecmp(otherOverrideMapName, 
			   myGetOverrideMapNameFunctor->invokeR()) == 0)
    {
      for (it = myDocks.begin(); it != myDocks.end(); it++)
      {
	dock = (*it);
	if ((dock->getPose().findDistanceTo(otherDockPose) < 
	     mySameDockDistanceTolerance) && 
	    (ArMath::subAngle(dock->getPose().getTh(), 
			      otherDockPose.getTh()) < 
	     mySameDockDistanceTolerance))
	{
	  matchedDock = true;
	  myOccupiedDocks.insert(dock->getName());
	  ArLog::log(ArLog::Normal, 
		     "Robot '%s' is using dock '%s' in this map.  It is dock '%s' in map '%s' (overrideMap '%s' pose %g %g %g)",
		     otherRobotName, dock->getName(), 
		     otherDockName, otherMapName, otherOverrideMapName, 
		     otherDockPose.getX(), otherDockPose.getY(), 
		     otherDockPose.getTh());
	  
	  
	}
      }
    }
    
    if (!matchedDock)
      ArLog::log(ArLog::Normal, 
		 "Robot '%s' is using an unmatched dock.  It is dock '%s' in map '%s' (overrideMap '%s' pose %g %g %g)",
		 otherRobotName, otherDockName, otherMapName, 
		 otherOverrideMapName, 
		 otherDockPose.getX(), otherDockPose.getY(), 
		 otherDockPose.getTh());
  }
  
  myDocksMutex.unlock();
}

void ArServerModeDock::dockMapChanged(void)
{
  ArMapInterface *arMap = myPathTask->getAriaMap();
  if (arMap == NULL)
    return;

  arMap->lock();
  myDocksMutex.lock();

  myDocksMapName = arMap->getFileName();

  myFailedDriveInDocks.clear();

  ArUtil::deleteSet(myDocks.begin(), myDocks.end());

  myDocks.clear();

  if (myPreferredDockMapObject != NULL)
  {
    delete myPreferredDockMapObject;
    myPreferredDockMapObject = NULL;
  }

  if (myPreferredDock[0] != '\0')
  {
    ArMapObject *dockObject = NULL;

    if (myPreferredDock[0] != '\0' && !myDockType.empty())
      dockObject = arMap->findMapObject(myPreferredDock, myDockType.c_str());
    if (dockObject == NULL && myPreferredDock[0] != '\0')
      dockObject = arMap->findMapObject(myPreferredDock, "Dock");

    if (dockObject != NULL)
    {
      myPreferredDockMapObject = new ArMapObject(*dockObject);
      std::string preferredOverrideMapName;
      if (myGetOverrideMapNameFunctor != NULL && 
	  myGetOverrideMapNameFunctor->invokeR() != NULL && 
	  myGetOverrideMapNameFunctor->invokeR()[0] != '\0')
	preferredOverrideMapName = myGetOverrideMapNameFunctor->invokeR();
      else
	preferredOverrideMapName = arMap->getFileName();
      
      myPreferredDockMapName = arMap->getFileName();
      myPreferredDockOverrideMapName = preferredOverrideMapName;
    }
    else
    {
      myPreferredDockMapName = "";
      myPreferredDockOverrideMapName = "";
    }
  }

  // first build a list of docks (we need to know how many we have
  // before we walk through them)
  std::list<ArMapObject *> tempList;
  std::list<ArMapObject *>::iterator it;
  if (!myDockType.empty())
  {
    tempList = arMap->findMapObjectsOfType(myDockType.c_str(), true);
    for (it = tempList.begin(); it != tempList.end(); it++)
      myDocks.insert(new ArMapObject(*(*it)));
  }
  tempList = arMap->findMapObjectsOfType("Dock", true);
  for (it = tempList.begin(); it != tempList.end(); it++)
    myDocks.insert(new ArMapObject(*(*it)));

  // if we're docking or docked walk through the docks in the map...
  // see if we have a dock in exactly the same position... if so toss
  // out a new dockingAtToServer packet.
  if (myFoundDockMapObject != NULL)
  {
    std::set<ArMapObject *>::iterator docksIt;
    ArMapObject *dock = NULL;
    ArMapObject *dockWithSameName = NULL;
    ArMapObject *dockWithSamePosition = NULL;
    
    for (docksIt = myDocks.begin(); docksIt != myDocks.end(); docksIt++)
    {
      dock = (*docksIt);
      // if we have one with the same name or the exact same position,
      // assume it's the right one
      if (ArUtil::strcasecmp(dock->getName(), 
			     myFoundDockMapObject->getName()) == 0)
	dockWithSameName = dock;
      if (dock->getPose().findDistanceTo(
		  myFoundDockMapObject->getPose()) <= 1 && 
	  (ArMath::subAngle(
		  dock->getPose().getTh(), 
		  myFoundDockMapObject->getPose().getTh()) <= 1))
	dockWithSamePosition = dock;
    }
    
    if (dockWithSameName != NULL || dockWithSamePosition != NULL)
    {
      myFoundDockMapName = arMap->getFileName();
      if (myFoundDockMapObject != NULL)
      {
	delete myFoundDockMapObject;
	myFoundDockMapObject = NULL;
      }
      std::string overrideMapName;
      if (myGetOverrideMapNameFunctor != NULL && 
	  myGetOverrideMapNameFunctor->invokeR() != NULL && 
	  myGetOverrideMapNameFunctor->invokeR()[0] != '\0')
	myFoundDockOverrideMapName = myGetOverrideMapNameFunctor->invokeR();
      else
	myFoundDockOverrideMapName = arMap->getFileName();
      // if we have a dock with the same position and same name and
      // it's the same dock, then it's unambiguously the same
      if (dockWithSamePosition != NULL && dockWithSameName != NULL && 
	  dockWithSameName == dockWithSamePosition)
      {
	myFoundDockMapObject = new ArMapObject(*dockWithSameName);
	ArLog::log(ArLog::Normal, 
		   "Dock::MapChanged: This robot is using dock '%s' in this map.",
		   myFoundDockMapObject->getName());
      }
      // if we have a dock with the same position and same name but
      // it's the same dock, then we go with the one with the same
      // position and note it's strange state
      else if (dockWithSamePosition != NULL && dockWithSameName != NULL && 	       dockWithSameName != dockWithSamePosition)
      {
	myFoundDockMapObject = new ArMapObject(*dockWithSameName);
	ArLog::log(ArLog::Normal, 
		   "Dock::MapChanged: This robot is using dock '%s' in this map because it had the same position as the old map (but another dock had the same name, which may be bad).",
		   myFoundDockMapObject->getName());
      }
      // if we have one in the same position but nothing with the same
      // name that's pretty unambiguous too
      else if (dockWithSamePosition != NULL && dockWithSameName == NULL)
      {
	myFoundDockMapObject = new ArMapObject(*dockWithSamePosition);
	ArLog::log(ArLog::Normal, 
		   "Dock::MapChanged: This robot is using dock '%s' in this map because it had the same position so it was hopefully just renamed.",
		   myFoundDockMapObject->getName());
      }
      // if we have one with the same name but nothing in the same
      // position then they probably moved it (hopefully not much)
      else if (dockWithSamePosition == NULL && dockWithSameName != NULL)
      {
	myFoundDockMapObject = new ArMapObject(*dockWithSameName);
	ArLog::log(ArLog::Normal, 
		   "Dock::MapChanged: This robot is using dock '%s' in this map because it had the same name (if the position isn't the same that may be bad...).",
		   myFoundDockMapObject->getName());
      }
      else
      {	
	ArLog::log(ArLog::Normal, 
		   "Dock::MapChanged: shouldn't be in this case");
      }
      //dockingAtChanged();
    }
  }

  // always call this now since the preferred dock behavior can change
  dockingAtChanged();

  arMap->unlock();

  checkOccupiedDocks();

  myDocksMutex.unlock();
}

AREXPORT void ArServerModeDock::clearInterrupted(void)
{
  myModeInterrupted = NULL;
  ArLog::log(myDockModeLogLevel, "DockMode: Clearing modeInterrupted");
}

AREXPORT void ArServerModeDock::resumeInterrupted(bool assureDeactivation)
{
  ArLog::log(myDockModeLogLevel, "DockMode: Resume interrupted start %d", assureDeactivation);
  std::list<ArServerMode *> *requestedActivateModes = getRequestedActivateModes();

  // if something else wanted to activate we can just deactivate and 
  // that'll get activated
  if (isActive() && 
      (requestedActivateModes != NULL) &&
      requestedActivateModes->begin() != requestedActivateModes->end())
  {
    deactivate();
    if (getActiveMode() != NULL)
      ArLog::log(myDockModeLogLevel, 
		 "DockMode: Resume interrupted deactivating and returning... %s got activated", 
		 getActiveMode()->getName());
    else
      ArLog::log(myDockModeLogLevel, 
		 "DockMode: Resume interrupted deactivating and returning... nothing active...");
    return;
  }
  // if we're still active and we interrupted something when we
  // activated then start that up again, if it was stop we interrupted
  // just stay at the dock

  if (myModeInterrupted != NULL)
    ArLog::log(myDockModeLogLevel, "DockMode: Resume interrupted another... isActive %d modeInterrupted %p %s", 
	       isActive(), myModeInterrupted, myModeInterrupted->getName());
  else
    ArLog::log(myDockModeLogLevel, "DockMode: Resume interrupted another... isActive %d modeInterrupted NULL", isActive());


  if (isActive() && myModeInterrupted != NULL && 
      strcmp(myModeInterrupted->getName(), "stop") != 0)
  {
    ArLog::log(myDockModeLogLevel, "DockMode: Trying to activate interrupted mode %s", 
	       myModeInterrupted->getName());
    myModeInterrupted->activate();
    myModeInterrupted = NULL;
    if (getActiveMode() != NULL)
      ArLog::log(myDockModeLogLevel, 
		 "DockMode: Did activate mode %s", 
		 getActiveMode()->getName());
    return;
  }

  ArLog::log(myDockModeLogLevel, "DockMode: Resume interrupted later");
  
  // if we're supposed to assure deactivation and we're still here
  // then deactivate
  if (isActive() && assureDeactivation)
  {
    ArLog::log(myDockModeLogLevel, "DockMode: Deactivating");
    deactivate();
    return;
  }
  ArLog::log(myDockModeLogLevel, "DockMode: Resume interrupted end");
}

AREXPORT void ArServerModeDock::addStateChangedCB(ArFunctor *functor, 
						 ArListPos::Pos position)
{
  if (position == ArListPos::FIRST)
    myStateChangedCBList.push_front(functor);
  else if (position == ArListPos::LAST)
    myStateChangedCBList.push_back(functor);
  else
    ArLog::log(ArLog::Terse, 
	       "ArServerModeDock::addStateChangeCB: Invalid position.");
}

AREXPORT void ArServerModeDock::remStateChangedCB(ArFunctor *functor)
{
  myStateChangedCBList.remove(functor);
}

void ArServerModeDock::stateChanged(void)
{
  if (myState == UNDOCKING && myLastState == DOCKED)
  {
    if (myHaveBeenNotDocked)
    {
      long secs;
      secs = myLastNotDocked.secSince();

      char buf[1024];

      myDockInfoMutex.lock();

      if (myLastForcedDock)
      {
	sprintf(buf, 
		"DockInfo:UndockingFromForced: Was docked %02ld:%02ld:%02ld",
		secs / 3600, (secs % 3600) / 60, secs % 60);
	myDockInfoUndockingFromForced = buf;
      }
      else
      {
	sprintf(buf, 
		   "DockInfo:Undocking: Was docked %02ld:%02ld:%02ld",
		   secs / 3600, (secs % 3600) / 60, secs % 60);
	myDockInfoUndocking = buf;
      }

      myDockInfoMutex.unlock();

      ArLog::log(myDockInfoLogLevel, buf);
    }
  }

  if (myLastState == DOCKING && myState == DOCKED)
  {
    long secs;
    secs = myLastDocked.secSince();

    char buf[1024];

    myDockInfoMutex.lock();

    if (myForcedDock)
    {
      sprintf(buf, 
	      "DockInfo:ForcedDockAchieved: Last docked %02ld:%02ld:%02ld ago",
	      secs / 3600, (secs % 3600) / 60, secs % 60);
      myDockInfoForcedDockAchieved = buf;
    }
    else
    {
      sprintf(buf, 
	      "DockInfo:DockAchieved: Last docked %02ld:%02ld:%02ld ago",
	      secs / 3600, (secs % 3600) / 60, secs % 60);
      myDockInfoDockAchieved = buf;
    }

    myDockInfoMutex.unlock();

    ArLog::log(myDockInfoLogLevel, buf);
  }

  // if we were and are docked see if forced toggled
  if (myLastState == DOCKED && myState == DOCKED && 
      myLastForcedDock != myForcedDock)
  {
    if (myForcedDock)
      myDockNowForcedCBList.invoke();
    else
      myDockNowUnforcedCBList.invoke();
  }
  myLastForcedDock = myForcedDock;
  myLastState = myState;

  std::list<ArFunctor *>::iterator it;
  for (it = myStateChangedCBList.begin(); 
       it != myStateChangedCBList.end(); 
       it++)
    (*it)->invoke();

  broadcastDockInfoChanged();
}

void ArServerModeDock::broadcastDockInfoChanged(void)
{
  // Think that the robot is already locked at this point.... (so it's 
  // not necessary here)
  if (myServer != NULL) 
  {
    ArNetPacket send;
    makeDockInfoPacket(&send);
    myServer->broadcastPacketTcp(&send, "dockInfoChanged");
  }
}

void ArServerModeDock::makeDockInfoPacket(ArNetPacket *packet)
{
  packet->empty();
  packet->uByteToBuf(myState);
  packet->uByteToBuf(myForcedDock);
  if (myShutdownFunctor == NULL)
    packet->uByte2ToBuf(0);
  else 
    packet->uByte2ToBuf(myShuttingDownSeconds);
}

AREXPORT void ArServerModeDock::activateAsDocked(void)
{
  myStatus = "Docked";
  switchState(DOCKED);
  activate();
}


AREXPORT void ArServerModeDock::setDockBaseDirectory(
	const char *dockBaseDirectory)
{
  if (dockBaseDirectory != NULL && strlen(dockBaseDirectory) > 0)
    myDockBaseDir = dockBaseDirectory;
  else
    myDockBaseDir = "";
}

AREXPORT const char *ArServerModeDock::getDockBaseDirectory(void) const
{
  return myDockBaseDir.c_str();
}

AREXPORT void ArServerModeDock::setDockFileName(const char *dockFileName)
{  
  if (dockFileName != NULL && strlen(dockFileName) > 0)
    myDockFileName = dockFileName;
  else
    myDockFileName = "";
}

AREXPORT const char *ArServerModeDock::getDockFileName(void) const
{
  return myDockFileName.c_str();
}


AREXPORT void ArServerModeDock::restoreFromDockFile(void)
{
  if (myDockFileName.size() == 0)
    return;

  FILE *file;
  std::string realFileName;

  if (myDockFileName.size() > 0 && (myDockFileName.c_str()[0] == '/' || 
				myDockFileName.c_str()[0] == '\\'))
  {
    realFileName = myDockFileName;
  }
  else 
  {
    realFileName = myDockBaseDir;
    realFileName += myDockFileName;
  }

  ArLog::log(ArLog::Normal, 
	     "Restoring from dock file %s", 
	     realFileName.c_str());

  // if there is no dock file, we're done
  if ((file = ArUtil::fopen(realFileName.c_str(), "r")) == NULL)
  {
    ArLog::log(ArLog::Normal, 
	       "Dock file did not exist, robot not docked");
    return;
  }
  else
  {      
    char rawLine[10000];
    if (fgets(rawLine, sizeof(rawLine), file) == NULL)
    {
      ArLog::log(ArLog::Normal, "Dock file had bad information, will be docked at unknown goal");
    }
    else
    {
      ArArgumentBuilder line;
      line.add(rawLine);

      line.compressQuoted(true);

      if (line.getArgc() < 6 || !line.isArgInt(3) || !line.isArgInt(4) || 
	  !line.isArgInt(5))
      {
	ArLog::log(ArLog::Normal, "Dock file had bad information (%s), will be docked at unknown goal", rawLine);
      }
      else
      {
	myFoundDockMapName = line.getArg(0);
	myFoundDockOverrideMapName = line.getArg(2);

	if (myFoundDockMapObject != NULL)
	{
	  delete myFoundDockMapObject;
	  myFoundDockMapObject = NULL;
	}
	myFoundDockMapObject = new ArMapObject(
		  "UnspecificMapObjectCreatedForDocking", 
		  ArPose(line.getArgInt(3), line.getArgInt(4), 
			 line.getArgInt(5)),
		  "", "", line.getArg(1), false, ArPose(), ArPose());

	ArLog::log(ArLog::Normal, "Restored dock as '%s' in map name '%s' (override map name '%s') at position %.0f %.0f %.0f",
		   myFoundDockMapName.c_str(), 
		   myFoundDockMapObject->getName(),
		   myFoundDockOverrideMapName.c_str(),
		   myFoundDockMapObject->getPose().getX(),
		   myFoundDockMapObject->getPose().getY(),
		   myFoundDockMapObject->getPose().getTh());
		   
      }
    }


    ArLog::log(ArLog::Normal, "Robot was docked, so activating dock mode");
    activateAsDocked();
    if (isActive())
      ArLog::log(ArLog::Normal, "Dock mode active");
    else
      ArLog::log(ArLog::Normal, "Dock mode NOT active");
  }

  fclose(file);
}

void ArServerModeDock::writeDockFile(void)
{
  if (myDockFileName.size() == 0)
    return;

  FILE *file;
  std::string realFileName;

  if (myDockFileName.size() > 0 && (myDockFileName.c_str()[0] == '/' || 
				myDockFileName.c_str()[0] == '\\'))
  {
    realFileName = myDockFileName;
  }
  else 
  {
    realFileName = myDockBaseDir;
    realFileName += myDockFileName;
  }

  ArLog::log(ArLog::Normal, 
	     "Writing dock file %s", 
	     realFileName.c_str());

  myPreWriteCBList.invoke();
  if ((file = ArUtil::fopen(realFileName.c_str(), "w")) == NULL)
  {
    ArLog::log(ArLog::Terse, 
	   "Could not open dock file %s for writing",
	       realFileName.c_str());
    myPostWriteCBList.invoke();
    return;
  }
  if (myFoundDockMapObject != NULL)
    fprintf(file, "\"%s\" \"%s\" \"%s\" %.0f %.0f %.0f", 
	    myFoundDockMapName.c_str(), 
	    myFoundDockMapObject->getName(),
	    myFoundDockOverrideMapName.c_str(),
	    myFoundDockMapObject->getPose().getX(),
	    myFoundDockMapObject->getPose().getY(),
	    myFoundDockMapObject->getPose().getTh());
  
  fclose(file);

  myPostWriteCBList.invoke();
}


void ArServerModeDock::eraseDockFile(void)
{
  if (myDockFileName.size() == 0)
    return;

  std::string realFileName;

  if (myDockFileName.size() > 0 && (myDockFileName.c_str()[0] == '/' || 
				myDockFileName.c_str()[0] == '\\'))
  {
    realFileName = myDockFileName;
  }
  else 
  {
    realFileName = myDockBaseDir;
    realFileName += myDockFileName;
  }

  ArLog::log(ArLog::Normal, 
	     "Erasing dock file %s", 
	     realFileName.c_str());

  myPreWriteCBList.invoke();
  unlink(realFileName.c_str());
  myPostWriteCBList.invoke();
}

  

AREXPORT ArServerModeDockPioneer::ArServerModeDockPioneer(
	ArServerBase *server, ArRobot *robot, 
	ArLocalizationTask *locTask, ArPathPlanningInterface *pathTask, 
	ArFunctor *shutdownFunctor) :

  ArServerModeDock(server, robot, locTask, pathTask, true, 
		   shutdownFunctor),
  myGroup(robot),
  myGoalDoneCB(this, &ArServerModeDockPioneer::goalDone),
  myGoalFailedCB(this, &ArServerModeDockPioneer::goalFailed)
{
  myPathTask->addGoalDoneCB(&myGoalDoneCB);
  myPathTask->addGoalFailedCB(&myGoalFailedCB);

  myGroup.addAction(new ArActionLimiterForwards("limiter", 150, 0, 0, 1.3),
		    70);
  myGroup.addAction(new ArActionLimiterBackwards,
		   69);

  myDriveTo.setMaxLateralDist(500);
  myDriveTo.setMaxDistBetweenLinePoints(50);
  myGroup.addAction(&myDriveTo, 60);

  myGroup.addAction(new ArActionStop, 50);
  myDockType = "DockPioneer";
  myNeedToPathPlanToDock = false;
}

AREXPORT ArServerModeDockPioneer::~ArServerModeDockPioneer()
{

}

AREXPORT void ArServerModeDockPioneer::userTask(void)
{
  bool printing = false;
  if (printing)
    printf("State %d\n", myState);

  if (myState == DOCKED)
  {
    if (!(myRobot->getFlags() & ArUtil::BIT10))
    {
      //myStatus = "Dock failed";
      //switchState(UNDOCKED);
      if (mySentDockCommandTime.secSince() > 10)
      {
	myFindingDockTry++;
	if (myFindingDockTry == 2)
	{
	  ArLog::log(ArLog::Normal, "Moving forwards to get dock back");
	  myNeedToResendDockCommand = true;
	  myRobot->move(25);
	  myRobot->enableMotors();
	  mySentDockCommandTime.setToNow();
	}
	if (myFindingDockTry == 3)
	{
	  ArLog::log(ArLog::Normal, "Moving backwards to get dock back");
	  myRobot->enableMotors();
	  myRobot->move(-50);
	  myNeedToResendDockCommand = true;
	  mySentDockCommandTime.setToNow();
	}
	if (myFindingDockTry > 3)
	{
	  ArLog::log(ArLog::Normal, 
		     "Failed dock: Failed 3 tries to dock");
	  myStatus = "Failed Dock";
	  undock();
	}
      }
      else if (myNeedToResendDockCommand && 
	       mySentDockCommandTime.secSince() > 2)
      {
	ArLog::log(ArLog::Normal, "Resending dock command");
	myNeedToResendDockCommand = false;
	myRobot->comInt(68, 1);
      }
    }

  }
  if (myState == DOCKING)
  {
    if (printing)
      printf("Docking... %ld %d\n", myStartedDriveTo.secSince(),
	     myRobot->getFlags() & ArUtil::BIT10);
    // if we're docked
    if (mySentDockCommand && (myRobot->getFlags() & ArUtil::BIT10))
    {
      ArLog::log(ArLog::Normal, "Docked");
      if (myForcedDock && myRobot->haveStateOfCharge())
	myStatus = "Docked because of low state of charge";
      else if (myForcedDock)
	myStatus = "Docked because of low voltage";
      else
	myStatus = "Docked";

      switchState(DOCKED);
    }
    // if we sent the command but it didn't work
    else if (mySentDockCommand && mySentDockCommandTime.secSince() > 10)
    {
      myFindingDockTry++;
      if (myFindingDockTry == 2)
      {
	ArLog::log(ArLog::Normal, "Moving forwards to find dock");
	myNeedToResendDockCommand = true;
	myRobot->move(25);
	myRobot->enableMotors();
	mySentDockCommandTime.setToNow();
      }
      if (myFindingDockTry == 3)
      {
	ArLog::log(ArLog::Normal, "Moving backwards to find dock");
	myRobot->enableMotors();
	myRobot->move(-50);
	myNeedToResendDockCommand = true;
	mySentDockCommandTime.setToNow();
      }
      if (myFindingDockTry > 3)
      {
	ArLog::log(ArLog::Normal, 
		   "Dock Failed: Failed 3 tries to dock");
	driveInFailed();
	undock();
      }
    }
    else if (mySentDockCommand && myNeedToResendDockCommand && 
	     mySentDockCommandTime.secSince() > 2)
    {
      ArLog::log(ArLog::Normal, "Resending dock command");
      myNeedToResendDockCommand = false;
      myRobot->comInt(68, 1);
    }
    // if we sent the dock command just chill
    else if (mySentDockCommand)
    {

    }
    // if we're there and haven't sent the command
    else if (myDriveTo.isActive() && 
	     myDriveTo.getState() == ArActionTriangleDriveTo::STATE_SUCCEEDED)
    {
      if (printing)
	printf("Sent dock\n");
      myStatus = "Trying to dock";
      mySentDockCommand = true;
      myRobot->comInt(68, 1);
      mySentDockCommandTime.setToNow();
      ArLog::log(ArLog::Normal, "Sending dock command");
    }
    // if we failed getting there
    else if (myDriveTo.isActive() && 
	     myDriveTo.getState() == ArActionTriangleDriveTo::STATE_FAILED)
    {
      ArLog::log(ArLog::Normal, 
		 "Dock Failed: Could not find dock target");
      driveInFailed();
      undock();
    }
    // if it took longer than 30 seconds to drive in
    else if (myDriveTo.isActive() && myStartedDriveTo.secSince() > 30)
    {
      ArLog::log(ArLog::Normal, 
		 "Dock Failed: Took too long to find target");
      myStatus = "Failed dock";
      driveInFailed();
      undock();
    }
    // if we've gotten to our goal and need to drive into the dock
    else if (myGoalDone && !myDriveTo.isActive())
    {
      ArLog::log(ArLog::Normal, "Driving to docking triangle");
      myGroup.activateExclusive();
      myGoalDone = false;
      myStartedDriveTo.setToNow();
      myDrivingIntoDockCBList.invoke();
    }
  }
  else if (myState == UNDOCKING)
  {
    if (printing)
      printf("Undocking %ld\n", myStartedState.secSince());
    if (myStartedState.secSince() >= 4)
    {
      myRobot->stop();
      ArLog::log(ArLog::Normal, "Undocked");
      switchState(UNDOCKED);
      resumeInterrupted(true);
    }
    else if (myStartedState.secSince() >= 2 && !myUndockingMoveSent)
    {
      myUndockingMoveSent = true;
      myRobot->enableMotors();      
      myRobot->move(-25);
    }
  }
  else if (myState == UNDOCKED)
  {
    if (myNeedToPathPlanToDock && !myServer->idleProcessingPending())
    {
      myGoalDone = false;
      myDriveTo.deactivate();
      mySentDockCommand = false;
      myNeedToResendDockCommand = false;
      myNeedToPathPlanToDock = false;

      ArMapInterface *arMap;
      ArMapObject *dockObject = NULL;
      arMap = myPathTask->getAriaMap();
      
      dockObject = findDock(arMap);
      
      if (dockObject == NULL || 
	  !myPathTask->pathPlanToPose(dockObject->getPose(), true))
      {
	myStatus = "Failed dock";
	resumeInterrupted(false);
	// no callback here since this'll only fail if the path planning
	// is completely hosed (if there's no path the goal failed will be
	// called, not this)
	return;
      }
      //myStatus = "Driving to dock";
      myDockName = dockObject->getName();
      myStatus = "Going to dock at ";
      myStatus += myDockName;
      ArLog::log(ArLog::Normal, "Docking at %s", myDockName.c_str());
      myFindingDockTry = 1;
      switchState(DOCKING);
      myDrivingToDockCBList.invoke();
    }
    else
    {
      ArLog::log(ArLog::Terse, "Unable to dock because no valid dock or path planning not set up correctly");
      myStatus = "Unable to dock";
      switchState(UNDOCKED);
      resumeInterrupted(true);
    }

  }
}

AREXPORT void ArServerModeDockPioneer::dock(void)
{
  if (myIsActive && myState == DOCKED)
    return;

  activate();
  if (!myIsActive || myState != UNDOCKED)
    return;
  myNeedToPathPlanToDock = true;
  switchState(UNDOCKED);
  /*
  myGoalDone = false;
  myDriveTo.deactivate();
  mySentDockCommand = false;
  myNeedToResendDockCommand = false;

  ArMapInterface *arMap;
  ArMapObject *dockObject = NULL;
  arMap = myPathTask->getAriaMap();

  dockObject = findDock(arMap);

  if (dockObject == NULL || 
      !myPathTask->pathPlanToPose(dockObject->getPose(), true))
  {
    myStatus = "Failed dock";
    resumeInterrupted(false);
    // no callback here since this'll only fail if the path planning
    // is completely hosed (if there's no path the goal failed will be
    // called, not this)
    return;
  }
  myStatus = "Driving to dock";
  ArLog::log(ArLog::Normal, "Docking");
  myDockName = dockObject->getName();
  myFindingDockTry = 1;
  switchState(DOCKING);
  myDrivingToDockCBList.invoke();
  */
}

AREXPORT void ArServerModeDockPioneer::undock(void)
{
  if (myState == DOCKED && myForcedDock)
    return;
  if (myState == UNDOCKED)
    deactivate();
  else if (myState == DOCKED)
  {
    myStatus = "Undocking";
    myUndockingMoveSent = false;
    switchState(UNDOCKING);
    ArLog::log(ArLog::Normal, "Undocking");
    myRobot->comInt(68, 0);
  }
  else if (myState == DOCKING)
  {
    ArLog::log(ArLog::Normal, "Undocking");
    if (mySentDockCommand)
    {
      myStatus = "Undocking";
      myUndockingMoveSent = false;
      switchState(UNDOCKING);
      myRobot->comInt(68, 0);
    }
    else
    {
      switchState(UNDOCKED);
      ArLog::log(ArLog::Normal, "Undocked");
      resumeInterrupted(true);
    }
  }
  // if we're already undocking ignore this
  else if (myState == UNDOCKING)
  {
  }
    

}

AREXPORT void ArServerModeDockPioneer::deactivate()
{
  myNeedToPathPlanToDock = false;
  ArServerModeDock::deactivate();
}

AREXPORT void ArServerModeDockPioneer::checkDock(void)
{
  if (myRobot->isConnected() && (myRobot->getFlags() & ArUtil::BIT10))
  {
    ArLog::log(ArLog::Normal, "Robot is already docked, activating dock mode");
    myStatus = "Docked";
    switchState(DOCKED);
    activate();
    clearInterrupted();
  }
}

AREXPORT void ArServerModeDockPioneer::forceUnlock(void)
{
  myRobot->comInt(68, 0);
  ArUtil::sleep(10);
  myState = UNDOCKED;
  ArServerMode::forceUnlock();
}

AREXPORT void ArServerModeDockPioneer::goalDone(ArPose /*pose*/)
{
  if (!myIsActive)
    return;
  myStatus = "Driving into dock";
  myGoalDone = true;
}

AREXPORT void ArServerModeDockPioneer::goalFailed(ArPose /*pose*/)
{
  if (!myIsActive || myState != DOCKING)
    return;
  gotoFailed();
  resumeInterrupted(false);
  switchState(UNDOCKED);
}

/**
   @param server the server where we put the docking commands

   @param robot the robot we're using

   @param locTask the task for localization (so we can say its okay to
   be in a wall)

   @param pathTask the task for driving around

   @param useChargeState whether to use the charge state to know when
   docking or not

   @param approachDist distance in front of triangle vertex to use as
   the approach point (before the final drive in)

   @param backOutDist distance to back out, 0 means use old method
   which goes out as far as it had to drive in
 **/
AREXPORT ArServerModeDockTriangleBump::ArServerModeDockTriangleBump(
	ArServerBase *server, ArRobot *robot, 
	ArLocalizationTask *locTask, ArPathPlanningInterface *pathTask,
	bool useChargeState, double approachDist, double backOutDist, 
	ArFunctor *shutdownFunctor) : 
  ArServerModeDock(server, robot, locTask, pathTask, useChargeState,
		   shutdownFunctor),
  myGroup(robot),
  myDriveTo("triangleDriveTo", 0, approachDist, 100, 100, 30),
  myBackGroup(robot),
  myLimiterBackwards("backwards limiter",
		     -600, -1200, -200, 1.2),
  myGoalDoneCB(this, &ArServerModeDockTriangleBump::goalDone),
  myGoalFailedCB(this, &ArServerModeDockTriangleBump::goalFailed)
{
  myPathTask->addGoalDoneCB(&myGoalDoneCB);
  myPathTask->addGoalFailedCB(&myGoalFailedCB);
  myStallsAsBumps = false;

  myDisableDockCalled = true;
  myDriveFromValid = false;
  myHitDock = false;

  myDriveTo.setGotoVertex(true);
  myDriveTo.setMaxLateralDist(500);
  myDriveTo.setMaxDistBetweenLinePoints(50);
  myGroup.addAction(&myDriveTo, 60);
  myGroup.addAction(new ArActionStop, 50);
  
  myDrivingInMovementParamsAction.setParameters(0, 0, 0, 0, 25, 25, 25);
  myGroup.addAction(&myDrivingInMovementParamsAction, 30);

  myDesiredBackOutDist = backOutDist;

  // original
  //myBackGroup.addAction(new ArActionLimiterBackwards("backwards limiter",
  //					     -200, 0, -200, 1.2), 51);
  /* attempted new
  myLimiterBackwards.setParameters(150, 
				   200, 50, 0, 
				   200, 50, 0,
				   600, false, 0);
  myLimiterBackwards.setUseLocationDependentDevices(false);
  */
  // params on the backwards limiter are up in the constructor
  myBackGroup.addAction(&myLimiterBackwards, 51);
  myBackGroup.addAction(&myGoto, 50);

  myBackingOutMovementParamsAction.setParameters(100, 100, 100, 100, 
						 25, 25, 25);
  myBackGroup.addAction(&myBackingOutMovementParamsAction, 60);

  myUndockingEStopped = false;

  myIsSim = false;
  myDockType = "";
  myNeedToPathPlanToDock = false;
}

AREXPORT ArServerModeDockTriangleBump::~ArServerModeDockTriangleBump()
{

}

AREXPORT void ArServerModeDockTriangleBump::userTask(void)
{
  bool printing = false;
  int frontBump;
  int frontBumpMask = (ArUtil::BIT1 | ArUtil::BIT2 | ArUtil::BIT3 | 
		       ArUtil::BIT4 | ArUtil::BIT5 | ArUtil::BIT6);
  if (printing)
    printf("State %d\n", myState);
  if (myState == DOCKED)
  {
    // if we've been docked for more than a second and our motors are
    // enabled disable them
    if (!myIsSim && myRobot->areMotorsEnabled() && 
	myStartedState.mSecSince() > 1000)
    {
      ArLog::log(ArLog::Normal, "Dock disabling motors");
      myRobot->disableMotors();
    }

    if (isDocked())
    {
      myLastPowerGood.setToNow();
    }
    if (myLastPowerGood.secSince() >= 30)
    {
      //switchState(UNDOCKED);
      ArLog::log(ArLog::Normal, 
	  "Failed dock lost power after there, last power was %d seconds ago", 
		 myLastPowerGood.secSince());
      
      myStatus = "Failed Dock";
      myRetryDock = true;
      undock();
    }
  }
  if (myState == DOCKING)
  {
    frontBump = ((myRobot->getStallValue() & 0xff00) >> 8) & frontBumpMask;
    if (myStallsAsBumps && (myRobot->isLeftMotorStalled() ||
			    myRobot->isRightMotorStalled()))
      frontBump |= ArUtil::BIT0;
      
    if (printing)
      printf("Docking... %ld %d\n", myStartedDriveTo.secSince(),
	     isDocked());
    // if we're there and haven't sent the command
    if (myHitDock && isDocked())
    {
      ArLog::log(ArLog::Normal, "Docked");
      if (myForcedDock && myRobot->haveStateOfCharge())
	myStatus = "Docked because of low state of charge";
      else if (myForcedDock)
	myStatus = "Docked because of low voltage";
      else
	myStatus = "Docked";
      myLastPowerGood.setToNow();
      switchState(DOCKED);
    }
    // if we sent the command but it didn't work
    else if (myHitDock && myHitDockTime.secSince() > 10)
    {
      ArLog::log(ArLog::Normal, 
		 "Dock Failed");
      myRetryDock = true;
      driveInFailed();
      undock();
    }
    // if we sent the dock command just chill
    else if (myHitDock)
    {

    }
    else if (myDriveTo.isActive() && 
	     frontBump)
    {
      ArLog::log(ArLog::Normal, "Hit dock");
      enableDock();
      myHitDock = true;
      myHitDockTime.setToNow();
      myDriveTo.deactivate();
    }
    // if we failed getting there
    else if (myDriveTo.isActive() && 
	     myDriveTo.getState() == ArActionTriangleDriveTo::STATE_FAILED)
    {
      ArLog::log(ArLog::Normal, 
		 "Dock Failed: Could not find dock target");
      myRetryDock = true;

      // if we haven't seen a vertex we will deactivate myDriveTo
      // since it didn't do anything... which will make the undock
      // call so that it will skip the backing up straight to
      // undock... this is to make it so the robot won't back up if it
      // didn't see a triangle without changing the code path much
      if (!myDriveTo.getVertexSeen())
	myDriveTo.deactivate();
      driveInFailed();
      undock();
    }
    // if it took longer than 30 seconds to drive in
    else if (myDriveTo.isActive() && myStartedDriveTo.secSince() > 30)
    {
      ArLog::log(ArLog::Normal, 
		 "Dock Failed: Took too long to find target");
      myStatus = "Failed dock";
      myRetryDock = true;
      driveInFailed();
      undock();

    }
    // if we've gotten to our goal and need to drive into the dock
    else if (myGoalDone && !myDriveTo.isActive())
    {
      ArLog::log(ArLog::Normal, "Driving into dock");
      beforeDriveInCallback();
      myDriveFromValid = true;
      myDriveFrom = myRobot->getEncoderPose();
      myGroup.activateExclusive();
      myGoalDone = false;
      myStartedDriveTo.setToNow();
      myDrivingIntoDockCBList.invoke();
    }
  }
  else if (myState == UNDOCKING)
  {
    // if the robot is estopped, then do nothing and return... this is
    // so that the robot will stay docked until the estop is pressed,
    // but keep trying to undock, for the least confusing interaction
    if (myRobot->isEStopPressed())
    {
      myStatus = "Undocking failing, robot estopped";
      myUndockingEStopped = true;
      return;
    }

    if (myUndockingEStopped && !myRobot->isEStopPressed())
    {
      myUndockingEStopped = false;
      myStartedState.secSince();
      myStatus = "Undocking";
    }

    myHitDock = false;
    if (printing)
      printf("Undocking %ld\n", myStartedState.secSince());

    // make it so that we're always trying to move while undocking, so
    // the sonar kick in before we start backing up
    myRobot->forceTryingToMove();

    if (!myDisableDockCalled)
    {
      disableDock();
      myDisableDockCalled = true;
    }

    if (myStartedState.secSince() < 2 )
    {
      //ArLog::log(ArLog::Normal, "Sending motor enable");
      myRobot->enableMotors();
    }

    if (myStartedState.secSince() >= 3 && !myRobot->areMotorsEnabled())
      myRobot->enableMotors();

    if (myStartedState.secSince() >= 3 && !myStartedBacking)
    {
      myStartedBacking = true;
      //myGoto.setEncoderGoalRel(myDistanceToBack, -180, true);
      myGoto.setDistance(-myDistanceToBack, true);
      myRobot->clearDirectMotion();
      myBackGroup.activateExclusive();
    }

    if (myStartedBacking && myGoto.haveAchievedDistance())
    {
      backoutCallback();
    }

    /// MPL taking out the isDocked since it has caused some problems
    if (myStartedBacking && myGoto.haveAchievedDistance())// && !isDocked())
    {
      afterDriveOutCallback();
      if (myRetryDock)
      {
	myRetryDock = false;
	switchState(UNDOCKED);
	ArLog::log(ArLog::Normal, "Undocked, retrying dock");
	dock();
	return;
      }
      myStatus = "Stopped";
      switchState(UNDOCKED);
      ArLog::log(ArLog::Normal, "Undocked");
      resumeInterrupted(true);
    }
    if (myStartedState.secSince() >= 60)
    {
      ArLog::log(ArLog::Normal, "Error undocking, taken too long");
      myStartedState.setToNow();
      myStatus = "Undocking failing";
    }
  }
  else if (myState == UNDOCKED)
  {
    if (myNeedToPathPlanToDock && !myServer->idleProcessingPending())
    {
      myNeedToPathPlanToDock = false;
      myGoalDone = false;
      myDriveTo.deactivate();
      myRobot->clearDirectMotion();
      
      
      ArMapInterface *arMap;
      ArMapObject *dockObject = NULL;
      arMap = myPathTask->getAriaMap();
      dockObject = findDock(arMap);
      // go to either the first dock object or to the goal dock if thats not around
      if (dockObject == NULL || 
	  !myPathTask->pathPlanToPose(dockObject->getPose(), true))
      {
	if (dockObject != NULL)
	{
	  myStatus = "Cannot drive to dock at ";
	  myStatus += dockObject->getName();
	}
	else
	{
	  myStatus = "Cannot drive to dock";
	}
	if (myForcedDock)
	{
	  myRetryDock = true;
	  undock();
	  // no callback here since this'll only fail if the path planning
	  // is completely hosed (if there's no path the goal failed will
	  // be called, not this)... in fact this whole my retry dock and
	  // all that is probably not needed
	}
	else
	  resumeInterrupted(false);
	return;
      }
      myDockName = dockObject->getName();
      myStatus = "Going to dock at ";
      myStatus += dockObject->getName();
      ArLog::log(ArLog::Normal, "Docking at %s", dockObject->getName());
      switchState(DOCKING);
      myDrivingToDockCBList.invoke();
    }
    else
    {
      ArLog::log(ArLog::Terse, "Unable to dock because no valid dock or path planning not set up correctly");
      myStatus = "Unable to dock";
      myRetryDock = false;
      switchState(UNDOCKED);
      resumeInterrupted(true);
    }

  }
 
}

AREXPORT void ArServerModeDockTriangleBump::dock(void)
{
  if (myIsActive && myState == DOCKED)
    return;

  activate();
  /// MPL changed
  //if (!myIsActive || myState != UNDOCKED)
  // to this  for the shutdown change
  if (!myIsActive || (myState != UNDOCKED && !myRetryDock))
    return;
  myNeedToPathPlanToDock = true;
  switchState(UNDOCKED);
  /*
  myGoalDone = false;
  myDriveTo.deactivate();
  myRobot->clearDirectMotion();


  ArMapInterface *arMap;
  ArMapObject *dockObject = NULL;
  arMap = myPathTask->getAriaMap();
  dockObject = findDock(arMap);
  // go to either the first dock object or to the goal dock if thats not around
  if (dockObject == NULL || 
      !myPathTask->pathPlanToPose(dockObject->getPose(), true))
  {
    if (dockObject != NULL)
    {
      myStatus = "Cannot drive to dock at ";
      myStatus += dockObject->getName();
    }
    else
    {
      myStatus = "Cannot drive to dock";
    }
    if (myForcedDock)
    {
      myRetryDock = true;
      undock();
      // no callback here since this'll only fail if the path planning
      // is completely hosed (if there's no path the goal failed will
      // be called, not this)... in fact this whole my retry dock and
      // all that is probably not needed
    }
    else
      resumeInterrupted(false);
    return;
  }
  myDockName = dockObject->getName();
  myStatus = "Going to dock at ";
  myStatus += dockObject->getName();
  ArLog::log(ArLog::Normal, "Docking at %s", dockObject->getName());
  switchState(DOCKING);
  myDrivingToDockCBList.invoke();
  */
}

AREXPORT void ArServerModeDockTriangleBump::undock(void)
{
  if (myState == DOCKED && myForcedDock && !myRetryDock)
    return;

  myRobot->enableMotors();
  if (myState == UNDOCKED)
  {
    resumeInterrupted(true);
  }
  else if (myState == DOCKED)
  {
    if (myDesiredBackOutDist > .1)
      myDistanceToBack = myDesiredBackOutDist;
    else if (myDriveFromValid)
      myDistanceToBack = myRobot->getEncoderPose().findDistanceTo(myDriveFrom);
    else
      myDistanceToBack = myRobot->getRobotRadius() + 1000;
    myStatus = "Undocking";
    myStartedBacking = false;
    myDisableDockCalled = false;
    switchState(UNDOCKING);
    ArLog::log(ArLog::Normal, "Undocking");
  }
  else if (myState == DOCKING)
  {
    myRobot->stop();
    ArLog::log(ArLog::Normal, "Undocking");
    if (myDesiredBackOutDist > .1)
      myDistanceToBack = myDesiredBackOutDist;
    else if (myDriveFromValid)
      myDistanceToBack = myRobot->getEncoderPose().findDistanceTo(myDriveFrom);
    else
      myDistanceToBack = myRobot->getRobotRadius() + 1000;
    myStartedBacking = false;
    if (myHitDock || myDriveTo.isActive())
    {
      myStatus = "Undocking";
      myDisableDockCalled = false;
      switchState(UNDOCKING);
    }
    else
    {
      myDisableDockCalled = true;
      if (myRetryDock)
      {
	myRetryDock = false;
	switchState(UNDOCKED);
	ArLog::log(ArLog::Normal, "Undocked, retrying dock");
	dock();
	return;
      }
      myStatus = "Stopped";
      switchState(UNDOCKED);
      ArLog::log(ArLog::Normal, "Undocked");
      resumeInterrupted(true);
    }
  }
  // if we're already undocking ignore this
  else if (myState == UNDOCKING)
  {
  }
}

AREXPORT void ArServerModeDockTriangleBump::deactivate()
{
  // if we're not forced docking and retry dock is set then cancel retry dock
  if (!myForcedDock && myRetryDock)
  {
    ArLog::log(ArLog::Normal, "Cancelling dock retry");
    myRetryDock = false;
  }
  myNeedToPathPlanToDock = false;
  ArServerModeDock::deactivate();
}

AREXPORT void ArServerModeDockTriangleBump::checkDock(void)
{
  if (myRobot->isConnected() && isDocked())
  {
    ArLog::log(ArLog::Normal, "Robot is already docked, activating dock mode");
    myStatus = "Docked";
    switchState(DOCKED);
    myHitDock = true;
    activate();
    clearInterrupted();
  }
}

AREXPORT void ArServerModeDockTriangleBump::forceUnlock(void)
{
  // taking out the explicit command and having it call disable dock
  //myRobot->comInt(68, 0);
  disableDock();
  ArUtil::sleep(10);
  myState = UNDOCKED;
  ArServerMode::forceUnlock();
}


AREXPORT void ArServerModeDockTriangleBump::goalDone(ArPose /*pose*/ )
{
  if (!myIsActive)
    return;
  myStatus = "Driving into dock";
  myGoalDone = true;
}

AREXPORT void ArServerModeDockTriangleBump::goalFailed(ArPose /*pose*/)
{
  if (!myIsActive || myState != DOCKING)
    return;
  gotoFailed();
  ArLog::log(ArLog::Normal, "Couldn't get to dock");
  myStatus = "Failed dock";
  // MPL commented out this line for shutting down change
  //switchState(UNDOCKED);
  // if we're not forced see if anything else wants to go
  if (!myForcedDock)
    resumeInterrupted(false);
  
  // if we're still active try to redock
  if (isActive())
  {
    ArLog::log(ArLog::Normal, "Retrying dock");
    // MPL added this line for shutting down change
    myRetryDock = true;
    myNeedToPathPlanToDock = true;
    switchState(UNDOCKED); 
    //dock();
  }
  // MPL added these two lines for shutting down change
  else
  {
    myNeedToPathPlanToDock = false;
    switchState(UNDOCKED);
  }
}

AREXPORT void ArServerModeDockTriangleBump::addToConfig(ArConfig *config)
{
  std::string section;
  section = "Docking";

  if (config == NULL)
    return;

  ArServerModeDock::addToConfig(config);

  myDrivingInMovementParamsAction.addToConfig(config,
				     section.c_str(),
				     "DrivingIntoDock");

  myBackingOutMovementParamsAction.addToConfig(config,
				     section.c_str(),
				     "BackingOutOfDock");

}



AREXPORT ArServerModeDockPatrolBot::ArServerModeDockPatrolBot(
	ArServerBase *serverBase, ArRobot *robot,
	ArLocalizationTask *locTask, ArPathPlanningInterface *pathTask,
	ArFunctor *shutdownFunctor) :
  ArServerModeDockTriangleBump(serverBase, robot, locTask, pathTask, true,
			       300, 300, shutdownFunctor)
{
  //myRobot->requestIOPackets();
  myDockType = "DockPatrolBot";
  myTouchIgnoreIllegalPoseFlag = true;
}

AREXPORT ArServerModeDockPatrolBot::~ArServerModeDockPatrolBot()
{

}

AREXPORT bool ArServerModeDockPatrolBot::isDocked(void)
{
  //unsigned char powerbits = myRobot->getIODigIn(4);
  // powerbits & ArUtil::BIT3);
  return (myRobot->getFlags() & ArUtil::BIT10);
}

AREXPORT void ArServerModeDockPatrolBot::enableDock(void)
{
  myRobot->comInt(68, 1);
  return;
}

AREXPORT void ArServerModeDockPatrolBot::disableDock(void)
{
  myRobot->comInt(68, 0);
  return;
}

AREXPORT void ArServerModeDockPatrolBot::checkDock(void)
{
  // first we see if it looks like we might be docked (laser readings in the robot)
  ArRangeDevice *laser;
  const std::list<ArSensorReading *> *rawReadings;
  std::list<ArSensorReading *>::const_iterator rawReadingsIt;
  const ArSensorReading *reading;

  int numClose, numReadings;

  if ((laser = myRobot->findLaser(1)) != NULL &&
      (rawReadings = laser->getRawReadings()) != NULL)
  {
    for (rawReadingsIt = rawReadings->begin(), numClose = 0, numReadings = 0; 
	 rawReadingsIt != rawReadings->end();
	 rawReadingsIt++)
    {
      reading = (*rawReadingsIt);
      if (!reading->getIgnoreThisReading())
      {
	numReadings++;
	if (reading->getRange() < myRobot->getRobotWidth() / 2)
	  numClose++;
      }
    }

    if (numReadings == 0)
    {
      ArLog::log(ArLog::Normal, "No laser readings yet, checking dock");
    }

    if (numClose < .20 * numReadings)
    {
      ArLog::log(ArLog::Normal, "Not checking for dock since not enough laser readings to be in dock (%d %d %4.1f%%)", numClose, numReadings, numClose / (double) numReadings * 100.0);
      return;
    }
    if (numClose >= .20 * numReadings)
    {
      ArLog::log(ArLog::Normal, "Robot is already docked (because of laser readings %d %d %4.1f%%), activating dock mode", numClose, numReadings, numClose / (double) numReadings * 100.0);
      myStatus = "Docked";
      switchState(DOCKED);
      myHitDock = true;
      activate();
      clearInterrupted();
    }
  }
  else
  {
    ArLog::log(ArLog::Normal, "No laser, will check for dock");
    ArServerModeDockTriangleBump::checkDock();
  }
  if (isActive())
  {
    if (myTouchIgnoreIllegalPoseFlag && myLocTask == NULL)
    {
      ArLog::log(ArLog::Normal, "Not touching ignore illegal pose flag (cd no loc)");
    }
    else if (myTouchIgnoreIllegalPoseFlag)
    {
      ArLog::log(myDockInfoLogLevel, "Setting ignore illegal pose flag (cd)");
      myLocTask->setIgnoreIllegalPoseFlag(true);
    }
    else
    {
      ArLog::log(ArLog::Normal, "Not touching ignore illegal pose flag (cd)");
    }
  }
}

AREXPORT void ArServerModeDockPatrolBot::beforeDriveInCallback()
{
  if (myTouchIgnoreIllegalPoseFlag && myLocTask == NULL)
  {
    ArLog::log(ArLog::Normal, "Not touching ignore illegal pose flag (bd no loc)");
  }
  else if (myTouchIgnoreIllegalPoseFlag)
  {
    ArLog::log(myDockInfoLogLevel, "Setting ignore illegal pose flag (bd)");
    myLocTask->setIgnoreIllegalPoseFlag(true);
  }
  else
  {
    ArLog::log(ArLog::Normal, "Not touching ignore illegal pose flag (bd)");
  }
}

AREXPORT void ArServerModeDockPatrolBot::afterDriveOutCallback()
{
  if (myTouchIgnoreIllegalPoseFlag && myLocTask == NULL)
  {
    ArLog::log(ArLog::Normal, "Not touching ignore illegal pose flag (ad no loc)");
  }
  else if (myTouchIgnoreIllegalPoseFlag)
  {
    ArLog::log(myDockInfoLogLevel, "Clearing ignore illegal pose flag");
    myLocTask->setIgnoreIllegalPoseFlag(false);
  }
  else
  {
    ArLog::log(ArLog::Normal, "Not touching ignore illegal pose flag (ad)");
  }
}

void ArServerModeDockPatrolBot::setTouchIgnoreIllegalPoseFlag(
	bool touchIgnoreIllegalPoseFlag)
{
  myTouchIgnoreIllegalPoseFlag = touchIgnoreIllegalPoseFlag;
}

AREXPORT ArServerModeDockPatrolBotNiMH::ArServerModeDockPatrolBotNiMH(
	ArServerBase *serverBase, ArRobot *robot,
	ArLocalizationTask *locTask, ArPathPlanningInterface *pathTask,
	ArFunctor *shutdownFunctor) :
  ArServerModeDockPatrolBot(serverBase, robot, locTask, pathTask, 
			    shutdownFunctor),
  myProcessFileCB(this, &ArServerModeDockPatrolBotNiMH::processFile),
  myPBNiMHUserTaskCB(this, &ArServerModeDockPatrolBotNiMH::pbNiMHUserTask)
{
  myDockType = "DockPatrolBotNiMH";

  myStallsAsBumps = true;
  myNiMHDockUntilDone = true;
  myNiMHUseMicrocycling = false;
  myNiMHMicrocyclingDockingStateOfCharge = 60;
  sprintf(myBalanceChargeTimeOfDay, "0:00");
  sprintf(myBalanceChargeDayOfWeek, "Sunday");
  myAutoDockForBalanceCharge = false;
  myDoneChargingVoltage = 0;
  myDoneChargingStateOfCharge = 0;
  myDoneChargingAtFloat = false;
  myDoneChargingMinutes = 0;
  myNiMHDockVoltage = 22.5;
  myWasActive = false;
  myNeedBalance = false;

  myLastChargeState = ArRobot::CHARGING_UNKNOWN;

  myPBNiMHUserTaskCB.setName("PatrolbotNiMHUserTask");
  myRobot->addUserTask("PatrolbotNiMHUserTask", 60,  &myPBNiMHUserTaskCB);
}

AREXPORT ArServerModeDockPatrolBotNiMH::~ArServerModeDockPatrolBotNiMH()
{

}

AREXPORT bool ArServerModeDockPatrolBotNiMH::subclassAddParamsDoneCharging(
	ArConfig *config, const char *section)
{

		   
  config->addParam(
	  ArConfigArg("DockUntilDoneCharging", &myNiMHDockUntilDone,
		      "If true then the robot will dock until it is done charging (to balance or float)"),
	  section, ArPriority::NORMAL);

  baseAddParamMinutesToChargeFor(config, section);

  baseAddParamToChargeTo(config, section);


  Aria::getConfig()->addParam(
	  ArConfigArg(ArConfigArg::SEPARATOR), section, 
	  ArPriority::NORMAL);
  
  config->addParam(
	  ArConfigArg("MicrocycleDocking", &myNiMHUseMicrocycling,
		      "If true then the robot will use microcycle docking, which docks at MicrocycleDockingStateOfCharge and charges until it is done with a microcycle (to overcharge, balance, or float).  Note that this will let the robot charge quickly but it won't run for very long, but this has by far the best runtime to charging ratio."),
	  section, ArPriority::NORMAL);


  config->addParam(
	  ArConfigArg("MicrocycleDockingStateOfCharge", 
		      &myNiMHMicrocyclingDockingStateOfCharge,
		      "If true then the robot will use microcycle docking, which docks at MicrocycleStateOfCharge and charges until it is done with a microcycle (to overcharge, balance, or float).  Note that this will let the robot charge quickly but it won't run for very long, but this has by far the best runtime to charging ratio.", 50, 70),
	  section, ArPriority::NORMAL);

  Aria::getConfig()->addParam(
	  ArConfigArg(ArConfigArg::SEPARATOR), section, 
	  ArPriority::NORMAL);

  config->addParam(
	  ArConfigArg("AutoDockFallbackVoltage", &myNiMHDockVoltage,
		      "Voltage to charge at, this should be set low as a fallback in case state of charge is misbehaving, not for regular docking"),
	  section, ArPriority::EXPERT);
  

  Aria::getConfig()->addParam(
	  ArConfigArg(ArConfigArg::SEPARATOR), section, 
	  ArPriority::NORMAL);

  config->addParam(
	  ArConfigArg("BalanceChargeDayOfWeek", 
		      myBalanceChargeDayOfWeek,
		      "The day of the week to do a balance charge, at BalanceChargeTimeOfDay",
		      sizeof(myBalanceChargeDayOfWeek)),
	  section, ArPriority::NORMAL, "Choices:Sunday;;Monday;;Tuesday;;Wednesday;;Thursday;;Friday;;Saturday");
  config->addParam(
	  ArConfigArg("BalanceChargeTimeOfDay", 
		      myBalanceChargeTimeOfDay,
		      "The time of day to do a balance charge, on BalanceChargeDayOfWeek",
		      sizeof(myBalanceChargeTimeOfDay)),
	  section, ArPriority::NORMAL, "Time:hh:mm AP");
  config->addParam(
	  ArConfigArg("AutoDockForBalanceCharge", 
		      &myAutoDockForBalanceCharge,
		      "True to autodock when the balance charge time of day and day of week happens, false to just do the balance charge next time the robot autodocks"),
	  section, ArPriority::NORMAL);
  Aria::getConfig()->addParam(
	  ArConfigArg(ArConfigArg::SEPARATOR), section, 
	  ArPriority::NORMAL);

  myProcessFileCB.setName("ArServerModeDockPatrolBotNiMH");
  Aria::getConfig()->addProcessFileCB(&myProcessFileCB);
  
  return true;
}

bool ArServerModeDockPatrolBotNiMH::processFile(void)
{
  calculateNextBalance();
  // return true no matter what since if this time is wrong its not
  // something the user can fix anyways
  return true;
}

bool ArServerModeDockPatrolBotNiMH::calculateNextBalance(void)
{

  // if we already need a balance charge, don't recalculate it until
  // we've done one
  if (myNeedBalance)
    return true;
  
  time_t todaysBalance;
  bool ok;
  todaysBalance = ArUtil::parseTime(myBalanceChargeTimeOfDay, &ok);
  if (!ok)
  {
    ArLog::log(ArLog::Normal, "ArServerModeDockPatrolBotNiMH: Bad time for balance charge time of day");
    myNextBalance = 0;
    return false;
  }

  int balanceDayOfWeek = -1;
  if (strcasecmp(myBalanceChargeDayOfWeek, "Sunday") == 0)
    balanceDayOfWeek = 0;
  else if (strcasecmp(myBalanceChargeDayOfWeek, "Monday") == 0)
    balanceDayOfWeek = 1;
  else if (strcasecmp(myBalanceChargeDayOfWeek, "Tuesday") == 0)
    balanceDayOfWeek = 2;
  else if (strcasecmp(myBalanceChargeDayOfWeek, "Wednesday") == 0)
    balanceDayOfWeek = 3;
  else if (strcasecmp(myBalanceChargeDayOfWeek, "Thursday") == 0)
    balanceDayOfWeek = 4;
  else if (strcasecmp(myBalanceChargeDayOfWeek, "Friday") == 0)
    balanceDayOfWeek = 5;
  else if (strcasecmp(myBalanceChargeDayOfWeek, "Saturday") == 0)
    balanceDayOfWeek = 6;

  if (balanceDayOfWeek < 0)
  {
    ArLog::log(ArLog::Normal, "ArServerModeDockPatrolBotNiMH: Bad day of week for balance charge day of week");
    myNextBalance = 0;
    return false;
  }

  struct tm now; 
  time_t nowSeconds = time(NULL);
  if (ArUtil::localtime(&nowSeconds, &now) == NULL || 
      now.tm_wday < 0 || now.tm_wday > 6) 
  {
    ArLog::log(ArLog::Normal, "ArServerModeDockPatrolBotNiMH: Can't get day of week...");
    myNextBalance = 0;
    return false;
  }

  int numDaysTillBalance = -1;
  if (balanceDayOfWeek > now.tm_wday)
    numDaysTillBalance = balanceDayOfWeek - now.tm_wday;
  else if (balanceDayOfWeek < now.tm_wday)
    numDaysTillBalance = balanceDayOfWeek + 7 - now.tm_wday;
  else if (balanceDayOfWeek == now.tm_wday && todaysBalance < time(NULL))
    numDaysTillBalance = 7;
  else if (balanceDayOfWeek == now.tm_wday && todaysBalance >= time(NULL))
    numDaysTillBalance = 0;

  if (numDaysTillBalance < 0)
  {
    ArLog::log(ArLog::Normal, "ArServerModeDockPatrolBotNiMH: Can't figure out days until next balance");
    myNextBalance = 0;
    return false;
  }

  myNextBalance = todaysBalance + numDaysTillBalance * 24 * 3600;

  char timeBuf[1024];
  char *timeStr;
  timeStr = ctime(&myNextBalance);
  strncpy(timeBuf, timeStr, 20);
  timeBuf[20] = '\0';
  ArLog::log(myDockModeLogLevel,
	     "Next docking balance charge from at %s, which is %ld seconds away", 
	     timeBuf, myNextBalance - time(NULL));
  return true; 
}

void ArServerModeDockPatrolBotNiMH::pbNiMHUserTask(void)
{
  if (!myWasActive && isActive() && myNeedBalance)
  {
    if (myState == DOCKED)
      ArLog::log(ArLog::Normal, "Docked for balance charge now");
    else 
      ArLog::log(ArLog::Normal, "Docking for balance charge");
  }

  myWasActive = isActive();

  if (myState == DOCKED)
  {
    ArRobot::ChargeState chargeState = myRobot->getChargeState();

    if (myLastChargeState != ArRobot::CHARGING_BALANCE && 
	chargeState == ArRobot::CHARGING_BALANCE)
      myStartedBalance.setToNow();
    if (myLastChargeState != ArRobot::CHARGING_FLOAT &&
	chargeState == ArRobot::CHARGING_FLOAT)
    {
      long secs;
      secs = myLastDocked.secSince();
      
      char buf[1024];

      myDockInfoMutex.lock();

      if (myNeedBalance)
      {
	sprintf(buf, 
		"DockInfo:NeededBalanceCharge: Balance took %02ld:%02ld:%02ld",
		secs / 3600, (secs % 3600) / 60, secs % 60);
	myDockInfoNeededBalanceCharge = buf;
      }
      else
      {
	sprintf(buf, 
		"DockInfo:BalanceCharge: Balance took %02ld:%02ld:%02ld",
		secs / 3600, (secs % 3600) / 60, secs % 60);
	myDockInfoBalanceCharge = buf;
      }

      myDockInfoMutex.unlock();
      
      ArLog::log(myDockInfoLogLevel, buf);
    }

    myLastChargeState = chargeState;
  }
  else
  {
    myLastChargeState = ArRobot::CHARGING_UNKNOWN;
  }

  if (!myNeedBalance && myNextBalance != 0 && myNextBalance <= time(NULL))
  {
    ArLog::log(ArLog::Normal, "Docking needs balance charge");
    myNeedBalance = true;
  }
}     

void ArServerModeDockPatrolBotNiMH::userTask(void)
{
  if (myState == DOCKED && myNeedBalance && 
      strstr(myStatus.c_str(), " for balance charge") == NULL)
  {
    myStatus = myStatus + " for balance charge";
    ArLog::log(ArLog::Normal, myStatus.c_str());
  }

  if (myRobot->getChargeState() == ArRobot::CHARGING_FLOAT)
  {
    myLastBalanced = time(NULL);
    if (myNeedBalance)
    {
      ArLog::log(ArLog::Normal, "Docking done with balance charge (got to float)");
      myNeedBalance = false;
      calculateNextBalance();
      char status[1024];
      char *balanceStr;
      strcpy(status, myStatus.c_str());
      if ((balanceStr = strstr(status, " for balance charge")) != NULL)
	*balanceStr = '\0';
      myStatus = status;
    }
  }

  ArServerModeDockPatrolBot::userTask();
}

AREXPORT bool ArServerModeDockPatrolBotNiMH::subclassDoneCharging(void)
{
  ArRobot::ChargeState chargeState = myRobot->getChargeState();

  // this shouldn't be needed since this shouldn't be called by the
  // base class if we're not done charging, but just in case
  if (subclassNotDoneCharging())
    return false;

  // if we're docking until done for this it means balance or float
  if (myNiMHDockUntilDone && 
      (chargeState == ArRobot::CHARGING_BALANCE || 
       chargeState == ArRobot::CHARGING_FLOAT)) 
  {
    ArLog::log(ArLog::Normal, 
	       "Done docking because charging finished (got to balanced or float)");
    return true;
  }

  // if we're microcycling then done just means anything but
  // bulk... the 10 second check is because CHARGING_NOT counts
  // because it happens between BULK and OVERCHARGE... but it also
  // starts as NOT
  if (myNiMHUseMicrocycling && myStartedState.secSince() > 10 && 
      (chargeState == ArRobot::CHARGING_NOT || 
       chargeState == ArRobot::CHARGING_OVERCHARGE || 
       chargeState == ArRobot::CHARGING_BALANCE || 
       chargeState == ArRobot::CHARGING_FLOAT))
  {
    ArLog::log(ArLog::Normal, 
	       "Done docking because microcycle charge finished");
    return true;
  }

  return false;
}

AREXPORT bool ArServerModeDockPatrolBotNiMH::isDocked(void)
{
  return (
	  (myRobot->getFlags() & ArUtil::BIT10) || 
	  myRobot->getChargeState() == ArRobot::CHARGING_BULK ||
	  myRobot->getChargeState() == ArRobot::CHARGING_OVERCHARGE ||
	  myRobot->getChargeState() == ArRobot::CHARGING_BALANCE ||
	  myRobot->getChargeState() == ArRobot::CHARGING_FLOAT);
}

AREXPORT bool ArServerModeDockPatrolBotNiMH::subclassNotDoneCharging(void)
{
  if (myNeedBalance)
    return true;
  return false;
}

AREXPORT bool ArServerModeDockPatrolBotNiMH::subclassNeedsAutoDock(
	char *reason, size_t lenOfReason)
{
  if (myUsingAutoDock && myAutoDockForBalanceCharge && myNeedBalance)
  {
    if (reason != NULL && lenOfReason > 0)
    {
      snprintf(reason, lenOfReason,
	       "Docking because balance charge is needed");
      reason[lenOfReason] = '\0';
    }
    return true;
  }
  if (myUsingAutoDock && 
      myRobot->getRealBatteryVoltage() < myNiMHDockVoltage)
  {
    if (reason != NULL && lenOfReason > 0)
    {
      snprintf(reason, lenOfReason,
	       "Docking because voltage (%.1fV) is below docking fallback voltage (%.1fV)",
	       myRobot->getRealBatteryVoltage(), myNiMHDockVoltage);
      reason[lenOfReason] = '\0';
    }
    return true;
  }
  if (myUsingAutoDock && myNiMHUseMicrocycling && 
      myRobot->haveStateOfCharge() && 
      myRobot->getStateOfCharge() < myNiMHMicrocyclingDockingStateOfCharge)
  {
    if (reason != NULL && lenOfReason > 0)
    {
      snprintf(reason, lenOfReason,
 "Docking because state of charge is %.1f%% (docks for microcycle at %.1f%%)",
	       myRobot->getStateOfCharge(), 
	       myNiMHMicrocyclingDockingStateOfCharge);
      reason[lenOfReason] = '\0';
    }
    return true;
  }
  else
    return false;
}

AREXPORT void ArServerModeDockPatrolBotNiMH::setAutoDock(bool autoDocking)
{
  printf("%d %d %d\n", myUsingAutoDock, autoDocking, myNeedBalance);
  if (myUsingAutoDock && !autoDocking && myNeedBalance)
  {
    ArLog::log(ArLog::Normal, "Auto dock being disabling, cancelling balance charge");
    myNeedBalance = false;
    calculateNextBalance();
  }
  ArServerModeDockPatrolBot::setAutoDock(autoDocking);
}

AREXPORT void ArServerModeDockPatrolBotNiMH::setNeedBalance(bool needBalance)
{
  myNeedBalance = needBalance;
  if (needBalance)
    ArLog::log(ArLog::Normal, "Need balance being set");
  else
    ArLog::log(ArLog::Normal, "Need balance being cleared");
}

AREXPORT void ArServerModeDockPatrolBotNiMH::subclassGetDockInfoString(
	char *dockInfoStr, size_t dockInfoStrLength, bool terse)
{
  myDockInfoMutex.lock();

  if (myDockInfoNeededBalanceCharge.empty() && 
      myDockInfoBalanceCharge.empty())
  {
    myDockInfoMutex.unlock();
    return;
  }

  // return what info we have
  if (!myDockInfoNeededBalanceCharge.empty() && 
      !myDockInfoBalanceCharge.empty())
    snprintf(dockInfoStr, dockInfoStrLength, "%s\n%s\n", 
	     myDockInfoNeededBalanceCharge.c_str(),
	     myDockInfoBalanceCharge.c_str());
  else if (!myDockInfoNeededBalanceCharge.empty())
    snprintf(dockInfoStr, dockInfoStrLength, "%s\n", 
	     myDockInfoNeededBalanceCharge.c_str());
  else if (!myDockInfoBalanceCharge.empty())
    snprintf(dockInfoStr, dockInfoStrLength, "%s\n", 
	     myDockInfoBalanceCharge.c_str());

  dockInfoStr[dockInfoStrLength - 1] = '\0'; 

  myDockInfoMutex.unlock();
}

AREXPORT ArServerModeDockSimulator::ArServerModeDockSimulator(
	ArServerBase *serverBase, ArRobot *robot,
	ArLocalizationTask *locTask, ArPathPlanningInterface *pathTask, 
	ArFunctor *shutdownFunctor, const char *dockType) :
  ArServerModeDockTriangleBump(serverBase, robot, locTask, pathTask, 
			       false, 300, 300, shutdownFunctor)
{
  setStallsAsBumps(true);
  myIsSim = true;
  myDockType = dockType;
}

AREXPORT ArServerModeDockSimulator::~ArServerModeDockSimulator()
{

}

AREXPORT bool ArServerModeDockSimulator::isDocked(void)
{
  if (myState == UNDOCKING || myState == UNDOCKED)
    return false;
  else
    return true;
}

AREXPORT void ArServerModeDockSimulator::enableDock(void)
{
  return;
}

AREXPORT void ArServerModeDockSimulator::disableDock(void)
{
  return;
}

/**
   The simulator redefines this so that the simulated robot doesn't
   always come up docked.
 **/
AREXPORT void ArServerModeDockSimulator::checkDock(void)
{
  return;
}

// This use to have 1000, 0 for docking params
AREXPORT ArServerModeDockPowerBot::ArServerModeDockPowerBot(
	ArServerBase *serverBase, ArRobot *robot,
	ArLocalizationTask *locTask, ArPathPlanningInterface *pathTask,
	bool isOldDock, ArFunctor *shutdownFunctor, bool useChargeState, 
	int oldDockAnalogPort) :
  ArServerModeDockTriangleBump(serverBase, robot, locTask, pathTask, 
			       useChargeState, 1000, 1000, 
			       shutdownFunctor) 
{
  myIsOldDock = isOldDock;
  myOldDockAnalogPort = oldDockAnalogPort;
  if (myIsOldDock)
  {
    myRobot->requestIOPackets();
    myBatteryChargingGood = 2.40;
  }
  myDockType = "DockPowerBot";
}

AREXPORT ArServerModeDockPowerBot::~ArServerModeDockPowerBot()
{

}

AREXPORT void ArServerModeDockPowerBot::addToConfig(ArConfig *config)
{
  std::string section;
  section = "Docking";

  if (config == NULL)
    return;

  ArServerModeDockTriangleBump::addToConfig(config);

  if (myIsOldDock)
  {
    config->addParam(
	    ArConfigArg("BatteryChargingGood", &myBatteryChargingGood, 
			"This is the maximum analog voltage from the charge current sensor that this class will take as charging... the sensor should read at 2.50v with no load and every amp charging reduces this by .04... so 2.3 should be at least 5 amps... this is not always at the center and has some noise so its a parameter here but shouldn't really need to be adjusted",
			0, 5),
	    section.c_str(), ArPriority::TRIVIAL);
  }
}

AREXPORT bool ArServerModeDockPowerBot::isDocked(void)
{
  //ArLog::log(ArLog::Normal, "## %d %d", myIsOldDock, (myRobot->getFlags() & ArUtil::BIT10));
  // if its the new dock check the flags
  if (!myIsOldDock && (myRobot->getFlags() & ArUtil::BIT10))
    return true;
  // if its the old dock check the current
  else if (myIsOldDock && 
      myRobot->getIOAnalogVoltage(myOldDockAnalogPort) < myBatteryChargingGood)
    return true;
  else
    return false;
}

AREXPORT void ArServerModeDockPowerBot::enableDock(void)
{
  ArLog::log(ArLog::Normal, "comInt 68, 1");
  myRobot->comInt(68, 1);
  return;
}

AREXPORT void ArServerModeDockPowerBot::disableDock(void)
{
  ArLog::log(ArLog::Normal, "comInt 68, 0");
  myRobot->comInt(68, 0);
  return;
}

AREXPORT void ArServerModeDockPowerBot::backoutCallback(void)
{
  ArLog::log(ArLog::Normal, "comInt 68, 0");
  myRobot->comInt(68, 0);
  return;
}

/**
   @param server the server where we put the docking commands

   @param robot the robot we're using

   @param locTask the task for localization (so we can say its okay to
   be in a wall)

   @param pathTask the task for driving around

   @param useChargeState whether to use the charge state to know when
   docking or not

   @param approachDist distance in front of triangle vertex to use as
   the approach point (before the final drive in)

   @param pullOutDist distance to back out, 0 means use old method
   which goes out as far as it had to drive in
 **/
AREXPORT ArServerModeDockTriangleBumpBackwards::ArServerModeDockTriangleBumpBackwards(
	ArServerBase *server, ArRobot *robot, 
	ArLocalizationTask *locTask, ArPathPlanningInterface *pathTask,
	bool useChargeState, double approachDist, double finalDist, 
	double backIntoDockDist, double pullOutDist, ArFunctor *shutdownFunctor) :
  ArServerModeDock(server, robot, locTask, pathTask, useChargeState,
		   shutdownFunctor),
  myGroup(robot),
  myDriveTo("triangleDriveTo", finalDist, approachDist, 100, 100, 30),
  myDrivingInLimiter("drivingInDeceleratingLimiter"),
  myBackIntoDockGroup(robot),
  myTurnGroup(robot),
  myPullOutGroup(robot),
  myLimiterForwards("limiter forwards",
		     600, 1200, 200, 1.2),
  myLimiterBackwards("limiter backwards",
		     600, 1200, 200, 1.2),
  myGoalDoneCB(this, &ArServerModeDockTriangleBumpBackwards::goalDone),
  myGoalFailedCB(this, &ArServerModeDockTriangleBumpBackwards::goalFailed)
{
  myDesiredApproachDist = approachDist; 
  myDesiredFinalDist = finalDist;
  myDesiredBackIntoDockDist = backIntoDockDist;
  myDesiredPullOutDist = pullOutDist;

  myPathTask->addGoalDoneCB(&myGoalDoneCB);
  myPathTask->addGoalFailedCB(&myGoalFailedCB);
  myStallsAsBumps = true;

  myDisableDockCalled = true;
  myHitDock = false;
  myDriveFromValid = false;

  myMaxLateralDist = 150;
  myMaxAngleMisalignment = 3;
  myMaxDistBetweenLinePoints = 50;

  myDrivingInLimiter.setStopRotationToo(true);
  myGroup.addAction(&myDrivingInLimiter, 70);
  myDriveTo.setGotoVertex(false);
  myDriveTo.setMaxLateralDist(myMaxLateralDist);
  myDriveTo.setMaxAngleMisalignment(myMaxAngleMisalignment);
  myDriveTo.setMaxDistBetweenLinePoints(myMaxDistBetweenLinePoints);
  myDriveTo.setTriangleParams(157, -146, 157);
  myGroup.addAction(&myDriveTo, 60);
  myGroup.addAction(new ArActionStop, 50);
  
  myMovementParamsAction.setParameters(0, 0, 0, 0, 25, 25, 25);
  myGroup.addAction(&myMovementParamsAction, 30);

  myDesiredPullOutDist = pullOutDist;

  /// MPL TODO these should come from the dock type (ie lynx) rather
  /// than be common, but since that's our only dock yet I didn't
  /// delay a shipment for it
  myDrivingInAvoidanceAbsoluteWidth = 700;
  myDrivingInAvoidanceAbsoluteLengthFront = 450; 

  myPullOutGoto.setSpeed(200);
  myPullOutGroup.addAction(&myLimiterForwards, 52);
  myPullOutGroup.addAction(&myLimiterBackwards, 51);
  myPullOutGroup.addAction(&myPullOutGoto, 50);

  myBackIntoDockGoto.setSpeed(50);
  myBackIntoDockGroup.addAction(&myBackIntoDockGoto, 50);

  myTurnMovementParamsAction.setParameters(0, 0, 0, 0, 25, 25, 25);
  myTurnGroup.addAction(&myTurnMovementParamsAction, 30);
  myTurnGroup.addAction(&myTurnActionInput, 50);



  myUndockingEStopped = false;
  
  myIsSim = false;
  myDockType = "";
  myNeedToPathPlanToDock = false;

}

AREXPORT ArServerModeDockTriangleBumpBackwards::~ArServerModeDockTriangleBumpBackwards()
{

}

AREXPORT void ArServerModeDockTriangleBumpBackwards::userTask(void)
{
  bool printing = false;
  int bump;
  int bumpMask = (ArUtil::BIT1 | ArUtil::BIT2 | ArUtil::BIT3 | 
		       ArUtil::BIT4 | ArUtil::BIT5 | ArUtil::BIT6);

  if (printing)
    printf("State %d\n", myState);
  if (myState == DOCKED)
  {
    // if we've been docked for more than a second and our motors are
    // enabled disable them
    /* not doing this since the robot can get pushed out...
    if (!myIsSim && myRobot->areMotorsEnabled() && 
	myStartedState.mSecSince() > 1000)
    {
      ArLog::log(ArLog::Normal, "Dock disabling motors");
      myRobot->disableMotors();
    }
    */
    if (isDocked())
    {
      myLastDocked.setToNow();
    }
    bool needRedock = false;
    
    if (myLastDocked.secSince() >= 30)
    {
      ArLog::log(ArLog::Normal, 
	  "Failed dock lost power after there, last power was %d seconds ago", 
		 myLastDocked.secSince());
      needRedock = true;
    }
    /*
    else if (myRobot->haveStateOfCharge() && 
	     myRobot->getStateOfCharge() < myLastDockedSOC)
    {
      ArLog::log(ArLog::Normal, 
		 "Failed dock lost power after there, SOC went from %g to %g",
		 myLastDockedSOC, myRobot->getStateOfCharge());
      needRedock = true;
    }
    */

    if (needRedock)
    {
      myStatus = "Failed Dock";
      myRetryDock = true;
      driveInFailed();
      undock();
    }
    myLastDockedSOC = myRobot->getStateOfCharge();
    myLastDockedVoltage = myRobot->getRealBatteryVoltage();
  }

  if (myState == DOCKING)
  {
    bump = (myRobot->getStallValue() & 0x00ff) & bumpMask;

    if (myStallsAsBumps && (myRobot->isLeftMotorStalled() ||
			    myRobot->isRightMotorStalled()))
      bump |= ArUtil::BIT0;
      
    if (printing)
      printf("Docking... %ld %d\n", myStartedDriveTo.secSince(),
	     isDocked());

    // we can't tell if we're charging or not... but only if we're
    // charging, so just assume we're good if we're in there

    // if we're there and haven't sent the command
    if (myHitDock && isDocked())
    {

      if (myForcedDock && myRobot->haveStateOfCharge())
      {
	myStatus = "Docked because of low state of charge";
	ArLog::log(ArLog::Normal, "Docked because of low state of charge");
      }
      else if (myForcedDock)
      {
	myStatus = "Docked because of low voltage";
	ArLog::log(ArLog::Normal, "Docked because of low voltage");
      }
      else
      {
	myStatus = "Docked";
	ArLog::log(ArLog::Normal, "Docked");
      }
      myLastDocked.setToNow();
      myLastDockedSOC = myRobot->getStateOfCharge();
      myLastDockedVoltage = myRobot->getRealBatteryVoltage();
      switchState(DOCKED);
    }
    // if we sent the command but it didn't work
    else if (myHitDock && myHitDockTime.secSince() > 20)
    {
      ArLog::log(ArLog::Normal, 
		 "Dock Failed (no charging after charger power was good)");
      myRetryDock = true;
      driveInFailed();
      undock();
    }
    // if we sent the dock command just chill
    else if (myHitDock)
    {

    }
    else if (myBackIntoDockGoto.isActive() && 
	     (bump || myRobot->isChargerPowerGood()))
    {
      if (bump)
      {
	ArLog::log(ArLog::Normal, "Hit dock");
      }
      else
      {
	ArLog::log(ArLog::Normal, "Charger power good");
      }
      myRobot->stop();
      myRobot->clearDirectMotion();
      enableDock();
      myHitDock = true;
      myHitDockTime.setToNow();
      myBackIntoDockGoto.deactivate();
    }
    // if it took longer than 30 seconds to drive in
    else if (myBackIntoDockGoto.isActive() && 
	     (myBackIntoDockGoto.haveAchievedDistance() || 
	      myStartedBackIntoDock.secSince() > 30))
    {
      if (myBackIntoDockGoto.haveAchievedDistance())
      {
	ArLog::log(ArLog::Normal, 
		   "Dock Failed: Backed all the way without seeing power good");
      }
      else
      {
	ArLog::log(ArLog::Normal, 
		   "Dock Failed: Took too long to back in");
      }
      myStatus = "Failed dock";
      myRetryDock = true;
      driveInFailed();
      undock();
    }
    // if we turned around, then back in...
    else if (myTurnActionInput.isActive() &&
	     fabs(ArMath::subAngle(myRobot->getTh(), myTurnHeading)) <= 2.1 && 
	     ArMath::fabs(myRobot->getVel()) < 5 &&
	     ArMath::fabs(myRobot->getRotVel()) < 2 && 
	     myStartedTurn.mSecSince() > 500)
    {
      ArLog::log(ArLog::Normal, "Turned around, backing in");
      /*
      myLastDockingSOC = myRobot->getStateOfCharge();
      myDockingLargestSOCJump = 0;
      myLastDockingVoltage = myRobot->getRealBatteryVoltage();
      myDockingLargestVoltageJump = 0;
      */
      beforeDriveInCallback();
      myDrivingIntoDockCBList.invoke();
      myBackIntoDockGoto.setPrinting(true);
      myBackIntoDockGoto.setDistance(-myDesiredBackIntoDockDist, true);
      myBackIntoDockGroup.activateExclusive();
      myStartedBackIntoDock.setToNow();
    }
    // if it took longer than 30 seconds to drive in
    else if (myTurnActionInput.isActive() && 
	     myStartedTurn.secSince() > 30)
    {
      ArLog::log(ArLog::Normal, 
		 "Dock Failed: Took too long to turn");
      myStatus = "Failed dock";
      myRetryDock = true;
      driveInFailed();
      undock();
    }
    // if we got there, then turn around
    else if (myDriveTo.isActive() && 
	     myDriveTo.getState() == ArActionTriangleDriveTo::STATE_SUCCEEDED)
    {
      ArLog::log(ArLog::Normal, "Got to triangle, turning around");
      myStartedTurn.setToNow();
      myTurnActionInput.setVel(0);
      // this was just using the angle from the dock goal now, but
      // we'll use the opposite of the robot heading to account for
      // when the triangle isn't aligned with the goal
      myTurnHeading = ArMath::addAngle(myRobot->getTh(), 180);
      myTurnActionInput.setHeading(myTurnHeading);
      myTurnGroup.activateExclusive();
    }
    // if we failed getting there
    else if (myDriveTo.isActive() && 
	     myDriveTo.getState() == ArActionTriangleDriveTo::STATE_FAILED)
    {
      ArLog::log(ArLog::Normal, 
		 "Dock Failed: Could not find dock target");
      myRetryDock = true;

      // if we haven't seen a vertex we will deactivate myDriveTo
      // since it didn't do anything... which will make the undock
      // call so that it will skip the backing up straight to
      // undock... this is to make it so the robot won't back up if it
      // didn't see a triangle without changing the code path much
      if (!myDriveTo.getVertexSeen())
	myDriveTo.deactivate();
      driveInFailed();
      undock();
    }
    // if it took longer than 30 seconds to drive in
    else if (myDriveTo.isActive() && 
	     myStartedDriveTo.secSince() > 30)
    {
      ArLog::log(ArLog::Normal, 
		 "Dock Failed: Took too long to find target");
      myStatus = "Failed dock";
      myRetryDock = true;
      driveInFailed();
      undock();

    }
    // if we've gotten to our goal and need to drive to the triangle
    else if (myGoalDone && !myDriveTo.isActive())
    {
      ArLog::log(ArLog::Normal, "Driving into dock (from %.0f %.0f %.1f)", myRobot->getX(), myRobot->getY(), myRobot->getTh());
      myDriveTo.setParameters(myDesiredFinalDist, myDesiredApproachDist);
      myDriveTo.setMaxLateralDist(myMaxLateralDist);
      myDriveTo.setMaxAngleMisalignment(myMaxAngleMisalignment);
      myDriveTo.setMaxDistBetweenLinePoints(myMaxDistBetweenLinePoints);

      double sideClearance = (myDrivingInAvoidanceAbsoluteWidth - 
			      myRobot->getRobotWidth()) / 2;
      double frontClearance = (myDrivingInAvoidanceAbsoluteLengthFront - 
			       myRobot->getRobotLengthFront());
      myDrivingInLimiter.setParameters(
	      frontClearance, // clearance
	      sideClearance, // side clearance at slow speed
	      0, // padding at slow speed
	      0, // slow speed
	      sideClearance, // side clearance at fast speed
	      0, // padding at fast speed
	      2000, // fast speed
	      600,
	      false,
	      0);

      myDriveFromValid = true;
      myDriveFrom = myRobot->getEncoderPose();
      myGroup.activateExclusive();

      if (myDrivingInAvoidanceAbsoluteWidth < .1 || myDrivingInAvoidanceAbsoluteLengthFront < .1)
	myDrivingInLimiter.deactivate();
      else
	ArLog::log(ArLog::Normal, "Driving in avoiding frontClearance %.0f sideClearance %.0f", frontClearance, sideClearance);

      myGoalDone = false;
      myStartedDriveTo.setToNow();
    }
  }
  else if (myState == UNDOCKING)
  {
    // if the robot is estopped, then do nothing and return... this is
    // so that the robot will stay docked until the estop is pressed,
    // but keep trying to undock, for the least confusing interaction
    if (myRobot->isEStopPressed())
    {
      myStatus = "Undocking failing, robot estopped";
      myUndockingEStopped = true;
      return;
    }

    if (myUndockingEStopped && !myRobot->isEStopPressed())
    {
      myUndockingEStopped = false;
      myStartedState.secSince();
      myStatus = "Undocking";
    }

    myHitDock = false;
    if (printing)
      printf("Undocking %ld\n", myStartedState.secSince());

    // make it so that we're always trying to move while undocking, so
    // the sonar kick in before we start backing up
    myRobot->forceTryingToMove();

    if (!myDisableDockCalled)
    {
      disableDock();
      myDisableDockCalled = true;
    }

    if (myStartedState.secSince() < 2 )
    {
      //ArLog::log(ArLog::Normal, "Sending motor enable");
      myRobot->enableMotors();
    }

    if (myStartedState.secSince() >= 3 && !myRobot->areMotorsEnabled())
      myRobot->enableMotors();

    if (myStartedState.secSince() >= 3 && !myStartedPullingOut)
    {
      myStartedPullingOut = true;
      //myPullOutGoto.setEncoderGoalRel(myDistanceToPullOut, -180, true);
      myPullOutGoto.setDistance(myDistanceToPullOut, true);
      myRobot->clearDirectMotion();
      myPullOutGroup.activateExclusive();
    }

    if (myStartedPullingOut && myPullOutGoto.haveAchievedDistance())
    {
      pulloutCallback();
    }

    /// MPL taking out the isDocked since it has caused some problems
    if (myStartedPullingOut && myPullOutGoto.haveAchievedDistance())// && !isDocked())
    {
      afterDriveOutCallback();
      if (myRetryDock)
      {
	myRetryDock = false;
	switchState(UNDOCKED);
	ArLog::log(ArLog::Normal, "Undocked, retrying dock");
	dock();
	return;
      }
      myStatus = "Stopped";
      switchState(UNDOCKED);
      ArLog::log(ArLog::Normal, "Undocked");
      resumeInterrupted(true);
    }
    if (myStartedState.secSince() >= 60)
    {
      ArLog::log(ArLog::Normal, "Error undocking, taken too long");
      myStartedState.setToNow();
      myStatus = "Undocking failing";
    }
  }
  else if (myState == UNDOCKED)
  {
    if (myNeedToPathPlanToDock && !myServer->idleProcessingPending())
    {
      myNeedToPathPlanToDock = false;
      myGoalDone = false;
      myDriveTo.deactivate();
      myRobot->clearDirectMotion();
      
      
      ArMapInterface *arMap;
      ArMapObject *dockObject = NULL;
      arMap = myPathTask->getAriaMap();
      dockObject = findDock(arMap);
      // go to either the first dock object or to the goal dock if thats not around
      if (dockObject == NULL || 
	  !myPathTask->pathPlanToPose(dockObject->getPose(), true))
      {
	if (dockObject != NULL)
	{
	  myStatus = "Cannot drive to dock at ";
	  myStatus += dockObject->getName();
	}
	else
	{
	  myStatus = "Cannot drive to dock";
	}
	if (myForcedDock)
	{
	  myRetryDock = true;
	  undock();
	  // no callback here since this'll only fail if the path planning
	  // is completely hosed (if there's no path the goal failed will
	  // be called, not this)... in fact this whole my retry dock and
	  // all that is probably not needed
	}
	else
	  resumeInterrupted(false);
	return;
      }
      // this sets it to the goal heading for sanity, though it'll get
      // reset later
      myTurnHeading = ArMath::subAngle(dockObject->getPose().getTh(), 180);
      myDockName = dockObject->getName();
      myStatus = "Going to dock at ";
      myStatus += dockObject->getName();
      ArLog::log(ArLog::Normal, "Docking at %s", dockObject->getName());
      // MPL added this extra
      myHitDock = false;
      switchState(DOCKING);
      myDrivingToDockCBList.invoke();
    }
    else
    {
      ArLog::log(ArLog::Terse, "Unable to dock because no valid dock or path planning not set up correctly");
      myStatus = "Unable to dock";
      myRetryDock = false;
      switchState(UNDOCKED);
      resumeInterrupted(true);
    }

  }
 
}

AREXPORT void ArServerModeDockTriangleBumpBackwards::dock(void)
{
  if (myIsActive && myState == DOCKED)
    return;

  activate();
  /// MPL changed
  //if (!myIsActive || myState != UNDOCKED)
  // to this  for the shutdown change
  if (!myIsActive || (myState != UNDOCKED && !myRetryDock))
    return;
  myNeedToPathPlanToDock = true;
  switchState(UNDOCKED);
  /*
  myGoalDone = false;
  myDriveTo.deactivate();
  myRobot->clearDirectMotion();


  ArMapInterface *arMap;
  ArMapObject *dockObject = NULL;
  arMap = myPathTask->getAriaMap();
  dockObject = findDock(arMap);
  // go to either the first dock object or to the goal dock if thats not around
  if (dockObject == NULL || 
      !myPathTask->pathPlanToPose(dockObject->getPose(), true))
  {
    if (dockObject != NULL)
    {
      myStatus = "Cannot drive to dock at ";
      myStatus += dockObject->getName();
    }
    else
    {
      myStatus = "Cannot drive to dock";
    }
    if (myForcedDock)
    {
      myRetryDock = true;
      undock();
      // no callback here since this'll only fail if the path planning
      // is completely hosed (if there's no path the goal failed will
      // be called, not this)... in fact this whole my retry dock and
      // all that is probably not needed
    }
    else
      resumeInterrupted(false);
    return;
  }
  myDockName = dockObject->getName();
  myStatus = "Going to dock at ";
  myStatus += dockObject->getName();
  ArLog::log(ArLog::Normal, "Docking at %s", dockObject->getName());
  switchState(DOCKING);
  myDrivingToDockCBList.invoke();
  */
}

AREXPORT void ArServerModeDockTriangleBumpBackwards::undock(void)
{
  if (myState == DOCKED && myForcedDock && !myRetryDock)
    return;

  myRobot->enableMotors();
  if (myState == UNDOCKED)
  {
    resumeInterrupted(true);
  }
  else if (myState == DOCKED)
  {
    if (myDesiredPullOutDist > .1)
      myDistanceToPullOut = myDesiredPullOutDist;
    else if (myDriveFromValid)
      myDistanceToPullOut = myRobot->getEncoderPose().findDistanceTo(myDriveFrom);
    else
      myDistanceToPullOut = myRobot->getRobotRadius() + 1000;
    myStatus = "Undocking";
    myStartedPullingOut = false;
    myDisableDockCalled = false;
    switchState(UNDOCKING);
    ArLog::log(ArLog::Normal, "Undocking");
  }
  else if (myState == DOCKING)
  {
    myRobot->stop();
    ArLog::log(ArLog::Normal, "Undocking");
    if (myDesiredPullOutDist > .1)
      myDistanceToPullOut = myDesiredPullOutDist;
    else if (myDriveFromValid)
      myDistanceToPullOut = myRobot->getEncoderPose().findDistanceTo(myDriveFrom);
    else
      myDistanceToPullOut = myRobot->getRobotRadius() + 1000;
    myStartedPullingOut = false;
    if (myHitDock || myBackIntoDockGoto.isActive())
    {
      myStatus = "Undocking";
      myDisableDockCalled = false;
      switchState(UNDOCKING);
    }
    else
    {
      myDisableDockCalled = true;
      if (myRetryDock)
      {
	myRetryDock = false;
	switchState(UNDOCKED);
	ArLog::log(ArLog::Normal, "Undocked, retrying dock");
	dock();
	return;
      }
      myStatus = "Stopped";
      switchState(UNDOCKED);
      ArLog::log(ArLog::Normal, "Undocked");
      resumeInterrupted(true);
    }
  }
  // if we're already undocking ignore this
  else if (myState == UNDOCKING)
  {
  }
}

AREXPORT void ArServerModeDockTriangleBumpBackwards::deactivate()
{
  // if we're not forced docking and retry dock is set then cancel retry dock
  if (!myForcedDock && myRetryDock)
  {
    ArLog::log(ArLog::Normal, "Cancelling dock retry");
    myRetryDock = false;
  }
  myNeedToPathPlanToDock = false;
  ArServerModeDock::deactivate();
}

AREXPORT void ArServerModeDockTriangleBumpBackwards::checkDock(void)
{
  if (myRobot->isConnected() && isDocked())
  {
    ArLog::log(ArLog::Normal, "Robot is already docked, activating dock mode");
    myStatus = "Docked";
    switchState(DOCKED);
    myHitDock = true;
    activate();
    clearInterrupted();
  }
}

AREXPORT void ArServerModeDockTriangleBumpBackwards::forceUnlock(void)
{
  disableDock();
  ArUtil::sleep(10);
  myState = UNDOCKED;
  ArServerMode::forceUnlock();
}


AREXPORT void ArServerModeDockTriangleBumpBackwards::goalDone(ArPose /*pose*/ )
{
  if (!myIsActive)
    return;
  myStatus = "Driving into dock";
  myGoalDone = true;
}

AREXPORT void ArServerModeDockTriangleBumpBackwards::goalFailed(ArPose /*pose*/)
{
  if (!myIsActive || myState != DOCKING)
    return;
  gotoFailed();
  ArLog::log(ArLog::Normal, "Couldn't get to dock");
  myStatus = "Failed dock";
  // MPL commented out this line for shutting down change
  //switchState(UNDOCKED);
  // if we're not forced see if anything else wants to go
  if (!myForcedDock)
    resumeInterrupted(false);
  
  // if we're still active try to redock
  if (isActive())
  {
    ArLog::log(ArLog::Normal, "Retrying dock");
    // MPL added this line for shutting down change
    myRetryDock = true;
    myNeedToPathPlanToDock = true;
    switchState(UNDOCKED); 
    //dock();
  }
  // MPL added these two lines for shutting down change
  else
  {
    myNeedToPathPlanToDock = false;
    switchState(UNDOCKED);
  }
}

AREXPORT void ArServerModeDockTriangleBumpBackwards::addToConfig(ArConfig *config)
{
  std::string section;
  section = "Docking";

  if (config == NULL)
    return;

  ArServerModeDock::addToConfig(config);

  myMovementParamsAction.addToConfig(config,
				     section.c_str(),
				     "DrivingIntoDock");

  Aria::getConfig()->addParam(
	  ArConfigArg(ArConfigArg::SEPARATOR), section.c_str(), 
	  ArPriority::EXPERT);

  config->addParam(
	  ArConfigArg("TriangleApproachDist", &myDesiredApproachDist,
		      "This is the approach distance for driving into the dock... the minimum is 0, but be very careful with this since it can break everything... only Adept should adjust it", 0),
	  section.c_str(), ArPriority::EXPERT);

  config->addParam(
	  ArConfigArg("TriangleFinalDist", &myDesiredFinalDist,
		      "This is the final distance for driving into the dock... the minimum is 0, but be very careful with this since it can break everything... only Adept should adjust it", 0),
	  section.c_str(), ArPriority::EXPERT);

  config->addParam(
	  ArConfigArg("BackIntoDockDist", &myDesiredBackIntoDockDist,
		      "This is the distance the robot will back into the dock after turning around... the minimum is 0, but be very careful with this since it can break everything... only Adept should adjust it (it will also stop if the bumper triggers)", 0),
	  section.c_str(), ArPriority::EXPERT);

  config->addParam(
	  ArConfigArg("PullOutDist", &myDesiredPullOutDist,
		      "This is the distance the robot will pull out of the dock dock to undock... the minimum is 0, but be very careful with this since it can break everything... only Adept should adjust it (it will also stop if the bumper triggers)", 0),
	  section.c_str(), ArPriority::EXPERT);

  Aria::getConfig()->addParam(
	  ArConfigArg(ArConfigArg::SEPARATOR), section.c_str(), 
	  ArPriority::EXPERT);

  config->addParam(
	  ArConfigArg("DrivingInAvoidanceAbsoluteWidth", 
		      &myDrivingInAvoidanceAbsoluteWidth,
		      "The width to avoid (from the robot center, ie absolute) while driving into the dock.  0 means don't avoid anything (mm)", 0),
	  section.c_str(), ArPriority::EXPERT);

  config->addParam(
	  ArConfigArg("DrivingInAvoidanceAbsoluteLengthFront", 
		      &myDrivingInAvoidanceAbsoluteLengthFront,
		      "The length front to avoid (from the robot center, ie absolute) while driving into the dock.  0 means don't avoid anything (mm)", 0),
	  section.c_str(), ArPriority::EXPERT);

  Aria::getConfig()->addParam(
	  ArConfigArg(ArConfigArg::SEPARATOR), section.c_str(), 
	  ArPriority::EXPERT);

  config->addParam(
	  ArConfigArg("MaxLateralDist", &myMaxLateralDist,
		      "The maximum of how far to the left or right the robot will look for a vertex.  (Technically the maximum distance between the robot's center and an imaginary line that comes out from the vertex).  Any vertex more than that amount off will be totally ignored.  0 means don't use this feature (mm)", 0),
	  section.c_str(), ArPriority::EXPERT);

  config->addParam(
	  ArConfigArg("MaxAngleMisalignment", &myMaxAngleMisalignment,
		      "The maximum of how the angle of the robot and the angle of the vertex line may be off.  Any vertex line more than that amount off will be totally ignored.  0 means don't use this feature (deg)", 0),
	  section.c_str(), ArPriority::EXPERT);

  config->addParam(
	  ArConfigArg("MaxDistBetweenLinePoints", &myMaxDistBetweenLinePoints,
		      "Maximum distance between points that are turned into lines, this is a parameter to use to try and get rid of the points where a laser saw part of the target and part of the background.  0 means don't use this feature (mm)", 0),
	  section.c_str(), ArPriority::EXPERT);

  Aria::getConfig()->addParam(
	  ArConfigArg(ArConfigArg::SEPARATOR), section.c_str(), 
	  ArPriority::EXPERT);
}

AREXPORT ArServerModeDockLynx::ArServerModeDockLynx(
	ArServerBase *serverBase, ArRobot *robot,
	ArLocalizationTask *locTask, ArPathPlanningInterface *pathTask,
	ArFunctor *shutdownFunctor) :
  ArServerModeDockTriangleBumpBackwards(serverBase, robot, locTask, pathTask, 
  				true,   // Use Charge State?
          825,    // TriangleApproachDist default
          675,    // TriangleFinalDist default
          410,    // BackIntoDockDist default
          300,    // PullOutDist default
					shutdownFunctor)
{
  myDockType = "DockLynx";
  
  // doing this since simpler way since it's how the battery decides
  // it's going to be the active one
  myBattery = myRobot->findBattery(1);
  if (myBattery == NULL)
  {
    ArLog::log(ArLog::Normal, "DockLynx: Battery1 not set up, will not work correctly");
  }

  // for docking we only pay attention to the first connected battery
  // (except on disabling docking, there we tell them all to stop)
  /*
  ArBatteryMTX *battery;
  std::map<int, ArBatteryMTX *> *batteryMap;
  std::map<int, ArBatteryMTX *>::iterator it;
  batteryMap = myRobot->getBatteryMap();

  if (batteryMap == NULL)
  {
    ArLog::log(ArLog::Normal, "DockLynx: batterymap not set up, will not work correctly");
  }
  else
  {
    for (it = batteryMap->begin(); it != batteryMap->end(); it++)
    {
      battery = (*it).second;
      if (battery->isConnected())
      {
	myBattery = battery;
	break;
      }
    }
  }
  */
}

AREXPORT ArServerModeDockLynx::~ArServerModeDockLynx()
{

}

AREXPORT bool ArServerModeDockLynx::isDocked(void)
{
  if (myBattery == NULL)
    return false;

  // ignore this for now
  if (myRobot->getChargeState() == ArRobot::CHARGING_BULK || 
      myRobot->getChargeState() == ArRobot::CHARGING_OVERCHARGE || 
      myRobot->getChargeState() == ArRobot::CHARGING_FLOAT || 
      myRobot->getChargeState() == ArRobot::CHARGING_BALANCE)
    return true;
  else
    return false;
}

// we don't need to do anything to enable the dock, since it'll do the magic itself
AREXPORT void ArServerModeDockLynx::enableDock(void)
{
  return;
}

AREXPORT void ArServerModeDockLynx::disableDock(void)
{
  if (myBattery == NULL)
  {
    ArLog::log(ArLog::Normal, "DockLynx: No battery so can't disable dock");
    return;
  }

  myBattery->sendStopCharging();

  ArBatteryMTX *battery;
  std::map<int, ArBatteryMTX *> *batteryMap;
  std::map<int, ArBatteryMTX *>::iterator it;
  batteryMap = myRobot->getBatteryMap();

  if (batteryMap == NULL)
  {
    ArLog::log(ArLog::Normal, "DockLynx: No batteries so can't disable dock");
    return;
  }

  for (it = batteryMap->begin(); it != batteryMap->end(); it++)
  {
    battery = (*it).second;
    // if we've already sent it once don't send it again... if the
    // battery isn't connected don't send it either
    if (battery == myBattery || !battery->isConnected())
      continue;
    battery->sendStopCharging();
  }
}

// for the lynx we only want to restore if there's a chance we really
// are docked
AREXPORT void ArServerModeDockLynx::restoreFromDockFile(void)
{
  if (myBattery == NULL)
    return;
  
  int statusFlags = myBattery->getStatusFlags();
  
  // so only restore if the possibility is there
  bool isDockedNow = isDocked();
  if (isDockedNow || 
      (statusFlags & ArBatteryMTX::STATUS_CHARGER_ON) || 
      (statusFlags & ArBatteryMTX::STATUS_ON_CHARGER))
  {
    ArServerModeDockTriangleBumpBackwards::restoreFromDockFile();
  }
  // otherwise remove the file
  else
  {
    ArLog::log(ArLog::Normal, "ArServerModeDockLynx: Lynx was docked before, but cannot be docked now, so removing docked file");
    eraseDockFile();
  }
}

AREXPORT void ArServerModeDockLynx::checkDock(void)
{
  if (myBattery == NULL)
    return;
  
  int statusFlags = myBattery->getStatusFlags();
  
  bool isDockedNow = isDocked();
  if (isDockedNow || 
      (statusFlags & ArBatteryMTX::STATUS_CHARGER_ON) || 
      (statusFlags & ArBatteryMTX::STATUS_ON_CHARGER))
  {
    if (isDockedNow)
      ArLog::log(ArLog::Normal, "Robot is charging, activating dock mode");
    else if (statusFlags & ArBatteryMTX::STATUS_CHARGER_ON) 
      ArLog::log(ArLog::Normal, "Robot charger is on, activating dock mode");
    else if (statusFlags & ArBatteryMTX::STATUS_ON_CHARGER) 
      ArLog::log(ArLog::Normal, "Robot is on charger, activating dock mode");
    else
      ArLog::log(ArLog::Normal, "For unknown reason, activating dock mode");
    myStatus = "Docked";
    switchState(DOCKED);
    myHitDock = true;
    activate();
    clearInterrupted();
  }
}

void ArServerModeDockLynx::beforeDriveInCallback()
{
}

void ArServerModeDockLynx::afterDriveOutCallback()
{
}
