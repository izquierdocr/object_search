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
#ifndef ARSERVERCLASSES_H
#define ARSERVERCLASSES_H

#include "Aria.h"
#include "ArServerBase.h"
#include "ArServerMode.h"
#include "ArPathPlanningTask.h"
#include "ArBaseLocalizationTask.h"

#include <deque>
#include <string>

//class ArPathPlanningTask;
class ArActionPlanAndMoveToGoal;

/// Sends path information to the client
class ArServerInfoPath
{
public:
  /// Constructor
  AREXPORT ArServerInfoPath(ArServerBase *server, ArRobot *robot,
			    ArPathPlanningTask *pathTask);
  /// Destructor
  AREXPORT virtual ~ArServerInfoPath();
  /// Gets the path information
  AREXPORT void getPath(ArServerClient *client, ArNetPacket *packet);
  /// Gets the path information
  AREXPORT void getPathCurrent(ArServerClient *client, ArNetPacket *packet);
  /// Gets the local search rectangle information
  AREXPORT void getSearchRectangle(ArServerClient *client, 
				   ArNetPacket *packet);
  /// Adds the search area drawing
  AREXPORT void addSearchRectangleDrawing(ArServerInfoDrawings *drawings);
  /// Adds the simple commands to turn on and off the search rectangle
  AREXPORT void addControlCommands(ArServerHandlerCommands *handlerCommands);
  /// Enables the drawing of the search rectangle
  AREXPORT void searchRectangleEnable(void);
  /// Disables the drawing of the search rectangle
  AREXPORT void searchRectangleDisable(void);
  /// Sets the drawing data for the search rectangle
  AREXPORT void setSearchRectangleDrawingData(ArDrawingData *data,
					      bool takeOwnershipOfData);
#ifndef SWIG
  /// Gets the drawing data for the search rectangle
  AREXPORT ArDrawingData *getSearchRectangleDrawingData(void);
#endif
protected:
  void serverPathPlannerStatus(ArServerClient* client, ArNetPacket* pkt);
  void getPathInternal(ArServerClient *client, bool shortCount);
  void pathPlannerStateChanged();

  ArServerBase *myServer;
  ArRobot *myRobot;
  ArServerInfoDrawings *myDrawings;
  ArPathPlanningTask *myPathTask;
  ArServerHandlerCommands *myHandlerCommands;
  ArActionPlanAndMoveToGoal* myAction;
  bool myDrawSearchRectangle;
  ArServerInfoDrawings *myServerInfoDrawings;
  ArDrawingData *mySearchRectangleDrawingData;
  bool myOwnSearchRectangleDrawingData;
  char myPathPlannerStatus[32];
  ArFunctor2C<ArServerInfoPath, ArServerClient *, ArNetPacket *> myGetPathCB;
  ArFunctor2C<ArServerInfoPath, ArServerClient *, ArNetPacket *> myGetPathCurrentCB;
  ArFunctor2C<ArServerInfoPath, ArServerClient *, ArNetPacket *> myGetSearchRectangleCB;
  ArFunctorC<ArServerInfoPath> mySearchRectangleEnableCB;
  ArFunctorC<ArServerInfoPath> mySearchRectangleDisableCB;
  ArFunctor2C<ArServerInfoPath, ArServerClient *, ArNetPacket *> myServerPathPlannerStatusCB;
  ArFunctorC<ArServerInfoPath> myPathPlannerStateChangeCB;
    
};

/// Sends localization point information to the client
class ArServerInfoLocalization
{
public:
  /// Constructor
  AREXPORT ArServerInfoLocalization(ArServerBase *server, ArRobot *robot,
				    ArBaseLocalizationTask *locTask,
				    ArBaseLocalizationTask *locTaskForPoints = NULL);
  /// Destructor
  AREXPORT virtual ~ArServerInfoLocalization();
  /// Networking request callback to get the localization points
  AREXPORT void getLocPoints(ArServerClient *client, ArNetPacket *packet);
  /// Networking request callback to get the state of the localization
  AREXPORT void getLocState(ArServerClient *client, ArNetPacket *packet);

  /// Adds a callback for when localization starts initializing
  AREXPORT void addInitializingCB(ArFunctor *functor, int position = 50)
    { myInitCBList.addCallback(functor, position); }
  /// Removes a plain callback for when localization starts initializing
  AREXPORT void remInitializingingCB(ArFunctor *functor)
    { myInitCBList.remCallback(functor); }
  /// Adds a callback for when localization finishes initializing successfully
  AREXPORT void addInitializingSucceededCB(ArFunctor *functor, 
					   int position = 50)
    { myInitSucceededCBList.addCallback(functor, position); }
  /// Removes a plain callback for when localization finishes initializing successfully
  AREXPORT void remInitializingSucceededCB(ArFunctor *functor)
    { myInitSucceededCBList.remCallback(functor); }
  /// Adds a callback for when localization finishes initializing but is still lost
  AREXPORT void addInitializingFailedCB(ArFunctor *functor, int position = 50)
    { myInitFailedCBList.addCallback(functor, position); }
  /// Removes a plain callback for when localization finishes initializing but is still lost
  AREXPORT void remInitializingFailedCB(ArFunctor *functor)
    { myInitFailedCBList.remCallback(functor); }
  /// Adds a callback for when localization becomes lost
  AREXPORT void addLostCB(ArFunctor *functor, int position = 50)
    { myLostCBList.addCallback(functor, position); }
  /// Removes a plain callback for when localization becomes lost
  AREXPORT void remLostCB(ArFunctor *functor)
    { myLostCBList.remCallback(functor); }
  /// Sets the state to lost, for internal use
  AREXPORT void setToLostState(void);
protected:
  void stateChange(void);
  
  ArServerBase *myServer;
  ArRobot *myRobot;
  ArBaseLocalizationTask *myLocTask;
  ArBaseLocalizationTask *myLocTaskForPoints;
  enum State
  {
    GOOD = 0,
    IDLE = 1,
    LOST = 2,
    INITING =3
  };

  State myState;
  State myLastState;

  ArMutex myDataMutex;

  ArCallbackList myLostCBList;
  ArCallbackList myInitCBList;
  ArCallbackList myInitSucceededCBList;
  ArCallbackList myInitFailedCBList;

  ArFunctor2C<ArServerInfoLocalization, ArServerClient *, ArNetPacket *> myGetLocPointsCB;
  ArFunctor2C<ArServerInfoLocalization, ArServerClient *, ArNetPacket *> myGetLocStateCB;
  ArFunctorC<ArServerInfoLocalization> myStateChangeCB;
};

/// Handles calls from the client to relocalize
class ArServerHandlerLocalization
{
public:
  /// Constructor
  AREXPORT ArServerHandlerLocalization(ArServerBase *server, ArRobot *robot,
				      ArBaseLocalizationTask *locTask,
				      bool addResetToHome = true,
				      bool setSimPoseOnLocalize = true);
  /// Destructor
  AREXPORT virtual ~ArServerHandlerLocalization();
  /// Localizes to a point given
  AREXPORT void localizeToPose(ArPose pose, bool lockRobot = true,
			       double xySpread = -1, double thSpread = -1);
  /// Function that'll do the work of resetting the sim pose on localize
  AREXPORT void setSimPose(ArPose pose, bool lockRobot = true);
  /// Networking request callback that localizes the robot to home
  AREXPORT void netResetToHome(ArServerClient *client, ArNetPacket *packet);
  /// Networking request callback that localizes to a point given
  AREXPORT void netLocalizeToPose(ArServerClient *client, ArNetPacket *packet);
  /// Adds the simple relative localize commands
  AREXPORT void addSimpleRelativeCommands(ArServerHandlerCommands *commands);
  /// Adds a callback for when something localizes the robot
  AREXPORT void addLocalizeCallback(
	  ArFunctor3<ArPose, ArPose, int> *functor, int position = 50);
  /// Removes a callback for when something localizes the robot
  AREXPORT void remLocalizeCallback(
	  ArFunctor3<ArPose, ArPose, int> *functor);
  /// Calls the callbacks for when something localizes the robot
  AREXPORT void callLocalizeCallbacks(ArPose pose, ArPose spread, int nsam);

  /// Adds a callback for before something localizes the robot robot
  AREXPORT void addBeforeLocalizeCallback(
	  ArFunctor3<ArPose, ArPose, int> *functor, int position = 50);
  /// Removes a callback for when something localizes the robot
  AREXPORT void remBeforeLocalizeCallback(
	  ArFunctor3<ArPose, ArPose, int> *functor);
  /// Calls the callbacks for when something localizes the robot
  AREXPORT void callBeforeLocalizeCallbacks(ArPose pose, ArPose spread, int nsam);

  /// Gets the last pose, spread and num samples localized to IF the
  /// robot hasn't moved since the last localize
  AREXPORT bool getLastLocalizationIfNotMovedSince(ArPose *pose, ArPose *spread, int *nsam);
protected:
  AREXPORT void relativeLocalizeToPose(ArArgumentBuilder *arg);
  AREXPORT void relativeLocalizeHeading(ArArgumentBuilder *arg);
  /// Function that'll reinforce the set pose
  void userTask(void);

  ArServerBase *myServer;
  ArRobot *myRobot;
  bool mySetSimPoseOnLocalize;
  ArBaseLocalizationTask *myLocTask;
  // last localized pose
  ArPose myPose;
  // spread parameter
  ArPose mySpread;
  // last localized counter
  unsigned int myLastLocalizeSimCounter;

  // the encoder pose we were at last localization
  ArPose myLastLocalizeEncoderPose;
  // if we've moved since the last localization
  bool myLastLocalizeMovedSince;

  std::multimap<int, ArFunctor3<ArPose, ArPose, 
				int> *> myLocalizeCallbacks;
  std::multimap<int, ArFunctor3<ArPose, ArPose, 
				int> *> myBeforeLocalizeCallbacks;
  ArMutex myLocalizeCallbackMutex;

  ArFunctor2C<ArServerHandlerLocalization, ArServerClient *, ArNetPacket *> myNetResetToHomeCB;
  ArFunctor2C<ArServerHandlerLocalization, ArServerClient *, ArNetPacket *> myNetLocalizeToPoseCB;
  ArFunctorC<ArServerHandlerLocalization> myUserTaskCB;
  ArFunctor1C<ArServerHandlerLocalization, ArArgumentBuilder *> myRelativeLocalizeToPoseCB;
  ArFunctor1C<ArServerHandlerLocalization, ArArgumentBuilder *> myRelativeLocalizeHeadingCB;
};

/** For backwards compatability.
 *  @deprecated This name is deprecated, use ArServerHandlerLocalization
 *  instead.
 */
typedef ArServerHandlerLocalization ArServerHandleLocalization;

class ArServerModeGotoLLA : public ArServerMode
{
public:
  AREXPORT ArServerModeGotoLLA(ArServerBase *server, ArRobot *robot, 
			       ArBaseLocalizationTask* locTask,
			       ArPathPlanningTask *pathTask, 
			       ArMapInterface *arMap, 
			       ArPose home = ArPose(0, 0, 0), 
			       ArRetFunctor<ArPose> *getHomePoseCB = NULL);
  AREXPORT virtual ~ArServerModeGotoLLA();
  AREXPORT virtual void activate(void);
  AREXPORT virtual void deactivate(void);
  AREXPORT void gotoPose(ArPose pose, ArPose llaPose, bool useHeading);
  AREXPORT void tourGoalsInList(std::list<ArPose> goalList,
				std::list<ArPose> llaGoalList);

  AREXPORT void addGotoLLACommand(ArServerHandlerCommands *commandsServer);
  AREXPORT void gotoLLA(ArArgumentBuilder *args); 

  AREXPORT void addTourGoalsInListCommand(
	  ArServerHandlerCommands *commandsServer);
  AREXPORT void tourGoalsInListCommand(ArArgumentBuilder *arg);

protected:
  AREXPORT virtual void userTask(void);

  /** Reset state */
  void reset(void);

  /** Callback from ARNL when a goal is reached. Set myStatus and other state.
      If touring, plan the next goal in the tour.
  */
  void goalDone(ArPose pose);

  /** Callback from ARNL when a goal fails. Set myStatus and other state.
      If touring, plan the next goal in the tour.
  */
  void goalFailed(ArPose pose);

  /// Set myGoalName to the name of the next goal in the tour
  void findNextTourGoal(void);

  /** @return number of goals in current tour, or 0 if none */
  size_t numGoalsTouring();

  /// Keep trying to plan paths to goals in a tour.
  void planToNextTourGoal();


  ArPose myGoalPose;
  ArPose myLLAGoalPose;
  bool myDone;
  bool myUseHeading;
  ArPose myGoal;
  ArPose myLLAGoal;
  bool myGoingHome;
  ArMapInterface *myMap;
  ArPose myHome;
  ArRetFunctor<ArPose> *myGetHomePoseCB;
  bool myTouringGoals;
  std::list<ArPose> myTouringGoalsList; 
  std::list<ArPose> myTouringLLAGoalsList; 
  bool myAmTouringGoalsInList;
  ArBaseLocalizationTask *myLocTask;
  ArPathPlanningTask *myPathTask;
  ArFunctor1C<ArServerModeGotoLLA, ArPose> myGoalDoneCB;
  ArFunctor1C<ArServerModeGotoLLA, ArPose> myGoalFailedCB;
  ArFunctor1C<ArServerModeGotoLLA, ArArgumentBuilder*> myGotoLLACB;
  ArFunctor1C<ArServerModeGotoLLA, ArArgumentBuilder*> myTourGoalsInListLLACB;
};

/** Handles requests to go to a goal or point, to go to the home point,
 *  or to tour all goals. It sets the server Status and Mode strings
 *  when active.
 *
 *  This class adds the following data requests to the server:
 *  <dl>
 *   <dt>gotoGoal</dt>  <dd>Sends the robot to the goal with the given name.</dd>
 *   <dt>gotoPose</dt>  <dd>Sends the robot to the given pose. Parameters are
 *      2 4-byte integers for X and Y, and an optional 4-byte integer for
 *      heading.</dd>
 *    <dt>home</dt>     <dd>Sends the robot to a home point from the map, or
 *      0,0,0 if none.</dd>
 *    <dt>goalName</dt> <dd>Requests a reply packet containing the name of the
 *      current goal</dd>
 *    <dt>tourGoals</dt> <dd>Sends the robot to each goal in the map in
 *      turn.</dd>
 *    <dt>getGoals</dt> <dd>Requests a reply packet containing the names of all
 *      goals in the map (separated by null byte)</dd>
 *  </dl>
 */
/// Handles going to a goal or point
class ArServerModeGoto : public ArServerMode
{
public:
  AREXPORT ArServerModeGoto(ArServerBase *server, ArRobot *robot, 
			    ArPathPlanningTask *pathTask, ArMapInterface *arMap, 
			    ArPose home = ArPose(0, 0, 0), 
			    ArRetFunctor<ArPose> *getHomePoseCB = NULL);
  AREXPORT virtual ~ArServerModeGoto();
  AREXPORT virtual void activate(void);
  AREXPORT virtual void deactivate(void);
  AREXPORT void home(void);
  AREXPORT void gotoGoal(const char *goal);
  AREXPORT void gotoPose(ArPose pose, bool useHeading);

  /** Enter a "tour goals" mode, in which the robot is sent to each goal in the
   *  map in turn. This mode can be entered using the tourGoals networking
   *  request.
   */
  AREXPORT void tourGoals(void);

  /** Enter a "tour goals" mode, in which the robot is sent to each goal in the
   *  given list in turn.  
   *
   *  @todo Use an ArArgumentBuilder instead of a deque?
   */
  AREXPORT void tourGoalsInList(std::deque<std::string> goalList);

  /** Add a "tour" command to the given "simple commands" object. This
   *  simple (custom) command accepts a comma-separated list of goals,
   *  builds a list of goals, expanding items ending in a wildcard (*)
   *  to maching goals, and omitting invalid goals,
   *  and then calls tourGoalsInList().
   */
  AREXPORT void addTourGoalsInListSimpleCommand(ArServerHandlerCommands *commandsServer);
  //AREXPORT void addGotoLLACommand(ArServerHandlerCommands *commandsServer);

  AREXPORT virtual bool isAutoResumeAfterInterrupt(void);

protected:
  AREXPORT void serverGetGoals(ArServerClient *client,
			       ArNetPacket *packet);
  AREXPORT void serverGotoPose(ArServerClient *client,
			       ArNetPacket *packet);
  AREXPORT void serverGotoGoal(ArServerClient *client,
			       ArNetPacket *packet);
  AREXPORT void serverHome(ArServerClient *client,
			   ArNetPacket *packet);
  AREXPORT void serverTourGoals(ArServerClient *client,
				ArNetPacket *packet);
  AREXPORT virtual void userTask(void);

  /** Reset state */
  void reset(void);

  /** Callback from ARNL when a goal is reached. Set myStatus and other state.
      If touring, plan the next goal in the tour.
  */
  void goalDone(ArPose pose);

  /** Callback from ARNL when a goal fails. Set myStatus and other state.
      If touring, plan the next goal in the tour.
  */
  void goalFailed(ArPose pose);

  /// Set myGoalName to the name of the next goal in the tour
  void findNextTourGoal(void);

  /** @return number of goals in current tour, or 0 if none */
  size_t numGoalsTouring();


  /// Keep trying to plan paths to goals in a tour, until either a plan succeeds or all the goals fail.
  void planToNextTourGoal();

  ArPose myGoalPose;
  bool myDone;
  bool myUseHeading;
  std::string myGoalName;
  bool myGoingHome;
  ArMapInterface *myMap;
  ArPose myHome;
  ArRetFunctor<ArPose> *myGetHomePoseCB;
  bool myTouringGoals;
  ArPathPlanningTask *myPathTask;
  ArFunctor1C<ArServerModeGoto, ArPose> myGoalDoneCB;
  ArFunctor1C<ArServerModeGoto, ArPose> myGoalFailedCB;
  ArFunctor2C<ArServerModeGoto, ArServerClient *, ArNetPacket *> myServerGetGoalsCB;
  ArFunctor2C<ArServerModeGoto, ArServerClient *, ArNetPacket *> myServerGotoGoalCB;
  ArFunctor2C<ArServerModeGoto, ArServerClient *, ArNetPacket *> myServerGotoPoseCB;
  ArFunctor2C<ArServerModeGoto, ArServerClient *, ArNetPacket *> myServerHomeCB;
  ArFunctor2C<ArServerModeGoto, ArServerClient *, ArNetPacket *> myServerTourGoalsCB;
  ArFunctor2C<ArServerModeGoto, ArServerClient *, ArNetPacket *> myServerGoalNameCB;

  void serverGoalName(ArServerClient* client, ArNetPacket* pkt);
  void pathPlannerStateChanged();
  std::deque<std::string> myTouringGoalsList; ///< @todo use an ArArgumentBuilder instead of a deque?
  bool myAmTouringGoalsInList;
  ArFunctor1C<ArServerModeGoto, ArArgumentBuilder*> myTourGoalsInListSimpleCommandCB;
  AREXPORT void tourGoalsInListCommand(ArArgumentBuilder *args); ///< Used as callback from ArServerHandlerCommands (simple/custom commands)
};


/// This class is for drawing destinations on the screen
class ArServerDrawingDestination
{
public:
  /// Constructor
  AREXPORT ArServerDrawingDestination
                  (ArServerInfoDrawings *infoDrawings,
				           ArPathPlanningTask *pathTask,
				           const char *name = "destination");
  /// Destructor
  AREXPORT virtual ~ArServerDrawingDestination();

  /// Sets the parameters for the destination to flash (default is just on)
  AREXPORT void setFlashingParameters(int onMSec = 1, int offMSec = 0);

  /// Sets the drawing data
  AREXPORT void setDrawingData(ArDrawingData *drawingData, 
			                         bool ownDrawingData);
  /// the callback to get the data to draw
  AREXPORT void drawDestination(ArServerClient * client, ArNetPacket *packet);
  /// A call that'll add all the drawing data to the config
  AREXPORT void addToConfig(ArConfig *config);
protected:
  AREXPORT bool processFile(void);
  ArServerInfoDrawings *myInfoDrawings;
  ArPathPlanningTask *myPathTask;
  std::string myName;
  ArDrawingData *myDrawingData;
  bool myOwnDrawingData;
  int myOnMSec;
  int myOffMSec;
  bool myWasInitialized;
  bool myOn;
  ArPose myLastGoal;
  ArTime myLastSwitch;

  char myConfigDrawMode[512];
  int myConfigFlashMSec;
  char myConfigShape[512];
  int myConfigColorRGB;
  int myConfigSecondaryColorRGB;
  int myConfigSize;
  int myConfigLayer;
  int myConfigRefreshTime;

  ArFunctor2C<ArServerDrawingDestination, 
      ArServerClient *, ArNetPacket *> myDrawDestinationCB;
  ArRetFunctorC<bool, ArServerDrawingDestination> myProcessFileCB;
};

/// Action for stopping the robot if it gets lost or localization isn't active
class ArActionLost : public ArAction
{
public:
  AREXPORT ArActionLost(ArBaseLocalizationTask *locTask, 
			ArPathPlanningTask *pathTask, 
			ArServerMode *serverMode = NULL,
			const char *name = "lost stopper");
  /// Destructor
  AREXPORT virtual ~ArActionLost();
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  AREXPORT virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  AREXPORT virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
  /// Lets this action work (defaults to enabled) 
  AREXPORT void enable(void);
  /// Prevents this action from doing anything until enabled
  AREXPORT void disable(void); 
  /// Sees if this action can do anything or not
  AREXPORT bool isEnabled(void) const { return myIsEnabled;; }
  /// Gets a callback to enable the action
  AREXPORT ArFunctor *getEnableCB(void) { return &myEnableCB; } 
  /// Gets a callback to disable the action
  AREXPORT ArFunctor *getDisableCB(void) { return &myDisableCB; } 
protected:
  ArActionDesired myDesired;
  ArBaseLocalizationTask *myLocTask;
  ArPathPlanningTask *myPathTask;
  ArServerMode *myServerMode;
  ArMutex myDataMutex;
  bool myIsEnabled;
  ArFunctorC<ArActionLost> myEnableCB;
  ArFunctorC<ArActionLost> myDisableCB;
};

//
// This is the action which will regulate robot speed, accels and decels 
// according to the localization score. The robot is slowed down as
// its localization score drops from 1.0.
//
class ArActionSlowDownWhenNotCertain : public ArAction
{
public:
  /// Constructor, sets the maximums and the parent pointer.
  AREXPORT  ArActionSlowDownWhenNotCertain(ArBaseLocalizationTask* lt);
  /// Destructor, no new objects, we don't need to do anything
  AREXPORT virtual ~ArActionSlowDownWhenNotCertain(void);
  /// Fire, this is what the resolver calls to find out what this action wants
  AREXPORT virtual ArActionDesired* fire(ArActionDesired currentDesired);
  /// Sets the robot pointer, also gets the sonar device
  AREXPORT virtual void setRobot(ArRobot *robot);
  /// Overrides base deactivate to uninitialize ArPathPlanningTask
  AREXPORT virtual void deactivate(void);
  /// Sets the power to which the loca score is raised to reduce speed.
  AREXPORT void setPower(double p) { myPower = p; }
  /// Gets the power to which the loca score is raised to reduce speed.
  AREXPORT double getPower(double /*p*/) { return myPower; }
protected:
  /// Function to call when arnl.p is processed.
  bool     reconfigureAction(void);
public:
  AREXPORT virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  AREXPORT virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
protected:
  /// Parent task whose score will control this action.
  ArBaseLocalizationTask*    myLocalizationTask;
  /// Robot
  ArRobot*                    myRobot;
  /// What the action wants to do
  ArActionDesired myDesired;
  /// Call back this when arnl.p is changed.
  ArRetFunctorC<bool, ArActionSlowDownWhenNotCertain>* myProcessFileCB;
  /// Bounds on the linear and rotational velocities.
  double myMaxVel;
  double myMinVel;
  double myMaxRotVel;
  double myMinRotVel;
  double myMaxLatVel;
  double myMinLatVel;
  /// Exponent to raise the localization score which will be the slowdown
  /// factor.
  double myPower;
  double myMaxThreshold;
  double myMinThreshold;
  bool   myEnableFlag;
}; 

class ArSimMapSwitcher
{
public:
  AREXPORT ArSimMapSwitcher(ArRobot *robot, ArMapInterface *arMap);
  AREXPORT virtual ~ArSimMapSwitcher();
  AREXPORT void mapChanged(void);
protected:
  ArRobot *myRobot;
  ArMapInterface *myMap;
  ArFunctorC<ArSimMapSwitcher> myMapChangedCB;
  
};


/// Class to save and restore pose from a file
/**
  After an attempt is made to restore a pose (successful or not) this
  class will start to write the robot's pose into the file that is
  attempted to restore from.  Unless told otherwise in the constructor
  it'll also save to the file when Aria exits.
*/
class ArPoseStorage
{
public:
  /// Constructor
  AREXPORT ArPoseStorage(ArRobot *robot, const char *baseDirectory = "",
			 int poseWriteIntervalInMSecs = 1000,
			 bool addAriaExitCB = true);
  /// Destructor
  AREXPORT ~ArPoseStorage();
  /// Saves the pose of a robot to a file for restoring later
  AREXPORT bool savePose(void);
  /// Restores the pose of a robot from a file saved earlier
  AREXPORT bool restorePose(const char *fileName);
protected:
  /// Internal user task call that will occasionally write the pose to disk
  AREXPORT void userTask(void);

  ArRobot *myRobot;
  std::string myBaseDir;
  std::string myFileName;
  std::string myRealFileName;
  std::string myTmpFileName;
  bool mySaving;
  int myWriteInterval;
  ArTime myLastWrite;
  ArRetFunctorC<bool, ArPoseStorage> myAriaExitCB;
  ArFunctorC<ArPoseStorage> myUserTaskCB;

};

#endif


