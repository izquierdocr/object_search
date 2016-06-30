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
#ifndef ARPATHPLANNINGTASK_H
#define ARPATHPLANNINGTASK_H

#include "Aria.h"
#include "ArPathPlan.h"
#include "ArNetworking.h"
#include "ArPathPlanningInterface.h"

#include <set>
#include <vector>
#include <algorithm>

//
// Advance declarations.
//
class ArPathPlan;
class ArPlanParams;
class ArPathPlanningTask;
class ArActionPlanAndMoveToGoal;

/**
   @class ArPathPlanningTask
   @brief Task that performs path planning and path following in a seperate asynchronous thread.

   The path planning task uses a grid based search to compute the shortest and safe path from the present robot pose to any reachable point in the given robot enviroment map. It then enables an action that follows the planned path, all the while using the dynamic window method to avoid unmapped obstacles in its path. The path planning thread requires fairly accurate robot location. Hence a localization task (ArLocalizationTask or ArSonarLocalizationTask) must also be run concurrently with the path planning task.

   See @ref PathPlanning for an overview of path planning and following in ARNL.

   @note ArPathPlanningTask::ArPathPlanningTask will start the new thread automatically when an object is created, so you must not call ArASyncTask::runAsync() on an ArPathPlanningTask object.

*/

class ArPathPlanningTask: public ArASyncTask, public ArPathPlanningInterface
{
  public:
  /*
  /// State of the path plan (accessible using getState()).
  enum PathPlanningState
  {
    NOT_INITIALIZED,    ///< Task not initialized 
    PLANNING_PATH,      ///< Planning the inital path
    MOVING_TO_GOAL,     ///< Moving to the goal.
    REACHED_GOAL,       ///< Reached the goal.
    FAILED_PLAN,        ///< Failed to plan a path to goal.
    FAILED_MOVE,        ///< Failed to reach goal after plan obtained.
    ABORTED_PATHPLAN,   ///< Aborted plan before done.
    INVALID             ///< Invalid state.
  };
  */
  /// State of the local motion plan
  enum LocalPlanningState
  {
    NO_LOCAL_PLAN_GFD,  ///< Failed to plan a path to goal + Goal Fail Dist.
    NO_LOCAL_PLAN_LFD,  ///< Failed to plan a path to goal + Local Fail Dist.
    NO_LOCAL_PLAN,      ///< Failed to plan a path to goal.
    GOAL_OCCUPIED,      ///< Goal is occupied.
    NO_MOVE,            ///< Failed to reach goal after plan obtained.
    OBSTACLE_TOO_CLOSE, ///< Obstacle too close for move.
    COMPLETELY_BLOCKED, ///< All directions blocked.
    GOT_PLAN_AND_VEL_HEADING,  ///< Plan obtained with heading and vel.
    GOT_PLAN_AND_VELS,  ///< Plan obtained with lin and ang velocities.
    GOT_BLOCKED_PLAN_AND_VEL_HEADING,  ///< Plan obtained with heading and vel.
    GOT_BLOCKED_PLAN_AND_VELS,  ///< Plan obtained with lin and ang velocities.
  };

  /// How to incorporate the range device data.
  enum RangeType
  {
    NEITHER,   ///< No current and no cumulative
    CURRENT,   ///< Current buffer only.
    CUMULATIVE,///< Cumulative buffer only.
    BOTH       ///< Both buffers.
  };

  friend class ArActionPlanAndMoveToGoal;
  friend class Localize;
  friend class ArGlobalReplanningRangeDevice;

  public:
  /** Constructor for a robot with two typical range devices 
   * (e.g. SICK LRF and Sonar) and the map. 
   * Use addRangeDevice() to add additional range devices.
   * Parameters are set to default values and state is initialized, then
   * the path planning task thread is automatically started.
   */
  AREXPORT ArPathPlanningTask(ArRobot* robot, 
			      ArRangeDevice* laser, 
			      ArRangeDevice* sonar, 
			      ArMapInterface* m,
			      bool legacyFreeSpaceCosting=false);
  /** Constructor for a robot with just sonar (or other comparable 
   *  range device) and map. Use addRangeDevice() to add additional 
   *  range devices. 
   * Parameters are set to default values and state is initialized, then
   * the path planning task thread is automatically started.
   */
  AREXPORT ArPathPlanningTask(ArRobot* robot, 
			      ArRangeDevice* sonar, 
			      ArMapInterface* map,
			      bool legacyFreeSpaceCosting=false);
protected:
  /** This contains code common to both constructors above. If mySonar or 
   * myLaser have been set non-null, then it adds them to the path planning. */
  void ArPathPlanningTask_CCommon(ArRobot* r, ArMapInterface* m);
public:
  /// Base destructor.
  AREXPORT virtual ~ArPathPlanningTask(void);
  /// Set a new destination pose for the path planning task to plan to. 
  AREXPORT virtual bool   pathPlanToPose(ArPose goal, bool headingFlag, 
				 bool printFlag=true);
  /// Set a new goal (from the map) for the path planning task  to plan to.
  AREXPORT virtual bool   pathPlanToGoal(const char* goalname, 
				 bool strictGoalTypeMatching = true,
				 const char *goalType = "Goal");

  /// Function to local path planning
  /// If the localPlanMap is true, the path planning will take use its
  /// local map to plan for local paths instead of bound by the starting map.
  /// @internal
  AREXPORT bool   startPathPlanToLocalPose(bool printFlag = true);
  /// Set a local pose for the path planning while local path planning
  /// @internal
  AREXPORT bool   continuePathPlanToLocalPose(ArPose goal, bool headingFlag, 
					      bool printFlag = true);
  /// Function to end local path planning
  /// @internal
  AREXPORT bool   endPathPlanToLocalPose(bool printFlag);
  /// Gets just the path planning/following action object.
  ArActionPlanAndMoveToGoal*  getPathPlanAction(void) 
  { return myPathPlanAction; }
  /// Gets the path planning action/following group (move, stall recovery etc)
  ArActionGroup* getPathPlanActionGroup(void) 
  { return myPlanAndMoveGroup; }
  /// Sets the path planning action/following group
  AREXPORT void   setPathPlanActionGroup(ArActionGroup *group, 
					 bool takeOwnershipOfGroup = false);
  /// Load the path planning params from config file.
  AREXPORT bool   loadParamFile(const char *file);
  /// Gets the minimum distance it must look for the speed at which it is 
  /// going (mostly internal use).
  AREXPORT double getSafeCollisionRange(void);
  /// Set the path planning and following parameters for customized sectors.
  AREXPORT void setTemporaryDrivingParams(double freeSpace=-1.0,
					  double obsThreshold=-1.0,
					  double frontClearance=-1.0,
					  double slowSpeed=-1.0,
					  double sideClearanceAtSlowSpeed=-1.0,
					  double frontPaddingAtSlowSpeed=-1.0,
					  double fastSpeed=-1.0,
					  double sideClearanceAtFastSpeed=-1.0,
					  double frontPaddingAtFastSpeed=-1.0,
					  double goalOccupiedFailDistance=-1.0,
					  double localPathFailDistance=-1.0)
  {
    myMutex.lock();
    myUsingPlanFreeSpace = freeSpace;
    myUsingObsThreshold = obsThreshold;
    myUsingFrontClearance = frontClearance;
    myUsingSlowSpeed = slowSpeed;
    myUsingSideClearanceAtSlowSpeed = sideClearanceAtSlowSpeed;
    myUsingFrontPaddingAtSlowSpeed = frontPaddingAtSlowSpeed;
    myUsingFastSpeed = fastSpeed;
    myUsingSideClearanceAtFastSpeed = sideClearanceAtFastSpeed;
    myUsingFrontPaddingAtFastSpeed = frontPaddingAtFastSpeed;
    myUsingGoalOccupiedFailDistance = goalOccupiedFailDistance;
    myUsingLocalPathFailDistance = localPathFailDistance;
    myMutex.unlock();
  }
  /// @internal
  AREXPORT void setCustomParamsTest(bool flag)
  {
    if(flag)
    {
      ArLog::log(ArLog::Normal, "Setting custom params");
      myMutex.lock();
      myUsingPlanFreeSpace = 100;
      myUsingObsThreshold = 1.0;
      myUsingFrontClearance = 10.0;
      myUsingSlowSpeed = 10.0;
      myUsingSideClearanceAtSlowSpeed = 10.0;
      myUsingFrontPaddingAtSlowSpeed = 10.0;
      myUsingFastSpeed = 2000.0;
      myUsingSideClearanceAtFastSpeed = 10.0;
      myUsingFrontPaddingAtFastSpeed = 10.0;
      myUsingGoalOccupiedFailDistance = 1000.0;
      myUsingLocalPathFailDistance = 1000.0;
      myMutex.unlock();
    }
    else
    {
      ArLog::log(ArLog::Normal, "Unsetting custom params");
      myMutex.lock();
      myUsingPlanFreeSpace = -1.0;
      myUsingObsThreshold = -1.0;
      myUsingFrontClearance = -1.0;
      myUsingSlowSpeed = -1.0;
      myUsingSideClearanceAtSlowSpeed = -1.0;
      myUsingFrontPaddingAtSlowSpeed = -1.0;
      myUsingFastSpeed = -1.0;
      myUsingSideClearanceAtFastSpeed = -1.0;
      myUsingFrontPaddingAtFastSpeed = -1.0;
      myUsingGoalOccupiedFailDistance = -1.0;
      myUsingLocalPathFailDistance = -1.0;
      myMutex.unlock();
    }
  }
  /// Saves the default params to the filename
  bool   saveParams(char* filename)
  {
    if(myParams == NULL)
      return false;
    return myParams->writeFile(filename);
  }
  /// If localization fails, then call this to alert path planning.
  /// @internal
  AREXPORT void   trackingFailed(int failedTimes);
  /// Cancel any current path following
  AREXPORT virtual void   cancelPathPlan(void);
  /// Set the verbose flag.
  void   setVerboseFlag(bool f)
  {
    myMutex.lock();
    myVerboseFlag = f;
    myMutex.unlock();
  }
  /// Get the current goal of the path plan.
  ArPose getCurrentGoal()
  {
    myMutex.lock();
    ArPose p = myCurrentGoal;
    myMutex.unlock();
    return p;
  }
  /// Get the value of path plan initialized flag.
  bool   getInitializedFlag(void)
  {
    bool p;
    myMutex.lock();
    p = myInitializedFlag;
    myMutex.unlock();
    return p;
  }
  /// Get the verbose flag. (debugging only)
  bool   getVerboseFlag(void)
  {
    bool p;
    myMutex.lock();
    p = myVerboseFlag;
    myMutex.unlock();
    return p;
  }
  /// Gets the state of path planning.
  virtual PathPlanningState getState(void)
  {
    ArPathPlanningTask::PathPlanningState p;
    myMutex.lock();
    p = myState;
    myMutex.unlock();
    return p;
  }
  /// Gets a textual description of failure or current planning status.
  virtual void   getFailureString(char *str, size_t len)
  {
    myMutex.lock();
    strncpy(str, myFailureString.c_str(), len);
    myMutex.unlock();
  }
  /// Gets a textual description of failure or current planning status.
  virtual void   getStatusString(char *str, size_t len)
  {
    getFailureString(str, len);
  }
  /// Gets the list of path points as poses.
  AREXPORT std::list<ArPose> getCurrentPath(ArPose from, bool local=false);
  /// Gets the Aria map used in the path planning.
  virtual ArMapInterface* getAriaMap(void) {return myPathPlan->getAriaMap();}

  /** @name Accessors for current configuration values 
   *
   *  These values are normally set via ArConfig (the "Path Planning" section)
   *  by loading a file or other means, or by calling the modifier functions
   *  below.
   */
  //@{
  
  /// Get the use laser flag.
  bool   getUseLaserFlag(void)
  {
    bool p;
    myMutex.lock();
    p = myUseLaserFlag;
    myMutex.unlock();
    return p;
  }
  /// Get the use sonar flag.
  bool   getUseSonarFlag(void)
  {
    bool p;
    myMutex.lock();
    p = myUseSonarFlag;
    myMutex.unlock();
    return p;
  }
  /// Gets the map changed flag to trigger reloading of map for path planning.
  bool   getMapChangedFlag(void)
  {
    bool p;
    myMutex.lock();
    p = myMapChangedFlag;
    myMutex.unlock();
    return p;
  }
  /// Gets the use one way flag.
  bool   getUseOneWaysFlag(void)
  {
    bool p;
    myMutex.lock();
    p = myUseOneWaysFlag;
    myMutex.unlock();
    return p;
  }
  /// Gets the use resistance flag.
  bool   getUseResistanceFlag(void)
  {
    bool p;
    myMutex.lock();
    p = myUseResistanceFlag;
    myMutex.unlock();
    return p;
  }
  /// Gets the mark old path factor (mostly internal use).
  double getMarkOldPathFactor(void)
  {
    double p;
    myMutex.lock();
    p = myMarkOldPathFactor;
    myMutex.unlock();
    return p;
  }
  /// Gets the starting fraction of the old path marking.
  double getStartMarkPoint(void)
  {
    double p;
    myMutex.lock();
    p = myStartMarkPoint;
    myMutex.unlock();
    return p;
  }
  /// Gets the preferred line width.
  double getPreferredLineWidth(void)
  {
    double p;
    myMutex.lock();
    p = myPreferredLineWidth;
    myMutex.unlock();
    return p;
  }
  /// Gets the resistance for restrictive sectors (mostly internal use).
  short  getResistance(void)
  {
    short p;
    myMutex.lock();
    p = myResistance;
    myMutex.unlock();
    return p;
  }
  /// Gets the resistance for preferred lines (mostly internal use).
  short  getPreference(void)
  {
    short p;
    myMutex.lock();
    p = myPreference;
    myMutex.unlock();
    return p;
  }
  /// Gets the Offset from the center for the sided sectors 
  /// (mostly internal use).
  double  getPreferredDirectionSideOffset(void)
  {
    double p;
    myMutex.lock();
    p = myPreferredDirectionSideOffset;
    myMutex.unlock();
    return p;
  }
  /// Gets the maximum speed for the robot.
  double getMaxSpeed(void) const { return myMaxSpeed; }
  /// Gets the maximum rotational speed for the robot.
  double getMaxRotSpeed(void) const { return myMaxRotSpeed; }
  /// Gets the maximum obstacle range for collision avoidance.
  double getCollisionRange(void) const { return myCollisionRange; }
  /// Gets the slow speed.
  double getSlowSpeed(void) const { return mySlowSpeed; }
  /// Gets the fast speed.
  double getFastSpeed(void) const { return myFastSpeed; }
  /// Gets the side clearance at slow speed.
  double getSideClearanceAtSlowSpeed(void) const 
  { return mySideClearanceAtSlowSpeed; }
  /// Gets the side clearance at fast speed.
  double getSideClearanceAtFastSpeed(void) const 
  { return mySideClearanceAtFastSpeed; }
  /// Gets the front clearance.
  double getFrontClearance(void) const { return myFrontClearance; }
  /// Gets the front padding at slow speed.
  double getFrontPaddingAtSlowSpeed(void) const 
  { return myFrontPaddingAtSlowSpeed; }
  /// Gets the front padding at high speed.
  double getFrontPaddingAtFastSpeed(void) const 
  { return myFrontPaddingAtFastSpeed; }
  /// Gets the decel allowed when there is still +ve clearance + padding.
  double getSuperMaxTransDecel(void) const 
  { return mySuperMaxTransDecel; }
  /// Gets the emergency max decel allowed for collision avoidance.
  double getEmergencyMaxTransDecel(void) const 
  { return myEmergencyMaxTransDecel; }
  /// Gets the robot width.
  double getWidth(void) const { return myWidth; }
  /// Gets the robot front length.
  double getLengthFront(void) const { return myLengthFront; }
  /// Gets the robot rear length.
  double getLengthRear(void) const { return myLengthRear; }
  /// Gets the path planning map resolution.
  double getPlanMapResolution(void) const { return myPlanRes; }
  /// Gets the distance from the side of the robot to obstacles to plan with.
  double getPlanFreeSpace(void) 
  const { return myPlanFreeSpace; }
  /// Gets the heading weight.
  double getHeadingWt(void) const { return myHeadingWt; }
  /// Gets the distance weight.
  double getDistanceWt(void) const { return myDistanceWt; }
  /// Gets the velocity weight.
  double getVelocityWt(void) const { return myVelocityWt; }
  /// Gets the smoothing weight.
  double getSmoothingWt(void) const { return mySmoothingWt; }
  /// Gets the linear velocity increments
  int    getLinVelIncrements(void) const { return myLinVelIncrements;}
  /// Gets the rotational velocity increments
  int    getRotVelIncrements(void) const { return myRotVelIncrements;}
  /// Gets the smoothing window size.
  int    getSmoothSize(void) const { return mySmoothSize; }
  /// Gets the obstacle threshold
  double getObsThreshold(void) const { return myObsThreshold; }
  /// Gets the maximum expansion factor when local planning fails.
  double getMaxExpansionFactor(void) const 
  { return myMaxExpansionFactor; }
  /// Gets the flag which decides if collision range is used for planning.
  bool   getUseCollisionRangeForPlanningFlag(void) const 
  { return myUseCollisionRangeForPlanningFlag; }
  /// Gets the distance tolerance to goal in mm.
  double getGoalDistanceTolerance(void) 
  const { return myGoalDistanceTol; }
  /// Gets the angle tolerance to goal in mm.
  double getGoalAngleTolerance(void) const { return myGoalAngleTol; }
  /// Gets the speed for end move.
  double getGoalSpeed(void) const { return myGoalSpeed; }
  /// Gets the rotational speed for end move.
  double getGoalRotSpeed(void) const { return myGoalRotSpeed; }
  /// Gets the multiplier used to correct direction to goal.
  double getGoalRotVelKp(void) const { return myGoalRotVelKp; }
  /// Gets the accel for end move.
  double getGoalTransAccel(void) const { return myGoalTransAccel; }
  /// Gets the rotational accel for end move.
  double getGoalRotAccel(void) const { return myGoalRotAccel; }
  /// Gets the decel for end move.
  double getGoalTransDecel(void) const { return myGoalTransDecel; }
  /// Gets the rotational decel for end move.
  double getGoalRotDecel(void) const { return myGoalRotDecel; }
  /// Gets the time to allow for slowing down.
  double getGoalSwitchTime(void) const { return myGoalSwitchTime; }
  /// Gets the use encoder pose for end move.
  bool   getGoalUseEncoderFlag(void) const 
  { return myGoalUseEncoderFlag; }
  /// Gets the distance to which the robot will drive when goal is occupied.
  double  getGoalOccupiedFailDistance(void) const 
  { return myGoalOccupiedFailDistance; }
  /// Gets the distance to which the robot will drive when path is blocked.
  double  getLocalPathFailDistance(void) const 
  { return myLocalPathFailDistance; }
  /// Gets the rotational speed for end move.
  double getHeadingRotSpeed(void) const { return myHeadingRotSpeed; }
  /// Gets the rotational accel for end move.
  double getHeadingRotAccel(void) const { return myHeadingRotAccel; }
  /// Gets the rotational decel for end move.
  double getHeadingRotDecel(void) const { return myHeadingRotDecel; }
  /// Gets the secs for local path search to fail in.
  int    getSecsToFail(void) const { return mySecsToFail; }
  /// Gets the max speed at which to go to align mode.
  double getAlignSpeed(void) const { return myAlignSpeed; }
  /// Gets the min angle to local dest to go to align mode.
  double getAlignAngle(void) const { return myAlignAngle; }
  /// Gets the degree of the spline.
  int    getSplineDegree(void) const { return mySplineDegree; }
  /// Gets the increments in path to set control points.
  int    getNumSplinePoints(void) const  { return myNumSplinePoints; }
  /// Gets the flag for stopping local planning.
  bool   getNoLocalPlanFlag(void) const 
  { return myNoLocalPlanFlag; }
  /// Gets the look ahead when there is no local planning allowed.
  double getNoLocalPlanLookAhead(void) const 
  { return myNoLocalPlanLookAhead; }
  /// Gets the wrong side cost for going on sided sectors.
  double getPreferredDirectionWrongWayCost(void) const
  { return myPreferredDirectionWrongWayCost; }
  /// Gets the Sided to old cost factor for cells in a sided sector.
  double getPreferredDirectionToOldCostFactor(void) const
  { return myPreferredDirectionToOldCostFactor; }
  /// Gets the Sided to old cost factor for cells in a sided sector 
  /// for mainpath computation.
  double getPreferredDirectionToOldCostFactorMain(void) const
  { return myPreferredDirectionToOldCostFactorMain; }
  /// Gets the boolean indicating legacy one way costing.
  bool   getLegacyFreeSpaceCosting(void) const
  { return myLegacyFreeSpaceCosting; }
  /// Gets the side away cost for going on sided sectors.
  double getPreferredDirectionAwayCost(void) const
  { return myPreferredDirectionAwayCost; }
  /// Get the clear on fail flag.
  bool   getClearOnFailFlag(void) const
  { return myClearOnFailFlag; }
  /// Gets the number of times to replan 
  int getNumTimesToReplan(void) const
  { return myNumTimesToReplan; }
  /// Gets use emergency stop if collision imminent flag.
  bool   getUseEStopFlag(void) const 
  { return myUseEStopFlag; }
  /// Gets the curvature speed factor.
  double getCurvatureSpeedFactor(void) const
  { return myCurvatureSpeedFactor; }
  /// Gets the cost of going against the one way direction.
  double getOneWayCost(void) const
  { return myOneWayCost; }
  /// Gets the one way to old cost factor for cells in a sided sector.
  double getOneWayToOldCostFactor(void)
  { return myPreferredDirectionToOldCostFactor; }
  /// Gets the cost of going away from the central spine of the one way area.
  double getCenterAwayCost(void) const
  { return myCenterAwayCost; }
  /// Gets the factor by which an existing path point costs less.
  bool getLogFlag(void) const
  { return myLogFlag; }
  /** Gets the currently set logging level
    * @swig use getPathPlanningLogLevel() instead 
    */
  int getLogLevel(void) const
  { return myLogLevel; }
  /// Gets the factor to multiply the robot lengths to match path.
  double getMatchLengths(void) const
  { return myMatchLengths; }
  /// Get the local path planning flag.
  bool   getLocalPathPlanningFlag(void) const
  { return myLocalPathPlanningFlag; }
  /// Get the local path planning flag.
  bool   getCheckInsideRadiusFlag(void) const
  { return myCheckInsideRadiusFlag; }
  /// Gets the flag for checking if obstacles are inside bounds.
  bool   getCheckInsideBoundsFlag(void) const
  { return myCheckInsideBoundsFlag; }


  //@}

  /** @name Modifiers for configuration values.  
   *  Normally these values
   *  are automatically set via ArConfig (e.g. by loading them from a 
   *  file or other source) in the "Path Planning" section, and these 
   *  methods would only be
   *  used internally by ARNL to set stored values from ArConfig.
   */
  //@{
  
  /// Sets the use laser flag.
  void   setUseLaserFlag(bool f)
  {
    myMutex.lock();
    myUseLaserFlag = f;
    myMutex.unlock();
  }
  /// Sets the use sonar flag.
  void   setUseSonarFlag(bool f)
  {
    myMutex.lock();
    myUseSonarFlag = f;
    myMutex.unlock();
  }
  /// Sets the map changed flag to trigger reloading of map for path planning.
  void   setMapChangedFlag(bool f)
  {
    myMutex.lock();
    myMapChangedFlag = f;
    myMutex.unlock();
  }
  /// Sets the use one way flag.
  void   setUseOneWaysFlag(bool f)
  {
    myMutex.lock();
    myUseOneWaysFlag = f;
    myMutex.unlock();
  }
  /// Sets the reisistance.
  void   setResistance(short f)
  {
    myMutex.lock();
    myResistance = f;
    myMutex.unlock();
  }
  /// Sets the preference.
  void   setPreference(short f)
  {
    myMutex.lock();
    myPreference = f;
    myMutex.unlock();
  }
  /// Sets the side offset for sided sectors.
  void   setPreferredDirectionSideOffset(double f)
  {
    myMutex.lock();
    myPreferredDirectionSideOffset = f;
    myMutex.unlock();
  }
  /// Sets the callback which will be called after planning and before action.
  void   setPlanDoneCallBack(ArFunctor* prcb)
  {
    myMutex.lock();
    myPlanDoneCB = prcb;
    myMutex.unlock();
  }
  /// Gets the flag which decides if collision range is used for planning.
  void   setUseCollisionRangeForPlanningFlag(bool f)
  { 
    myMutex.lock();
    myUseCollisionRangeForPlanningFlag = f; 
    if(myPlanParams)
      myPlanParams->setUseCollisionRangeForPlanningFlag(f); 
    myMutex.unlock();
  }
  /// Sets the maximum speed during path following.
  void   setMaxSpeed(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myMaxSpeed = f;
    if(myPlanParams)
      myPlanParams->setMaxLinVel(f);
    myMutex.unlock();
  }
  /// Sets the maximum rotational speed during path following.
  void   setMaxRotSpeed(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myMaxRotSpeed = f;
    if(myPlanParams)
      myPlanParams->setMaxRotVel(f);
    myMutex.unlock();
  }
  /// Sets the maximum range for obstacles in sensor view.
  void   setCollisionRange(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myCollisionRange = f;
    if(myPlanParams)
      myPlanParams->setMaxObsDist(f);
    myMutex.unlock();
  }
  /// Sets the slow speed for dynamic clearance computation.
  void   setSlowSpeed(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    mySlowSpeed = f;
    myMutex.unlock();
  }
  /// Sets the fast speed for dynamic clearance computation.
  void   setFastSpeed(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myFastSpeed = f;
    myMutex.unlock();
  }
  /// Sets the side clearance at slow speed.
  void   setSideClearanceAtSlowSpeed(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    mySideClearanceAtSlowSpeed = f;
    myMutex.unlock();
  }
  /// Sets the side clearance at fast speed.
  void   setSideClearanceAtFastSpeed(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    mySideClearanceAtFastSpeed = f;
    myMutex.unlock();
  }
  /// Sets the front clearance.
  void   setFrontClearance(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myFrontClearance = f;
    myMutex.unlock();
  }
  /// Sets the front padding at slow speed.
  void   setFrontPaddingAtSlowSpeed(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myFrontPaddingAtSlowSpeed = f;
    myMutex.unlock();
  }
  /// Sets the front padding at fast speed.
  void   setFrontPaddingAtFastSpeed(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myFrontPaddingAtFastSpeed = f;
    myMutex.unlock();
  }
  /// Sets the deceleration to be used to avoid obstacles inside padding.
  void   setSuperMaxTransDecel(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    mySuperMaxTransDecel = f;
    myMutex.unlock();
  }
  /// Set the deceleration to be used when obstacles inside clearance.
  void   setEmergencyMaxTransDecel(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myEmergencyMaxTransDecel = f;
    myMutex.unlock();
  }
  /// Sets the robot width used for path planning.
  void   setWidth(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myWidth = f;
    if(myPlanParams)
      myPlanParams->setWidth(f);
    myMutex.unlock();
  }
  /// Sets the robot length used for path planning.
  void   setLengthFront(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myLengthFront = f;
    if(myPlanParams)
      myPlanParams->setLengthFront(f);
    myMutex.unlock();
  }
  /// Sets the robot rear length used for path planning.
  void   setLengthRear(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myLengthRear = f;
    if(myPlanParams)
      myPlanParams->setLengthRear(f);
    myMutex.unlock();
  }
  /// Sets the grid size for the path planning map.
  void   setPlanMapResolution(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myPlanRes = f;
    if(myPlanParams)
      myPlanParams->setPlanRes(f);
    myMutex.unlock();
  }
  /// Sets the distance from obstacles beyond which costs equal free space.
  void   setPlanFreeSpace(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myPlanFreeSpace = f;
    if(myPlanParams)
      myPlanParams->setPlanFreeSpace(f);
    myMutex.unlock();
  }
  /// Sets the heading weight for DWA.
  void   setHeadingWt(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myHeadingWt = f;
    if(myPlanParams)
      myPlanParams->setHeadingWt(f);
    myMutex.unlock();
  }
  /// Sets the distance weight for DWA.
  void   setDistanceWt(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myDistanceWt = f;
    if(myPlanParams)
      myPlanParams->setDistanceWt(f);
    myMutex.unlock();
  }
  /// Sets the velocity weight for DWA.
  void   setVelocityWt(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myVelocityWt = f;
    if(myPlanParams)
      myPlanParams->setVelocityWt(f);
    myMutex.unlock();
  }
  /// Sets the velocity weight for DWA.
  void   setSmoothingWt(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    mySmoothingWt = f;
    if(myPlanParams)
      myPlanParams->setSmoothingWt(f);
    myMutex.unlock();
  }
  /// Sets the linear velocity increments for the DWA table.
  void   setLinVelIncrements(int f)
  { 
    myMutex.lock();
    myLinVelIncrements = f;
    if(myPlanParams)
      myPlanParams->setLinVelIncrements(f);
    myMutex.unlock();
  }
  /// Sets the rotational velocity increments for the DWA table.
  void   setRotVelIncrements(int f)
  { 
    myMutex.lock();
    myRotVelIncrements = f;
    if(myPlanParams)
      myPlanParams->setRotVelIncrements(f);
    myMutex.unlock();
  }
  /// Sets the smooth windows size for the DWA.
  void   setSmoothWinSize(int f)
  { 
    myMutex.lock();
    mySmoothSize = f;
    if(myPlanParams)
      myPlanParams->setSmoothWinSize(f);
    myMutex.unlock();
  }
  /// Sets the threshold above which a cell is considered an obstacle.
  void   setObsThreshold(double f)
  { 
    myMutex.lock();
    myObsThreshold = f;
    if(myPlanParams)
      myPlanParams->setObsThreshold(f);
    myMutex.unlock();
  }
  /// Sets the factor to which the search will expand if it fails.
  void   setMaxExpansionFactor(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myMaxExpansionFactor = f;
    myMutex.unlock();
  }
  /// Sets the distance from goal within which end move will kick in.
  void   setGoalDistanceTolerance(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myGoalDistanceTol = f;
    if(myPlanParams)
      myPlanParams->setGoalDistanceTolerance(f);
    myMutex.unlock();
  }
  /// Sets the orientation from goal angle within which end move will kick in.
  void   setGoalAngleTolerance(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myGoalAngleTol = f;
    if(myPlanParams)
      myPlanParams->setGoalAngleTolerance(f);
    myMutex.unlock();
  }
  /// Sets the speed at which end move will be executed.
  void   setGoalSpeed(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myGoalSpeed = f;
    myMutex.unlock();
  }
  /// Sets the rotational speed at which end move will be executed.
  void   setGoalRotSpeed(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myGoalRotSpeed = f;
    myMutex.unlock();
  }
  /// Sets the multiplier for correcting the goal direction.
  void   setGoalRotVelKp(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myGoalRotVelKp = f;
    myMutex.unlock();
  }
  /// Sets the acceleration at which goal will be attempted.
  void   setGoalTransAccel(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myGoalTransAccel = f;
    myMutex.unlock();
  }
  ///Sets the rotational acceleration at which goal will be attempted.
  void   setGoalRotAccel(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myGoalRotAccel = f;
    myMutex.unlock();
  }
  /// Sets the deceleration at which goal will be attempted.
  void   setGoalTransDecel(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myGoalTransDecel = f;
    myMutex.unlock();
  }
  /// Sets the rotational deceleration at which goal will be attempted.
  void   setGoalRotDecel(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myGoalRotDecel = f;
    myMutex.unlock();
  }
  /// Sets the extra time beyond computed slow down time to do the end move.
  void   setGoalSwitchTime(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myGoalSwitchTime = f;
    myMutex.unlock();
  }
  /// Sets the flag to allow encoder pose for the end move.
  void   setGoalUseEncoderFlag(bool f)
  { 
    myMutex.lock();
    myGoalUseEncoderFlag = f;
    myMutex.unlock();
  }
  /// Sets the distance to which the robot will drive when goal is occupied.
  void   setGoalOccupiedFailDistance(double f)
  { 
    myMutex.lock();
    myGoalOccupiedFailDistance = f;
    if(myPlanParams)
      myPlanParams->setGoalOccupiedFailDistance(f);
    myMutex.unlock();
  }

#if 0
  /// Sets the distance to which the robot will drive path is blocked.
  AREXPORT void   setLocalPathFailDistance(double f);
#endif
  /// Sets the rotation speed for the Aria heading command.
  void   setHeadingRotSpeed(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myHeadingRotSpeed = f;
    myMutex.unlock();
  }
  /// Sets the rotation accel for the Aria heading command.
  void   setHeadingRotAccel(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myHeadingRotAccel = f;
    myMutex.unlock();
  }
  /// Sets the rotation decel for the Aria heading command.
  void   setHeadingRotDecel(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myHeadingRotDecel = f;
    myMutex.unlock();
  }
  /// Sets the no of seconds the local planning will keep retrying a fail.
  void   setSecsToFail(int f)
  { 
    f = abs(f);
    myMutex.lock();
    mySecsToFail = f;
    if(myPlanParams)
      myPlanParams->setSecsToFail(f);
    myMutex.unlock();
  }
  /// Sets the minimum angle from the path orientation to allow linear motion.
  void   setAlignAngle(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myAlignAngle = f;
    if(myPlanParams)
      myPlanParams->setAlignAngle(f);
    myMutex.unlock();
  }
  /// Sets the minimum speed at which align to path orient will take place.
  void   setAlignSpeed(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myAlignSpeed = f;
    if(myPlanParams)
      myPlanParams->setAlignSpeed(f);
    myMutex.unlock();
  }
  /// Sets the degree of the smoothing spline.
  void   setSplineDegree(int f)
  { 
    myMutex.lock();
    mySplineDegree = f;
    if(myPlanParams)
      myPlanParams->setSplineDegree(f);
    myMutex.unlock();
  }
  /// Sets the distance between knot points on the path to smooth with.
  void   setNumSplinePoints(int f)
  { 
    myMutex.lock();
    myNumSplinePoints = f;
    if(myPlanParams)
      myPlanParams->setNumSplinePoints(f);
    myMutex.unlock();
  }
  /// Sets the local plan flag.
  void   setNoLocalPlanFlag(bool f)
  { 
    myMutex.lock();
    myNoLocalPlanFlag = f; 
    myMutex.unlock();
  }
  /// Sets the old style heading move flag.
  void   setCheckInsideBounds(bool f)
  { 
    myMutex.lock();
    myCheckInsideBoundsFlag = f; 
    myMutex.unlock();
  }
  /// Set the clear on fail flag.
  void   setClearOnFailFlag(bool f)
  { 
    myMutex.lock();
    myClearOnFailFlag = f; 
    myMutex.unlock();
  }
  /// Sets use emergency stop if collision imminent flag.
  void   setUseEStopFlag(bool f)
  { 
    myMutex.lock();
    myUseEStopFlag = f;
    myMutex.unlock();
  }
#if 0
  /// Sets the curvature speed factor.
  AREXPORT void   setCurvatureSpeedFactor(double f);
  /// Sets the cost of going against the one way direction.
  AREXPORT void   setOneWayCost(double f);
  /// Sets the cost of going away from the central spine of the one way area.
  AREXPORT void   setCenterAwayCost(double f);
  /// Sets the cost of path point inside a resistance area.
  AREXPORT void   setResistance(double f);
#endif
  /// Sets the use resistance areas flag.
  void   setUseResistanceFlag(bool f)
  {
    myMutex.lock();
    myUseResistanceFlag = f;
    myMutex.unlock();
  }
  /// Sets the factor by which an existing path point costs less.
  void   setMarkOldPathFactor(double f)
  {
    myMutex.lock();
    myMarkOldPathFactor = f;
    myMutex.unlock();
  }
  /// Sets the fraction of the path from which the old path marking starts.
  void   setStartMarkPoint(double f)
  {
    myMutex.lock();
    myStartMarkPoint = f;
    myMutex.unlock();
  }
  /// Sets the preferred line width.
  void   setPreferredLineWidth(double f)
  {
    myMutex.lock();
    myPreferredLineWidth = f;
    myMutex.unlock();
  }
  /// Sets the factor by which an existing path point costs less.
  void   setLogFlag(bool f)
  {
    myMutex.lock();
    myLogFlag = f;
    myMutex.unlock();
  }
  /// Sets the factor by which an existing path point costs less.
  void   setLogLevel(int f)
  {
    myMutex.lock();
    myLogLevel = f;
    myMutex.unlock();
  }
  /// Sets the factor to multiply the robot lengths to match path.
  void   setMatchLengths(double f)
  { 
    f = ArMath::fabs(f);
    myMutex.lock();
    myMatchLengths = f;
    if(myPlanParams)
      myPlanParams->setMatchLengths(f);
    myMutex.unlock();
  }
  /// Get the local path planning flag.
  void   setLocalPathPlanningFlag(bool f)
  {
    myMutex.lock();
    myLocalPathPlanningFlag = f;
    myMutex.unlock();
  }
  /// Get the local path planning flag.
  void   setCheckInsideRadiusFlag(bool f)
  {
    myMutex.lock();
    myCheckInsideRadiusFlag = f;
    myMutex.unlock();
  }
  /** @brief Sets path plan parameters which affect obstacle avoidance & 
      tracking (for internal use). 
      *  @internal
      */
  AREXPORT void   setPlanParams(double robotWidth, 
				double robotRadius,
				double robotLengthFront, 
				double robotLengthRear,
				double useRearFraction,
				bool useRadiusForPlan,
				double frontClearance, double sideClearance, 
				double obsThreshold,
				double maxLinAcc, double maxRotAcc,
				double maxLinDec, double maxRotDec,
				double maxLinVel, double maxRotVel,
				double headingWt, double distanceWt, 
				double velocityWt, double smoothingWt,
				double gridRes,
				int nli, int nri, int sws,
				double maxObsDistance, 
				double goalDistTol, double goalAngTol,
				double PlanFreeSpace, 
				int secsToFail, 
				double alignAngle, double alignSpeed,
				int splineDegree, int numSplinePoints,
				double goalOccupiedFailDistance,
				double curvatureSpeedFactor,
				bool useCollisionRangeForPlanningFlag,
				double oneWayCost,
				double centerAwayCost,
				double onewayToOldCostFactor,
				double localPathFailDistance,
				short resistance,
				double markOldPathFactor,
				double matchLengths,
				bool checkInsideRadius,
				double preferWidth,
				double noLocalPlanLookAhead,
				double wrongSideCost,
				double sidedToOldCostFactor,
				double sidedToOldCostFactorMain,
				bool legacyFreeSpaceCosting,
				double sideAwayCost);

  /** @brief Sets only the dynamic motion params (for internal use).
   *  @internal
   */
  AREXPORT void   setMovementParams(double linVel, double rotVel,
				    double linAcc, double rotAcc,
				    double linDec, double rotDec);
  //@}

  /** @name Modify the set of range devices used by ARNL. */
  //@{
  
  /// Adds a range sensing device
  AREXPORT void   addRangeDevice(ArRangeDevice *device, RangeType type);

  /// Remove a range device from the robot's list, by name
  AREXPORT void   remRangeDevice(const char *name, RangeType type);
  /// Remove a range device from the robot's list, by instance
  AREXPORT void   remRangeDevice(ArRangeDevice *device, RangeType type);

  /// Adds a global range device.
  AREXPORT void   addGlobalPlanningRangeDevice(ArRangeDevice *device, 
						RangeType type);
  /// Remove a global range device from the robot's list, by name.
  AREXPORT void   remGlobalPlanningRangeDevice(const char *name, 
					       RangeType type);
  /// Remove a global range device from the robot's list, by instance
  AREXPORT void   remGlobalPlanningRangeDevice(ArRangeDevice *device, 
					       RangeType type);
  /// Gets the range device list
  std::list<ArRangeDevice*>* getRangeDeviceList(void)
  {
    return &myRangeDeviceList;
  }
  /// Gets the cumulative range device list
  std::list<ArRangeDevice*>* getCumRangeDeviceList(void)
  {
    return &myCumRangeDeviceList;
  }
  /// Gets the global range device list
  std::list<ArRangeDevice*>* getGlobalPlanningRangeDeviceList(void)
  {
    return &myGlobalPlanningRangeDeviceList;
  }
  /// Gets the global range device list
  std::list<ArRangeDevice*>* getGlobalPlanningCumRangeDeviceList(void)
  {
    return &myGlobalPlanningCumRangeDeviceList;
  }

  /// Clears the range devices.
  AREXPORT void   clearRangeDevices(void);
  /// Clears the global range devices.
  AREXPORT void   clearGlobalPlanningRangeDevices(void);
  /// Clears the global cum range devices.
  AREXPORT void   clearGlobalPlanningCumRangeDevices(unsigned int cyt);

  /// Clears the cumulative range devices.
  AREXPORT void   clearCumRangeDevices(unsigned int cyt);
  
  //@}
 
  
  /** What to do when goal cannot be reached.
   *  @internal
   */
  AREXPORT void   goalFailed(ArPose goal,
		     const char* failureString = "Failed going to goal",
			     PathPlanningState state = FAILED_MOVE);
  /// Get the time robot was not moving.
  double getLastMoveTime(void)
  {
    double p;
    myMutex.lock();
    p = myLastMoveTime;
    myMutex.unlock();
    return p;
  }
  /// Get the time robot was not moving.
  double getLastLinearMoveTime(void)
  {
    double p;
    myMutex.lock();
    p = myLastLinearMoveTime;
    myMutex.unlock();
    return p;
  }
  /// Set the time the robot was not moving.
  virtual void   setLastMoveTime(double t)
  {
    myMutex.lock();
    myLastMoveTime = t;
    myMutex.unlock();
  }
  /// Set the time the robot was not moving.
  void   setLastLinearMoveTime(double t)
  {
    myMutex.lock();
    myLastLinearMoveTime = t;
    myMutex.unlock();
  }
  /// Compute the progress from start to given pose.
  AREXPORT double computeProgress(ArPose robotPose);

  /// Compute the progress from start to robot's present pose.
  double computeProgress() 
  {
    return myRobot ? computeProgress(myRobot->getPose()) : 0;
  }
  /// Estimates an approximate time from given pose to goal in seconds..
  AREXPORT double estimateTimeToGoal(ArPose robotPose);

  /// Estimates an approximate time from robot's current pose to goal in seconds..
  double estimateTimeToGoal() 
  {
    return myRobot ? estimateTimeToGoal(myRobot->getPose()) : 0;
  }
  /// Estimates an approximate distance from given pose to goal in mm.
  AREXPORT double estimateDistanceToGoal(ArPose robotPose);

  /// Estimates an approximate distance from robot's current pose to 
  /// goal in mm.
  double estimateDistanceToGoal() 
  {
    return myRobot ? estimateDistanceToGoal(myRobot->getPose()) : 0;
  }

  /** @name ArNetworking callback methods
      Provide ArFunctor objects which invoke these methods to ArServerHandlerDrawings::addDrawing(),
	  and MobileEyes (enable them in the Map menu) and other clients will be able to draw these
	  diagnostic visualizations.
   */
  //@{
  
  /// Draws the local search rectangle.
  AREXPORT void   drawSearchRectangle(ArServerClient* client, 
				      ArNetPacket* packet);

  /// Draws the points seen by the sensor and not in the map.
  AREXPORT void   drawNewPoints(ArServerClient* client, ArNetPacket* packet);

  /// Draws the local special points.
  AREXPORT void   drawGridPoints(ArServerClient* client, ArNetPacket* packet);
  /// Draws the resistance points.
  AREXPORT void   drawResistancePoints(ArServerClient* client, 
				       ArNetPacket* packet);
  /// Draws the local path points.
  AREXPORT void   drawPathPoints(ArServerClient* client, ArNetPacket* packet);
  /// Draws the local obstacle points.
  AREXPORT void   drawObsPoints(ArServerClient* client, ArNetPacket* packet);
  /// Draws the obs points causing path plan failure.
  AREXPORT void   drawFailPoints(ArServerClient* client, ArNetPacket* packet);

  /// Draws the display points.
  AREXPORT void   drawVelocityPath(ArServerClient* client,
				   ArNetPacket* packet);
  /// Draws the robot boundary.
  AREXPORT void   drawRobotBounds(ArServerClient* client, 
				  ArNetPacket* packet);
  /// Draws the collide track.
  AREXPORT void   drawCollidePath(ArServerClient* client, 
				  ArNetPacket* packet);
  /// Draws the collide track.
  AREXPORT void   drawCollidePathFront(ArServerClient* client, 
				       ArNetPacket* packet);
  /// Draws the collide track.
  AREXPORT void   drawCollidePathLeft(ArServerClient* client, 
				      ArNetPacket* packet);
  /// Draws the collide track.
  AREXPORT void   drawCollidePathBack(ArServerClient* client, 
				      ArNetPacket* packet);
  /// Draws the collide track.
  AREXPORT void   drawCollidePathRight(ArServerClient* client, 
				       ArNetPacket* packet);

  //@}

  /// Returns the last local path planning result.
  ArPathPlan::LocalPathState  getLocalPathState(void) 
  { return myLocalPathState; }
  /// Returns the last velocity search state from the DWA.
  ArPathPlan::SearchVelocityState  getDWAState(void) 
  { return myDWAState; }
  /** Returns cost of the robots cell in the occupancy grid.
    @internal
  */
  AREXPORT double  getCost(void);
  /** Returns util of the robots cell in the occupancy grid.
    @internal
  */
  AREXPORT double  getUtil(void);



  /* -- Not implemented --
 /// Incorporate sensor readings when path is blocked.
 AREXPORT bool    incorporateBlockedPathRangeSensor();
  */

  /// Returns just the grid based search path from a point to another.
  AREXPORT std::list<ArPose> getPathFromTo(ArPose from, ArPose to, 
					   bool threadSafe = true,
					   ArPathPlan *pathPlan = NULL);
  /// Creates a path plan for use with getPathFromTo (and the plan params it needs)... delete these whenever the config changes but not while you're using getPathFromTo with it
  AREXPORT void createPathFromToHelpers(ArPlanParams **planParams, 
					ArPathPlan **pathPlan);
  /// Get the pointer to the list of obstacle points used for path planning.
  AREXPORT std::list<ArPose>* getObsListPtr(void);

private:
  /// Finds if any obstacles inside bounds.
  bool obstaclePointsInsideRobotBounds(void);
  /// Finds if any obstacles inside radius.
  bool obstaclePointsInsideRobotCircle(void);

public:
  /// Add a callback function from the list to call when path blocked.
  void    addBlockedPathCB(ArFunctor1<const std::list<ArPose>* >* cb)
  {
    myBlockedPathCallbacks.push_back(cb);
  }

  /// Remove a callback function from the list to call when path blocked.
  void    remBlockedPathCB(ArFunctor1<const std::list<ArPose>* >* cb)
  {
    myBlockedPathCallbacks.remove(cb);
  }

  /// Adds a plain callback for when the path is blocked
  void addPlainBlockedPathCB(ArFunctor *functor, int position = 50)
  { myPlainBlockedPathCBList.addCallback(functor, position); }
  /// Removes a plain callback for when the path is blocked
  void remPlainBlockedPathCB(ArFunctor *functor)
  { myPlainBlockedPathCBList.remCallback(functor); }
  
  /** Invoke all the callbacks after a replan due to path block.
   *  @internal
   */
  AREXPORT void invokeBlockedPathCB(void);

  /// Get the waiting to fail flag.
  bool    getWaitingToFailFlag(void)
  {
    bool p;
    myMutex.lock();
    p = myWaitingToFailFlag;
    myMutex.unlock();
    return p;
  }

protected:
  // Function used to run the task as a thread.
  virtual void* runThread(void* ptr);
  // The sensor interpretation callback. Called every 100msec.
  void          robotCallBack(void)
  {
    myMutex.lock();
    mySensorSetFlag = true;
    myMutex.unlock();
  }



  // Needed if the laser does not connect first.
  bool          configureLaser(void)
  {
    return true;
  }
  // Get the action fire flag
  bool          getActionFireFlag(void)
  {
    bool p;
    myMutex.lock();
    p = myActionFireFlag;
    myMutex.unlock();
    return p;
  }
  // Get the goal reached flag.
  bool          getGoalReachedFlag(void)
  {
    bool p;
    myMutex.lock();
    p = myGoalReachedFlag;
    myMutex.unlock();
    return p;
  }
  // Get the sensor set flag indicating new range data (set every 100ms).
  bool          getSensorSetFlag(void)
  {
    bool p;
    myMutex.lock();
    p = mySensorSetFlag;
    myMutex.unlock();
    return p;
  }
  // Get the plan failed flag.
  bool          getPlanFailedFlag(void)
  {
    bool p;
    myMutex.lock();
    p = myPlanFailedFlag;
    myMutex.unlock();
    return p;
  }
  // Get the goal altered flag.
  bool          getGoalAlteredFlag(void)
  {
    bool p;
    myMutex.lock();
    p = myGoalAlteredFlag;
    myMutex.unlock();
    return p;
  }
  // Get the goal set flag
  bool          getGoalSetFlag(void)
  {
    bool p;
    myMutex.lock();
    p = myGoalSetFlag;
    myMutex.unlock();
    return p;
  }
  // Get the goal planned flag
  bool          getGoalPlannedFlag(void)
  {
    bool p;
    myMutex.lock();
    p = myGoalPlannedFlag;
    myMutex.unlock();
    return p;
  }
  // Get the goal heading flag.
  bool          getGoalHeadingFlag(void)
  {
    bool p;
    myMutex.lock();
    p = myGoalHeadingFlag;
    myMutex.unlock();
    return p;
  }
  // Get the path is blocked flag.
  bool          getReplanGlobalPathFlag(void)
  {
    bool p;
    myMutex.lock();
    p = myReplanGlobalPathFlag;
    myMutex.unlock();
    return p;
  }
  // Return time of last replanning.
  unsigned int  getLastPlanTime(void)
  {
    myMutex.lock();
    unsigned int ret = myPlanTime;
    myMutex.unlock();
    return ret;
  }

  // Set the goal set Flag
  void          setActionFireFlag(bool f)
  {
    myMutex.lock();
    myActionFireFlag = f;
    myMutex.unlock();
  }

  // Set the current goal for path planning.
  void          setCurrentGoal(ArPose p)
  {
    myMutex.lock();
    myCurrentGoal = p;
    myMutex.unlock();
  }
  // Set the value of flag indicating new sensor data.
  void          setSensorSetFlag(bool f)
  {
    myMutex.lock();
    mySensorSetFlag = f;
    myMutex.unlock();
  }
  // Set the value of initialized flag.
  void          setInitializedFlag(bool f)
  {
    myMutex.lock();
    myInitializedFlag = f;
    myMutex.unlock();
  }
  // Set the goal reached flag.
  void          setGoalReachedFlag(bool f)
  {
    myMutex.lock();
    myGoalReachedFlag = f;
    myMutex.unlock();
  }
  // Set the plan failed flag.
  void          setPlanFailedFlag(bool f)
  {
    myMutex.lock();
    myPlanFailedFlag = f;
    myMutex.unlock();
  }
  // Set the goal set Flag
  void          setGoalSetFlag(bool f)
  {
    myMutex.lock();
    myGoalSetFlag = f;
    myMutex.unlock();
  }
  // Set the goal planned Flag
  void          setGoalPlannedFlag(bool f)
  {
    myMutex.lock();
    myGoalPlannedFlag = f;
    myMutex.unlock();
  }
  // Set the goal heading Flag.
  void          setGoalHeadingFlag(bool f)
  {
    myMutex.lock();
    myGoalHeadingFlag = f;
    myMutex.unlock();
  }
  // Set the goal heading Flag.
  void          setGoalAlteredFlag(bool f)
  {
    myMutex.lock();
    myGoalAlteredFlag = f;
    myMutex.unlock();
  }
  // Set the blocked path flag.
  void          setReplanGlobalPathFlag(bool f)
  {
    myMutex.lock();
    myReplanGlobalPathFlag = f;
    myMutex.unlock();
  }
  // Set the waiting to failsecs flag.
  void          setWaitingToFailFlag(bool f)
  {
    myMutex.lock();
    myWaitingToFailFlag = f;
    myMutex.unlock();
  }
  // Set the time of last plan.
  void          setLastPlanTime(void)
  {
    myMutex.lock();
    myPlanTime = ArUtil::getTime();
    myMutex.unlock();
  }
  // Sets the state of path planning.
  void          setState(PathPlanningState s, 
			 const char *failureString = NULL);
  // Sets the progress and time to now.
  void          setProgressNow(double p)
  {
    myMutex.lock();
    myProgress.setPose(p, 0);
    myProgress.setTimeToNow();
    myMutex.unlock();
  }
  // Set up the path planning stuff and follow.
  bool          planAndSetupAction(ArPose from, bool sensorSeesBlock = false);
  // Actually drives the robot along the planned path.
  LocalPlanningState computeLocalMotion(ArPose robotPose, ArPose goalPose, 
					double& linVel, double& rotVel, 
					double& heading, double searchFactor,
					bool replan,
					double goalDistThres);
  // Wrapper to get to ArPathPlan to clear its obs point list and map.
  bool          clearMapAndObsList(double cd, ArPose rp)
  {
    if(myPathPlan)
      return myPathPlan->clearMapAndObsList(cd, rp);
    else
      return false;
  }

  // Clears all transient readings.
  void          clearRangeMap(void)
  {
    if(myPathPlan)
      return myPathPlan->clearRangeMap();
  }

  // Incorporates the sensor readings into the local map for obstacle avoid.
  bool          incorporateRangeIntoMap(ArRangeDevice* rangeDev, 
					double rangeDist, 
					ArPose robotPose,
					bool useSensorMap,
					bool notCumulative)
  {
    return myPathPlan->incorporateRangeIntoMap(rangeDev, 
					       rangeDist, 
					       robotPose,
					       useSensorMap,
					       notCumulative);
  }

  // Incorporate all the global range sensors.
  bool          incorporateGlobalPlanningRangeSensors(ArPose rp);
  // Checks whether there are any global range sensors.
  bool          hasGlobalPlanningRangeSensors(void)
  {
    return !(getGlobalPlanningRangeDeviceList()->empty() &&
	     getGlobalPlanningCumRangeDeviceList()->empty());
  }

  // Checks to see if path to goal is a straight line.
  bool          pathEqualsStraightLine(void);

  // Sets the clearances according to speed and return those vals.
  void          setVariableClearances(double lvel, double avel,
				      double& front, double& side);
  // Fills the local obstacle list.
  void          fillLocalObstacleList(double lvel, double avel,
				      ArPose robotPose);
  // Find the closest obstacle within collision range.
  double        findDistanceToCollision(double lvel, double avel,
					ArPose robotPose);
  // What to do when goal is reached.
  void          goalDone(ArPose goal);
  // What to do when goal was interrupted.
  void          goalInterrupted(ArPose goal);
  // Needed if the params are changed or loaded again.
  bool          reconfigurePathPlanning(void);
  // Setup the path planing params with the aria config thing.
  void          setupPathPlanningParams(void);
  // Function for the things to do if map changes.
  void          mapChanged(void);
  // Get the linear deceleration.
  double        getMaxLinDecel(void) 
  {
    if(myPlanParams)
      return myPlanParams->getMaxLinDecel();
    else
      return 0.0;
  }
  // Get the progress with time.
  ArPoseWithTime getProgress(void) {return myProgress; }
  // Do a sane check on the param values.
  void          doSanityCheck(double width, double radius,
			      double lengthFront, double lengthRear,
			      double robotVel, double robotRotVel, 
			      double robotRotAccel, double robotRotDecel,
			      double robotAccel, double robotDecel);
  // Clear drawings in ArPathPlan.
  void clearDrawings(void)
  {
    if(myPathPlan)
      myPathPlan->clearDrawings();
  }
  private:

  bool     myActionFireFlag;          // Flag to say if the action should fire
  bool     myInitializedFlag;         // Robot pose is initialized.
  bool     myVerboseFlag;             // Print details flag.
  bool     mySensorSetFlag;           // Flag to set off laser incorporation.
  bool     myGoalReachedFlag;         // Flag to stop robot in runThread.
  bool     myPlanFailedFlag;          // Flag to indicate local planning failed
  bool     myGoalSetFlag;             // Flag to start main plan.
  bool     myGoalPlannedFlag;         // Flag to tell path is computed.
  bool     myGoalHeadingFlag;         // Flag to tell if goal heading needed.
  bool     myGoalAlteredFlag;         // Flag to tell if goal changed in move.
  ArPose   myCurrentGoal;             // Current goal.
  bool     myUseLaserFlag;            // Use the laser for collision avoid?
  bool     myUseSonarFlag;            // Use the sonar for collision avoid?
  bool     myMapChangedFlag;          // Use to trigger reloading map.
  bool     myReplanGlobalPathFlag;    // Checks if path blocked.
  bool     myWaitingToFailFlag;       // Set when counting up to failSecs.

  PathPlanningState myState;          // Current state
  std::string myFailureString;        // Reason for failure

  ArPathPlan::LocalPathState myLocalPathState; // Local planning result.
  ArPathPlan::SearchVelocityState myDWAState;  // Local DWA result.
  
  unsigned int                             myPlanTime;
  double                                   myLastMoveTime;
  double                                   myLastLinearMoveTime;
  ArPoseWithTime                           myProgress;

  ArRobot*                                 myRobot;
  ArRangeDevice*                           myLaser;
  ArRangeDevice*                           mySonar;
  ArMutex                                  myMutex;

  ArPathPlan*                              myPathPlan;
  ArPlanParams*                            myPlanParams;
  ArConfig*                                myParams;

  ArFunctorC<ArPathPlanningTask>*          myRobotCB;
  ArFunctor1C<int, ArPathPlanningTask>*    myFailedCB;

  ArRetFunctorC<bool, ArPathPlanningTask>* myLaserConnectedCB;
  ArRetFunctorC<bool, ArPathPlanningTask>* myRobotConnectedCB;

  ArRetFunctorC<bool, ArPathPlanningTask>* myProcessFileCB;
  ArFunctorC<ArPathPlanningTask>*          myMapChangedCB;

  ArActionPlanAndMoveToGoal*               myPathPlanAction;
  ArActionMovementParameters*              myMovementParamsAction;
  ArActionStallRecover*                    myStallAction;
  ArActionGroup*                           myPlanAndMoveGroup;
  bool                                     myOwnMyPlanAndMoveGroup;
  ArFunctor*                               myPlanDoneCB;

  std::list<ArRangeDevice*>                myRangeDeviceList;
  std::list<ArRangeDevice*>                myCumRangeDeviceList;
  std::list<ArRangeDevice*>                myGlobalPlanningRangeDeviceList;
  std::list<ArRangeDevice*>                myGlobalPlanningCumRangeDeviceList;

  protected:
  double myMaxSpeed;
  double myMaxRotSpeed;
  double myCollisionRange;
  double myFrontClearance;
  double myUsingFrontClearance;
  double mySlowSpeed;
  double myUsingSlowSpeed;
  double mySideClearanceAtSlowSpeed;
  double myUsingSideClearanceAtSlowSpeed;
  double myFrontPaddingAtSlowSpeed;
  double myUsingFrontPaddingAtSlowSpeed;
  double myFastSpeed;
  double myUsingFastSpeed;
  double mySideClearanceAtFastSpeed;
  double myUsingSideClearanceAtFastSpeed;
  double myFrontPaddingAtFastSpeed;
  double myUsingFrontPaddingAtFastSpeed;
  double mySuperMaxTransDecel;
  double myEmergencyMaxTransDecel;
  double myWidth;
  double myRadius;
  double myLengthFront;
  double myLengthRear;
  double myUseRearFraction;
  double myPlanRes;
  bool   myUseRadiusForPlan;
  double myPlanFreeSpace;
  double myUsingPlanFreeSpace;
  double myHeadingWt;
  double myDistanceWt;
  double myVelocityWt;
  double mySmoothingWt;
  int    myLinVelIncrements;
  int    myRotVelIncrements;
  int    mySmoothSize;
  double myObsThreshold;
  double myUsingObsThreshold;
  double myMaxExpansionFactor;
  bool   myUseCollisionRangeForPlanningFlag;
  double myGoalDistanceTol;
  double myGoalAngleTol;
  double myGoalSpeed;
  double myGoalRotSpeed;
  double myGoalRotVelKp;
  double myGoalTransAccel;
  double myGoalRotAccel;
  double myGoalTransDecel;
  double myGoalRotDecel;
  double myGoalSwitchTime;
  bool   myGoalUseEncoderFlag;
  double myGoalOccupiedFailDistance;
  double myUsingGoalOccupiedFailDistance;
  double myLocalPathFailDistance;
  double myUsingLocalPathFailDistance;
  double myHeadingRotSpeed;
  double myHeadingRotAccel;
  double myHeadingRotDecel;
  int    mySecsToFail;
  double myAlignSpeed;
  double myAlignAngle;
  int    mySplineDegree;
  int    myNumSplinePoints;
  bool   myClearOnFailFlag;
  int    myNumTimesToReplan;
  bool   myUseEStopFlag;
  double myCurvatureSpeedFactor;
  double myOneWayCost;
  double myCenterAwayCost;
  bool   myUseOneWaysFlag;
  double myOneWayToOldCostFactor;
  short  myResistance;
  short  myPreference;
  bool   myUseResistanceFlag;
  double myMarkOldPathFactor;
  double myStartMarkPoint;
  double myMatchLengths;
  bool   myCheckInsideRadiusFlag;
  bool   myCheckInsideBoundsFlag;
  bool   myLocalPathPlanningFlag;
  bool   myLogFlag;
  int    myLogLevel;
  double myPreferredLineWidth;
  bool   myLocalPlanMap;
  bool   myNoLocalPlanFlag;
  double myNoLocalPlanLookAhead;
  double myPreferredDirectionWrongWayCost;
  double myPreferredDirectionToOldCostFactor;
  double myPreferredDirectionToOldCostFactorMain;
  bool   myLegacyFreeSpaceCosting;
  double myPreferredDirectionAwayCost;
  double myPreferredDirectionSideOffset;
};

/*!
 * This is the action for local obstacle avoidance and path following.
 * This class uses the ArPathPlanningTask methods to drive the robot safely.
 */
class ArActionPlanAndMoveToGoal : public ArAction
{
  public:
  /// Constructor, sets the maximums and the parent pointer.
  AREXPORT  ArActionPlanAndMoveToGoal(double maxVel, double maxRotVel, 
				      ArPathPlanningTask* ppt, 
				      ArRangeDevice *laser, 
				      ArRangeDevice *sonar);
  /// Destructor, no new objects, we don't need to do anything
  AREXPORT virtual ~ArActionPlanAndMoveToGoal(void) {};
  /// Fire, this is what the resolver calls to find out what this action wants
  AREXPORT virtual ArActionDesired* fire(ArActionDesired currentDesired);
  /// Sets the robot pointer, also gets the sonar device (unused now)
  //AREXPORT virtual void setRobot(ArRobot *robot);
  /// Overrides base deactivate to uninitialize ArPathPlanningTask
  AREXPORT virtual void deactivate(void);
  /// Set the two vels.
  AREXPORT void         setTotalVel(double lvel, double avel);
  /// Set end move params such as accels and decels for fine positioning.
  AREXPORT void         setEndMoveParams(void);
  /// Set params such as accels and decels for heading only situations.
  AREXPORT void         setHeadingParams(void);

  /// Clear the range devices.
  void         clearRangeDevices(void)
  {
    if(myPathPlanningTask)
      myPathPlanningTask->clearRangeDevices();
  }
  /// Clear the cumulative range devices.
  void         clearCumRangeDevices(unsigned int cyt)
  {
    if(myPathPlanningTask)
      myPathPlanningTask->clearCumRangeDevices(cyt);
  }

  /// Set max vel.
  void         setMaxVel(double v) 
  { myMaxVel = ArMath::fabs(v); }
  /// Set max rot vel.
  void         setMaxRotVel(double r) 
  { myMaxRotVel = ArMath::fabs(r); }
  /// Set times failed
  void         setFailedTimes(int i)  { myFailedTimes = i; }
  /// Set range factor.
  void         setRangeFactor(double r)  { myRangeFactor = r; }
  /// Set end move flag.
  void         setEndMoveStopRotationFlag(bool f)  
  { myEndMoveStopRotationFlag = f; }
  /// Set end move flag.
  void         setEndMoveFlag(bool f)  { myEndMoveFlag = f; }
  /// Set plan state
  void         setPlanState(ArPathPlanningTask::LocalPlanningState s) 
  { myPlanState = s; }
  /// Set the no of times we have replanned a goal
  void         setReplannedTimes(int f) { myReplannedTimes = f; }
  /// Set the no of times to replan to a goal.
  void         setNumTimesToReplan(int f) { myNumTimesToReplan = f; }
  /// Get max vel.
  double       getMaxVel(void) { return myMaxVel; }
  /// Get max rot vel.
  double       getRotMaxVel(void) { return myMaxRotVel; }
  /// Get times failed
  int          getFailedTimes(void)  { return myFailedTimes; }
  /// Get range factor.
  double       getRangeFactor(void)  { return myRangeFactor; }
  /// Get end move flag.
  bool         getEndMoveFlag(void)  { return myEndMoveFlag; }
  /// Get end move stop rotation flag.
  bool         getEndMoveStopRotationFlag(void)  
  { return myEndMoveStopRotationFlag; }
  /// Get last plan state.
  ArPathPlanningTask::LocalPlanningState getPlanState(void)  
  { return myPlanState; }
  /// Get the no of times we have replanned a goal
  int          getReplannedTimes(void) { return myReplannedTimes; }
  /// Get the no of times to replan to a goal
  int          getNumTimesToReplan(void) { return myNumTimesToReplan; }
  
  /// Set the desired action.
  AREXPORT void         setCurrentDynamicParams(ArActionDesired curDesired);
  /// This gets how close we are to failing as a ratio between 0 and 1
  AREXPORT double       getFailedTimeRatio(void);

  /// Get the desired action.
  virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  virtual const ArActionDesired *getDesired(void) const 
  { return &myDesired; }
#endif
  /// Computes the safe deceleration from obstacle distance if any.
  AREXPORT double computeSafeDecel(double obsDist,
				   double linVel,
				   double rotVel,
				   double currentDecel,
				   bool& eStopFlag,
				   bool& brakeFlag);
  // This is a helper class that sets vels to 0 and then returns 
  AREXPORT ArActionDesired* stop(double obsDist,
				 double linVel,
				 double rotVel,
				 double currentDecel,
				 bool skipComputeDecel = true,
				 bool eStop = false);

  protected:

  /// This holds the laser info.
  ArRangeDevice* myLaser;
  /// Holds sonar stuff.
  ArRangeDevice* mySonar;
  /// What the action wants to do
  ArActionDesired myDesired;
  /// Max translational velocity.
  double myMaxVel;
  /// Max rotational velocity.
  double myMaxRotVel;
  /// Current linear velocity.
  double myVel;
  /// Current angular velocity.
  double myRotVel;
  /// Current heading;
  double myHeading;
  /// Link to the class that does the actual path planning.
  ArPathPlanningTask* myPathPlanningTask;
  /// Failed times.
  int myFailedTimes;
  /// Range factor.
  double myRangeFactor;
  /// End move flag.
  bool myEndMoveFlag;
  /// End move stop rotatinon flag.
  bool myEndMoveStopRotationFlag;
  /// End move position
  ArPose myEndMoveEncoderGoal;
  /// Planning state.
  ArPathPlanningTask::LocalPlanningState myPlanState;
  /// No of times the same goal was replanned.
  int myReplannedTimes;
  /// Time
  ArTime myFireTime;
  /// E Stop Flag.
  bool myEStopFlag;
  /// How many times we should replan
  int myNumTimesToReplan;
};
/*!
 * This is the range device which can hold global information which can then
 * be used to replan a path when the robot finds a block in a planned path.
 * Unlike other approaches where the robot just replan again from the blocked
 * pose using just the local information, this structure will hold the blocks
 * in memory until the goal is reached. This will avoid cases where the robot
 * will shuttle between two closed doors in an attempt to get inside a room.
 */

class ArGlobalReplanningRangeDevice : public ArRangeDevice
{

  public:
  ArGlobalReplanningRangeDevice(
	  ArPathPlanningTask *pathTask, size_t currentBufferSize=10000,
	  size_t cumulativeBufferSize=10000, 
	  const char *name="Global Replanning") : 
  ArRangeDevice(currentBufferSize, cumulativeBufferSize, name, 0),
  myClearCB(this, &ArGlobalReplanningRangeDevice::clear),
  myAddReadingsCB(this, &ArGlobalReplanningRangeDevice::addReadings)
  {
    myPathTask = pathTask;
    myPathTask->addGlobalPlanningRangeDevice(this, 
					       ArPathPlanningTask::CUMULATIVE);
    myPathTask->addGoalFinishedCB(&myClearCB);
    myPathTask->addBlockedPathCB(&myAddReadingsCB);
    setCumulativeDrawingData(new ArDrawingData("polyDots", 
					       ArColor(0xaa, 0xff, 0xff),
					       100, 
					       55, 200, "DefaultOff"), true);
  }

  virtual ~ArGlobalReplanningRangeDevice() {} 

  void clear(ArPose pose) 
  {
    lockDevice();
    myCumulativeBuffer.beginRedoBuffer();
    myCumulativeBuffer.endRedoBuffer();
    unlockDevice();
  }

  void addReadings(const std::list<ArPose> *readings)
  {
    lockDevice();
    std::list<ArPose>::const_iterator it;
    for (it = readings->begin(); it != readings->end(); it++)
      myCumulativeBuffer.addReading((*it).getX(), (*it).getY());
    unlockDevice();
  }

  protected:
  ArPathPlanningTask *myPathTask;
  ArFunctor1C<ArGlobalReplanningRangeDevice, ArPose> myClearCB;
  ArFunctor1C<ArGlobalReplanningRangeDevice, 
  const std::list<ArPose>*> myAddReadingsCB;

};

#endif // ARPATHPLANNINGTASK_H
