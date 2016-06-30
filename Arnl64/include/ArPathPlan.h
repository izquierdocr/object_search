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
 * File: ArPathPlan.h
 * 
 * Function: Header for the pathplan.cpp program file.
 *
 * Created:  George V. Paul. gvp@activmedia.com. April 25 2003.
 * Modified: George V. Paul. gvp@activmedia.com. July 9 2003. To add the
 *           param class.
 *
 *****************************************************************************/
#ifndef ARPATHPLAN_H
#define ARPATHPLAN_H

#define RADS2DEGS 57.29577951
#define DEGS2RADS 0.017453293

#include <math.h>
#include "Aria.h"
#include "ArNetworking.h"
// KMC #include "ArExport.h"
#include "ArValueIteration.h"
#include "ArOccGrid.h"
#include "ArNetworking.h"

#include <set>
#include <vector>
#include <algorithm>

class ArPathPlan;
class ArPlanParams;
class ArRunningStats;

/*
  Class to hold parameters related to collision avoidance & pathplaning.
*/

class ArPlanParams
{
  public:

  /// Base Constructor.
  ArPlanParams() : myRobotWidth(400.0), myRobotRadius(300.0), 
  myRobotLengthFront(250.0), myRobotLengthRear(250.0), 
  myUseRearFraction(1.0),
  myUseRadiusForPlan(false),
  myFrontClearance(100.0), mySideClearance(50.0),
  myObsThreshold(0.2),
  myMaxLinAccel(600.0), myMaxRotAccel(400.0), 
  myMaxLinDecel(600.0), myMaxRotDecel(400.0),
  myMaxLinVel(600.0), myMaxRotVel(100.0), myHeadingWt(0.8),
  myDistanceWt(0.1), myVelocityWt(0.1),  mySmoothingWt(0.0), 
  myPlanRes(100.0),
  myLinVelIncrements(4), myRotVelIncrements(4), mySmoothWinSize(2), 
  myMaxObsDist(3000.0), 
  myGoalDistanceTolerance(200.0), myGoalAngleTolerance(10),
  myPlanFreeSpace(1000.0),
  mySecsToFail(4), myAlignAngle(10), myAlignSpeed(10),
  mySplineDegree(3), myNumSplinePoints(5),
  myGoalOccupiedFailDistance(1000),
  myCurvatureSpeedFactor(1.0),
  myUseCollisionRangeForPlanningFlag(false),
  myOneWayCost(1.0), myCenterAwayCost(1.0), myOneWayToOldCostFactor(0.9),
  myLocalPathFailDistance(1000),
  myResistance(1),
  myMarkOldPathFactor(0.75),
  myMatchLengths(2.0),
  myCheckInsideRadiusFlag(true),
  myNoLocalPlanLookAhead(200.0),
  myWrongSideCost(10.0),
  mySidedToOldCostFactor(0.25),
  mySidedToOldCostFactorMain(0.9),
  myLegacyFreeSpaceCosting(false),
  mySideAwayCost(1.0) {}

  /// Constructor with more params.
  ArPlanParams(double a, double r, double bf, double br, 
	       double c, double d, double e) :
  myRobotWidth(a), myRobotRadius(r),
  myRobotLengthFront(bf), myRobotLengthRear(br),
  myUseRearFraction(1.0),
  myUseRadiusForPlan(false),
  myFrontClearance(c), mySideClearance(d),
  myObsThreshold(e),
  myMaxLinAccel(600.0), myMaxRotAccel(400.0), 
  myMaxLinDecel(600.0), myMaxRotDecel(400.0),
  myMaxLinVel(600.0), myMaxRotVel(100.0), myHeadingWt(0.8),
  myDistanceWt(0.1), myVelocityWt(0.1), mySmoothingWt(0.0), myPlanRes(100.0),
  myLinVelIncrements(4), myRotVelIncrements(4), mySmoothWinSize(2), 
  myMaxObsDist(3000.0), 
  myGoalDistanceTolerance(200.0), myGoalAngleTolerance(10),
  myPlanFreeSpace(1000.0),
  mySecsToFail(4), myAlignAngle(10), myAlignSpeed(10),
  mySplineDegree(3), myNumSplinePoints(5),
  myGoalOccupiedFailDistance(1000),
  myCurvatureSpeedFactor(1.0),
  myUseCollisionRangeForPlanningFlag(false),
  myOneWayCost(1.0), myCenterAwayCost(1.0), myOneWayToOldCostFactor(0.9),
  myLocalPathFailDistance(1000),
  myResistance(1),
  myMarkOldPathFactor(0.75), 
  myMatchLengths(2.0),
  myCheckInsideRadiusFlag(true),
  myNoLocalPlanLookAhead(200.0),
  myWrongSideCost(10.0),
  mySidedToOldCostFactor(0.25),
  mySidedToOldCostFactorMain(0.9),
  myLegacyFreeSpaceCosting(false),
  mySideAwayCost(1.0) {}

  /// Returns width of the robot.
  double getWidth(void) {return myRobotWidth;}
  /// Returns the radius.
  double getRadius(void) {return myRobotRadius;}
  /// Returns length of the robot.
  double getLength(void) {return myRobotLengthFront+myRobotLengthRear;}
  /// Returns front length of the robot.
  double getLengthFront(void) {return myRobotLengthFront;}
  /// Returns rear length of the robot.
  double getLengthRear(void) {return myRobotLengthRear;}
  /// Returns fraction of rear lenght to use.
  double getUseRearFraction(void) {return myUseRearFraction;}
  /// Returns flag for using radius instead of half width.
  bool getUseRadiusForPlan(void) {return myUseRadiusForPlan;}
  /// Returns safety distance in front. Note that myFrontClearance is the 
  /// clearance + variable speed based padding updated every robot cycle.
  double getVariableFrontClearance(void) {return myFrontClearance;}
  /// Returns safety clearance in the sides. Note mySideClearance is the
  /// variable side clearance based on speed updated every robot cycle.
  double getVariableSideClearance(void) {return mySideClearance;}
  /// Returns threshold to consider as obstacle in the occupancy map.
  double getObsThreshold(void) {return myObsThreshold;}
  /// Returns maximum linear acceleration
  double getMaxLinAccel(void) {return myMaxLinAccel;}
  /// Returns maximum rotational acceleration
  double getMaxRotAccel(void) {return myMaxRotAccel;}
  /// Returns maximum linear deceleration
  double getMaxLinDecel(void) {return myMaxLinDecel;}
  /// Returns maximum rotational deceleration
  double getMaxRotDecel(void) {return myMaxRotDecel;}
  /// Returns maximum linear velocity.
  double getMaxLinVel(void) {return myMaxLinVel;}
  /// Returns maximum rotational velocity.
  double getMaxRotVel(void) {return myMaxRotVel;}
  /// Returns heading weight.
  double getHeadingWt(void) {return myHeadingWt;}
  /// Returns distance weight.
  double getDistanceWt(void) {return myDistanceWt;}
  /// Returns velocity weight.
  double getVelocityWt(void) {return myVelocityWt;}
  /// Returns smoothing weight.
  double getSmoothingWt(void) {return mySmoothingWt;}
  /// Returns path planning resolution.
  double getPlanRes(void) {return myPlanRes;}
  /// Gets no of linear velocity increments in the dynamic window.
  int    getLinVelIncrements(void) {return myLinVelIncrements;}
  /// Gets no of rotational velocity increments in the dynamic window.
  int    getRotVelIncrements(void) {return myRotVelIncrements;}
  /// Gets maximum obstacle distance.
  double getMaxObsDist(void) {return myMaxObsDist;}
  /// Gets smoothing window size.
  int    getSmoothWinSize(void) {return mySmoothWinSize;}
  /// Gets the goal distance tolerance
  double getGoalDistanceTolerance(void) {return myGoalDistanceTolerance;}
  /// Gets the goal angle tolerance
  double getGoalAngleTolerance(void) {return myGoalAngleTolerance;}
  /// Gets distance to free space from the robot.
  double getPlanFreeSpace(void) {return myPlanFreeSpace;}
  /// Gets seconds to wait till local search fails.
  int    getSecsToFail(void) {return mySecsToFail;}
  /// Gets minimum angle to local destination for robot to move to align.
  double getAlignAngle(void) {return myAlignAngle;}
  /// Gets maximum speed for robot to move to align.
  double getAlignSpeed(void) {return myAlignSpeed;}
  /// Gets degree of the spline.
  int    getSplineDegree(void) {return mySplineDegree;}
  /// Gets increments at which to set control points.
  int    getNumSplinePoints(void) {return myNumSplinePoints;}
  /// Gets min distance to approach goal when it is occupied.
  double getGoalOccupiedFailDistance(void) {return myGoalOccupiedFailDistance;}
  /// Gets the factor by which speed is reduced for curved paths.
  double getCurvatureSpeedFactor(void) {return myCurvatureSpeedFactor;}
  /// Gets the flag to allow use of collision range to plan look ahead.
  bool   getUseCollisionRangeForPlanningFlag(void) 
  {return myUseCollisionRangeForPlanningFlag;}
  /// Gets the one way cost
  double getOneWayCost(void) {return myOneWayCost;}
  /// Gets the reistance
  short  getResistance(void) {return myResistance;}
  /// Gets the center way cost
  double getCenterAwayCost(void) {return myCenterAwayCost;}
  /// Gets the one way to old cost factor.
  double getOneWayToOldCostFactor(void) {return myOneWayToOldCostFactor; }
  /// Gets min distance to approach even when path is blocked.
  double getLocalPathFailDistance(void) {return myLocalPathFailDistance;}
  /// Gets mark old path factor.
  double getMarkOldPathFactor(void) {return myMarkOldPathFactor;}
  /// Gets no of robot lengths to match path with.
  double getMatchLengths(void) {return myMatchLengths; }
  /// Gets the flag for checking if an obstacle is inside the robot circle.
  bool   getCheckInsideRadiusFlag(void) {return myCheckInsideRadiusFlag;}
  /// Gets the preferred line width.
  double getPreferredLineWidth(void) {return myPreferredLineWidth;}
  /// Gets the lookahead when there is no replanning.
  double getNoLocalPlanLookAhead(void) {return myNoLocalPlanLookAhead;}
  /// Gets the wrong side cost.
  double getWrongSideCost(void) {return myWrongSideCost;}
  /// Gets the side away cost.
  double getSideAwayCost(void) {return mySideAwayCost;}
  /// Gets the sided to old cost factor for cells.
  double getSidedToOldCostFactor(void) {return mySidedToOldCostFactor;}
  /// Gets the sided to old cost factor for cells. Main path.
  double getSidedToOldCostFactorMain(void) {return mySidedToOldCostFactorMain;}
  /// Gets the legacy free space costing for one ways. (for RMT)
  bool   getLegacyFreeSpaceCosting(void) {return myLegacyFreeSpaceCosting;}

  /// Sets width of robot.
  void   setWidth(double w) {myRobotWidth = w;}
  /// Sets radius of robot.
  void   setRadius(double w) {myRobotRadius = w;}
  /// Sets front length of robot.
  void   setLengthFront(double l) {myRobotLengthFront = l;}
  /// Sets rear length of robot.
  void   setLengthRear(double l) {myRobotLengthRear = l;}
  /// Returns fraction of rear lenght to use.
  void   setUseRearFraction(double l) {myUseRearFraction = l;}
  /// Returns flag for using radius instead of half width.
  void   setUseRadiusForPlan(bool l) {myUseRadiusForPlan = l;}
  /// Sets safety distance in front
  void   setFrontClearance(double f) {myFrontClearance = f;}
  /// Sets safety clearance in the sides.
  void   setSideClearance(double s) {mySideClearance = s;}
  /// Sets threshold to consider obstacle in the occupancy map.
  void   setObsThreshold(double t) {myObsThreshold = t;}
  /// Sets maximum linear acceleration
  void   setMaxLinAccel(double v) {myMaxLinAccel = v;}
  /// Sets maximum rotational acceleration
  void   setMaxRotAccel(double v) {myMaxRotAccel = v;}
  /// Sets maximum linear deceleration
  void   setMaxLinDecel(double v) {myMaxLinDecel = v;}
  /// Sets maximum rotational deceleration
  void   setMaxRotDecel(double v) {myMaxRotDecel = v;}
  /// Sets maximum linear velocity.
  void   setMaxLinVel(double v) {myMaxLinVel = v;}
  /// Sets maximum rotational velocity.
  void   setMaxRotVel(double v) {myMaxRotVel = v;}
  /// Sets heading weight.
  void   setHeadingWt(double v) {myHeadingWt = v;}
  /// Sets distance weight.
  void   setDistanceWt(double v) {myDistanceWt = v;}
  /// Sets velocity weight.
  void   setVelocityWt(double v) {myVelocityWt = v;}
  /// Sets smoothing weight.
  void   setSmoothingWt(double v) {mySmoothingWt = v;}
  /// Sets plan resolution.
  void   setPlanRes(double pres) {myPlanRes = pres;}
  /// Sets no of linear velocity increments for the dynamic window.
  void   setLinVelIncrements(int nli) {myLinVelIncrements = nli;}
  /// Sets no of rotational velocity increments for the dynamic window.
  void   setRotVelIncrements(int nri) {myRotVelIncrements = nri;}
  /// Sets maximum obstacle distance.
  void   setMaxObsDist(double sdist) {myMaxObsDist = sdist;}
  /// Sets smoothing window size.
  void   setSmoothWinSize(int sws) {mySmoothWinSize = sws;}
  /// Sets goal distance tolerance
  void   setGoalDistanceTolerance(double g) {myGoalDistanceTolerance = g;}
  /// Sets goal angle tolerance
  void   setGoalAngleTolerance(double g) {myGoalAngleTolerance = g;}
  /// Sets distance to free space from the robot.
  void   setPlanFreeSpace(double g) {myPlanFreeSpace = g;}
  /// Sets seconds to wait till local search fails.
  void   setSecsToFail(int g) {mySecsToFail = g;}
  /// Sets minimum angle to local destination for robot to move to align.
  void   setAlignAngle(double g) {myAlignAngle = g;}
  /// Sets maximum speed for robot to move to align.
  void   setAlignSpeed(double g) {myAlignSpeed = g;}
  /// Sets degree of the spline.
  void   setSplineDegree(int g) {mySplineDegree = g;}
  /// Sets increments at which to set control points.
  void   setNumSplinePoints(int g) {myNumSplinePoints = g;}
  /// Sets min distance to approach goal when it is occupied.
  void   setGoalOccupiedFailDistance(double g) {myGoalOccupiedFailDistance=g;}
  /// Sets the factor by which speed will be reduced for curved paths.
  void   setCurvatureSpeedFactor(double g) {myCurvatureSpeedFactor = g;}
  /// Sets the flag to allow use of collision range to plan look ahead.
  void   setUseCollisionRangeForPlanningFlag(double g) 
  {myUseCollisionRangeForPlanningFlag = g;}
  /// Sets the one way cost
  void   setOneWayCost(double g) {myOneWayCost = g;}
  /// Sets the resistance.
  void   setResistance(short g) {myResistance = g;}
  /// Sets the center way cost
  void   setCenterAwayCost(double g) {myCenterAwayCost = g;}
  /// Sets the one way to old cost factor.
  void   setOneWayToOldCostFactor(double g) {myOneWayToOldCostFactor = g;}
  /// Sets min distance to approach even when path is blocked.
  void   setLocalPathFailDistance(double g) {myLocalPathFailDistance=g;}
  /// Sets mark old path factor.
  void   setMarkOldPathFactor(double g) {myMarkOldPathFactor=g;}
  /// Sets no of robot lengths to match path with.
  void   setMatchLengths(double g) {myMatchLengths=g;}
  /// Sets the flag for checking if an obstacle is inside the robot circle.
  void   setCheckInsideRadiusFlag(bool g) {myCheckInsideRadiusFlag=g;}
  /// Sets the preferred line width.
  void   setPreferredLineWidth(double g) {myPreferredLineWidth=g;}
  /// Sets the lookahead when there is no replanning.
  void   setNoLocalPlanLookAhead(double g) {myNoLocalPlanLookAhead=g;}
  /// Sets the wrong side cost.
  void   setWrongSideCost(double g) {myWrongSideCost=g;}
  /// Sets the side away cost.
  void   setSideAwayCost(double g) {mySideAwayCost=g;}
  /// Sets the sided to old cost factor for cells.
  void   setSidedToOldCostFactor(double g) {mySidedToOldCostFactor=g;}
  /// Sets the sided to old cost factor for cells. Main path.
  void   setSidedToOldCostFactorMain(double g) {mySidedToOldCostFactorMain=g;}
  /// Sets the legacy free space costing for one ways. (for RMT)
  void   setLegacyFreeSpaceCosting(bool g) {myLegacyFreeSpaceCosting=g;}

  /// Sets up all numbers for path planning.
  void   setPlanParams(double robotWidth, double robotRadius,
		       double robotLengthFront, double robotLengthRear, 
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
		       double planFreeSpace,
		       int secsToFail, 
		       double alignAngle, double alignSpeed,
		       int splineDegree, int numSplinePoints,
		       double goalOccupiedFailDistance,
		       double curvatureSpeedFactor,
		       bool useCollisionRangeForPlanning,
		       double oneWayCost, double centerAwayCost,
		       double oneWayToOldCostFactor,
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
  /// Sets up all numbers for path planning.
  void   setMovementParams(double maxLinVel, double maxRotVel,
			   double maxLinAcc, double maxRotAcc,
			   double maxLinDec, double maxRotDec);
  private:

  double myRobotWidth;
  double myRobotRadius;
  double myRobotLengthFront;
  double myRobotLengthRear;
  double myUseRearFraction;
  bool   myUseRadiusForPlan;
  double myFrontClearance;
  double mySideClearance;
  double myObsThreshold;
  double myMaxLinAccel;
  double myMaxRotAccel;
  double myMaxLinDecel;
  double myMaxRotDecel;
  double myMaxLinVel;
  double myMaxRotVel;
  double myHeadingWt;
  double myDistanceWt;
  double myVelocityWt;
  double mySmoothingWt;
  double myPlanRes;
  int    myLinVelIncrements;
  int    myRotVelIncrements;
  int    mySmoothWinSize;
  double myMaxObsDist;
  double myGoalDistanceTolerance;
  double myGoalAngleTolerance;
  double myPlanFreeSpace;
  int    mySecsToFail;
  double myAlignAngle;
  double myAlignSpeed;
  int    mySplineDegree;
  int    myNumSplinePoints;
  double myGoalOccupiedFailDistance;
  double myCurvatureSpeedFactor;
  bool   myUseCollisionRangeForPlanningFlag;
  double myOneWayCost;
  double myCenterAwayCost;
  double myOneWayToOldCostFactor;
  double myLocalPathFailDistance;
  short  myResistance;
  double myMarkOldPathFactor;
  double myMatchLengths;
  bool   myCheckInsideRadiusFlag;
  double myPreferredLineWidth;
  double myNoLocalPlanLookAhead;
  double myWrongSideCost;
  double mySidedToOldCostFactor;
  double mySidedToOldCostFactorMain;
  bool   myLegacyFreeSpaceCosting;
  double mySideAwayCost;
};


// This is a class for computing a running average of a number of elements
class ArRunningStats
{
public:
  /// Constructor, give it the number of elements you want to average
  ArRunningStats(size_t numToMax)
  {
    myNumToMax = numToMax;
    myMax = 0;
    myMin = 0;
    myAve = 0;
    myNum = 0;
  }
  /// Destructor
  virtual ~ArRunningStats() {}
  /// Gets the Max
  double getMax(void) const { return myMax;}
  /// Gets the Min
  double getMin(void) const { return myMin;}
  /// Gets the Average
  double getAverage(void) const { return myAve;}
  /// Adds a number
  void add(double val) 
  {
    myNum++;
    myVals.push_back(val);
    if (myVals.size() > myNumToMax || myNum > myNumToMax)
    {
      myNum--;
      myVals.erase(myVals.begin());
    }
    size_t size = myVals.size();
    myMax = val;
    myMin = val;
    myAve = 0;
    for(size_t i = 0; i < size; i++)
    {
      if(myMax < myVals[i])
	myMax = myVals[i];
      if(myMin > myVals[i])
	myMin = myVals[i];
      myAve += myVals[i];
    }
    if(size > 0)
      myAve /= ((double) size);
  }
  /// Clears the average
  void clear(void)
  {
    while (myVals.size() > 0)
      myVals.pop_back();
    myNum = 0;
    myMax = 0;
    myMin = 0;
    myAve = 0;
  }
  /// Gets the number of elements
  size_t getNumToMax(void) const { return myNumToMax;}
  /// Sets the number of elements
  void setNumToMax(size_t numToMax)
  {
    myNumToMax = numToMax;
    while (myVals.size() > myNumToMax)
    {
      myNum--;
      myVals.erase(myVals.begin());
    }
    size_t size = myVals.size();
    myMax = myVals[0];
    myMin = myVals[0];
    myAve = 0;
    for(size_t i = 0; i < size; i++)
    {
      if(myMax < myVals[i])
	myMax = myVals[i];
      if(myMin > myVals[i])
	myMin = myVals[i];
      myAve += myVals[i];
    }
    if(size > 0)
      myAve /= ((double) size);
  }

protected:
  size_t myNumToMax;
  double myMax;
  double myMin;
  double myAve;
  size_t myNum;
  std::vector<double> myVals;
};

/* 
  @class ArPathPlan
  @internal
  @brief Class holds everything related to path planing and following.
*/
class ArPathPlan
{

public:

  /// State of the dynamic window search
  enum SearchVelocityState
  {
    GOT_VELS,           /// Computed desired velocities.
    GOT_VEL_HEADING,    /// Computed linear vel and heading.
    OBSTACLE_TOO_CLOSE, /// Obstacle too close for move.
    COMPLETELY_BLOCKED, /// All directions blocked.
    NOT_INITIALIZED     /// States are not initiailized.
  };
  /// State of the local path plan
  enum LocalPathState
  {
    BAD_MAIN_PLAN,      /// The mother plan is bad.
    ROBOT_TOO_FAR,      /// Robot is too far of main plan.
    GOAL_OCCUPIED,      /// Goal is unreachable.
    PLAN_INIT_FAILED,   /// Initialization of search failed.
    PLAN_MEM_FAILED,    /// Memory allocation of search failed.
    PLAN_PATH_FAILED_GFD,/// Search failed + Goal Failed Distance.
    PLAN_PATH_FAILED_LFD,/// Search failed + Local Failed Distance.
    PLAN_PATH_FAILED,   /// Search failed.
    LOOKAHEAD_SMALL,    /// Adaptive lookahead too small.
    GOT_BLOCKED_PLAN,   /// Plan found.
    GOT_PLAN,           /// Plan found.
    GOT_FOLLOW_PLAN_LFD /// Follow plan found but local failed distance.

  };

friend class Localize;
friend class ArPathPlanningTask;

  /// Base constructor.
  ArPathPlan(void);
  /// Elaborate constructor.
  ArPathPlan(ArRobot* ro, ArPlanParams* pp, ArMapInterface* am);
  /// Base destructor.
  ~ArPathPlan(void);
  /// Initialize stuff for fence following.
  bool   initializeLineToGoal(bool useSensorMap);
  /// Computes the local line to a goal for fence or wall following.
  int    computeLineToGoal(ArPose from, ArPose to, 
			   bool useSensorMap = false);
  /// Computes the local path to a goal for fence or wall following.
  int    computeLocalPathToGoal(ArPose from, ArPose to, bool useSensorMap);

  /// Computes the list of successive cells making path and return path 
  /// length.
  int    computeMainPathToGoal(ArPose from, ArPose to, 
			       bool useSensorMap = false);
  /// Scan the range device buffer into the two arrays, and adds to ObsList.
  bool   scanRangeIntoArray(ArRangeDevice* rdev,
			    std::vector<ArPose>& xyArray,
			    bool notCumulative=true);
  /// Reset both local grid maps using map data.
  bool   initializeGridsAndVI(bool changed=false, bool useOneWays=true,
			      short resistance=1, short preference=1,
			      double sideOffset=0.25,
			      bool localPlanMap = false);
  /// Check if the grid has been initialized.
  bool   isGridInitialized(void) 
  {
    if(myGridMap && myPlanParams)
      return true;
    else 
      return false;
  }
  /// Finds the average distance to path from the line.
  double matchPathWithLine(double lv, double av,
			   double rx, double ry, double rt, 
			   double maxobsdist, std::vector<ArPose>& path,
			   std::vector<double>& cumPDist,
			   bool drawPath);
  /// Finds the average distance to path from the line.
  double matchPathWithCircle(double lv, double av,
			     double rx, double ry, double rt, 
			     double maxobsdist, std::vector<ArPose>& path,
			     std::vector<double>& cumPDist,
			     bool drawPath);
  /// Finds the average distance to path from the line.
  double matchPathWithCourse(double lv, double av,
			     double rx, double ry, double rt, 
			     double maxobsdist, std::vector<ArPose>& path,
			     std::vector<double>& cumPDist,
			     bool drawPath=false);
  /// Finds the distance to intersection of a given velocity pair.
  double findDistanceToCollisionMapped(double lv, double av, 
				       double rx, double ry, double rt, 
				       double maxobsdist);
  /// Finds the exact distance to the nearest obstacle with st line move.
  double intersectLineWithEnvSensed(double lv, double av, 
				    double rx, double ry, double rt, 
				    double maxobsdist,
				    bool drawHit=false);
  /// Finds the exact distance to the nearest obstacle with cirular arc.
  double intersectCircleWithEnvSensed(double lv, double av, 
				      double rx, double ry, double rt, 
				      double maxobsdist,
				      bool drawHit=false);
  /// Finds the exact distance to intersection of a given velocity pair.
  double findDistanceToCollisionSensed(double lv, double av, 
				       double rx, double ry, double rt, 
				       double maxobsdist,
				       bool drawHit=false);
  /// Finds if any obstacle point inside robot boundary.
  bool   obstaclePointsInsideRobotBounds(void);
  /// Finds if any obstacle point inside robot circle.
  bool   obstaclePointsInsideRobotCircle(void);
  /// Finds if a given point is inside a polygon.
  bool   insidePolygon(std::vector<ArPose> points, ArPose p);

  /// Does the key dynamic window approach to collision avoidance.
  SearchVelocityState searchVelocitySpace(ArPose desp, ArPose gdest, 
					  double curvature, 
					  ArPose realp, ArPose goal,
					  double& lvel, double& avel, 
					  double& heading, 
					  double goalDecel,
					  double goalTime,
					  double goalDistThres,
					  bool followLFD,
					  bool oldHeadingMove);
  /// Smooth the objective array.
  bool   smoothArray(int m, int n);

  /// Predict a pose in the path.
  double predictPose(double lv, double av, 
		     double rx, double ry, double rt,
		     double& fx, double& fy, double& ft, 
		     double ti, double lmdec);
  /// Finds the heading objective.
  double findHeadingObjective(double lv, double av, 
			      double dx, double dy,
			      double rx, double ry, double rt,
			      double it, double lmdec);
  /// Fills the local obstacle list from the sensor.
  void   fillLocalObstacleList(double lvel, double avel, 
			       ArPose robotPose,
			       double maxObsDist);
  /// Fills the robot boundary points.
  void fillRobotBounds(void);
  /// Finds the objective function value due to heading for given velocity.
  double findHeadingObjective(double av, double dangle, double ti);
  /// Finds the objective function value due to heading for given velocity.
  double findRotVelObjective(double av, double dangle, double ti);
  /// Clear transients.
  void   clearRangeMap(void);
  /// Use the range data to set the appropriate grid map values.
  bool   incorporateRangeIntoMap(ArRangeDevice* rangeDev, 
				 double rangeLimit,
				 ArPose robotPose,
				 bool useSensorMap,
				 bool notCumulative);
  /// Search locally for path in case of obstacles.
  LocalPathState  searchForLocalGoal(ArPose robot, ArPose goal, 
				     ArPose& dest, ArPose& gdest, 
				     double& curvature, 
				     double lvel, double avel, double factor,
				     bool replan, 
				     double startMarkPoint,
				     bool localPlanMap = false);
  LocalPathState  searchForLocalGoal2(ArPose robot, ArPose goal, 
				      ArPose& dest, ArPose& gdest, 
				      double& curvature, 
				      double lvel, double avel, double factor,
				      bool replan, bool localPlanMap = false);
  /// Smooth the path into spline.
  void   smoothPath(std::vector<ArPose>& path, 
		    std::vector<ArPose>& smoothpath, 
		    double& tangentAngle,
		    int incr,
		    bool passThrough);
  /// Reduce the path to cell size segments.
  int    reducePath(std::vector<ArPose> path, 
		    std::vector<ArPose>& reducedpath, double res);
  /// Finds the index in the path to a goal la distance away.
  int    findLocalGoalIndex(int start, int la, std::vector<ArPose>& path);
  /// Finds the bounding box of the local path.
  void   findPathBounds(std::vector<ArPose>& path, int start, int end, 
			int& binx, int& biny, int& baxx, int& baxy);
  /// Finds the bounding box of the local path.
  void   findObstacleBounds(std::list<ArPose>& obsList,
			    ArPose robotPose, double range, double margin,
			    int& binx, int& biny, int& baxx, int& baxy);
  /// Finds the closest index in the path to the robot location.
  int    findClosestIndex(ArPose robot, std::vector<ArPose>& path, int i=0);
  /// Finds the exit of path in a box.
  int    findLocalExitIndex(int xstart, int ystart, int xend, int yend, 
			    int start,
			    std::vector<ArPose>& path);
  /// Finds the point within fail distance of goal.
  int    findFailGoalIndex(int xStart, int yStart, int xEnd, int yEnd,
			   int start, int end,
			   int xSt, int ySt, int xSi, int ySi,
			   std::vector<ArPose>& path);
  /// Finds the point within fail distance of local path.
  int    findFailLocalIndex(int xStart, int yStart, int xEnd, int yEnd,
			    int start, int end,
			    int xSt, int ySt, int xSi, int ySi,
			    std::vector<ArPose>& path);
  /// Finds if there is a block in the path from unmapped obstacle.
  bool   findBlock(int start, int end, int& blockIndex, int& safeIndex,
		   double& blockDist, std::vector<ArPose>& path);
  /// Set plan param pointer.
  void   setPlanParamsPtr(ArPlanParams* pptr) {myPlanParams = pptr;}
  /// Gets new obstacle Factor.
  double  getNewObsFactor(void);
  /// Sets new obstacle Factor.
  void    setNewObsFactor(double f);
  /// Local distance check.
  double  findClosestObstacle(double rx, double ry, double rt, 
			      double lvel, double avel, 
			      double maxobsdist);
  /// Finds the closest point on the path.
  ArPose  findClosestOnPath(double dist,
			    int& first,
			    std::vector<ArPose>& path,
			    std::vector<double>& cumPDist);

  /// Clear the local obstacle point list and the sensor map.
  bool    clearMapAndObsList(double cd, ArPose rp);
  /// Gets the list of poses from current pose to dest.
  std::list<ArPose> getCurrentPath(ArPose robp, bool local=false);
  /// Get the aria map ptr.
  ArMapInterface*  getAriaMap(void) {return myAriaMap;}
  /// Gets the list of poses from start pose to dest.
  std::list<ArPose> getOriginalPath(void);
  /// Compute the progress to the goal from the start pose.
  double  computeProgress(ArPose robPose);
  /// Estimates an approximate time to goal in seconds..
  double  estimateTimeToGoal(ArPose robPose);
  /// Estimates an approximate distance to goal in mm.
  double  estimateDistanceToGoal(ArPose robPose);
  /// Gets the list of obstacle points expanded in CSpace.
  std::vector<ArPose> getCSpaceObstaclePoints(void);
  /// Gets the list of resistance points.
  std::vector<ArPose> getResistancePoints(void);
  /// Draw the search space.
  void    drawSearchRectangle(ArServerClient* client, ArNetPacket* packet,
			      bool initFlag);
  /// Draws the new obs points.
  void    drawNewPoints(ArServerClient* client, ArNetPacket* packet,
			bool initFlag);
  /// Draws the local special points.
  void    drawGridPoints(ArServerClient* client, ArNetPacket* packet,
			 bool initFlag);
  /// Draws the local special points.
  void    drawResistancePoints(ArServerClient* client, ArNetPacket* packet,
			       bool initFlag);
  /// Draws the local obstacle points.
  void    drawObsPoints(ArServerClient* client, ArNetPacket* packet,
			bool initFlag);
  /// Draws the obs points causing path plan failure.
  void    drawFailPoints(ArServerClient* client, ArNetPacket* packet,
			 bool initFlag);
  /// Draws the cspace points causing path plan failure.
  void    drawCSpacePoints(ArServerClient* client, ArNetPacket* packet,
			   bool initFlag);
  /// Draws the points.
  void    drawVelocityPath(ArServerClient* client, ArNetPacket* packet,
			   bool initFlag);
  /// Draws the collision points
  void    drawCollidePath(ArServerClient* client, ArNetPacket* packet,
			  bool initFlag);
  /// Draws the collide track.
  void   drawCollidePathFront(ArServerClient* client, 
			      ArNetPacket* packet,
			      bool initFlag);
  /// Draws the collide track.
  void   drawCollidePathLeft(ArServerClient* client, 
			     ArNetPacket* packet,
			     bool initFlag);
  /// Draws the collide track.
  void   drawCollidePathBack(ArServerClient* client, 
			     ArNetPacket* packet,
			     bool initFlag);
  /// Draws the collide track.
  void   drawCollidePathRight(ArServerClient* client, 
			      ArNetPacket* packet,
			      bool initFlag);
  /// Draws the robot boundary.
  void    drawRobotBounds(ArServerClient* client, ArNetPacket* packet,
			  bool initFlag);
  /// Clear drawings.
  void    clearDrawings(void);

  /// Debug function.
  bool    writePath(char* filename, std::vector<ArVINode*> path,
		    int dx, int dy, ArValueIteration* arVal);
  /// Debug stuff.
  bool    writeMemory(char* filename, int lx, int ly, ArnlFloat** mapPtr);

  /* Not implemented anymore.
  /// Debug function.
  bool    writePoints(char* filename,
		      int xStart, int yStart,
		      int lXSize, int lYSize,
		      ArValueIteration* arVI);
  */
  /// Returns cost of the robots cell in the occupancy grid.
  double  getCost(void);
  /// Returns util of the robots cell in the occupancy grid.
  double  getUtil(void);
  /// Return a pointer to the list of obstacle points used in path planning.
  std::list<ArPose>* getObsListPtr(void) {return &myObsList;}
  /// Limits velocity based on looking ahead for curves.
  double  lookAheadForCurvature(double linVel, double rotVel,
				double linVelLim, double rotVelLim);
  /// Integrates a path from current pose and velocity commands.
  std::vector<ArPose> simulatePath(ArPose robotPose, 
				   std::vector<ArPose>& commands,
				   double cyt);
  /// Checks to see if path to goal is a straight line.
  bool    pathEqualsStraightLine(double tolerance);

  private:

  ArOccGrid*                myGridMap;
  ArOccGrid*                mySensorMap;

  ArValueIteration*         myMainVI;
  ArValueIteration*         myLocalVI;

  ArRobot*                  myRobot;

  ArMapInterface*           myAriaMap;

  ArMutex                   myVIMutex;
  ArMutex                   myGridMutex;
  ArMutex                   mySenMutex;
  ArMutex                   myMaPaMutex;
  ArMutex                   myDisplayMutex;
  ArMutex                   myLocalVIMutex;
  ArMutex                   myLocalObsMutex;

  std::vector<ArPose>       myLocalPath;
  std::vector<ArPose>       myMainPath;
  std::vector<ArPose>       myOriginalPath;

  double                    myNewObsFactor;
  std::vector<ArPose>       myPathCurvature;
  double                    myMaxRotAccRequired;
  double                    myMaxRotDecRequired;
  ArPose                    mySearchCenter;
  ArPose                    mySearchSize;
  ArPose                    myDestPose;
  ArPose                    myTangentPose;
  ArPose                    myPreviousBackEndPose;
  ArPose                    myNearestPathPose;
  std::vector<ArPose>       myNewObsPoints;
  ArPose                    myPreviousStartPose;
  ArPose                    myPreviousEndPose;

  ArNetPacket               mySearchRectanglePacket;
  ArNetPacket               myNewPointsPacket;
  ArNetPacket               myGridPointsPacket;
  ArNetPacket               myResistancePointsPacket;
  ArNetPacket               myObsPointsPacket;
  ArNetPacket               myFailPointsPacket;
  ArNetPacket               myVelocityPathPacket;
  ArNetPacket               myRobotBoundsPacket;
  ArNetPacket               myCollidePathPacket;

  ArNetPacket               myCollidePathFrontPacket;
  ArNetPacket               myCollidePathLeftPacket;
  ArNetPacket               myCollidePathBackPacket;
  ArNetPacket               myCollidePathRightPacket;
  

  int                       myNOLI;
  int                       myNORI;
  double*                   myVl;
  double*                   myVa;
  double**                  mySd;
  double**                  mySh;
  double**                  mySv;
  double**                  mySm;
  double**                  mySp;
  double                    myLookAhead;
  
  public:

  ArPlanParams*             myPlanParams;   // Plan param holder.
  std::list<ArPose>         myObsList;      // Holds all sensor obstacles.
  std::vector<ArPose>       myFailObsList;  // Holds the points causing fail.
  std::vector<ArPose>       myCSpaceObsList;// Holds the CSpace Obs Points.
  std::list<ArPose>         myLocalObsList; // Holds closest obstacles.
  ArTransform               myLocalObsGlobalTrans; // Holds the last global.
  ArRunningStats*           myRunningStats;
  std::vector<ArPose>       myArcPoints;    // For display.
  std::vector<ArPose>       myRobotBounds;  // For display.
  std::vector<ArPose>       myCollidePath;  // For display.
  std::vector<ArPose>       myResistancePoints; // For display.
  LocalPathState            myPathState;    // For holding earlier state.

  std::vector<ArPose>       myCollidePathFront;  // For display.
  std::vector<ArPose>       myCollidePathLeft;   // For display.
  std::vector<ArPose>       myCollidePathBack;   // For display.
  std::vector<ArPose>       myCollidePathRight;  // For display.

  bool                      myLocalPlanMap;

};


#endif // ARPATHPLAN_H
