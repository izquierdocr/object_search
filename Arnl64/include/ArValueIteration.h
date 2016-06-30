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
/* ***************************************************************************
 * 
 * File: ArValueIteration.h
 * 
 * Function: Header for the valueiteration.cpp program file.
 *
 * Created:  George V. Paul. gvp@activmedia.com. February 25 2003.
 *
 *****************************************************************************/
#ifndef ARVALUEITERATION_H
#define ARVALUEITERATION_H

#include <math.h>
#include <set>
#include <vector>
#include <algorithm>
#include "ArOccGrid.h"

#define OBS_VAL 1000000.0
#define OBSTACLE_VAL OBS_VAL
#define FREE_VAL 0.1

#define EXIT_VALUE -2000000.0
#define MAX_VALUE 1000000.0
#define UNSET_VALUE -1.0

class ArVINode;
class ArValueIteration;

/* 
  @class ArVINode
  @internal
  @brief Class holds the nodes used in the search.
*/
class ArVINode
{
public:

  /// Base Constructor.	
  ArVINode() : myValue(0.0), myX(0),  myY(0) {}
  /// Base Constructor with location.
  ArVINode(int px, int py) : myValue(0.0), myX(px), myY(py) {}
  /// Base Constructor with location and value.
  ArVINode(int px, int py, double gf) : myValue(gf), myX(px), myY(py) {}
  /// Finds the neighbors.
  bool getEightNeighbors(ArValueIteration* as, std::vector<ArVINode*> *ns);
  /// Finds cells.
  bool getEightCells(ArValueIteration* as, std::vector<ArVINode*> *neighbors);

  /// Get Value.
  double getValue(void) {return myValue;}
  /// Get X.
  int getX(void) {return myX;}
  /// Get Y.
  int getY(void) {return myY;}
  /// Copy Constructor
  ArVINode(const ArVINode &nn) 
  {
    myValue = nn.myValue;
    myX = nn.myX;
    myY = nn.myY;
  }

private:

  double myValue;
  int myX;
  int myY;	

};

/* 
  @class ArValueIteration
  @internal
  @brief Class holds everything related to the search.
*/
class ArValueIteration
{

public:
  /// Result of the search.
  enum
  {
    NOT_INITIALISED,
    SEARCHING,
    SUCCEEDED,
    FAILED,
    OUT_OF_MEMORY,
    LOCAL_MINIMA,
    INVALID
  };

friend class ArVINode;

  /// Base constructor.
  ArValueIteration():  
  myState(NOT_INITIALISED),
  myNSteps(0),
  myCosts(NULL),
  myUtils(NULL),
  myAcc(NULL),
  myAngle(NULL),
  myResistance(NULL),
  myPreference(NULL),
  mySidedness(NULL),
  myGridSizeX(0),
  myGridSizeY(0),
  myActions(NULL),
  myNActions(0),
  myLegacyFreeSpaceCosting(false)
  {}
  /// Constructor.
  ArValueIteration(int x, int y);
  /// Destructor.
  ~ArValueIteration();
  /// Gets the state of search.
  int       getState(void) {return myState;}
  /// Gets the cost of the map location.
  ArnlFloat     getCost(int x, int y)
  {
    if((myCosts != NULL) && 
       (x >= 0) && (x < myGridSizeX) && (y >= 0) && (y < myGridSizeY))
      return myCosts[x][y];
    else
      return MAX_VALUE;
  }
  /// Gets the value of the map location.
  double    getUtil(int x, int y)
  {
    if((myUtils != NULL) && 
       (x >= 0) && (x < myGridSizeX) && (y >= 0) && (y < myGridSizeY))
      return myUtils[x][y];
    else
      return UNSET_VALUE;
  }
  /// Gets the acc of the map location.
  ArnlFloat getAcc(int x, int y)
  {
    if((myAcc != NULL) && 
       (x >= 0) && (x < myGridSizeX) && (y >= 0) && (y < myGridSizeY))
      return myAcc[x][y];
    else
      return UNSET_VALUE;
  }
  /// Gets the value of the map location.
  double    getAngle(int x, int y)
  {
    if((myAngle != NULL) && 
       (x >= 0) && (x < myGridSizeX) && (y >= 0) && (y < myGridSizeY))
      return ((double)(myAngle[x][y].angle));
    else
      return NO_DIRECTION;
  }
  /// Gets the value of the map location.
  double    getCenterDistance(int x, int y)
  {
    if((myAngle != NULL) && 
       (x >= 0) && (x < myGridSizeX) && (y >= 0) && (y < myGridSizeY))
    {
      if(myAngle[x][y].angle != NO_DIRECTION)
	return (((double)myAngle[x][y].distance)/CENTER_FACTOR);
      else
	return 0.0;
    }
    else
      return 0.0;
  }
  /// Gets the resistance of the cell
  short     getResistance(int x, int y)
  {
    if((myResistance != NULL) && 
       (x >= 0) && (x < myGridSizeX) && (y >= 0) && (y < myGridSizeY))
      return myResistance[x][y];
    else
      return NO_RESISTANCE;
  }
  /// Gets the preference of the cell
  short     getPreference(int x, int y)
  {
    if((myPreference != NULL) && 
       (x >= 0) && (x < myGridSizeX) && (y >= 0) && (y < myGridSizeY))
      return myPreference[x][y];
    else
      return NO_PREFERENCE;
  }
  /// Gets the angle of the sided area
  double     getSideAngle(int x, int y)
  {
    if((mySidedness != NULL) && 
       (x >= 0) && (x < myGridSizeX) && (y >= 0) && (y < myGridSizeY))
      return (double)(mySidedness[x][y].angle);
    else
      return NO_SIDEDNESS;
  }
  /// Gets the distance to the side.
  double    getSideDistance(int x, int y)
  {
    if((mySidedness != NULL) && 
       (x >= 0) && (x < myGridSizeX) && (y >= 0) && (y < myGridSizeY))
    {
      if(mySidedness[x][y].angle != NO_SIDEDNESS)
	return (((double) mySidedness[x][y].distance)/CENTER_FACTOR);
      else
	return 0.0;
    }
    else
      return 0.0;
  }
  /// Gets the closeness of a cell to nearest obstacle.
  double    getCloseness(int x, int y, 
			 double robotrad, double gres, double freedist);
  /// Gets the gradient at the location
  double    getGradient(int x, int y, double& gang, double& gmag);
  /// Gets the center away cost for one ways.
  double    getCenterAwayCost(void)
  {
    return myCenterAwayCost;
  }
  /// Gets the center away cost for one ways.
  double    getOneWayToOldCostFactor(void)
  {
    return myOneWayToOldCostFactor;
  }
  /// Gets the wrong side cost for sided sectors.
  double    getWrongSideCost(void)
  {
    return myWrongSideCost;
  }
  /// Gets the side away cost for sided sectors.
  double    getSideAwayCost(void)
  {
    return mySideAwayCost;
  }
  /// Gets the sided to old cost factor for sided sectors.
  double    getSidedToOldCostFactor(void)
  {
    return mySidedToOldCostFactor;
  }
  /// Gets the bool indicating use of legacy costing for one ways.
  bool      getLegacyFreeSpaceCosting(void)
  {
    return myLegacyFreeSpaceCosting;
  }
  /// Sets the cost of the map location.
  void      setCost(int x, int y, ArnlFloat v)
  {
    if((x >= 0) && (x < myGridSizeX) && (y >= 0) && (y < myGridSizeY))
      myCosts[x][y] = v;
  }
  /// Sets the value of the map location.
  void      setUtil(int x, int y, double v)
  {
    if((myUtils != NULL) && 
       (x >= 0) && (x < myGridSizeX) && (y >= 0) && (y < myGridSizeY))
      myUtils[x][y] = v;
  }
  /// Sets the acc of the map location.
  void      setAcc(int x, int y, ArnlFloat v)
  {
    if((myAcc != NULL) &&
       (x >= 0) && (x < myGridSizeX) && (y >= 0) && (y < myGridSizeY))
      myAcc[x][y] = v;
  }
  /// Sets the angle of the map location.
  void      setAngle(int x, int y, AngDis v)
  {
    if((myAngle != NULL) &&
       (x >= 0) && (x < myGridSizeX) && (y >= 0) && (y < myGridSizeY))
      myAngle[x][y] = v;
  }
  /// Sets the resistance at x y.
  void      setResistance(int x, int y, short v)
  {
    if((myResistance != NULL) &&
       (x >= 0) && (x < myGridSizeX) && (y >= 0) && (y < myGridSizeY))
      myResistance[x][y] = v;
  }
  /// Sets the preference at x y.
  void      setPreferencce(int x, int y, short v)
  {
    if((myPreference != NULL) &&
       (x >= 0) && (x < myGridSizeX) && (y >= 0) && (y < myGridSizeY))
      myPreference[x][y] = v;
  }
  /// Sets the sidedness of the map location.
  void      setSidedness(int x, int y, AngDis v)
  {
    if((mySidedness != NULL) &&
       (x >= 0) && (x < myGridSizeX) && (y >= 0) && (y < myGridSizeY))
      mySidedness[x][y] = v;
  }
  /// Get the number of iterations used in the search.
  int       getStepCount() {return myNSteps;}
  /// Allocate the memory for a node.
  ArVINode* allocateNode(int x = 0, int y = 0, double gf = 1)
  {
    return new ArVINode(x, y, gf);
  }
  /// Initialize the cost table.
  bool      initializeCosts(ArnlFloat** gridData,
			    double obsThres, double robotHalfWidth, 
			    double gridRes, double sideFreeClearance,
			    bool legacyFreeSpaceCosting);
  /// Expand the resistance areas by the robot half width.
  bool      expandResistanceAreas(short** resistData, double robotHalfWidth, 
				  double gridRes, short value);

  /// Expand the preference areas.
  bool      expandPreferenceAreas(short** preferData, double robotHalfWidth, 
				  double gridRes, double preferWidth);

  /// Mark the old path in the cost array.
  bool      markOldPath(std::vector<ArPose> oldPath, double factor,
			int start);
  /// Dynamic progamming to compute the values array.
  bool      iterateValuesInMap(int gx, int gy, 
			       double oneWayCost, double centerAwayCost,
			       double oneWayToOldCostFactor,
			       double wrongSideCost, 
			       double sidedToOldCostFactor,
			       double sideAwayCost,
			       int fx=-1, int fy=-1);
  /// 
  bool      iterateValuesInMapTrue(int gx, int gy);
  /// Function to initialize the main util map with dummy values for use
  /// when only local path planning is needed.
  bool      initializeUtilsInMap(void);
  /// Trace back best path.
  int       findPath(int fx, int fy, int gx, int gy, 
		     std::vector<ArVINode*>* path);
  /// Steepest descent.
  bool      findBestMove(int fx, int fy, int& dx, int& dy);
  /// Returns map dimensions.
  void      getGridSize(int& sx, int& sy) {sx=myGridSizeX; sy=myGridSizeY;}
  /// Returns map dimensions.
  int       getGridSizeX(void) { return myGridSizeX; }
  /// Returns map dimensions.
  int       getGridSizeY(void) { return myGridSizeY; }
  /// Return pointer to pathmap.
  double**  getPathMapData(void) {return myUtils;}
  /// Return pointer to actions
  int**     getActionData(void) {return myActions;}
  /// Returns no of actions.
  int       getNumActions(void) {return myNActions;}
  /// Gets Resistance data ptr.
  short**   getResistanceDataPtr(void) {return myResistance;}
  /// Finds the closest legal point.
  bool      findClosestLegalPoint(int& rx, int& ry, double maxDisplace);
  /// Frees the memory allocated for the path nodes.
  void      freePathMemory(std::vector<ArVINode*> path)
  {
    size_t pathSize = path.size();
    for(size_t i = 0; i < pathSize; i++)
      delete(path[i]);
  }
  /// Sets Angle data ptr.
  void      setAngleDataPtr(AngDis** ptr) {myAngle = ptr;}
  /// Sets Resistance data ptr.
  void      setResistanceDataPtr(short** ptr) {myResistance = ptr;}
  /// Sets Preference data ptr.
  void      setPreferenceDataPtr(short** ptr) {myPreference = ptr;}
  /// Sets Side data ptr.
  void      setSidednessDataPtr(AngDis** ptr) {mySidedness = ptr;}
  /// Writes costs into disk.
  bool      writeCosts(char* filename);
  /// Writes utils into disk.
  bool      writeUtils(char* filename);
  /// Writes costs into disk.
  bool      writeAngle(char* filename);

private: 

  int myState;
  int myNSteps;
  ArnlFloat** myCosts;
  double** myUtils;
  ArnlFloat** myAcc;
  AngDis** myAngle;
  short** myResistance;
  short** myPreference;
  AngDis** mySidedness;
  int myGridSizeX;
  int myGridSizeY;
  double myGridRes;
  int** myActions;
  int myNActions;
  double mySideFreeClearance;
  double myMinUtil;
  double myOneWayUtil;
  double myOneWayCost;
  double myCenterAwayCost;
  double myOneWayToOldCostFactor;
  double myWrongSideCost;
  double mySideAwayCost;
  double mySidedToOldCostFactor;
  bool myLegacyFreeSpaceCosting;
};

    
#endif // ARVALUEITERATION_H
