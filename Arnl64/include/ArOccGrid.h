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
 * File: ArOccGrid.h
 * 
 * Function: Header file for the grid.cpp file.
 *
 * Created:  George V. Paul. gvp@activmedia.com. December 4 2002.
 *
 *****************************************************************************/
#ifndef AROCCGRID_H
#define AROCCGRID_H

#define NO_SIDEDNESS 360
#define NO_DIRECTION 360
#define SPECIAL_DIRECTION 720
#define CENTER_FACTOR 1e3
#define NO_RESISTANCE 1
#define NO_PREFERENCE 1

#define RADS2DEGS  57.29577951
#define DEGS2RADS  0.017453293

#include <stdio.h>
#include "ariaInternal.h"
#include "ArMap.h"
#include "arnlInternal.h"

#define MEMORY_IS_CHEAP
#ifdef MEMORY_IS_CHEAP
typedef double ArnlFloat;
#else
typedef float ArnlFloat;
#endif
//
// Struct holding the direction and the distance to center data for oneway 
// and two way drive on side sectors.
//
struct tagAngDis
{
  short angle;
  short distance;
};

typedef tagAngDis AngDis;

/* 
  @class ArOccGrid.
  @internal
  @brief Class holds details of the occupancy grid made from the map.
*/
class ArOccGrid
{

public:

  /// Base Constructor.
  AREXPORT ArOccGrid(void);
  /// Base Destructor.
  AREXPORT ~ArOccGrid(void);

  /// Make the kernel and set its size.
  AREXPORT bool     makeKernel(int kn);
  /// Blur the grid values to allow for some uncertainity.
  AREXPORT bool     blurGrid(ArnlFloat** data, bool setToOne=false);
  /// Blur the grid values to allow for some uncertainity.
  AREXPORT bool     blurSubGrid(int xStart, int yStart,
				int xEnd, int yEnd,
				ArnlFloat** data, bool setToOne=false);
  /// Get the value of the occupancy at the (x,y) coords.
  ArnlFloat    getGridVal(double x, double y)
  {
    int xx = (int)((x - myMinMapX) / myRes);
    int yy = (int)((y - myMinMapY) / myRes);
    if(xx < 0 || xx >= myXLength || yy < 0 || yy >= myYLength)
      return 0;
    else
    {
      if(myNewData)
	return ArUtil::findMax(myGridData[xx][yy], 
			       (ArnlFloat)myNewData[xx][yy]);
      else
	return myGridData[xx][yy];
    }
  }
  /// Trace the ray from x1, y1 to x2, y2 and see if it is intercepted.
  AREXPORT ArnlFloat    traceGridVal(double x1, double y1, double x2, 
				     double y2,
				     double& retX, double& retY,
				     double occThres, double ignoreRadius);
  /// Get the value of the direction at the (x,y) coords.
  AngDis      getDirection(double x, double y)
  {
    AngDis NO_DIRECTION_STRUCT = {NO_DIRECTION, NO_DIRECTION};
    if(!myDirection)
      return NO_DIRECTION_STRUCT;
    int xx = (int)((x - myMinMapX) / myRes);
    int yy = (int)((y - myMinMapY) / myRes);
    if(xx < 0 || xx >= myXLength || yy < 0 || yy >= myYLength)
      return NO_DIRECTION_STRUCT;
    else
      return myDirection[xx][yy];
  }
  /// Get the value of the sidedness at the (x,y) coords.
  AngDis      getSidedness(double x, double y)
  {
    AngDis NO_SIDEDNESS_STRUCT = {NO_SIDEDNESS, NO_SIDEDNESS};

    if(!mySidedness)
      return NO_SIDEDNESS_STRUCT;
    int xx = (int)((x - myMinMapX) / myRes);
    int yy = (int)((y - myMinMapY) / myRes);
    if(xx < 0 || xx >= myXLength || yy < 0 || yy >= myYLength)
      return NO_SIDEDNESS_STRUCT;
    else
      return mySidedness[xx][yy];
  }
  /// Get the value of the resistance at the (x, y) coords.
  short       getResistance(double x, double y)
  {
    if(!myResistance)
      return NO_RESISTANCE;
    int xx = (int)((x - myMinMapX) / myRes);
    int yy = (int)((y - myMinMapY) / myRes);
    if(xx < 0 || xx >= myXLength || yy < 0 || yy >= myYLength)
      return 0;
    else
      return myResistance[xx][yy];
  }
  /// Get the value of the resistance at the (x, y) coords.
  short       getPreference(double x, double y)
  {
    if(!myPreference)
      return NO_PREFERENCE;
    int xx = (int)((x - myMinMapX) / myRes);
    int yy = (int)((y - myMinMapY) / myRes);
    if(xx < 0 || xx >= myXLength || yy < 0 || yy >= myYLength)
      return 0;
    else
      return myPreference[xx][yy];
  }
  /// Get the current X resolution in mm.
  double   getRes(void) {return myRes;}
  /// Get the min map X coord in mm.
  double   getMinMapX(void) {return myMinMapX;}
  /// Get the min map Y coord in mm.
  double   getMinMapY(void) {return myMinMapY;}
  /// Get the max map X coord in mm.
  double   getMaxMapX(void) {return myMaxMapX;}
  /// Get the max map Y coord in mm.
  double   getMaxMapY(void) {return myMaxMapY;}
  /// Get the no of cells in the X dimension.
  int      getXLength(void) {return myXLength;}
  /// Get the no of cells in the Y dimension.
  int      getYLength(void) {return myYLength;}
  /// Get the kernel size.
  int      getKernelSize(void) {return myKSize;}
  /// Get the kernel data
  ArnlFloat**  getKernelData(void) {return myKernel;}
  /// Set the resolution in the X direction.
  void     setRes(double r) {myRes = fabs(r);}
  /// Set the kernel size.
  void     setKernelSize(int ks) {if(ks >= 0) myKSize = ks;}
  /// Set the value of the occupancy at the (x,y) coords.
  bool     setGridVal(double x, double y, ArnlFloat val)
  {
    if(!myGridData)
      return false;
    int xx = (int)((x - myMinMapX) / myRes);
    int yy = (int)((y - myMinMapY) / myRes);
    if(xx < 0 || xx >= myXLength || yy < 0 || yy >= myYLength)
    {
      return false;
    }else
    {
      myGridData[xx][yy] = val;
      return true;
    }
  }
  /// Set the value of the direction at x, y.
  bool     setDirection(double x, double y, AngDis val)
  {
    if(!myDirection)
      return false;
    int xx = (int)((x - myMinMapX) / myRes);
    int yy = (int)((y - myMinMapY) / myRes);
    if(xx < 0 || xx >= myXLength || yy < 0 || yy >= myYLength)
    {
      return false;
    }else
    {
      myDirection[xx][yy] = val;
      return true;
    }
  }
  /// Set the value of the sidedness at x, y.
  bool     setSidedness(double x, double y, AngDis val)
  {
    if(!mySidedness)
      return false;
    int xx = (int)((x - myMinMapX) / myRes);
    int yy = (int)((y - myMinMapY) / myRes);
    if(xx < 0 || xx >= myXLength || yy < 0 || yy >= myYLength)
    {
      return false;
    }else
    {
      mySidedness[xx][yy] = val;
      return true;
    }
  }
  /// Set the value of the resistance at x, y.
  bool     setResistance(double x, double y, short val)
  {
    if(!myResistance)
      return false;
    int xx = (int)((x - myMinMapX) / myRes);
    int yy = (int)((y - myMinMapY) / myRes);
    if(xx < 0 || xx >= myXLength || yy < 0 || yy >= myYLength)
    {
      return false;
    }else
    {
      myResistance[xx][yy] = val;
      return true;
    }
  }
  /// Set the value of the preference at x, y.
  bool     setPreference(double x, double y, short val)
  {
    if(!myPreference)
      return false;
    int xx = (int)((x - myMinMapX) / myRes);
    int yy = (int)((y - myMinMapY) / myRes);
    if(xx < 0 || xx >= myXLength || yy < 0 || yy >= myYLength)
    {
      return false;
    }else
    {
      myPreference[xx][yy] = val;
      return true;
    }
  }
  /// Get the pointer to the XxY array of grid cell data.
  ArnlFloat**  getGridData(void) {return myGridData;}
  /// Get the pointer to the XxY array of one way direction data.
  AngDis**     getDirectionData(void) {return myDirection;}
  /// Get the pointer to the XxY array of sided data.
  AngDis**     getSidednessData(void) {return mySidedness;}
  /// Get the pointer to the XxY array of resistance data.
  short**      getResistanceData(void) {return myResistance;}
  /// Get the pointer to the XxY array of prefererance data.
  short**      getPreferenceData(void) {return myPreference;}
  /// Mark with forbidden areas.
  AREXPORT bool     markForbiddenAreas(ArMapInterface* ariamap);
  
  /// Allocate new grid memory to fit both points and lines.
  AREXPORT bool     allocateGridMemory(ArMapInterface* ariaMap,
				       bool localPlanMap = false);
  /// Allocate new grid memory to take care of new known objects.
  AREXPORT bool     allocateNewMemory(void);
  
  /// Mark with map points.
  AREXPORT bool     markMapPoints(ArMapInterface* ariamap);
  
  /// Mark with map lines.
  AREXPORT bool     markMapLines(ArMapInterface* ariaMap);
  
  /// Mark known objects.
  AREXPORT bool     markKnownObjects(std::list<ArMultiRobotPoseAndRadius> 
				     known);
  /// Unmark the known objects.
  AREXPORT bool     clearKnownObjects(void);
  /// Support function to mark part of sectors.
  bool              markAreaWithAngDis(double fx,
				       double fy,
				       double tx,
				       double ty,
				       double angle,
				       double offset,
				       AngDis**& ptr);
  /// Non linear version of the one above.
  bool              markAreaWithAngDisNonLin(double fx,
					     double fy,
					     double tx,
					     double ty,
					     double angle,
					     double offset,
					     bool rightNotLeft,
					     AngDis**& ptr,
					     double sideOffset,
					     bool nonLinear);
  /// Mark one way areas.
  AREXPORT bool     markOneWayAreas(ArMapInterface* ariaMap);
  /// Mark sided sectors.
  AREXPORT bool     markSidedAreas(ArMapInterface* ariaMap, double sideOffset);
  /// Mark resistance areas.
  AREXPORT bool     markResistanceAreas(ArMapInterface* ariaMap, 
					short resistance, short preference);

  /// Mark (record) reflectors.
  AREXPORT bool     markReflectors(ArMapInterface* ariaMap);
  
  /// Fill the occupancy grid with values using data from the map.
  AREXPORT ArOccGrid* fillOccGridForLocalization(ArMapInterface* ariamap);
  
  /// Fills the map points and forbidden areas.
  AREXPORT ArOccGrid* fillOccGridForPathPlanning(ArMapInterface* ariamap,
						 bool useOneWays = true,
						 short resistanceValue = 1, 
						 short preferenceValue = 1,
						 double sideOffset = 0.25,
						 bool localPlanMap = false);
  /// Check if the resolutions changed, and if so reset the grid.
  AREXPORT bool     checkAndSetResolution(double res, ArMapInterface* ariamap);
  
  /// Set the resolution and fill map with mapdata and forbids, one-ways...
  AREXPORT bool     checkAndSetResolutionForPathPlanning(
	  double r, ArMapInterface* amap, bool useOneWays = true,
	  short resistance = 1, short preference = 1,
	  double sideOffset = 0.25,
	  bool localPlanMap = false);
  /// Check if the map is valid (helper fn to avoid working with dummy maps)
  AREXPORT bool     isMapValid(void);
  /// Finds the closest reflector to a given pose in a given map.
  AREXPORT bool     findClosestReflector(ArPose rPose, ArPose& pose,
				double maxDist, double maxLaserAngle);
  /// Finds the closest reflector by line to a given pose in a given map.
  AREXPORT bool     findClosestReflectorLine(ArPose rPose, ArPose& pose,
				    double maxDist, double maxLaserAngle);
  /// Finds all the centers and normal of reflectors.
  AREXPORT bool     findAllReflectorCentersAndNormals(std::vector<ArPose>& 
						      pList);
  /// Converts the map coords to grid coords.
  AREXPORT ArPose   mapToGridCoords(ArPose mPose);
  /// Converts the grid coords to map coords.
  AREXPORT ArPose   gridToMapCoords(ArPose gPose);
  /// Converts the map X coords to grid X coords.
  AREXPORT int      mapToGridX(double mX);
  /// Converts the map Y coords to grid Y coords.
  AREXPORT int      mapToGridY(double mY);
  /// Converts the grid X coords to map X coords.
  AREXPORT double   gridToMapX(int gX);
  /// Converts the grid Y coords to map Y coords.
  AREXPORT double   gridToMapY(int gY);
  /// checks if map is empty or not.
  AREXPORT bool     mapLocalizable()
  {
    return myMapLocalizable;
  }

private:
  int      myXLength;
  int      myYLength;
  double   myRes;
  double   myMinMapX;
  double   myMinMapY;
  double   myMaxMapX;
  double   myMaxMapY;
  ArnlFloat** myGridData;  // Given map date.
  short**  myNewData;   // New known objects.
  short*   myZeroData;  // Zero line.
  AngDis** myDirection; // One way data.
  AngDis** mySidedness; // Sidedness data.
  ArnlFloat** myKernel;    // Smoothing window.
  short**  myResistance;// My resistance data.
  short**  myPreference;// My resistance data.
  int      myKSize;
  std::list<ArMapObject*> myReflectors;
  int      myKnownMaxI;
  int      myKnownMinI;
  int      myMaxPreference;
  int      myMaxResistance;
  bool     myMapLocalizable;
};

#endif // AROCCGRID.H
