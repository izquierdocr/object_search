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
/* ****************************************************************************
 * 
 * File: ArLocalizationTask.h
 * 
 * Function: Header file for the localizationtask.cpp file.
 *
 * Created:  George V. Paul. gvp@activmedia.com. January 10 2003.
 *
 *****************************************************************************/
#ifndef ARLOCALIZATIONTASK_H
#define ARLOCALIZATIONTASK_H

#include "Aria.h"
#include "ArNetworking.h"
#include "ArRobotPoseProb.h"
#include "ArRobotPoseSamples.h"
#include "ArOccGrid.h"
#include "ArMatrix.h"
#include "arnlInternal.h"
#include "ArBaseLocalizationTask.h"
#include "ArRobotAndLaser.h"

#include <set>
#include <vector>
#include <algorithm>


class ArLocalizationTask;

#ifndef SWIG
/*
 * The kalman filter class which takes odometry and other data to compute
 * the robot state.
 *
 * @internal
 * This class is used internally by ArLocalizationTask
 */
class ArKalmanFilterReflector
{
  public:
  /// Constructor.
  ArKalmanFilterReflector(int stateSize, ArMatrix x0, ArMatrix p0, 
			  ArLocalizationTask* lightPtr) :
  myXSize(stateSize), myX(x0), myP(p0), myLaserLocaPtr(lightPtr)
  { 
    myMutex.setLogName("ArKalmanFilterReflector::myMutex");
  }
  /// Actual function that does the filter step.
  bool runFilterStep(
	  const std::vector<ArPose>& global,
	  const std::vector<ArPose>& local,
	  ArPose& pose,
	  ArPose& encoderPose,
	  ArPose& deltaPose,
	  ArPose& llp,
	  ArPose& mcp,
	  ArPose& vel,
	  double dt,
	  bool skipMeasureUpdate,
	  ArPose& meanPose,
	  ArPose& stdDev,
	  ArPose& maxInnov,
	  double& refScore,
	  bool useAllK);
  /// Get size of state vector.
  int  getStateSize(void) 
  { 
    myMutex.lock();
    int p = myXSize; 
    myMutex.unlock();
    return p;
  }
  /// Get P
  ArMatrix getP(void) 
  { 
    myMutex.lock();
    ArMatrix p = myP; 
    myMutex.unlock();
    return p;
  }
  /// Get P
  ArMatrix getR(void) 
  { 
    myMutex.lock();
    ArMatrix p = myR; 
    myMutex.unlock();
    return p;
  }
  /// Get X
  ArMatrix getX(void) 
  { 
    myMutex.lock();
    ArMatrix p = myX; 
    myMutex.unlock();
    return p;
  }
  /// Get encoder pose at last compute.
  ArPose getLastLocaEncoderPose(void) 
  { 
    myMutex.lock();
    ArPose p = myLastLocaEncoderPose; 
    myMutex.unlock();
    return p;
  }
  /// Get mean at last loca.
  ArPose getLastLocaMean(void) 
  {
    myMutex.lock();
    ArPose p = myLastLocaMean; 
    myMutex.unlock();
    return p;
  }
  /// Get the P at last loca.
  ArMatrix getLastLocaP(void) 
  {
    myMutex.lock();
    ArMatrix p = myLastLocaP; 
    myMutex.unlock();
    return p;
  }

protected:
  int myXSize; // Kalman base state size.
  ArMatrix myX; // Kalman state.
  ArMatrix myP; // Kalman plant covariance.
  ArMatrix myR; // Kalman sensor covariance.
  ArMutex myMutex;
  ArLocalizationTask* myLaserLocaPtr;
  ArPose myLastLocaEncoderPose;
  ArPose myLastLocaMean;
  ArMatrix myLastLocaP;
};
#endif

/*!
  @class ArLocalizationTask
  @brief Task that performs continuous localization of the robot with a laser range sensor in a seperate asynchronous thread.

  The localization software can be used to localize a robot in a given map using the laser rangefinder. The system is meant to be used along with ARIA.

  The localization system uses a Monte Carlo Localization Algorithm to accurately localize the robot in the given map using its laser data. The localization task is meant to be initialized and run in a separate thread in ARIA. It can be used with a real robot or with a simulator.

  In order to get the localization task thread going, the ArLocalizationTask class needs an instantiated ArRobot, ArRangeDevice and a map file of the robot's environment. The output of the localization will be reflected directly in the pose of the ArRobot. (unless explicitly set to not do so)

  An initial localization must be performed using one of the localization
  functions: localizeRobotAtHomeBlocking(), setRobotPose(), forceUpdatePose(), or
  localizeRobotInMapInit().  These will set or choose a starting position for
  the robot, do one or more attempted localizations, and may also examine the
  positions of mapped obstacles vs. "ray traced" laser data to check that
  laser data is not impossible from hypothetical positions.

  ArLocalizationTask automatically creates and runs its background thread when constructed. 

  This thread continuously relocalizes the robot as it moves, or if a new robot
  pose is set with setRobotPose() or forceUpdatePose(), or if triggered by an
  optional time interval (see configuration parameters).

  As an additional optional features, the basic MCL system has also been augmented with a Kalman filter which will fuse the wheel encoder data with the reflections from known laser reflecting landmarks (these are constructed with special reflective plastic manufactured by laser) in the map. To use this functionality you will need a map with reflector positions in it, and will have to configure the laser to return reflectance information. ARNL will incrementally incorporate the reflections from the landmarks using a EKF formulation. See the main ARNL overview page for details on this extra feature, including how to enable reflectance data in the laser.

 ArLocalizationTask has many parameters that affect localization, enable
  optional features, etc.  The map to use is also a parameter. These parameters
  are normally accessed via the global program ArConfig object (see
  Aria::getConfig()).
*/
class ArLocalizationTask: public ArBaseLocalizationTask
{
  friend class ArKalmanFilterReflector;

  public:

  /// Base constructor with all the necessary inputs.
  AREXPORT ArLocalizationTask(ArRobot* robot, ArRangeDevice* laser, 
			      char* mapName);
  /// Base constructor with all the necessary inputs.
  AREXPORT ArLocalizationTask(ArRobot* robot, 
                              ArRangeDevice* laser, 
                              ArMapInterface* ariaMap,
                              bool noReflectors = false);
  /// Base destructor.
  AREXPORT virtual ~ArLocalizationTask(void);

  /// Perform an initial localization in the map
  AREXPORT bool   localizeRobotInMapInit(ArPose given, int numSamples,
					 double stdX, double stdY, double stdT,
					 double thresFactor, bool warn=true,
					 bool setInitializedToFalse = true,
					 bool rayTrace=true);
  /// Used by localization task thread to update localization as robot moves
  AREXPORT bool   localizeRobotInMapMoved(int numSamples,
					  double distFactor, 
					  double angFactor,
					  double thresFactor);
  /** 
      @copydoc localizeRobotAtHomeBlocking()
  */
  AREXPORT bool   localizeRobotAtHomeBlocking(double distSpread,
					      double angleSpread, 
					      double probThreshold) 
  {
    return localizeRobotAtHomeBlocking(distSpread, distSpread, 
				       angleSpread, probThreshold);
  }
  /// @copydoc localizeRobotAtHomeBlocking()
  AREXPORT bool   localizeRobotAtHomeBlocking(double spreadX, double spreadY,
      double angleSpread, double probThreshold);
  /// @copydoc localizeRobotAtHomeBlocking()
  virtual bool localizeRobotAtHomeBlocking(double distSpread, 
						    double angleSpread)
  {
    return localizeRobotAtHomeBlocking(distSpread, distSpread, 
				       angleSpread, getUsingPassThreshold());
  }
  /**  
   *  @brief Try to localize at each home point at the map, 
       and set the robot pose to the point with best score
   *
   *   This function attempts to localize the robot in the most
   *   likely home position: first it checks the robot's current pose
   *   and then checks all of the home positions in the map.  The position
   *   with the highest score is then used.
   *   Call this function after the localization thread has
   *   been initialized but needs to be reset, or as an initial localization
   *   (i.e. at program start up).  This function blocks
   *   while the localization takes place, and returns after it either succeeds
   *   (Either localization at a home position succeeds, or all fail.)
   *
   *   This method is "blocking", in that it returns after all localization
   *   attempts have been performed (and either succeeds or fails). This could
   *   be any amount of time, depending on the map and configuration parameters.
   *
   *   @note Localization will fail if no sensor data has yet been obtained
   *         when this function is called (e.g. if called before or immediately
   *         after connecting to the robot and/or laser sensor).
   *
   *  @sa ArBaseLocalizationTask::setRobotPose()
   *  
   */
      
  virtual bool localizeRobotAtHomeBlocking() 
  {
    return localizeRobotAtHomeBlocking(getStdX(), getStdY(), getStdTh(),
				       getUsingPassThreshold());
  }
  /** @brief Request that the task later localize the robot at a map home \
      position, but then return immediately */
  AREXPORT bool   localizeRobotAtHomeNonBlocking(void);
  /// Gets the pose that robot was localized to by localizeRobotAtHomeBlocking()
  AREXPORT virtual ArPose getRobotHome(void);
  /// Sets parameters to be used when forceUpdatePose() is called
  AREXPORT void   setForceUpdateParams(int numSamples, 
				       double xStd, double yStd, double tStd);
  /// Reset the robot position to a new known pose and relocalize at that position.
  AREXPORT void   forceUpdatePose(ArPose forcePose, bool rayTrace=false);
  /// Adds a callback which will be called when localization fails.
  AREXPORT void   addFailedLocalizationCB(ArFunctor1<int>* functor);
  /// Removes a localization callback
  AREXPORT void   remFailedLocalizationCB(ArFunctor1<int>* functor);
  /** (Re)set the occupancy grid with a new map and resolution
      @internal Normally resolution is determed using ArConfig parameters.
  */
  bool   setGridResolution(double res, ArMapInterface* ariaMap)
  {
    if(myOccGridPtr)
      return myOccGridPtr->checkAndSetResolution(res, ariaMap);
    else
      return false;
  }
  
  /// Sets tracking failed Callback. 
  void   setFailedCallBack(ArFunctor1<int>* fcb) 
  {
    myFailedCB=fcb;
  }
  /** @name Modifiers for configuration values.
   * Normally these values are automatically set via ArConfig
   * (e.g. by loading them from a file or other source) in the
   * "Localization" section, and these methods would
   * only be used internally by ARNL.
   * However, they can be used if you are not using ArConfig or wish to 
   * override a setting.
   */
  //@{
  
  /// Sets the motion trigger value for distance.
  void   setTriggerDelR(double tr)
  {
    myMutex.lock();
    myTriggerDelR = tr;
    myMutex.unlock();
  }
  /// Sets the motion trigger value for angle.
  void   setTriggerDelT(double tt)
  {
    myMutex.lock();
    myTriggerDelT = tt;
    myMutex.unlock();
  }
  /// Sets the flag to trigger on idle.
  void   setTriggerTimeFlag(bool tt)
  {
    myMutex.lock();
    myTriggerTimeFlag = tt;
    myMutex.unlock();
  }
  /// Sets the idle trigger time in millisecs.
  void   setTriggerTime(double tt)
  {
    myMutex.lock();
    myTriggerDelT = tt;
    myMutex.unlock();
  }
  /// Sets the X range when localization triggers due to idle time.
  void   setTriggerTimeX(double tt)
  {
    myMutex.lock();
    myTriggerTimeX= tt;
    myMutex.unlock();
  }
  /// Sets the Y range when localization triggers due to idle time.
  void   setTriggerTimeY(double tt)
  {
    myMutex.lock();
    myTriggerTimeY= tt;
    myMutex.unlock();
  }
  /// Sets the Theta range when localization triggers due to idle time.
  void   setTriggerTimeTh(double tt)
  {
    myMutex.lock();
    myTriggerTimeTh= tt;
    myMutex.unlock();
  }
  /// Set the number of samples to use in the MCL.
  void   setNumSamples(int n)
  {
    if(n < 1)
      return;
    myMutex.lock();
    myNumSamples = n;
    myMutex.unlock();
  }
  /// Set the number of samples to use during initialization.
  void   setNumSamplesAtInit(int n)
  {
    if(n < 1)
      return;
    myMutex.lock();
    myNumSamplesAtInit = (n == 0) ? myNumSamples : n;
    myMutex.unlock();
  }
  /// Set the flag which decides on ray tracing for localize at init.
  void   setRayTraceAtInit(bool f)
  {
    myMutex.lock();
    myRayTraceAtInit = f;
    myMutex.unlock();
  }
  /// Set the variable number of samples (usually adjusted automatically).
  void   setCurrentNumSamples(int n)
  {
    if(n < 1)
      return;
    myMutex.lock();
    myCurrentNumSamples = n;
    myMutex.unlock();
  }
  /// Sets the pass threshold for fraction of laser points matched to env.
  void   setPassThreshold(double f)
  {
    myMutex.lock();
    myPassThreshold = f;
    myMutex.unlock();
  }
  /// Sets the lost threshold.
  void   setLostThreshold(double f)
  {
    myMutex.lock();
    myLostThreshold = f;
    myMutex.unlock();
  }
  /// Sets the sensor belief.
  void   setSensorBelief(double sensorBelief)
  {
    if(!mySystemErrorPtr)
      return;

    myMutex.lock();
    mySystemErrorPtr->setSensorBelief(sensorBelief);
    myMutex.unlock();
  }
  /// Sets the current pose.
  void   setCurrentLocaPose(double x, double y, double th)
  {
    myMutex.lock();
    myCurrentLocaPose.setPose(x, y, th);
    myMutex.unlock();
  }
  /// Sets the current pose.
  void   setCurrentLocaPose(ArPose p)
  {
    myMutex.lock();
    myCurrentLocaPose = p;
    myMutex.unlock();
  }
  /// Sets the verbose flag value. (for debugging)
  void   setVerboseFlag(bool a)
  {
    myMutex.lock();
    myVerboseFlag = a;
    myMutex.unlock();
  }
  /// Sets the laser increments.
  void   setAngleIncrement(double f)
  { 
    myMutex.lock();
    myAngleIncrement = f > 0 ? f : 1; 
    myMutex.unlock();
  }
  // Sets the occupancy threshold at which samples are discarded
  void   setKillThreshold(double f)
  { 
    myMutex.lock();
    myKillThreshold = f;
    myMutex.unlock();
  }
  /// Sets the recoverOnFailed flag.
  void   setRecoverOnFailedFlag(bool f)
  {
    myMutex.lock();
    myRecoverOnFailFlag = f;
    myMutex.unlock();
  }
  /// Sets the idle flag.
  AREXPORT void   setIdleFlag(bool f);
  /// Sets the map reloading flag.
  void   setReloadingMapFlag(bool f)
  {
    myMutex.lock();
    myReloadingMapFlag = f;
    myMutex.unlock();
  }
  /// Sets the flag to fuse all sensors.
  void   setEnableReflectorLocalizationFlag(bool f)
  {
    myMutex.lock();
    myEnableReflectorLocalizationFlag = f;
    myMutex.unlock();
  }
  /// Sets the variance in the XY for the reflector centers.
  void   setReflectorVar(double f)
  {
    myMutex.lock();
    myReflectorVar = ArMath::fabs(f);
    myMutex.unlock();
  }
  /// Sets the radius to search for a reflector to match a given reflection.
  void   setReflectorMatchDist(double f)
  {
    myMutex.lock();
    myReflectorMatchDist = ArMath::fabs(f);
    myMutex.unlock();
  }
  /// Sets the angle to search for a reflector to match a given reflection.
  void   setReflectorMatchAngle(double f)
  {
    myMutex.lock();
    myReflectorMatchAngle = ArMath::fabs(f);
    myMutex.unlock();
  }
  /// Sets the maximum distance the reflector can be seen.
  void   setReflectorMaxRange(double f)
  {
    myMutex.lock();
    myReflectorMaxRange = ArMath::fabs(f);
    myMutex.unlock();
  }
  /// Sets the maximum incident angle the reflector can be seen.
  void   setReflectorMaxAngle(double f)
  {
    myMutex.lock();
    myReflectorMaxAngle = ArMath::fabs(f);
    myMutex.unlock();
  }
  /// Sets the reflector size.
  void   setReflectorSize(double f)
  {
    myMutex.lock();
    myReflectorSize = ArMath::fabs(f);
    myMutex.unlock();
  }
  /// Sets the reflectance threshold.
  void   setReflectanceThreshold(int f)
  {
    myMutex.lock();
    myReflectanceThreshold = f;
    myMutex.unlock();
  }
  /// Sets the flag which decides to bypasses MCL and use reflectors.
  void   setBypassMCLFlag(bool f)
  {
    myMutex.lock();
    myBypassMCLFlag = f;
    myMutex.unlock();
  }

  //@}
  
  /** @name Accessors for current configuration values.
   *
   *  These values are normally set via ArConfig (the "Localization"
   *  section) by loading a file or other means, or by calling
   *  the modifier functions above.
   *
   */

  //@{

  /// Get parameters used when relocalization in manually performed using forceUpdatePose()
  AREXPORT void   getForceUpdateParams(ArPose& forcePose, int& numSamples, 
				       double& xStd, double& yStd, 
				       double& tStd);
  /// Gets the verbose flag. (for debugging only)
  bool   getVerboseFlag(void)
  {
    myMutex.lock();
    bool ret = myVerboseFlag;
    myMutex.unlock();
    return ret;
  }
  /// Gets the initialized flag indicating if localization thread is on. 
  bool   getInitializedFlag(void)
  {
    myMutex.lock();
    bool ret = myInitializedFlag;
    myMutex.unlock();
    return ret;
  }
  /// Get the maximum number of samples used in the MCL.
  int    getNumSamples(void)
  {
    myMutex.lock();
    int p = myNumSamples;
    myMutex.unlock();
    return p;
  }
  /// Get the number of samples used in the MCL during initialization.
  int    getNumSamplesAtInit(void)
  {
    myMutex.lock();
    int p = (myNumSamplesAtInit == 0) ? myNumSamples : myNumSamplesAtInit;
    myMutex.unlock();
    return p;
  }
  /// Get the flag which decides on ray tracing for localize at init.
  bool   getRayTraceAtInit(void)
  {
    myMutex.lock();
    bool p = myRayTraceAtInit;
    myMutex.unlock();
    return p;
  }
  /// Get the variable number of samples if adjusted during move.
  int    getCurrentNumSamples(void)
  {
    myMutex.lock();
    int p = myCurrentNumSamples;
    if(!myAdjustNumSamplesFlag)
      p = myNumSamples;
    myMutex.unlock();
    return p;
  }
  /// Get the current computed best robot pose.
  AREXPORT ArPose getRobotMaxProbPose(void);
  /// Gets the min distance to localize.
  double getTriggerDelR(void)
  {
    myMutex.lock();
    double p = myTriggerDelR;
    myMutex.unlock();
    return p;
  }
  /// Gets the min angle to localize.
  AREXPORT double getTriggerDelT(void)
  {
    myMutex.lock();
    double p = myTriggerDelT;
    myMutex.unlock();
    return p;
  }
  /// Gets the flag indicating if localization should trigger on idle.
  AREXPORT bool   getTriggerTimeFlag(void)
  {
    myMutex.lock();
    bool p = myTriggerTimeFlag;
    myMutex.unlock();
    return p;
  }
  /// Gets the min time in millisecs to be idle.
  AREXPORT double getTriggerTime(void)
  {
    myMutex.lock();
    double p = myTriggerTime;
    myMutex.unlock();
    return p;
  }
  /// Gets the X range of samples when localization triggers on idle.
  AREXPORT double getTriggerTimeX(void)
  {
    myMutex.lock();
    double p = myTriggerTimeX;
    myMutex.unlock();
    return p;
  }
  /// Gets the Y range of samples when localization triggers on idle.
  AREXPORT double getTriggerTimeY(void)
  {
    myMutex.lock();
    double p = myTriggerTimeY;
    myMutex.unlock();
    return p;
  }
  /// Gets the Theta range of samples when localization triggers on idle.
  AREXPORT double getTriggerTimeTh(void)
  {
    myMutex.lock();
    double p = myTriggerTimeTh;
    myMutex.unlock();
    return p;
  }
  /// Gets the Pass threshold for localization success.
  AREXPORT double getPassThreshold(void)
  {
    myMutex.lock();
    double p = myPassThreshold;
    myMutex.unlock();
    return p;
  }
  /// Gets the Lost threshold.
  AREXPORT double getLostThreshold(void)
  {
    myMutex.lock();
    double p = myLostThreshold;
    myMutex.unlock();
    return p;
  }
  /// Gets the Pass threshold for localization success being used right now
  AREXPORT double getUsingPassThreshold(void)
  {
    myMutex.lock();
    double p;
    if (myUseTempPassThreshold)
      p = myTempPassThreshold;
    else
      p = myPassThreshold;
    myMutex.unlock();
    return p;
  }
  /// Sets the Pass threshold to use until it is reset to config 
  AREXPORT void   setTempPassThreshold(double passThreshold)
  {
    myMutex.lock();
    myUseTempPassThreshold = true;
    myTempPassThreshold = passThreshold;
    myMutex.unlock();
  }
  /// Gets the temporary pass threshold to use until it is reset to config 
  AREXPORT double getTempPassThreshold(void)
  {
    myMutex.lock();
    double p;
    if (myUseTempPassThreshold)
      p = myTempPassThreshold;
    else
      p = -1;
    myMutex.unlock();
    return p;
  }

  /// Resets the Pass threshold to config
  AREXPORT void   clearTempPassThreshold(void)
  {
    myMutex.lock();
    myUseTempPassThreshold = false;
    myMutex.unlock();
  }
  /// Gets the higher of MCL and Reflector based localization.
  AREXPORT virtual double getLocalizationScore(void);
  /// Gets the localization threshold.
  AREXPORT virtual double getLocalizationThreshold(void);
  /// Gets the localization score based on Monte Carlo only.
  AREXPORT double getMCLocalizationScore(void);
  /// Gets the localization score based on reflectors only.
  AREXPORT double getRefLocalizationScore(void);
  /// Gets the sensor belief.
  AREXPORT double getSensorBelief(void)
  {
    if(!mySystemErrorPtr)
      return -1;

    myMutex.lock();
    double ret = mySystemErrorPtr->getSensorBelief();
    myMutex.unlock();

    return ret;
  }
  /// Gets the last successful MCLocalized pose.
  AREXPORT ArPose getCurrentLocaPose(void)
  {

    myMutex.lock();
    ArPose p = myCurrentLocaPose;
    myMutex.unlock();
    return p;
  }
  // Gets the standard deviation of the initialization X coords.
  AREXPORT double getStdX(void) const 
  { 
    return myInitStdX; 
  }
  // Gets the standard deviation of the initialization Y coords.
  AREXPORT double getStdY(void) const 
  { 
    return myInitStdY; 
  }
  // Gets the standard deviation of the initialization Th coords.
  AREXPORT double getStdTh(void) const 
  { 
    return myInitStdTh; 
  }
  // Gets the motion error mm per mm error coefficient.
  AREXPORT double getErrorMmPerMm(void) const 
  { 
    return myErrorMmPerMm; 
  }
  // Gets the motion error deg per deg error coefficient.
  AREXPORT double getErrorDegPerDeg(void) const 
  { 
    return myErrorDegPerDeg; 
  }
  // Gets the motion error deg per mm error coefficient.
  AREXPORT double getErrorDegPerMm(void) const 
  { 
    return myErrorDegPerMm; 
  }
  // Gets the sensor belief.
  AREXPORT double getSensorBelief(void) const 
  { 
    return mySensorBelief; 
  }
  // Gets the peak factor for multiple hypothesis.
  AREXPORT double getPeakFactor(void) const 
  { 
    return myPeakFactor; 
  }
  // Gets the obstacle threshold to be considered occupied grid cell.
  AREXPORT double getOccThreshold(void) const 
  { 
    return myOccThreshold; 
  }
  // Gets the occupancy grid resolution.
  AREXPORT double getGridRes(void) const 
  { 
    return myGridRes; 
  }
  // Gets the map name.
  AREXPORT char*  getMapName(void) 
  { 
    return myMapName; 
  }
  // Gets the peturbation range in X coords in mm.
  AREXPORT double getPeturbRangeX(void) const 
  { 
    return myPeturbX; 
  }
  // Gets the peturbation range in Y coords in mm.
  AREXPORT double getPeturbRangeY(void) const 
  { 
    return myPeturbY; 
  }
  // Gets the peturbation range in Theta coords in degs.
  AREXPORT double getPeturbRangeTh(void) const 
  { 
    return myPeturbTh; 
  }
  // Gets the failed range in X coords in mm.
  AREXPORT double getFailedRangeX(void) const 
  { 
    return myFailedX; 
  }
  // Gets the failed range in Y coords in mm.
  AREXPORT double getFailedRangeY(void) const 
  { 
    return myFailedY; 
  }
  // Gets the failed range in Theta coords in degs.
  AREXPORT double getFailedRangeTh(void) const 
  { 
    return myFailedTh; 
  }
  // Gets the peak pose X standard deviation in mm.
  AREXPORT double getPeakStdX(void) const 
  { 
    return myPeakStdX; 
  }
  // Gets the peak pose Y standard deviation in mm.
  AREXPORT double getPeakStdY(void) const 
  { 
    return myPeakStdY; 
  }
  // Gets the peak pose Th standard deviation in degs
  AREXPORT double getPeakStdTh(void) const 
  { 
    return myPeakStdTh; 
  }
  // Gets the increment by which laser readings are skipped.
  AREXPORT double getAngleIncrement(void) const 
  { 
    return myAngleIncrement; 
  }
  // Gets the occupancy threshold at which samples are discarded
  AREXPORT double getKillThreshold(void) const 
  { 
    return myKillThreshold; 
  }
  // Gets the Aria map.
  AREXPORT ArMapInterface* getAriaMap(void) 
  { 
    return myAriaMapPtr; 
  }
  
  /// Scan Buffer size. (internal laser scan buffer)
  AREXPORT int    getBufferSize(void) 
  {
    return myXYBuffer.size();
  }
  /// Scan Buffer XY.
  AREXPORT std::vector<ArPose> getXYBuffer(void) 
  {
    return myXYBuffer;
  }
  /// Scan Buffer pose taken. (internal laser scan buffer)
  AREXPORT ArPose getBufferPose(void) 
  {
    return myBufferPose;
  }
  /// Returns pointer to the occgrid.
  AREXPORT ArOccGrid* getOccGridPtr(void) 
  {
    return myOccGridPtr;
  }
  /// Gets the pose samples for client.
  AREXPORT virtual std::list<ArPose> getCurrentSamplePoses(void);
  /// Gets the recoverOnFailed flag.
  AREXPORT bool   getRecoverOnFailedFlag(void)
  {
    myMutex.lock();
    bool ret = myRecoverOnFailFlag;
    myMutex.unlock();
    return ret;
  }
  /// Gets the flag which allows poses overlaping occupied points on map.
  AREXPORT bool   getIgnoreIllegalPoseFlag(void)
  {
    myMutex.lock();
    bool ret = myIgnoreIllegalPoseFlag;
    myMutex.unlock();
    return ret;
  }
  /// Gets the flag which allows for changing numSamples with loca score.
  AREXPORT bool   getAdjustNumSamplesFlag(void)
  {
    myMutex.lock();
    bool ret = myAdjustNumSamplesFlag;
    myMutex.unlock();
    return ret;
  }
  /// Gets the minimum no of samples the localization will drop to.
  AREXPORT int    getMinNumSamples(void)
  {
    myMutex.lock();
    int ret = myMinNumSamples;
    myMutex.unlock();
    return ret;
  }
  /// Gets the rotation factor for adjusting no of samples with score.
  AREXPORT double getNumSamplesAngleFactor(void)
  {
    myMutex.lock();
    double ret = myNumSamplesAngleFactor;
    myMutex.unlock();
    return ret;
  }
  /// Gets the value of the sensor set flag which indicates new cycle.
  AREXPORT bool   getSensorSetFlag(void)
  {
    bool p;
    myMutex.lock();
    p = mySensorSetFlag;
    myMutex.unlock();
    return p;
  }
  /// Gets the flag to fuse all sensors.
  AREXPORT bool   getEnableReflectorLocalizationFlag(void)
  {
    double p;
    myMutex.lock();
    p = myEnableReflectorLocalizationFlag;
    myMutex.unlock();
    return p;
  }
  /// Gets the variance in the XY for the reflector centers.
  AREXPORT double getReflectorVar(void)
  {
    double p;
    myMutex.lock();
    p = myReflectorVar;
    myMutex.unlock();
    return p;
  }
  /// Gets the radius to search for a reflector to match a given reflection.
  AREXPORT double getReflectorMatchDist(void)
  {
    double p;
    myMutex.lock();
    p = myReflectorMatchDist;
    myMutex.unlock();
    return p;
  }
  /// Sets the angle to search for a reflector to match a given reflection.
  AREXPORT double getReflectorMatchAngle(void)
  {
    double p;
    myMutex.lock();
    p = myReflectorMatchAngle;
    myMutex.unlock();
    return p;
  }
  /// Gets the maximum distance the reflector can be seen.
  AREXPORT double getReflectorMaxRange(void)
  {
    double p;
    myMutex.lock();
    p = myReflectorMaxRange;
    myMutex.unlock();
    return p;
  }
  /// Gets the maximum incident angle the reflector can be seen.
  AREXPORT double getReflectorMaxAngle(void)
  {
    double p;
    myMutex.lock();
    p = myReflectorMaxAngle;
    myMutex.unlock();
    return p;
  }
  /// Gets the reflector size param.
  AREXPORT double getReflectorSize(void)
  {
    double p;
    myMutex.lock();
    p = myReflectorSize;
    myMutex.unlock();
    return p;
  }
  /// Gets the reflectance threshold.
  AREXPORT int    getReflectanceThreshold(void)
  {
    int p;
    myMutex.lock();
    p = myReflectanceThreshold;
    myMutex.unlock();
    return p;
  }
  /// Gets the flag which decides to bypasses MCL and use reflectors.
  AREXPORT bool   getBypassMCLFlag(void)
  {
    bool p;
    myMutex.lock();
    p = myBypassMCLFlag;
    myMutex.unlock();
    return p;
  }
  /// Gets the triangulation dist limit.
  AREXPORT double getReflectorTriDistLimit(void)
  {
    double p;
    myMutex.lock();
    p = myReflectorTriDistLimit;
    myMutex.unlock();
    return p;
  }
  /// Gets the triangulation angle limit.
  AREXPORT double getReflectorTriAngLimit(void)
  {
    double p;
    myMutex.lock();
    p = myReflectorTriAngLimit;
    myMutex.unlock();
    return p;
  }
  /// Gets the bad reflector factor.
  AREXPORT double getBadReflectorFactor(void)
  {
    double p;
    myMutex.lock();
    p = myBadReflectorFactor;
    myMutex.unlock();
    return p;
  }
  /// Gets the reflector range factor.
  AREXPORT double getReflectorRangeFactor(void)
  {
    double p;
    myMutex.lock();
    p = myReflectorRangeFactor;
    myMutex.unlock();
    return p;
  }
  /// Gets the bad reflector factor.
  AREXPORT ArMatrix getQParams(void);

  //@}

  /// Gets the state of localization.
  AREXPORT virtual LocalizationState getState(void)
  {
    myMutex.lock();
    LocalizationState p = myState;
    myMutex.unlock();
    return p;
  }

  /// Gets the idle flag.
  AREXPORT virtual bool   getIdleFlag(void)
  {
    myMutex.lock();
    bool ret = myIdleFlag;
    myMutex.unlock();
    return ret;
  }
  /// Gets the map reloading flag.
  AREXPORT bool   getReloadingMapFlag(void)
  {
    myMutex.lock();
    bool ret = myReloadingMapFlag;
    myMutex.unlock();
    return ret;
  }

  /// Read the Map data from a map file.
  AREXPORT ArMapInterface* readMapFromFile(char* mapName);
  /// Read the Map data from ArMap
  AREXPORT ArMapInterface* readAriaMap(ArMapInterface* ariaMap);
  /// Load parameters from the given filename into ArConfig
  AREXPORT bool   loadParamFile(const char *file);
  /// Saves all parameter values from ArConfig to the given file
  AREXPORT bool   saveParams(char* filename);

  /** @name Functions used internally by ARNL
   *  @internal
   */
  //@{

  /// Fill prob distribution histogram for debugging.
  AREXPORT bool   fillHistogram(double*& hist, double*& cumSum, 
				int& numSamples);

  /// Convert the laser data to x and y coord array.
  AREXPORT bool   scanToGlobalCoords(ArPose robPose, 
				     std::vector<ArPose>& xyLrf);

  /// Gets the last loca time to now.
  AREXPORT ArTime getLastLocaTime(void)
  {
    myMutex.lock();
    ArTime tim = myLastLocaTime;
    myMutex.unlock();
    return tim;
  }

  /// Set all the parameters for localization.
  AREXPORT bool   setLocaParams(double xStd, double yStd, double tStd,
				double kMmPerMm, double kDegPerDeg, 
				double kDegPerMm,
				double sensorBelief);
  /// Sets the flag which allows poses overlaping occupied points on map.
  AREXPORT void   setIgnoreIllegalPoseFlag(bool f)
  {
    myMutex.lock();
    myIgnoreIllegalPoseFlag = f;
    myMutex.unlock();
  }
  /// Sets the flag which changes numSamples with the localization score.
  AREXPORT void   setAdjustNumSamplesFlag(bool f)
  {
    myMutex.lock();
    myAdjustNumSamplesFlag = f;
    myMutex.unlock();
  }
  /// Sets the minimum no of samples the localization will drop to.
  AREXPORT void   setMinNumSamples(int n)
  {
    myMutex.lock();
    myMinNumSamples = n;
    myMutex.unlock();
  }
  /// Sets the rotation factor for adjusting no of samples with score.
  AREXPORT void   setNumSamplesAngleFactor(double f)
  {
    myMutex.lock();
    myNumSamplesAngleFactor = f;
    myMutex.unlock();
  }
  /// Sets the last loca time to now.
  AREXPORT void   setLastLocaTimeToNow(void)
  {
    myMutex.lock();
    myLastLocaTime.setToNow();
    myMutex.unlock();
  }
  /// Sets the value of the sensor set flag which indicates new cycle.
  AREXPORT void   setSensorSetFlag(bool p)
  {
    myMutex.lock();
    mySensorSetFlag = p;
    myMutex.unlock();
  }

  /// The compute the mean and var as soon as MCL finds a pose.
  AREXPORT bool   computeLastLocaMeanVar(ArPose& mean, ArMatrix& Var);
  /// Finds the mean and var of MCL after moving to present pose.
  AREXPORT bool   findMCLMeanVar(ArPose& mean, ArMatrix& Var);
  /// The actual mean var calculator for the virtual in the base class.
  AREXPORT virtual bool findLocalizationMeanVar(ArPose& mean, ArMatrix& Var);
  /// Sets the flag deciding whether to reflect localized pose onto the robot.
  AREXPORT virtual void setCorrectRobotFlag(bool f)
  {
    myMutex.lock();
    myCorrectRobotFlag = f;
    myMutex.unlock();
  }
  /// Used to set the robot pose usually at the start of localization. 
  /// This one with a spread around the set pose.
  AREXPORT virtual void setRobotPose(ArPose pose,
				     ArPose spread = ArPose(0, 0, 0), 
				     int nSam = 0);
  /// Gets the lost flag.
  AREXPORT virtual bool getRobotIsLostFlag(void) 
  {
    return !getInitializedFlag();
  }
  /// Set the localization idle
  AREXPORT virtual void setLocalizationIdle(bool f)
  {
    setIdleFlag(f);
    return;
  }
  //@}

  /** @name ArNetworking callback methods  
   * (used by server classes in ArServerClasses.cpp)
   */
  //@{
  
  /// Draws range data used for localization.
  AREXPORT void   drawRangePoints(ArServerClient* client, 
				  ArNetPacket* packet);
  /// Draw rays from the robot to the reflector.
  AREXPORT void   drawReflectorRays(ArServerClient* client, 
				    ArNetPacket* packet);
  /// Draws debugging objects if filled.
  AREXPORT void   drawSamplePoses(ArServerClient* client, 
				  ArNetPacket* packet);
  /// Draws the sample bounds.
  AREXPORT void   drawSampleBounds(ArServerClient* client, 
				   ArNetPacket* packet);
  /// Draws the sample bounds.
  AREXPORT void   drawKalmanVariance(ArServerClient* client, 
				     ArNetPacket* packet);
  /// Draws the sample bounds.
  AREXPORT void   drawMCLVariance(ArServerClient* client, 
				  ArNetPacket* packet);
private:
  std::vector<ArPose>  makeMeanVarPoints(ArPose& mean, ArMatrix& Var);
public:
/*
  /// Gets the position the robot was at at the given timestamp
  AREXPORT virtual int getPoseInterpPosition(ArTime timeStamp, 
					     ArPose *position)
  { return myInterpolation.getPose(timeStamp, position); }
*/
  /// Sets the callback function for localization to know about other robots.
  AREXPORT void   setMultiRobotCallback(
	  ArRetFunctor< std::list<ArMultiRobotPoseAndRadius> >* func) 
  { 
    myMultiRobotFunction = func;
  }

  //@}

  /// Sets the initialized flag value indicating robot was localized or not. Internal use only
  void          setInitializedFlag(bool a)
  {
    myMutex.lock();
    myInitializedFlag = a;
    myMutex.unlock();
  }

protected:
  /// Basic initializer (internal use only)
  void          basicInitialization(ArRobot* robot, ArRangeDevice* laser);
  /// Function used to run the task as a thread.
  virtual void* runThread(void* ptr);
  /// Function which does the main eqn for the actual MCL localization.
  bool          correctPoseFromSensor(double increment, bool rayTrace, 
				      bool ignoreBadProb);
  /// Gets the localized flag. (do not use)
  bool          getLocalizedFlag(void)
  {
    myMutex.lock();
    bool ret = myLocalizedFlag;
    myMutex.unlock();
    return ret;
  }
  /// Gets the localizing flag value. To avoid multiple localizing threads.
  bool          getLocalizingFlag(void)
  {
    myMutex.lock();
    bool ret = myLocalizingFlag;
    myMutex.unlock();
    return ret;
  }
  /// Gets the robot moved flag value. (Moved after last localization)
  bool          getRobotMovedFlag(void)
  {
    myMutex.lock();
    bool ret = myRobotMovedFlag;
    myMutex.unlock();
    return ret;
  }
  /// Gets the flag indicating if localization result will be set on robot.
  bool          getCorrectRobotFlag(void)
  {
    myMutex.lock();
    bool ret = myCorrectRobotFlag;
    myMutex.unlock();
    return ret;
  }
  /// Gets the flag indicating if logging.
  bool          getLogFlag(void)
  {
    myMutex.lock();
    bool ret = myLogFlag;
    myMutex.unlock();
    return ret;
  }
  /// Gets the integer indicating the level of logging.
  int           getLogLevel(void)
  {
    myMutex.lock();
    int ret = myLogLevel;
    myMutex.unlock();
    return ret;
  }
  /// Gets the error flag.
  bool          getErrorLogFlag(void)
  {
    myMutex.lock();
    bool ret = myErrorLogFlag;
    myMutex.unlock();
    return ret;
  }
  /// Sets the localized flag value. (do not use)
  void          setLocalizedFlag(bool a)
  {
    myMutex.lock();
    myLocalizedFlag = a;
    myMutex.unlock();
  }
  /// Sets the robot moved flag to set off MCL.
  void          setRobotMovedFlag(bool a)
  {
    myMutex.lock();
    myRobotMovedFlag = a;
    myMutex.unlock();
  }

  /// Initialize the MCL samples.
  bool          initializeSamples(int numSamples);
  /// Get the laser buffer into data structure.
  bool          scanLaserIntoArray(void);
  /// Set resolution of the occupancy grid and fill it.
  ArOccGrid*    initializeOccGrid(double res, ArMapInterface* ariaMap, 
				  bool lockMap=true);
  /// Legalize samples to eliminate illegal location such as on obstacles.
  void          killBadSamples(double obsThreshold);
  /// Save samples for backup 
  void          saveSamples(bool saveFile=false);
  /// Reset samples from backup.
  void          resetSamples(void);
  /// Find the best poses by looking at prob distribution peaks.
  int           findBestPoses(ArRobotPoseSamples* mrsp, double factor);
  /// Get the number of peaks.
  int           getNumBestPoses(ArRobotPoseSamples* mrsp)
  {
    return myBestPoses.size();
  }

  /// Compute statistics of samples after localization.
  bool          findSamplesStatistics(double& xMean,double& yMean, 
				      double& thMean,
				      double& xStd,double& yStd, double& tStd);
  /// Return the state.
  bool          findReflectorKalmanMeanVar(ArPose& mean, ArMatrix& var);


  /// Fuses the stats of two uniform dists into one.
  bool          fuseTwoDistributions(ArPose m1, ArMatrix V1,
				     ArPose m2, ArMatrix V2,
				     ArPose& mean, ArMatrix& Var);
  /// Return time of last good localization.
  unsigned int  getLocaTime() 
  {
    return myLocaTime;
  }
  /// Set motion error params.
  void          setMotionErrorParam(int index, double value);
  /// Get motion error params.
  double        getMotionErrorParam(int index);
  /// The sensor interpretation callback. Called every 100msec.
  void          robotCallBack(void);
  /// Needed if the laser does not connect first.
  bool          configureLaser(void);
  /// Needed if the params are changed or loaded again.
  bool          reconfigureLocalization(void);
  /// Setup the path planing params with values from ArConfig.
  void          setupLocalizationParams(bool noReflectors = false);
  /// Actual thing that gets called when localization failed.
  void          failedLocalization(int times);
  /// Change the sample size to reflect localization score.
  int           dynamicallyAdjustNumSamples(double dr, double dt);

  /// Function for the things to do if map changes.
  void          mapChanged(void);
  /// Solve the triangulation eqn.
  bool          findPoseFromLandmarks(const std::vector<ArPose>& global,
				      const std::vector<ArPose>& local,
				      ArPose& rPose,
				      ArMatrix& Var);
  /// Solve the Kalman thing.
  bool          kalmanFilter(const std::vector<ArPose>& global,
			     const std::vector<ArPose>& local,
			     ArPose& pose,
			     ArPose& deltaPose,
			     ArPose& delta,
			     bool failedLocalization,
			     ArPose& mean,
			     ArPose& stdDev,
			     ArPose& maxInnov,
			     double& refScore);
  /// Find the reflectors in local and global coords.
  int           getReflectionCoords(ArPose& poseTaken,
				    ArPose& rPose,
				    std::vector<ArPose>& gList,
				    std::vector<ArPose>& lList);
  /// Find the reflector centers in local and global coords.
  int           getReflectorCenterCoords(ArPose& poseTaken,
					 ArPose& rPose,
					 std::vector<ArPose>& gList,
					 std::vector<ArPose>& lList);
  /// Fuse all the sensor information.
  bool          updateKalmanState(bool failedLocalization);
  /// Set the flags and call the right callbacks when loca fails ntimes.
  void          setFlagsAndCallbacksOnFail(int ntimes);

  private:
  bool     myLocalizingFlag;          // Localizing going on.
  bool     myInitializedFlag;         // Robot pose is initialized.
  bool     myRobotMovedFlag;          // Robot Moved Flag.
  bool     myLocalizedFlag;           // Robot Localized Flag.
  bool     myVerboseFlag;             // Print details flag.
  bool     myCorrectRobotFlag;        // Modify pose on robot flag.
  bool     myOwnArMapFlag;            // Does the ArMap belong to us?
  int      myNumSamples;              // Number of samples to use in MCL.
  int      myNumSamplesAtInit;        // Number of samples at initialization.
  bool     myRayTraceAtInit;          // Ray trace for localizeInit.
  ArPose   myHomePose;                // where we started off localizing too
  bool     myHomePoseSet;             // if the starting point was a home point
  bool     myIgnoreIllegalPoseFlag;   // Ignore if pose overlaps a occupied pt.
  bool     myAdjustNumSamplesFlag;    // Flag to change numSamples with score.
  bool     mySensorSetFlag;           // Flags the start of a new cycle.

  LocalizationState myState;          // Current state

  double   myTriggerDelR;             // Min distance to trigger Loc.
  double   myTriggerDelT;             // Min angle to trigger Loc.
  bool     myTriggerTimeFlag;         // Trigger on idle time?
  double   myTriggerTime;             // Min idle time in msecs to trigger loc.
  double   myTriggerTimeX;            // Range for X at idle time localization.
  double   myTriggerTimeY;            // Range for Y at idle time localization.
  double   myTriggerTimeTh;           // Range for Theta ...
  double   myPassThreshold;           // Theshold to pass localization.
  double   myLostThreshold;           // Theshold for lost declaration.
  double   myUseTempPassThreshold;    // Whether to use the temporary passThres
  double   myTempPassThreshold;       // The temporary passThres

  bool     myForceUpdateFlag;         // Flag to force an update in thread.
  int      myForceUpdateNumSamples;   // No of samples to force update with.
  ArPose   myForceUpdatePose;         // Pose to force update with.
  double   myForceUpdateXStd;         // X Std deviation.
  double   myForceUpdateYStd;         // Y Std deviation.
  double   myForceUpdateTStd;         // Theta Std deviation.
  bool     myForceUpdateRayTrace;     // Flag to force ray tracing

  std::vector<ArPose> myXYBuffer;     // Laser XY buffer.
  ArPose   myBufferPose;              // Pose at which buffer was filled.
  
  ArPose   myCurrentLocaPose;         // Current robot pose.
  ArPose   myTrueCurrentPose;         // True Current robot pose.
  ArPose   myTrueCurrentEncoderPose;         // True Current robot pose. Enc.
  ArPose   myLastLocaPose;            // Last localized pose.
  ArPose   myLastLocaEncoderPose;     // Last localized pose.Enc
  ArTime   myLastLocaTime;            // Last localization time.
  ArPose   myLastDelPose;             // Last delpose after localization.
  ArPose   myLastLocaMean;            // Kept for later.
  ArMatrix myLastLocaVar;             // Kept for later.
  std::vector<ArRobotPoseProb*> myBestPoses; // Heap of probable poses.
  unsigned int  myLocaTime;           // Time at which good localization done.
  int      myCurrentNumSamples;       // Variable sample size for the cycle.
  int      myMinNumSamples;           // Min no of samples when adjustable.
  double   myNumSamplesAngleFactor;   // Special factor when rotated.
  ArPose   myLastEncoderPose;         // For computing Kalman movement.

  ArRobot* myRobot;
  ArRangeDevice*  myLaser;
  ArMutex  myMutex;

  ArRobotPoseSamples* myRobotSamplesPtr; // MCL samples data struct.
  ArRobotPoseSamples* myPreResamplesPtr; // Before resampling.
  ArRobotAndLaser*    myRobotAndLaserPtr;// Robot and Laser related stuff.
  ArSystemError*      mySystemErrorPtr;  // Error parameter holder.
  ArOccGrid*          myOccGridPtr;      // Occupancy grid.
  ArMapInterface*     myAriaMapPtr;      // Map data.

  ArFunctorC<ArLocalizationTask>*          myRobotCB;  // Robot->getPose()
  ArFunctor1<int>*                         myFailedCB; // Failed callback.

  ArRetFunctorC<bool, ArLocalizationTask>* myLaserConnectedCB;

  ArRetFunctorC<bool, ArLocalizationTask>* myProcessFileCB; // Config.
  ArFunctorC<ArLocalizationTask>*          myMapChangedCB;

  ArRetFunctor<std::list<ArMultiRobotPoseAndRadius> >*
                                           myMultiRobotFunction;

  ArConfig*                                myParams; // Default params holder.
  std::list<ArFunctor1<int>*>              myFailedLocalizationCBList;
  std::vector<ArPose>                      myDisplayPoints;
  std::vector<ArPose>                      myDisplayPoints2;
  std::vector<ArPose>                      myDisplayPoints3;
  
  ///
  ArTransform                              myEncToGlo;
  ArTransform                              myEncToLocalization;
  ///
  ArTime                                   myClock;

  int                                      myStateSize; // Kalman state size.
  ArKalmanFilterReflector*                 myKalman;

  bool                                     myDisplayReflectorRet;
  ArPose                                   myDisplayReflectorMean;
  ArMatrix                                 myDisplayReflectorVar;

  ArPose                                   myOdoPose;
  ArPose                                   myOdoEncoderPose;
  ArPose                                   myReflectorPose;
  ArPose                                   myReflectorOdoEncoderPose;
  ArPose                                   myKalmanPose;
  std::vector<ArPose>                      myReflectorDrawList;

  double                                   myRefLocaScore; 
  bool                                     myBypassMCLFlag;

  bool                                     myDisplayMCLRet;
  ArPose                                   myDisplayMCLMean;
  ArMatrix                                 myDisplayMCLVar;
  ArMatrix                                 myDisplayMCLBounds;

  bool                                     myDisplayRefRet;
  ArPose                                   myDisplayRefMean;
  ArMatrix                                 myDisplayRefVar;

protected:
  double myInitStdX;
  double myInitStdY;
  double myInitStdTh;
  double myErrorMmPerMm;
  double myErrorDegPerDeg;
  double myErrorDegPerMm;
  double mySensorBelief;
  double myPeakFactor;
  double myOccThreshold;
  double myGridRes;
  char   myMapName[1024];
  double myPeturbX;
  double myPeturbY;
  double myPeturbTh;
  double myFailedX;
  double myFailedY;
  double myFailedTh;
  int    myFailedTries;
  double myPeakStdX;
  double myPeakStdY;
  double myPeakStdTh;
  double myAngleIncrement;
  double myKillThreshold;
  bool   myRecoverOnFailFlag;
  bool   myIdleFlag;
  bool   myReloadingMapFlag;

  bool   myEnableReflectorLocalizationFlag;
  bool   myOldEnableReflectorLocalizationFlag;
  double myDistanceThreshold;
  double myReflectorVar;
  double myReflectorMatchDist;
  double myReflectorMatchAngle;
  double myReflectorMaxRange;
  double myReflectorMaxAngle;
  double myReflectorSize;
  double myReflectorAngleVarLimit;
  double myBadReflectorFactor;
  double myReflectorRangeFactor;
  bool   myEnableTriangulationFlag;
  double myReflectorTriDistLimit;
  double myReflectorTriAngLimit;
  int    myReflectanceThreshold;
  bool   myUseAllKFlag;
  double myQTra;
  double myQRot;
  double myQTraVel;
  double myQRotVel;
  double myQTraAcc;
  double myQRotAcc;
  double myQZeroSpeedFactor;
  double myXYLimit;
  double myThLimit;
  double myScoreToVarFactor;

  bool   myLogFlag;
  int    myLogLevel;
  bool   myErrorLogFlag;
  ArPose myError;

  int    myTransitionFromIdleNumSamples;
  double myTransitionFromIdleXStd;
  double myTransitionFromIdleYStd;
  double myTransitionFromIdleTStd;
  bool   myMapLocalizable;
};

#endif // ARLOCALIZATIONTASK_H
