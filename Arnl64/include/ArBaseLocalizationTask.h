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
#ifndef ARBASELOCALIZATIONTASK_H
#define ARBASELOCALIZATIONTASK_H

#include "Aria.h"
#include "ArMatrix.h"

/**
   This class has the similar items of ArLocalizationTask and
   ArSonarLocalizationTask, right now thats just one enum, later it
   may grow to become more.
**/

class ArBaseLocalizationTask: public ArASyncTask
{
public:
  ArBaseLocalizationTask(const char *name) : 
    myGetRobotHomeCB(this, &ArBaseLocalizationTask::getRobotHome),
    myGetPoseInterpPositionCB(this, 
			      &ArBaseLocalizationTask::getPoseInterpPosition),
    myEncoderToLocalizationTransformCB(
	    this, &ArBaseLocalizationTask::getEncoderToLocalizationTransform)
  { 
    myName = name; 
    std::string interpolationName;
    interpolationName = name;
    interpolationName += "::Interpolation";
    myInterpolation.setName(interpolationName.c_str());
    myInterpolation.setAllowedMSForPrediction(0);
  }
  /// State of the localization, accessible using getState().
  enum LocalizationState
  {
    NOT_INITIALIZED,        /// Task not initialized                     0
    LOCALIZATION_FAILED,    /// Failed localization.                     1
    LOCALIZATION_SUCCESS,   /// Localization succeeded.                  2
    LOCALIZATION_INIT_COMPUTING, /// Localization init in progress.      3
    LOCALIZATION_MOVE_COMPUTING, /// Localization move in progress.      4
    LOCALIZATION_IDLE,      /// Localization idled.                      5
    INVALID                 /// Invalid state.                           6
  };
  /// State of the lock, accessible using getLockState().
  enum LockState
  {
    LOCK_NOT_INITIALIZED, /// Lock not initialized                     0
    LOST,                 /// Failed lock for too long.                1
    LOST_LOCK,            /// Failed lock.                             2
    LOCKED                /// Localization succeeded.                  3
  };
  /// Function returns what the current mean pose and its variance.
  virtual bool findLocalizationMeanVar(ArPose& mean, ArMatrix& var)
  {
    return false;
  }
  /// Used to switch off moveTo inside the child localization tasks.
  virtual void setCorrectRobotFlag(bool f) 
  {
    return;
  }
  /** Set a new position for the localization task thread to use on its next
   * iteration.  
   * Some localization techniques may simply continue from the new position, others
   * may perform some extra checks against map data similar to an initial
   * startup localization.
   * Some paremeters may optionally be supplied. If not given, then previously
   * stored parameters are used (normally as given in ArConfig).
   * This function returns immediately after setting up the new position; it
   * does not block until the localization is finished. (You can use the 
   * localization status accessors and callbacks to determine whether
   * localization later fails.)
   * @param spread Use this instead of any stored "spread" parameters, if
   *  applicable to a particular localization technique
   * @param nSam Use this instead of any stored "number of samples" parameter,
   *  if applicable to a particular localization technique
   */
  virtual void setRobotPose(ArPose pose, 
				     ArPose spread = ArPose(-1., -1., -1.), 
				     int nSam = -1)
  {
    return;
  }
  /// Finds if the robot is lost.
  virtual bool getRobotIsLostFlag() 
  {
    return false;
  }
  /// Finds if the loca is idle
  virtual bool getIdleFlag() 
  {
    return false;
  }
  /// Gets the robot home pose
  /// This pose is the initial best localized position determined by 
  /// localizeRobotAtHomeBlocking() or localizeRobotAtHomeNonBlocking() methods
  virtual ArPose getRobotHome() 
  {
    return ArPose();
  }
  /// Get a functor that when called returns the robot's home pose
  /// This pose is the initial best localized position determined by 
  /// localizeRobotAtHomeBlocking() or localizeRobotAtHomeNonBlocking() methods
  ArRetFunctor<ArPose>* getRobotHomeCallback() 
  { 
    return &myGetRobotHomeCB; 
  }
  /// @deprecated
  ArPose getHomePose() 
  { 
    return getRobotHome(); 
  }
  /// Localize robot at start or while initialization.
  virtual bool localizeRobotAtHomeBlocking(double distSpread,
						    double angleSpread)
  {
    return false;
  }
  /// Try localizing at each home position, choosing the best to be the 
  /// current and the stored "home" position, using default parameters.
  virtual bool localizeRobotAtHomeBlocking() 
  {
    return false;
  }
  /// Gets the current sample poses if relevant.
  virtual std::list<ArPose> getCurrentSamplePoses() 
  {
    std::list<ArPose> l; return l;
  }
  /// Get the localization state.
  virtual LocalizationState getState(void)
  {
    return NOT_INITIALIZED;
  }
  /// Get the localization lock state.
  virtual LockState getLockState(void)
  {
    return LOCK_NOT_INITIALIZED;
  }
  /// Get the localization score.
  virtual double getLocalizationScore(void) 
  {
    return 0.0;
  }
  /// Get the localization score.
  virtual double getLocalizationLock(void) 
  {
    return 0.0;
  }
  /// Get the localization threshold.
  virtual double getLocalizationThreshold(void) 
  {
    return 0.0;
  }
  /// Get the localization idle
  virtual bool checkLocalizationIdle(void) 
  {
    return (getState() == LOCALIZATION_IDLE);
  }
  /// Set the localization idle
  virtual void setLocalizationIdle(bool f)
  {
    return;
  }
  /// Set the localization threshold.
  virtual void setLocalizationThreshold(double t) 
  {
    return;
  }
  /// Gets the name of this task
  const char *getName(void) 
  { 
    return myName.c_str(); 
  }
  /// Gets the position the robot was at at the given timestamp
  /** @see ArInterpolation::getPose 
   */
  virtual int getPoseInterpPosition(ArTime timeStamp, ArPose *position, 
				    ArPoseWithTime *mostRecent = NULL)
  { 
    return myInterpolation.getPose(timeStamp, position, mostRecent); 
  }
  /// Gets the interpolation object for more direct use
  virtual ArInterpolation *getInterpolation(void) 
    {
      return &myInterpolation;
    }
  /// Gets the callback that will call getPoseInterpPosition
  ArRetFunctor3<int, ArTime, ArPose *, ArPoseWithTime *> *
  getPoseInterpPositionCallback(void)
  { 
    return &myGetPoseInterpPositionCB; 
  }
  /// Convert LLA to Robot
  virtual bool convertLLA2RobotCoords(double lat, double lon, double alt,
				      double& ea, double& no, double& up)
  {
    return false;
  }
  /// Convert Robot to LLA.
  virtual bool convertRobot2LLACoords(double ea, double no, double up,
				      double& lat, double& lon, double& alt)
  {
    return false;
  }

  /// Return the transform converting the encoder pose to global pose for
  /// this localization. Needed for interpolation between two locas in time.
  ArTransform getEncoderToLocalizationTransform(void)
  {
    myTMutex.lock();
    ArTransform ret = myEncoderToLocalization;
    myTMutex.unlock();
    return ret;
  }
  /// Gets the callback for getEncoderToLocalizationTransform
  ArRetFunctor<ArTransform> *getEncoderToLocalizationTransformCallback(void)
  { 
    return &myEncoderToLocalizationTransformCB; 
  }

  /// Adds a plain callback for when the state changes.
  virtual void addStateChangeCB(ArFunctor *functor, int position = 50)
  { 
    myStateChangeCBList.addCallback(functor, position); 
  }
  /// Removes a plain callback for when the state changes.
  virtual void remStateChangeCB(ArFunctor *functor)
  { 
    myStateChangeCBList.remCallback(functor); 
  }

  /// Save the localization state (kalman) into a string.
  virtual std::string saveInternalStateToString(void)
  { 
    std::string ret;
    return ret;
  }
  /// Save the localization state (kalman) into a string.
  virtual void restoreInternalStateFromString(std::string str)
  { 
    return;
  }

protected:
  void setEncoderToLocalizationTransform(ArTransform tr)
  {
    myTMutex.lock();
    myEncoderToLocalization = tr;
    myTMutex.unlock();
  }

  std::string myName;
  ArRetFunctorC<ArPose, ArBaseLocalizationTask> myGetRobotHomeCB;
  ArRetFunctor3C<int, ArBaseLocalizationTask, ArTime, 
		 ArPose *, ArPoseWithTime *> myGetPoseInterpPositionCB;
  ArRetFunctorC<ArTransform, 
		ArBaseLocalizationTask> myEncoderToLocalizationTransformCB;

  ArInterpolation      myInterpolation;
  ArTransform          myEncoderToLocalization;
  ArMutex              myTMutex;

  ArCallbackList myStateChangeCBList;

};

#endif // ARBASELOCALIZATIONTASK_H
