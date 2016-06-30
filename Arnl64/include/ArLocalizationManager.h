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
#ifndef ARLOCALIZATIONMANAGER_H
#define ARLOCALIZATIONMANAGER_H

#include "Aria.h"
#include "ArNetworking.h"
#include "ArExport.h"
#include "ArBaseLocalizationTask.h"
/*!
  @class ArLocalizationManager
  @brief Task that fuses the output of the multiple localization threads.
  
 */
class ArLocalizationManager: public ArBaseLocalizationTask
{
/*
  friend class ArLocalizationTask;
  friend class ArGPSLocalizationTask;
  friend class ArLightLocalizationTask;
*/
public:

  /// Base constructor with all the necessary inputs.
  AREXPORT ArLocalizationManager(ArRobot* robot, ArMapInterface* ariaMap);

  /// Destructor.
  AREXPORT virtual ~ArLocalizationManager(void) 
  {
    if(myLogFile)
      fclose(myLogFile);
  }
   /// Fuse two distributions to make one mean pose and one variance matrix.
  AREXPORT bool fuseTwoDistributions(ArPose m1, ArMatrix V1, double s1,
				     double threshold1, 
				     ArPose m2, ArMatrix V2, double s2, 
				     double threshold2, 
				     ArPose& mean, ArMatrix& Var, double& s,
				     double& threshold);
  /// Adds a localization to be managed.
  AREXPORT bool addLocalizationTask(ArBaseLocalizationTask* loca);
  /// Removes a localization from the  managing.
  AREXPORT bool removeLocalizationTask(ArBaseLocalizationTask* loca);
  /// The actual mean var calculator for the virtual in the base class.
  AREXPORT virtual bool findLocalizationMeanVar(ArPose& mean, ArMatrix& Var);
  /// Sets the flag deciding whether to reflect localized pose onto the robot.
  AREXPORT virtual void setCorrectRobotFlag(bool a);
  /// Used to set the robot pose usually at the start of localization. 
  /// This one with a spread around the set pose.
  AREXPORT virtual void setRobotPose(ArPose pose,
				     ArPose spread = ArPose(0, 0, 0), 
				     int nSam = 0);
  /// Localize robot at start or while initialization.
  AREXPORT virtual bool localizeRobotAtHomeBlocking(double distSpread,
						    double angleSpread);
  /// Function to get the light loca pointer.
  AREXPORT virtual ArBaseLocalizationTask* getLightLocalizationPtr(void)
  {
    std::list<ArBaseLocalizationTask*>::iterator locaIt;
    ArBaseLocalizationTask* locaObj;

    myMutex.lock();
    std::list<ArBaseLocalizationTask*> locaList = myLocaList;
    /// Do the fusing.
    for(locaIt = locaList.begin(); locaIt != locaList.end(); locaIt++)
    {
      locaObj = (*locaIt);
      if(strcmp(locaObj->getName(), "Light") == 0)
      {
	myMutex.unlock();
	return locaObj;
      }
    }
    myMutex.unlock();
    return NULL;
  }
  /// Gets the lost flag.
  AREXPORT virtual bool getRobotIsLostFlag(void) 
  {
    return (myState == LOCALIZATION_FAILED);
  }
  /// Finds if the robot is lost.
  AREXPORT virtual bool getIdleFlag() 
  {
    return false;
  }
  /// Gets the state of localization.
  AREXPORT virtual LocalizationState getState(void)
  {
    myMutex.lock();
    LocalizationState p = myState;
    myMutex.unlock();
    return p;
  }
  /// Returns the weighted average of the scores from the localizations in
  /// its list. Only the XY pose uncertainity is used for now compute the
  /// weight. The theta uncertainity is ignored.
  AREXPORT virtual double getLocalizationScore(void)
  {
    myMutex.lock();
    double ret = myScore;
    myMutex.unlock();
    return ret;
  }
  // Get the localization score threshold.
  AREXPORT virtual double getLocalizationThreshold(void)
  {
    myMutex.lock();
    double ret = myLocalizationThreshold;
    myMutex.unlock();
    return ret;
  }
  /// Function to set the score.
  AREXPORT virtual void setLocalizationScore(double f)
  {
    myMutex.lock();
    myScore = f;
    myMutex.unlock();
  }
  // Set the localization score threshold.
  AREXPORT virtual void setLocalizationThreshold(double t)
  {
    myMutex.lock();
    myLocalizationThreshold = t;
    myMutex.unlock();
  }

protected:
  /// Function used to run the task as a thread.
  AREXPORT virtual void* runThread(void* ptr);
  ///
  void          setupLocalizationParams(void);
  /// Needed if the params are changed or loaded again.
  bool          reconfigureLocalization(void);


  ArRobot*                           myRobot;
  ArMapInterface*                    myMap;
  std::list<ArBaseLocalizationTask*> myLocaList;
  ArMatrix                           myVar;
  ArPose                             myMean;
  ArPose                             myLastLocaEncoderPose;

  ArRetFunctorC<bool, ArLocalizationManager>* myProcessFileCB; // Config.

  ArConfig*                          myParams; // Default params holder.
  double                             myDistanceLimit;
  double                             myAngleLimit;
  FILE*                              myLogFile;
  bool                               myLogFlag;
  bool                               myErrorLogFlag;
  ArPose                             myError;
  double                             myScore;
  double                             myLocalizationThreshold;

  LocalizationState                  myState;          // Current state
  bool                               myInitializedFlag;
  
};

#endif // ARLOCALIZATIONMANAGER_H
