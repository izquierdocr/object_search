#ifndef ARSONARAUTODISABLER_H
#define ARSONARAUTODISABLER_H

/// Class for automatically disabling sonar when the robot is stopped
/**
   If you create one of this class it will disable the sonar when the
   robot stops moving and then enable the sonar when the robot moves.
   Later this may get more parameters and the ability to be turned on
   and off and things like that (email on aria-users if you want
   them).

   Note that this class assumes it is the only class turning the sonar
   on or off and that the sonar start on.

    @ingroup OptionalClasses
 **/

#include "ariaTypedefs.h"
#include "ariaUtil.h"
#include "ArFunctor.h"

class ArRobot;

class ArSonarAutoDisabler
{
public:
  /// Constructor
  AREXPORT ArSonarAutoDisabler(ArRobot *robot);
  /// Destructor
  AREXPORT virtual ~ArSonarAutoDisabler();
  /// Supresses this disabler (which turns off the sonar)
  void supress(void) 
    { ArLog::log(ArLog::Normal, "ArSonarAutoDisabler::supress:"); 
      mySupressed = true; }
  /// Gets the callback to supress the autodisabler
  ArFunctor *getSupressCallback(void) { return &mySupressCB; }
  /// Unsupresses this disabler (goes back to auto enabling/disabling)
  void unsupress(void) 
    { ArLog::log(ArLog::Normal, "ArSonarAutoDisabler::unsupress:"); 
      mySupressed = false; }
  /// Gets the callback to supress the autodisabler
  ArFunctor *getUnsupressCallback(void) { return &myUnsupressCB; }

  /// Sets that we're autonomous drivign so we only enable some sonar
  void setAutonomousDriving(void) 
    { ArLog::log(ArLog::Normal, "ArSonarAutoDisabler::setAutonomousDriving:"); 
      myAutonomousDriving = true; }
  /// Gets the callback to set that we're driving autonomously
  ArFunctor *getSetAutonomousDrivingCallback(void) 
    { return &mySetAutonomousDrivingCB; }
  /// Sets that we're driving non-autonomously so we enable all sonar
  void clearAutonomousDriving(void) 
    { ArLog::log(ArLog::Normal, "ArSonarAutoDisabler::clearAutonomousDriving:"); 
      myAutonomousDriving = false; }
  /// Gets the callback to set that we're not driving autonomously
  ArFunctor *getClearAutonomousDrivingCallback(void) 
    { return &myClearAutonomousDrivingCB; }
protected:
  /// our user task
  AREXPORT void userTask(void);
  ArRobot *myRobot;
  ArTime myLastMoved;
  ArTime myLastSupressed;
  bool mySupressed;
  bool myAutonomousDriving;

  ArFunctorC<ArSonarAutoDisabler> myUserTaskCB;
  ArFunctorC<ArSonarAutoDisabler> mySupressCB;
  ArFunctorC<ArSonarAutoDisabler> myUnsupressCB;
  ArFunctorC<ArSonarAutoDisabler> mySetAutonomousDrivingCB;
  ArFunctorC<ArSonarAutoDisabler> myClearAutonomousDrivingCB;
};

#endif // ARSONARAUTODISABLER
