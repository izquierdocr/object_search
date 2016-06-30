#ifndef ARACTIONSPEEDLIMITER_H
#define ARACTIONSPEEDLIMITER_H

#include "ariaTypedefs.h"
#include "ArAction.h"

/// Action to limit the forwards motion of the robot based on range sensor readings.
/**
   This action uses the sensors to find a maximum forwared speed to travel at; when the range
   sensor (e.g. sonar or laser) detects obstacles closer than the given parameters,
   this action requests that the robot decelerate or stop.
   @ingroup ActionClasses
*/
class ArActionLimiterForwards : public ArAction
{
public:
  /// Constructor
  AREXPORT ArActionLimiterForwards(const char *name = "speed limiter", 
				   double stopDistance = 250,
				   double slowDistance = 1000,
				   double slowSpeed = 200,
				   double widthRatio = 1);
  /// Destructor
  AREXPORT virtual ~ArActionLimiterForwards();
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  AREXPORT virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  AREXPORT virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
  AREXPORT void setParameters(double stopDistance = 250,
			      double slowDistance = 1000,
			      double slowSpeed = 200,
			      double widthRatio = 1);
protected:
  bool myLastStopped;
  double myStopDist;
  double mySlowDist;
  double mySlowSpeed;
  double myWidthRatio;
  ArActionDesired myDesired;
};

#endif // ARACTIONSPEEDLIMITER_H
