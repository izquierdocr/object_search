#ifndef ARACTIONLIMITERROT_H
#define ARACTIONLIMITERROT_H

#include "ariaTypedefs.h"
#include "ArAction.h"

/// Action to limit the forwards motion of the robot based on range sensor readings
/**
   This action uses the robot's range sensors (e.g. sonar, laser) to find a 
   maximum speed at which to travel
   and will increase the deceleration so that the robot doesn't hit
   anything.  If it has to, it will trigger an estop to avoid a
   collision.

   Note that this cranks up the deceleration with a strong strength,
   but it checks to see if there is already something decelerating
   more strongly... so you can put these actions lower in the priority list so
   things will play together nicely.

   @ingroup ActionClasses
**/
class ArActionLimiterRot : public ArAction
{
public:
  /// Constructor
  AREXPORT ArActionLimiterRot(const char *name = "limitRot");
  /// Destructor
  AREXPORT virtual ~ArActionLimiterRot();
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  AREXPORT virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  AREXPORT virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
  /// Sets the parameters (don't use this if you're using the addToConfig)
  AREXPORT void setParameters(bool checkRadius = false,
			      double inRadiusSpeed = 0);
  /// Adds to the ArConfig given, in section, with prefix
  AREXPORT void addToConfig(ArConfig *config, const char *section,
			    const char *prefix = NULL);
  /// Sets if we're using locationDependent range devices or not
  bool getUseLocationDependentDevices(void) 
    { return myUseLocationDependentDevices; }
  /// Sets if we're using locationDependent range devices or not
  void setUseLocationDependentDevices(bool useLocationDependentDevices)
    { myUseLocationDependentDevices = useLocationDependentDevices; }
protected:
  bool myCheckRadius;
  double myInRadiusSpeed;
  bool myUseLocationDependentDevices;
  ArActionDesired myDesired;
};

#endif // ARACTIONSPEEDLIMITER_H
