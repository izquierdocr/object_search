#ifndef ARACTIONDECELERATINGLIMITER_H
#define ARACTIONDECELERATINGLIMITER_H

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
class ArActionDeceleratingLimiter : public ArAction
{
public:
  enum LimiterType {
    FORWARDS, ///< Limit forwards
    BACKWARDS, ///< Limit backwards
    LATERAL_LEFT, ///< Limit lateral left
    LATERAL_RIGHT ///< Limit lateral right

  };
  /// Constructor
  AREXPORT ArActionDeceleratingLimiter(const char *name = "limitAndDecel", 
				       LimiterType type = FORWARDS);
  /// Destructor
  AREXPORT virtual ~ArActionDeceleratingLimiter();
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  AREXPORT virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  AREXPORT virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
  /// Sets the parameters (don't use this if you're using the addToConfig)
  AREXPORT void setParameters(double clearance = 100,
			      double sideClearanceAtSlowSpeed = 50,
			      double paddingAtSlowSpeed = 50,
			      double slowSpeed = 200,
			      double sideClearanceAtFastSpeed = 400,
			      double paddingAtFastSpeed = 300,
			      double fastSpeed = 1000,
			      double preferredDecel = 600,
			      bool useEStop = false,
			      double maxEmergencyDecel = 0);
  /// Gets if this will control us when going forwards
  LimiterType getType(void) { return myType; }
  /// Sets if this will control us when going forwards
  void setType(LimiterType type) { myType = type; }
  /// Adds to the ArConfig given, in section, with prefix
  AREXPORT void addToConfig(ArConfig *config, const char *section,
			    const char *prefix = NULL);
  /// Sets if we're using locationDependent range devices or not
  bool getUseLocationDependentDevices(void) 
    { return myUseLocationDependentDevices; }
  /// Sets if we're using locationDependent range devices or not
  void setUseLocationDependentDevices(bool useLocationDependentDevices)
    { myUseLocationDependentDevices = useLocationDependentDevices; }
  // sets if we should stop rotation too if this action has stopped the robot
  void setStopRotationToo(bool stopRotationToo)
    { myStopRotationToo = stopRotationToo; }
protected:
  bool myLastStopped;
  LimiterType myType;
  double myClearance;
  double mySideClearanceAtSlowSpeed;
  double myPaddingAtSlowSpeed;
  double mySlowSpeed;
  double mySideClearanceAtFastSpeed;
  double myPaddingAtFastSpeed;
  double myFastSpeed;
  double myPreferredDecel;
  double myMaxEmergencyDecel;
  bool myUseEStop;
  bool myUseLocationDependentDevices;
  bool myStopRotationToo;

//unused?  double myDecelerateDistance;
  ArActionDesired myDesired;
};

#endif // ARACTIONSPEEDLIMITER_H
