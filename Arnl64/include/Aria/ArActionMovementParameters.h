#ifndef ARACTIONMOVEMENTPARAMTERS_H
#define ARACTIONMOVEMENTPARAMTERS_H

#include "ariaTypedefs.h"
#include "ArAction.h"
#include "ArMapObject.h"

/// This is a class for setting max velocities and accels and decels via ArConfig parameters (see addToConfig()) or manually (using setParameters())
/**
   @ingroup ActionClasses
 **/
class ArActionMovementParameters : public ArAction
{
public: 
  /// Constructor
  AREXPORT ArActionMovementParameters(const char *name = "MovementParameters",
				      bool overrideFaster = true, 
				      bool addLatVelIfAvailable = true);
  /// Destructor
  AREXPORT virtual ~ArActionMovementParameters();
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  AREXPORT virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  AREXPORT virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
  /// Sees if this action is enabled (separate from activating it)
  AREXPORT bool isEnabled(void) { return myEnabled; }
  /// Enables this action (separate from activating it)
  AREXPORT void enable(void) { myEnabled = true; }
  /// Enables this action in a way that'll work from the sector callbacks
  AREXPORT void enableOnceFromSector(ArMapObject *mapObject) 
    { myEnableOnce = true; }
  /// Disables this action (separate from deactivating it)
  AREXPORT void disable(void) { myEnabled = false; }
  /// Sets the parameters (don't use this if you're using the addToConfig)
  AREXPORT void setParameters(double maxVel = 0, double maxNegVel = 0,
			      double transAccel = 0, double transDecel = 0,
			      double rotVelMax = 0, double rotAccel = 0,
			      double rotDecel = 0, double latVelMax = 0, 
			      double latAccel = 0, double latDecel = 0);
  /// Adds to the ArConfig given, in section, with prefix
  AREXPORT void addToConfig(ArConfig *config, const char *section,
			    const char *prefix = NULL);
protected:
  bool myEnabled;
  bool myEnableOnce;
  bool myOverrideFaster;
  bool myAddLatVelIfAvailable;

  double myMaxVel;
  double myMaxNegVel;
  double myTransAccel;
  double myTransDecel;
  double myMaxRotVel;
  double myRotAccel;
  double myRotDecel;
  double myMaxLatVel;
  double myLatAccel;
  double myLatDecel;
  
  ArActionDesired myDesired;


};

#endif // ARACTIONMOVEMENTPARAMTERS_H
