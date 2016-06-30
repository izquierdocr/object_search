#ifndef ARACTIONMOVEMENTPARAMTERSDEBUGGING_H
#define ARACTIONMOVEMENTPARAMTERSDEBUGGING_H

#include "ariaTypedefs.h"
#include "ArAction.h"
#include "ArMapObject.h"

/// This is a class for setting max velocities and accels and decels via ArConfig parameters (see addToConfig());
/**
   @ingroup ActionClasses
 **/
class ArActionMovementParametersDebugging : public ArAction
{
public: 
  /// Constructor
  AREXPORT ArActionMovementParametersDebugging(const char *name = "MovementParametersDebugging");
  /// Destructor
  AREXPORT virtual ~ArActionMovementParametersDebugging();
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
  /// Adds to the ArConfig given, in section, with prefix
  AREXPORT void addToConfig(ArConfig *config, const char *section,
			    const char *prefix = NULL);
protected:
  bool myEnabled;
  bool myEnableOnce;

  bool mySetMaxVel;
  double myMaxVel;
  bool mySetMaxNegVel;
  double myMaxNegVel;
  bool mySetTransAccel;
  double myTransAccel;
  bool mySetTransDecel;
  double myTransDecel;
  bool mySetMaxRotVel;
  double myMaxRotVel;
  bool mySetRotAccel;
  double myRotAccel;
  bool mySetRotDecel;
  double myRotDecel;
  bool mySetMaxLeftLatVel;
  double myMaxLeftLatVel;
  bool mySetMaxRightLatVel;
  double myMaxRightLatVel;
  bool mySetLatAccel;
  double myLatAccel;
  bool mySetLatDecel;
  double myLatDecel;
  
  ArActionDesired myDesired;


};

#endif // ARACTIONMOVEMENTPARAMTERS_H
