#ifndef ARACTIONSTOP_H
#define ARACTIONSTOP_H

#include "ariaTypedefs.h"
#include "ArAction.h"

/// Action for stopping the robot
/**
   This action simply sets the robot to a 0 velocity and a deltaHeading of 0.
   @ingroup ActionClasses
*/
class ArActionStop : public ArAction
{
public:
  /// Constructor
  AREXPORT ArActionStop(const char *name = "stop");
  /// Destructor
  AREXPORT virtual ~ArActionStop();
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  AREXPORT virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  AREXPORT virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
protected:
  ArActionDesired myDesired;
};

#endif // ARACTIONSTOP_H
