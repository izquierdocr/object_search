#ifndef ARACTIONLIMITERTABLESNSOR_H
#define ARACTIONLIMITERTABLESNSOR_H

#include "ariaTypedefs.h"
#include "ArAction.h"

/// Action to limit speed (and stop) based on whether the "table"-sensors see anything
/**
   This action limits speed to 0 if the table-sensors see anything in front
   of the robot.  The action will only work if the robot has table sensors,
   meaning that the robots parameter file has them listed as true.

   @ingroup ActionClasses
*/
class ArActionLimiterTableSensor : public ArAction
{
public:
  /// Constructor
  AREXPORT ArActionLimiterTableSensor(const char *name = "TableSensorLimiter");
  /// Destructor
  AREXPORT virtual ~ArActionLimiterTableSensor();
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  AREXPORT virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  AREXPORT virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
protected:
  ArActionDesired myDesired;
};



#endif // ARACTIONLIMITERTABLESNSOR_H
