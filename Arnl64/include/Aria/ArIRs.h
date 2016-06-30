#ifndef ARIRS_H
#define ARIRS_H

#include "ariaTypedefs.h"
#include "ArRangeDevice.h"


/// A class that treats a robot's infrared sensors as a range device.
/**
  (Only Peoplebot and some Powerbots have IR sensors.)

  @ingroup OptionalClasses  
  @ingroup DeviceClasses
*/
class ArIRs : public ArRangeDevice
{
public:
  AREXPORT ArIRs(size_t currentBufferSize = 10, 
		     size_t cumulativeBufferSize = 10,
		     const char *name = "irs",
		     int maxSecondsToKeepCurrent = 15);
  AREXPORT virtual ~ArIRs(void);

  AREXPORT virtual void setRobot(ArRobot *robot);
  AREXPORT void processReadings(void);

protected:
  ArFunctorC<ArIRs> myProcessCB;
  ArRobotParams myParams;
  std::vector<int> cycleCounters;
};


#endif // ARIRS_H
