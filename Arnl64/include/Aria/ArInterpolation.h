#ifndef ARINTERPOLATION_H
#define ARINTERPOLATION_H

#include "ariaTypedefs.h"
#include "ariaUtil.h"

/** 
    This class takes care of storing in readings of position vs time, and then
    interpolating between them to find where the robot was at a particular 
    point in time.  It has two lists, one containing the times, and one 
    containing the positions at those same times (per position), they must be 
    walked through jointly to maintain cohesion.  The new entries are at the
    front of the list, while the old ones are at the back.  
    numberOfReadings and the setNumberOfReadings control the number of entries
    in the list.  If a size is set that is smaller than the current size, then
    the old ones are chopped off.
    
    This class now has a couple of variables for when it allows
    prediction, they're set with setAllowedMSForPrediction and
    setAllowedPercentageForPrediction.  If either is below 0 than they
    are ignored (if both are below 0 it means any prediction is
    allowed, which would be bad).  Previous there was no MS limit, and
    the percentage limit was 50 (and so that is what the default is
    now).
**/
class ArInterpolation
{
public:
  /// Constructor
  AREXPORT ArInterpolation(size_t numberOfReadings = 100);
  /// Destructor
  AREXPORT virtual ~ArInterpolation();
  /// Adds a new reading
  AREXPORT bool addReading(ArTime timeOfReading, ArPose position);
  /// Finds a position
  AREXPORT int getPose(ArTime timeStamp, ArPose *position, 
		       ArPoseWithTime *lastData = NULL);
  /// Sets the name
  AREXPORT void setName(const char *name);
  /// Gets the name
  AREXPORT const char *getName(void);
  /// Sets the allowed milliseconds for prediction
  AREXPORT void setAllowedMSForPrediction(int ms = -1);
  /// Sets the allowed milliseconds for prediction
  AREXPORT int getAllowedMSForPrediction(void);
  /// Sets the allowed percentage for prediction
  AREXPORT void setAllowedPercentageForPrediction(int percentage = 5000);
  /// Sets the allowed milliseconds for prediction
  AREXPORT int getAllowedPercentageForPrediction(void);
  /// Sets if we should log events for this interpolation
  AREXPORT void setLogPrediction(bool logPrediction = false);
  /// Gets if we should log events for this interpolation
  AREXPORT bool getLogPrediction(void);
  /// Sets the number of readings this instance holds back in time
  AREXPORT void setNumberOfReadings(size_t numberOfReadings);
  /// Gets the number of readings this instance holds back in time
  AREXPORT size_t getNumberOfReadings(void) const;
  /// Empties the interpolated positions
  AREXPORT void reset(void);
protected:
  ArMutex myDataMutex;
  std::string myName;
  std::list<ArTime> myTimes;
  std::list<ArPose> myPoses;
  size_t mySize;
  bool myLogPrediction;
  int myAllowedMSForPrediction;
  int myAllowedPercentageForPrediction;
};

#endif
